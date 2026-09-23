"""RKNN YOLO 检测器（小车端，使用 NPU AUTO 多核）。

输入约定 —— **三条都是实测踩出来的，不要凭直觉改**（见 doc/数据与模型方案.md §7.4）：

1. **NHWC 布局**，不是 NCHW。喂 NCHW 时 RKNN 会把它当 NHWC 解读，
   输出完全错位（同一张图最大置信度 0.42 vs 正确的 0.71，框位置也偏）。
2. **RGB 通道序**（摄像头给的是 BGR，要转）。喂 BGR 时置信度会掉到 0.01 量级。
3. **uint8 0~255**，归一化（/255）在 RKNN 图内完成——转换时设了 mean=0/std=255。
   若哪天改成喂 float 0~1，`dev/tools/convert_rknn.py` 的 mean/std 必须同步改。

关于尺寸与坐标：
模型输入就是摄像头原生分辨率 **640x480** 时，**不做任何缩放**，
模型输出的框坐标天然就是原图坐标，后处理无需换算。
若输入尺寸与模型不符，退化为 letterbox（等比例缩放 + 灰边填充），
并把变换参数记下来，由 `restore()` 把框坐标还原回原图——
不还原的话 y 会系统性偏差（历史上 `postprocess.py` 就因为这个把 y 报大了 33%）。

> 2026-09-19 说明：本文件曾在车端被直接改写却没同步回仓库，被一次单向同步覆盖；
> 现按车端 `__pycache__` 里的字节码恢复并整理。**改车上代码必须回传仓库**（AGENTS.md §1.3）。
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

DEFAULT_INPUT_SIZE: Tuple[int, int] = (480, 640)    # (h, w)，与摄像头原生分辨率一致
_PAD_VALUE = 114                                    # letterbox 灰边（与训练时一致）


class RKNNYoloDet:
    """RKNN YOLO 检测器（也兼容上一届的分割模型，输出解码方式一致）。"""

    def __init__(self, model_path: str, input_size: Sequence[int] = DEFAULT_INPUT_SIZE):
        self.model_path = model_path
        if isinstance(input_size, int):
            input_size = (input_size, input_size)
        self.input_size: Tuple[int, int] = (int(input_size[0]), int(input_size[1]))
        self.rknn = None
        self._transform: Optional[Tuple[float, int, int]] = None   # (ratio, pad_x, pad_y)

    # ------------------------------------------------------------------ 生命周期
    def load(self) -> bool:
        from rknnlite.api import RKNNLite        # 延迟导入：本地（无 NPU）也能 import 本模块做单测
        self.rknn = RKNNLite()
        if self.rknn.load_rknn(self.model_path) != 0:
            return False
        if self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO) != 0:
            return False
        return True

    def release(self) -> None:
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None

    # ------------------------------------------------------------------ 预处理
    @property
    def is_native(self) -> bool:
        """**本帧**是否走了「原图直喂」（无缩放）路径。"""
        return self._transform is None

    def preprocess(self, frame_bgr: np.ndarray) -> np.ndarray:
        """原图 → 模型输入张量（NHWC uint8 RGB）。尺寸相符时零缩放。"""
        if frame_bgr is None:
            raise ValueError("preprocess 收到空帧")
        h, w = frame_bgr.shape[:2]
        th, tw = self.input_size
        if (h, w) == (th, tw):
            self._transform = None
            canvas = frame_bgr
        else:
            ratio = min(th / h, tw / w)
            nw, nh = int(round(w * ratio)), int(round(h * ratio))
            resized = cv2.resize(frame_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
            canvas = np.full((th, tw, 3), _PAD_VALUE, dtype=np.uint8)
            pad_x, pad_y = (tw - nw) // 2, (th - nh) // 2
            canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
            self._transform = (ratio, pad_x, pad_y)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(rgb[None])          # (1, h, w, 3) uint8

    # ------------------------------------------------------------------ 后处理
    def restore(self, boxes: np.ndarray) -> np.ndarray:
        """把模型坐标系的框还原到原图坐标；走了原图直喂时原样返回。

        传给 `vision.postprocess.parse_elements(box_transform=...)` 使用。
        """
        arr = np.asarray(boxes, dtype=np.float32)
        if self._transform is None or arr.size == 0:
            return arr
        ratio, pad_x, pad_y = self._transform
        out = arr.copy()
        out[:, [0, 2]] = (out[:, [0, 2]] - pad_x) / ratio
        out[:, [1, 3]] = (out[:, [1, 3]] - pad_y) / ratio
        return out

    def infer(self, img: np.ndarray) -> list:
        if self.rknn is None:
            raise RuntimeError("RKNN model not loaded")
        return self.rknn.inference(inputs=[img])


# 老名字（vision_main / debug_view 一直用它），保留为别名，调用点不用改
RKNNYoloSeg = RKNNYoloDet
