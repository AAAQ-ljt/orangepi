"""RKNN 检测器的预处理 / 坐标还原单测：合成图，不碰摄像头也不碰 NPU。

覆盖的是历史上真出过问题的两点（见 doc/数据与模型方案.md §7.4）：
  1. 喂给 RKNN 的张量必须是 **NHWC + RGB + uint8**；
  2. 输入尺寸与模型不符时走了 letterbox，就必须把框坐标还原回原图，
     否则 y 会系统性偏大（曾经把 y 报大了 33%）。

跑法：
    cd dev && PYTHONPATH=. python tests/test_rknn_detector.py
"""
from __future__ import annotations

import numpy as np

from vision.elements import as_hw
from vision.postprocess import parse_elements
from vision.rknn_detector import DEFAULT_INPUT_SIZE, RKNNYoloDet


# --------------------------------------------------------------- imgsz 规范化
def test_as_hw_accepts_int_and_list():
    assert as_hw(640) == (640, 640)
    assert as_hw([480, 640]) == (480, 640)      # [h, w]
    assert as_hw((480, 640)) == (480, 640)
    assert as_hw([640]) == (640, 640)


def test_profile_imgsz_is_hw():
    from vision.elements import load_profile
    assert load_profile("smartcar2026").imgsz == (480, 640)
    assert load_profile("legacy").imgsz == (640, 640)


# --------------------------------------------------------------- 原生尺寸路径
def test_native_size_needs_no_transform():
    """输入分辨率与模型一致时：零缩放，restore 必须是恒等映射。"""
    det = RKNNYoloDet("dummy.rknn", DEFAULT_INPUT_SIZE)      # 不 load，只测纯函数
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    x = det.preprocess(frame)

    assert x.shape == (1, 480, 640, 3), f"必须是 NHWC，实际 {x.shape}"
    assert x.dtype == np.uint8
    assert det.is_native, "尺寸相符时应走原图直喂路径"

    boxes = np.array([[10.0, 20.0, 30.0, 40.0]], dtype=np.float32)
    assert np.allclose(det.restore(boxes), boxes), "原图直喂时坐标不应被改动"


def test_preprocess_is_rgb_not_bgr():
    """通道序必须是 RGB：BGR 输入经过预处理后，R/B 通道应当互换。"""
    det = RKNNYoloDet("dummy.rknn", DEFAULT_INPUT_SIZE)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:, :, 2] = 200          # BGR 里 index 2 是红色通道
    x = det.preprocess(frame)
    assert x[0, 0, 0, 0] == 200, "红色应落在第 0 个通道（RGB）"
    assert x[0, 0, 0, 2] == 0


# --------------------------------------------------------------- 需要缩放的路径
def test_letterbox_fallback_and_restore():
    """尺寸不符时走 letterbox，restore 必须把坐标换算回原图。"""
    det = RKNNYoloDet("dummy.rknn", (480, 640))
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)         # 1280x720
    x = det.preprocess(frame)

    assert x.shape == (1, 480, 640, 3)
    assert not det.is_native, "尺寸不符时必须标记为非原生路径"

    ratio, pad_x, pad_y = det._transform
    # 1280x720(16:9) 比 640x480(4:3) 更"宽"，所以是左右填满、上下补边
    assert ratio == 0.5, "缩放比应为 0.5"
    assert ratio * 1280 == 640 and ratio * 720 == 360
    assert pad_x == 0, "宽度方向刚好填满，不应有左右补边"
    assert pad_y == 60, f"上下各补 (480-360)/2=60，实际 {pad_y}"

    # 模型坐标系里画一个框，还原后应回到原图坐标系
    boxes = np.array([[100.0, 100.0, 300.0, 300.0]], dtype=np.float32)
    back = det.restore(boxes)
    assert np.allclose(back[0, 0], (100.0 - pad_x) / ratio)
    assert np.allclose(back[0, 1], (100.0 - pad_y) / ratio)

    # 关键性质：把原图正中间的物体放到模型坐标系再还原，应回到正中间
    mid = np.array([[640.0, 360.0, 640.0, 360.0]], dtype=np.float32)   # 原图中心
    to_model = mid * ratio
    to_model[:, [0, 2]] += pad_x
    to_model[:, [1, 3]] += pad_y
    assert np.allclose(det.restore(to_model), mid, atol=1e-3), "还原后应回到原图坐标"


def test_restore_handles_empty():
    det = RKNNYoloDet("dummy.rknn", (480, 640))
    det.preprocess(np.zeros((720, 1280, 3), dtype=np.uint8))
    out = det.restore(np.zeros((0, 4), dtype=np.float32))
    assert out.shape == (0, 4)


# --------------------------------------------------------------- 与后处理联通
def test_postprocess_applies_box_transform():
    """parse_elements 应把 box_transform 应用到最终框上（模型坐标 -> 原图坐标）。"""
    from vision.elements import load_profile

    prof = load_profile("smartcar2026")
    # 合成输出 (1, 4+nc, 2)：一个高置信框 + 一个背景
    det_tensor = np.zeros((1, 4 + prof.num_classes, 2), dtype=np.float32)
    det_tensor[0, 0, 0], det_tensor[0, 1, 0] = 100.0, 100.0   # cx, cy
    det_tensor[0, 2, 0], det_tensor[0, 3, 0] = 40.0, 40.0     # w, h
    det_tensor[0, 4, 0] = 0.9                                  # blue_board 置信度

    def shift(boxes):
        out = np.asarray(boxes, dtype=np.float32).copy()
        out[:, [1, 3]] -= 50.0
        return out

    els = parse_elements([det_tensor], prof, conf_threshold=0.25, box_transform=shift)
    assert len(els) == 1
    assert els[0].xyxy[1] == 30.0, f"应减去 50，实际 {els[0].xyxy[1]}"
    assert els[0].xyxy[3] == 70.0


if __name__ == "__main__":
    test_as_hw_accepts_int_and_list()
    test_profile_imgsz_is_hw()
    test_native_size_needs_no_transform()
    test_preprocess_is_rgb_not_bgr()
    test_letterbox_fallback_and_restore()
    test_restore_handles_empty()
    test_postprocess_applies_box_transform()
    print("test_rknn_detector: all passed")
