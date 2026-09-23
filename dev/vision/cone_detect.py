"""锥桶检测（P1-3）：蓝色连通域通道 + 可选的 RKNN 模型通道，统一输出 ConeBox。

背景（2026-09-23 起，操场实测后的设计）：
- 参考实现（oldCode hardware.cpp `blue()` + vision.cpp `Obstacles()`）就是用 HSV 蓝色
  + 开运算 + 最大连通域，尺寸闸门区分挡板与锥桶——本模块同源，但**按 640×480 重标定口径**，
  且给出**全部候选框**（两个锥桶都要看见，不是只取最大域）。
- 锥桶的蓝色比挡板浅（2026-09-07 采集图实测：H 中位 90~96、S 中位 63~72、V 中位 160+），
  阈值独立于挡板（`CONE_BLUE_HSV_*`），不要复用 START_BLUE_HSV（S≥80 会漏掉大半锥桶）。
- 通道选择：`--cone-method hsv|model`。hsv 不依赖权重（本地图片可验、操场上可兜底）；
  model 走 `rknn_detector` + `car4cls` profile 的 coneBucket 类（车上正式路线，权重只放车上）。

输入：BGR 640×480 帧（下摄 /dev/video2）。
输出：`ConeBox` 列表——xyxy、中心、底边（估距用）、尺寸、面积占比、置信度。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from config import settings


@dataclass
class ConeBox:
    xyxy: Tuple[float, float, float, float]     # (x0, y0, x1, y1) 像素（原图坐标）
    area_ratio: float                            # 蓝色连通域面积 / ROI 面积
    conf: float                                  # 置信度（hsv 通道=面积占比×形状系数；model 通道=模型分数）

    @property
    def center_x(self) -> float:
        return (self.xyxy[0] + self.xyxy[2]) / 2.0

    @property
    def bottom_y(self) -> float:
        """框底边：估距离用它（竖立物体会偏高）。"""
        return float(self.xyxy[3])

    @property
    def width(self) -> float:
        return float(self.xyxy[2] - self.xyxy[0])

    @property
    def height(self) -> float:
        return float(self.xyxy[3] - self.xyxy[1])

    def __repr__(self) -> str:                  # noqa: D105
        return (f"ConeBox(x={self.center_x:.0f} yb={self.bottom_y:.0f} "
                f"{self.width:.0f}x{self.height:.0f} conf={self.conf:.2f})")


def _roi_slice(frame_bgr: np.ndarray, roi: Tuple[float, float, float, float]):
    h, w = frame_bgr.shape[:2]
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    return frame_bgr[y0:y1, x0:x1]


def blue_mask(frame_bgr: np.ndarray,
              roi: Tuple[float, float, float, float] = None,
              hsv_low=None, hsv_high=None,
              open_px: int = 3) -> np.ndarray:
    """ROI 内的蓝色二值掩膜（开运算去噪），与原图同尺寸（ROI 外为 0）。"""
    if frame_bgr is None:
        return np.zeros((1, 1), dtype=np.uint8)
    h, w = frame_bgr.shape[:2]
    roi = roi or settings.CONE_GATE_ROI
    mask = np.zeros((h, w), dtype=np.uint8)
    patch = _roi_slice(frame_bgr, roi)
    if patch.size == 0:
        return mask
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    low = np.array(hsv_low or settings.CONE_BLUE_HSV_LOW, dtype=np.uint8)
    high = np.array(hsv_high or settings.CONE_BLUE_HSV_HIGH, dtype=np.uint8)
    m = cv2.inRange(hsv, low, high)
    if open_px > 1:
        kernel = np.ones((open_px, open_px), np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
    x0, _x1 = int(w * roi[0]), int(w * roi[1])
    y0, _y1 = int(h * roi[2]), int(h * roi[3])
    mask[y0:_y1, x0:_x1] = m
    return mask


def detect_cones_hsv(frame_bgr: np.ndarray,
                     roi: Tuple[float, float, float, float] = None,
                     hsv_low=None, hsv_high=None,
                     min_w: Optional[int] = None, min_h: Optional[int] = None,
                     max_w: Optional[int] = None, max_h: Optional[int] = None,
                     min_area_ratio: Optional[float] = None,
                     max_area_ratio: Optional[float] = None) -> List[ConeBox]:
    """HSV 蓝色连通域 → 锥桶候选框列表（全部候选，按面积降序）。

    闸门（口径 640×480，参考实现按 320×240 的 w>50/h>70 翻倍折算）：
    - 框宽 ∈ [CONE_MIN_W, CONE_MAX_W]、高 ∈ [CONE_MIN_H, CONE_MAX_H]
      （MAX 挡"近处挡板/大蓝物"：板几乎占满画面宽，锥桶远窄于它）；
    - 连通域面积 / ROI 面积 ∈ [CONE_MIN_AREA_RATIO, CONE_MAX_AREA_RATIO]
      （下限滤零散蓝点，上限挡贴脸大板）。
    """
    h, w = frame_bgr.shape[:2]
    roi = roi or settings.CONE_GATE_ROI
    min_w = settings.CONE_MIN_W if min_w is None else min_w
    min_h = settings.CONE_MIN_H if min_h is None else min_h
    max_w = settings.CONE_MAX_W if max_w is None else max_w
    max_h = settings.CONE_MAX_H if max_h is None else max_h
    min_area_ratio = (settings.CONE_MIN_AREA_RATIO if min_area_ratio is None
                      else min_area_ratio)
    max_area_ratio = (settings.CONE_MAX_AREA_RATIO if max_area_ratio is None
                      else max_area_ratio)

    mask = blue_mask(frame_bgr, roi, hsv_low, hsv_high)
    roi_area = float((int(w * roi[1]) - int(w * roi[0]))
                     * (int(h * roi[3]) - int(h * roi[2])))
    if roi_area <= 0:
        return []

    n, _labels, stats, _cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    boxes: List[ConeBox] = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if bw < min_w or bw > max_w or bh < min_h or bh > max_h:
            continue
        area_ratio = float(area) / roi_area
        if area_ratio < min_area_ratio or area_ratio > max_area_ratio:
            continue
        # 扁平条硬过滤：高 < 0.5×宽 的蓝色横带（反光带/板底边）不可能是立着的锥桶
        # （本地验证实测：负样本里 207×40、224×58 这类 4:1 以上的蓝条全是反光/杂物）
        if bh < 0.5 * bw:
            continue
        # 形状系数：竖立锥桶应显著"高≥宽"（≥0.6 即可，远处小框更方）。低分但非零仍保留，
        # 让上游按数量/防抖决策——参考实现只有面积闸门，这里只是弱先验。
        shape = 1.0 if bh >= 0.6 * bw else 0.6 * bh / max(bw, 1)
        conf = min(1.0, area_ratio * 25.0 * shape)
        boxes.append(ConeBox(xyxy=(float(x), float(y), float(x + bw), float(y + bh)),
                             area_ratio=area_ratio, conf=conf))
    boxes.sort(key=lambda b: b.area_ratio, reverse=True)
    return boxes


class ConeDetector:
    """锥桶检测器（通道可切）：hsv = 本地可验的蓝色连通域；model = 车上 RKNN。

    用法：
        det = ConeDetector(method="hsv")
        cones = det.detect(frame)          # -> List[ConeBox]
    给 `model_path`（且本机装得起来 rknnlite）时自动走模型；load 失败回退 hsv 并告警。
    """

    def __init__(self, method: str = "hsv", model_path: Optional[str] = None,
                 profile_name: str = "car4cls", conf_threshold: Optional[float] = None,
                 roi: Optional[Tuple[float, float, float, float]] = None):
        self.method = method
        self.roi = roi
        self.model_path = model_path
        self._rknn = None
        self._profile = None
        if method == "model":
            if not model_path:
                raise ValueError("cone-method=model 需要 --cone-model 给权重路径")
            self._load_model(profile_name, conf_threshold)

    def _load_model(self, profile_name: str, conf_threshold: Optional[float]) -> None:
        from vision.elements import load_profile
        from vision.postprocess import postprocess
        from vision.rknn_detector import RKNNYoloDet
        self._profile = load_profile(profile_name)
        self._postprocess = postprocess
        self._model = RKNNYoloDet(self.model_path, self._profile.imgsz)
        if not self._model.load():
            print(f"[CONE] ⚠️ RKNN 加载失败（{self.model_path}），回退 HSV 通道")
            self._model = None
            self.method = "hsv"
            return
        self._conf_threshold = conf_threshold
        print(f"[CONE] 模型通道：{profile_name} ({self._profile.num_classes} 类) conf≥"
              f"{conf_threshold if conf_threshold is not None else self._profile.conf}")

    def detect(self, frame_bgr: np.ndarray) -> List[ConeBox]:
        if self.method == "hsv":
            return detect_cones_hsv(frame_bgr, roi=self.roi)
        if self._model is None:
            return detect_cones_hsv(frame_bgr, roi=self.roi)
        try:
            elements = self._postprocess(self._model.infer(self._model.preprocess(frame_bgr)),
                                         self._profile,
                                         conf_threshold=self._conf_threshold,
                                         box_transform=self._model.restore)
        except Exception as exc:                       # 推理异常不许带崩主循环
            print(f"[CONE] ⚠️ 推理失败：{exc}；本帧按无锥桶")
            return []
        cones = [ConeBox(xyxy=e.xyxy, area_ratio=0.0,
                         conf=e.conf if e.conf is not None else 0.0)
                 for e in elements if e.name == "cone"]
        # 模型坐标即原图坐标（480×640 原生直喂）；ROI 过滤只留画面中下部的地面目标
        if self.roi is not None:
            h, w = frame_bgr.shape[:2]
            x0, x1 = int(w * self.roi[0]), int(w * self.roi[1])
            y0, y1 = int(h * self.roi[2]), int(h * self.roi[3])
            cones = [c for c in cones
                     if x0 <= c.center_x <= x1 and y0 <= c.bottom_y <= y1]
        cones.sort(key=lambda c: c.conf, reverse=True)
        return cones

    def release(self) -> None:
        if getattr(self, "_model", None) is not None:
            self._model.release()
            self._model = None


class ConeTracker:
    """锥桶防抖（进/出都要求连续 N 帧），给避让决策一个稳定的"有/无"。

    参考实现（oldCode）是单帧检测到就绕，AGENTS.md 规范要求防抖：进 N 帧、退 N 帧。
    """

    def __init__(self, enter_frames: Optional[int] = None,
                 exit_frames: Optional[int] = None):
        self.enter_frames = (settings.CONE_HYSTERESIS_FRAMES if enter_frames is None
                             else enter_frames)
        self.exit_frames = (max(2, self.enter_frames // 2) if exit_frames is None
                            else exit_frames)
        self._seen = 0
        self._missing = 0
        self.present = False
        self.last: Optional[ConeBox] = None

    def update(self, cones: List[ConeBox]) -> bool:
        if cones:
            self._seen += 1
            self._missing = 0
            c = cones[0]                    # 最大/最可信的那个
            if self._seen >= self.enter_frames:
                self.present = True
                self.last = c
        else:
            self._missing += 1
            if self.present:
                if self._missing >= self.exit_frames:
                    self.present = False
            else:
                self._seen = 0              # 从未成立时清掉积累，防止缓慢爬升误触发
        return self.present

    def reset(self) -> None:
        self._seen = 0
        self._missing = 0
        self.present = False
        self.last = None