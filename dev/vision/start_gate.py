"""发车遮挡检测：蓝板被移开 = 发车信号（主方案 §3.1.7）。

**必须边沿触发**：只有「先确认有遮挡 → 再确认遮挡消失」的跳变才算发车信号。
绝不能用"当前无遮挡"直接发车，否则一上电就冲出去。

判据（现场可调，见 config/settings.START_*）：
1. 主判据：ROI 内 HSV 蓝色像素占比；
2. 备选判据（默认关）：边缘密度（挡板贴近镜头时画质退化）。

超时只告警、不自动发车（保守策略）。
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from config import settings


@dataclass
class StartGateState:
    blocked: bool          # 本帧是否判定"有遮挡"
    armed: bool            # 是否已确认过"有遮挡"（边沿触发的前半段）
    released: bool         # 是否已确认"遮挡消失"（闩锁，一旦置位保持）
    blue_ratio: float      # ROI 内蓝色像素占比（调试窗口显示用）
    edge_density: float    # ROI 内边缘密度
    timed_out: bool        # 超过 START_TIMEOUT_S 仍未见遮挡（仅告警）


def blue_ratio(frame_bgr: np.ndarray,
               roi: Tuple[float, float, float, float] = None,
               hsv_low: Tuple[int, int, int] = None,
               hsv_high: Tuple[int, int, int] = None) -> float:
    """ROI 内蓝色像素占比 0~1。"""
    roi = roi or settings.START_GATE_ROI
    h, w = frame_bgr.shape[:2]
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return 0.0
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array(hsv_low or settings.START_BLUE_HSV_LOW, dtype=np.uint8),
                       np.array(hsv_high or settings.START_BLUE_HSV_HIGH, dtype=np.uint8))
    return float(np.count_nonzero(mask)) / float(mask.size)


def edge_density(frame_bgr: np.ndarray,
                 roi: Tuple[float, float, float, float] = None) -> float:
    """ROI 内边缘密度 0~1（Canny 边缘像素占比）。"""
    roi = roi or settings.START_GATE_ROI
    h, w = frame_bgr.shape[:2]
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return 0.0
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 140)
    return float(np.count_nonzero(edges)) / float(edges.size)


class StartGate:
    """边沿触发的发车检测器（有状态，逐帧调用 update）。"""

    def __init__(self,
                 roi: Optional[Tuple[float, float, float, float]] = None,
                 blue_ratio_thresh: float = None,
                 arm_frames: int = None,
                 release_frames: int = None,
                 timeout_s: float = None,
                 use_edge_density: bool = None,
                 edge_density_thresh: float = None):
        self.roi = roi or settings.START_GATE_ROI
        self.blue_ratio_thresh = (settings.START_BLUE_RATIO_THRESH
                                  if blue_ratio_thresh is None else blue_ratio_thresh)
        self.arm_frames = settings.START_ARM_FRAMES if arm_frames is None else arm_frames
        self.release_frames = settings.START_RELEASE_FRAMES if release_frames is None else release_frames
        self.timeout_s = settings.START_TIMEOUT_S if timeout_s is None else timeout_s
        self.use_edge_density = (settings.START_USE_EDGE_DENSITY
                                 if use_edge_density is None else use_edge_density)
        self.edge_density_thresh = (settings.START_EDGE_DENSITY_THRESH
                                    if edge_density_thresh is None else edge_density_thresh)
        self._t0: Optional[float] = None
        self._reset_counters()

    # ------------------------------------------------------------------ 内部
    def _reset_counters(self) -> None:
        self._blocked_frames = 0
        self._clear_frames = 0
        self._armed = False
        self._released = False
        self._warned = False

    # ------------------------------------------------------------------ 接口
    def reset(self) -> None:
        """复位（重新进入发车等待时调用）。"""
        self._t0 = None
        self._reset_counters()

    def update(self, frame_bgr: np.ndarray, now: Optional[float] = None) -> StartGateState:
        now = time.monotonic() if now is None else now
        if self._t0 is None:
            self._t0 = now

        r_blue = blue_ratio(frame_bgr, self.roi)
        r_edge = edge_density(frame_bgr, self.roi)
        blocked = r_blue >= self.blue_ratio_thresh
        if self.use_edge_density:
            blocked = blocked and (r_edge <= self.edge_density_thresh)

        if not self._armed:
            if blocked:
                self._blocked_frames += 1
                if self._blocked_frames >= self.arm_frames:
                    self._armed = True
                    print(f"[START_GATE] 已确认遮挡（blue={r_blue:.2f}），等待挡板移开")
            else:
                self._blocked_frames = 0
        elif not self._released:
            if blocked:
                self._clear_frames = 0
            else:
                self._clear_frames += 1
                if self._clear_frames >= self.release_frames:
                    self._released = True
                    print(f"[START_GATE] 挡板已移开（连续 {self._clear_frames} 帧无遮挡）→ 允许发车")

        timed_out = (not self._armed) and (now - self._t0) >= self.timeout_s
        if timed_out and not self._warned:
            self._warned = True
            print(f"[START_GATE] ⚠️ {self.timeout_s:.0f}s 内未检测到遮挡："
                  f"检查蓝板是否在视野内 / 阈值 START_BLUE_RATIO_THRESH={self.blue_ratio_thresh}"
                  f"（当前 blue={r_blue:.2f}）—— 保守起见不自动发车")

        return StartGateState(blocked=blocked, armed=self._armed, released=self._released,
                              blue_ratio=r_blue, edge_density=r_edge, timed_out=timed_out)
