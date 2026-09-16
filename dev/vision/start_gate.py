"""发车遮挡检测：蓝板被移开 = 发车信号（主方案 §3.1.7）。

**必须边沿触发**：只有「先确认有遮挡 → 再确认遮挡消失」的跳变才算发车信号。
绝不能用"当前无遮挡"直接发车，否则一上电就冲出去。

要覆盖的两种时序（都必须成立）：
  A. 程序启动**前**板已放好 → 启动即确认"见到板"，不动；板被移走才发车；
  B. 程序启动**后**板才放上 → 启动后绝不动；板放上 → 确认；板移走 → 才发车。
核心不变量：**没见过板，绝不放行**。

判据（三路，前两路必开，第三路可选）：
  ① 最大蓝色连通域面积 / ROI 面积   —— 比"全 ROI 蓝像素占比"耐噪（排除零散蓝色杂物）
  ② 蓝色主导度 mean(clip(B-max(R,G),0,255)) —— 对曝光/白平衡漂移比 HSV 硬阈值稳
  ③ 画面细节（拉普拉斯方差）骤降     —— 板贴到镜头前时大面积失焦，默认关闭
阈值都在 config/settings.py 的 START_* 里，现场可调。
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from config import settings


@dataclass
class BlueMetrics:
    """ROI 内的蓝色判据读数（都可用于现场标定与调试显示）。"""
    area_ratio: float      # 判据①：最大蓝色连通域面积 / ROI 面积
    pixel_ratio: float     # 蓝色像素总占比（参考值）
    dominance: float       # 判据②：B - max(R,G) 均值（0~255）
    detail: float          # 判据③：拉普拉斯方差（越大越清晰）

    @property
    def summary(self) -> str:
        return (f"面积占比={self.area_ratio:.3f} 蓝像素占比={self.pixel_ratio:.3f} "
                f"主导度={self.dominance:.1f} 细节={self.detail:.0f}")


@dataclass
class StartGateState:
    blocked: bool          # 本帧是否判定"有遮挡"
    armed: bool            # 是否已确认过"有遮挡"（边沿触发的前半段）
    released: bool         # 是否已确认"遮挡消失"（闩锁，一旦置位保持）
    metrics: BlueMetrics
    frames_blocked: int = 0
    frames_clear: int = 0
    timed_out: bool = False      # 超时（仅告警，不放行）
    note: str = ""


# ------------------------------------------------------------------ 判据计算
def _roi_slice(frame_bgr: np.ndarray, roi: Tuple[float, float, float, float]):
    h, w = frame_bgr.shape[:2]
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    return frame_bgr[y0:y1, x0:x1]


def blue_mask(frame_bgr: np.ndarray, roi: Tuple[float, float, float, float],
              hsv_low=None, hsv_high=None) -> np.ndarray:
    """ROI 内的蓝色二值掩膜（已做形态学开运算去噪）。"""
    patch = _roi_slice(frame_bgr, roi)
    if patch.size == 0:
        return np.zeros((1, 1), dtype=np.uint8)
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    low = np.array(hsv_low or settings.START_BLUE_HSV_LOW, dtype=np.uint8)
    high = np.array(hsv_high or settings.START_BLUE_HSV_HIGH, dtype=np.uint8)
    mask = cv2.inRange(hsv, low, high)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)


def blue_metrics(frame_bgr: np.ndarray,
                 roi: Tuple[float, float, float, float] = None,
                 hsv_low=None, hsv_high=None) -> BlueMetrics:
    """一次性算出三路判据的读数。"""
    roi = roi or settings.START_GATE_ROI
    patch = _roi_slice(frame_bgr, roi)
    if patch.size == 0:
        return BlueMetrics(0.0, 0.0, 0.0, 0.0)

    mask = blue_mask(frame_bgr, roi, hsv_low, hsv_high)
    roi_area = float(mask.size)
    pixel_ratio = float(np.count_nonzero(mask)) / roi_area

    # 判据①：最大连通域面积占比
    area_ratio = 0.0
    n, _labels, stats, _cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n > 1:
        area_ratio = float(stats[1:, cv2.CC_STAT_AREA].max()) / roi_area

    # 判据②：蓝色主导度（通道差，抗曝光/白平衡）。
    # 注意：整幅取均值会被背景稀释（板只占 3% 面积时均值只剩 5），
    # 所以取**最蓝的 10% 像素**的均值 —— 这样远处的板也能给出强信号。
    b = patch[:, :, 0].astype(np.int16)
    g = patch[:, :, 1].astype(np.int16)
    r = patch[:, :, 2].astype(np.int16)
    dom_vals = np.clip(b - np.maximum(r, g), 0, 255).ravel()
    if dom_vals.size:
        k = max(1, int(dom_vals.size * 0.10))
        topk = np.partition(dom_vals, -k)[-k:]
        dominance = float(topk.mean())
    else:
        dominance = 0.0

    # 判据③：细节量（拉普拉斯方差）
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    detail = float(cv2.Laplacian(gray, cv2.CV_64F).var())

    return BlueMetrics(area_ratio=area_ratio, pixel_ratio=pixel_ratio,
                       dominance=dominance, detail=detail)


# 兼容旧接口（测试与调试窗口的老用法）
def blue_ratio(frame_bgr: np.ndarray, roi=None, hsv_low=None, hsv_high=None) -> float:
    return blue_metrics(frame_bgr, roi, hsv_low, hsv_high).pixel_ratio


def edge_density(frame_bgr: np.ndarray, roi=None) -> float:
    roi = roi or settings.START_GATE_ROI
    patch = _roi_slice(frame_bgr, roi)
    if patch.size == 0:
        return 0.0
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 140)
    return float(np.count_nonzero(edges)) / float(edges.size)


class StartGate:
    """边沿触发的发车检测器（有状态，逐帧调用 update）。"""

    def __init__(self,
                 roi: Optional[Tuple[float, float, float, float]] = None,
                 area_thresh: Optional[float] = None,
                 area_thresh_low: Optional[float] = None,
                 dominance_thresh: Optional[float] = None,
                 arm_frames: Optional[int] = None,
                 release_frames: Optional[int] = None,
                 timeout_s: Optional[float] = None,
                 use_detail: Optional[bool] = None,
                 detail_drop_ratio: Optional[float] = None,
                 report_every_s: Optional[float] = None,
                 verbose: bool = True):
        self.roi = roi or settings.START_GATE_ROI
        self.area_thresh = (settings.START_BLUE_AREA_THRESH if area_thresh is None else area_thresh)
        self.area_thresh_low = (settings.START_BLUE_AREA_THRESH_LOW
                                if area_thresh_low is None else area_thresh_low)
        self.dominance_thresh = (settings.START_BLUE_DOMINANCE_THRESH
                                 if dominance_thresh is None else dominance_thresh)
        self.arm_frames = settings.START_ARM_FRAMES if arm_frames is None else arm_frames
        self.release_frames = (settings.START_RELEASE_FRAMES
                               if release_frames is None else release_frames)
        self.timeout_s = settings.START_TIMEOUT_S if timeout_s is None else timeout_s
        self.use_detail = (settings.START_USE_DETAIL if use_detail is None else use_detail)
        self.detail_drop_ratio = (settings.START_DETAIL_DROP_RATIO
                                  if detail_drop_ratio is None else detail_drop_ratio)
        self.report_every_s = (settings.START_REPORT_EVERY_S
                               if report_every_s is None else report_every_s)
        self.verbose = verbose
        self._t0: Optional[float] = None
        self._detail_baseline: Optional[float] = None
        self._reset_counters()

    # ------------------------------------------------------------------ 内部
    def _reset_counters(self) -> None:
        self._blocked_frames = 0
        self._clear_frames = 0
        self._armed = False
        self._released = False
        self._warned = False
        self._last_report = 0.0

    def reset(self) -> None:
        """复位（重新进入发车等待时调用）。"""
        self._t0 = None
        self._reset_counters()

    def _decide_blocked(self, m: BlueMetrics) -> bool:
        blocked = (m.area_ratio >= self.area_thresh) or \
                  (m.dominance >= self.dominance_thresh and m.area_ratio >= self.area_thresh_low)
        if not blocked and self.use_detail and self._detail_baseline:
            # 贴脸失焦：细节相对基线骤降，且有一点蓝色
            if (m.detail <= self._detail_baseline * (1.0 - self.detail_drop_ratio)
                    and m.area_ratio >= self.area_thresh_low * 0.5):
                blocked = True
        # 明显"没板"时更新细节基线（用于判据③）
        if m.area_ratio < self.area_thresh_low * 0.5:
            if self._detail_baseline is None:
                self._detail_baseline = m.detail
            else:
                self._detail_baseline = 0.9 * self._detail_baseline + 0.1 * m.detail
        return blocked

    # ------------------------------------------------------------------ 接口
    def update(self, frame_bgr: np.ndarray, now: Optional[float] = None) -> StartGateState:
        now = time.monotonic() if now is None else now
        if self._t0 is None:
            self._t0 = now

        m = blue_metrics(frame_bgr, self.roi)
        blocked = self._decide_blocked(m)

        if not self._armed:
            if blocked:
                self._blocked_frames += 1
                if self._blocked_frames >= self.arm_frames:
                    self._armed = True
                    if self.verbose:
                        print(f"[START_GATE] ✅ 已确认蓝板（{m.summary}）→ 等待移开，此时绝不动车")
            else:
                self._blocked_frames = 0
                # 还没见到板：定期报读数，方便操作员判断"车没对准"还是"板还没放"
                if self.verbose and (now - self._last_report) >= self.report_every_s:
                    self._last_report = now
                    print(f"[START_GATE] 等待蓝板… {m.summary} "
                          f"（阈值 面积≥{self.area_thresh:.3f} 或 主导度≥{self.dominance_thresh:.1f}）")
        elif not self._released:
            if blocked:
                self._clear_frames = 0
            else:
                self._clear_frames += 1
                if self._clear_frames >= self.release_frames:
                    self._released = True
                    if self.verbose:
                        print(f"[START_GATE] ✅ 挡板已移开（连续 {self._clear_frames} 帧无遮挡）→ 允许发车")

        timed_out = (not self._armed) and (now - self._t0) >= self.timeout_s
        if timed_out and not self._warned:
            self._warned = True
            print(f"[START_GATE] ⚠️ {self.timeout_s:.0f}s 内未检测到蓝板：{m.summary}\n"
                  f"            检查：板是否在视野内 / 阈值 START_BLUE_AREA_THRESH={self.area_thresh:.3f}、"
                  f"START_BLUE_DOMINANCE_THRESH={self.dominance_thresh:.1f} —— 保守起见不自动发车")

        return StartGateState(blocked=blocked, armed=self._armed, released=self._released,
                              metrics=m, frames_blocked=self._blocked_frames,
                              frames_clear=self._clear_frames, timed_out=timed_out)
