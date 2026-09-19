"""Hough 循线：Canny 边缘 + 霍夫直线 + 左右车道线回归（**不依赖颜色**）。

为什么要有这条通道（2026-09-19 实车结论，见 doc/实地调试清单.md）：
HSV 白线方案在打印跑道/暖光下不可靠 —— 白线饱和度 S≈69~125 与红底重叠、
反光斑比白线还亮 40 级，颜色/亮度阈值怎么调都会被反光骗。
本模块改用**亮度梯度**（Canny）找线：线的本质是"明暗交界的直线"，
与颜色无关，反光斑内部没有边缘、形不成直线，天然免疫。

逻辑移植自 `F:/smart_car/smart-car/oldCode/src/vision/vision.cpp` 的 `picture()`
（2025 届实车跑通版）， ours 化改造：
  1. Canny 阈值**跨帧自适应**（同 oldCode）：边缘多了提阈值、少了降阈值，
     相当于一个简易自动曝光补偿；加了上下限钳位（oldCode 没钳位，会漂到 0/255）；
  2. 斜率过滤 `|k| ∈ (0.25, 2.0)`：去掉近水平的斑马线/纸边和近垂直的噪声；
  3. 左线取最负 k、右线取最正 k（oldCode 同款）；
  4. 单侧丢线时用**图像边缘兜底**（oldCode 同款）——整帧一条直线方程是稳定的，
     不会像 HSV 逐行检测那样逐行乱跳（那是 2026-09-16 冲出赛道的根因）；
     但置信度降到 `HOUGH_SINGLE_SIDE_CONF`，让仲裁层按"降级"处理（半油门）；
  5. 在下方误差带内对 (左线x+右线x)/2 求平均 → center_x，输出与
     `lane_scan.LaneScanner` 完全同构的 `LaneObservation`，仲裁/PID 零改动。
"""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

from config import settings
from vision.lane_scan import LaneObservation


class HoughLaneScanner:
    """Canny+Hough 循线器。**有状态**（Canny 阈值跨帧自适应），但可任意帧调用。"""

    def __init__(self,
                 target_x: float = None,
                 img_w: int = None,
                 canny_lo: int = None,
                 canny_hi: int = None):
        self.target_x = float(settings.TARGET_X if target_x is None else target_x)
        self.img_w = int(settings.IMG_W if img_w is None else img_w)
        self.canny_lo = int(settings.HOUGH_CANNY_LO if canny_lo is None else canny_lo)
        self.canny_hi = int(settings.HOUGH_CANNY_HI if canny_hi is None else canny_hi)
        # 时序状态（跨帧连续性：线对选择用它拒"单帧误配"）
        self._last_center: Optional[float] = None   # 上一帧接受的中心
        self._last_pair_w: Optional[float] = None   # 上一帧的配对宽度（单侧兜底的半宽先验）
        self._last_seen: int = -10**9               # 上次有效帧号
        self._frame_count: int = 0
        # 最近一次诊断信息（debug_view / 探针用）
        self.last_info: dict = {}

    def _ref_center(self, img_w: int) -> float:
        """线对选择的参考中心：上一帧中心还新鲜就用它，否则用图像中心。"""
        fresh = (self._frame_count - self._last_seen) <= settings.HOUGH_HOLD_FRAMES
        return self._last_center if (fresh and self._last_center is not None) else img_w / 2.0

    # ------------------------------------------------------------------ 内部
    def _adapt_canny(self, edge_count: int, roi_area: int) -> None:
        """oldCode 的自适应阈值：边缘密度高了提阈值、低了降阈值（带钳位）。"""
        density = edge_count / float(max(1, roi_area))
        step_lo, step_hi = settings.HOUGH_CANNY_STEP
        if density > settings.HOUGH_EDGE_DENSITY_HI:
            self.canny_lo = min(self.canny_lo + step_lo, settings.HOUGH_CANNY_LO_LIMIT[1])
            self.canny_hi = min(self.canny_hi + step_hi, settings.HOUGH_CANNY_HI_LIMIT[1])
        elif density < settings.HOUGH_EDGE_DENSITY_LO:
            self.canny_lo = max(self.canny_lo - step_lo, settings.HOUGH_CANNY_LO_LIMIT[0])
            self.canny_hi = max(self.canny_hi - step_hi, settings.HOUGH_CANNY_HI_LIMIT[0])

    @staticmethod
    def _slope_filter(lines: np.ndarray, k_min: float, k_max: float,
                      y_offset: float = 0.0):
        """HoughLinesP 结果 -> [(k, b, 端点)]，只留 |k| 在窗口内的。

        k<0 是左线候选、k>0 是右线候选（y 向下坐标系）。
        ⚠️ b 用**全帧坐标**存（y_offset=ROI 上沿）：Hough 段的 y 是 ROI 局部坐标，
        误差带外推用的是全帧 y —— 不加偏移的话外推全是垃圾（oldCode 原版专门有
        `+ frame.rows/2` 这一步，漏掉就是 2026-09-19 真图 cx=874/-10 的根因之一）。
        """
        out = []
        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
            dx = float(x2 - x1)
            if abs(dx) < 1e-6:
                continue                      # 近垂直段不进 slope 窗口（k 会爆）
            k = float(y2 - y1) / dx
            if not (k_min <= abs(k) <= k_max):
                continue
            y1f = float(y1) + y_offset
            out.append((k, y1f - k * float(x1),
                        (float(x1), y1f, float(x2), float(y2) + y_offset)))
        return out

    # ------------------------------------------------------------------ 接口
    def scan(self, frame_bgr: np.ndarray) -> LaneObservation:
        """对一帧循线，输出与 lane_scan.LaneScanner 同构的观测量。

        线对选择（对 oldCode 的关键改造，2026-09-19 真图复盘）：
        oldCode 在"画面里只有一对车道线"的前提下用 min/max-k 独立取左右线；
        我们的实车画面是田径跑道，**平行白线到处都是**，min/max-k 会选中
        别的车道线，浅斜率线外推到误差带后中心能飞出画面（实测 cx=874 / -10）。
        现在改为：枚举左右候选对，要求
          ① 配对宽度（在误差带中点的间距）落在车道宽度先验 LANE_PAIR_W_* 内；
          ② 每条线外推到误差带两端后 x 不越出画面 ±30%；
          ③ 打分 = 线段支持度（越长越可信）+ 与上一帧中心的连续性；
          ④ 与上一帧中心偏差超过 HOUGH_CENTER_JUMP_PX 且上一帧还新鲜 → 拒收该对
            （防单帧误配把中心拉飞；连续多帧无效则允许重新捕获）。
        """
        h, w = frame_bgr.shape[:2]
        roi_y0 = int(h * settings.HOUGH_ROI_TOP_RATIO)
        roi_y1 = int(h * settings.HOUGH_ROI_BOTTOM_RATIO)
        band_y0 = int(h * settings.HOUGH_BAND_TOP_RATIO)
        band_y1 = int(h * settings.HOUGH_BAND_BOTTOM_RATIO)
        band_mid = (band_y0 + band_y1) / 2.0
        if roi_y1 <= roi_y0 or band_y1 <= band_y0:
            return LaneObservation(center_x=self.target_x, confidence=0.0, source="hough")

        roi = frame_bgr[roi_y0:roi_y1, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0.5, 0.5)
        edges = cv2.Canny(blurred, self.canny_lo, self.canny_hi, apertureSize=3)
        self._adapt_canny(int(np.count_nonzero(edges)), edges.size)

        raw = cv2.HoughLinesP(edges, rho=settings.HOUGH_RHO,
                              theta=np.deg2rad(settings.HOUGH_THETA_DEG),
                              threshold=settings.HOUGH_THRESH,
                              minLineLength=settings.HOUGH_MIN_LEN_PX,
                              maxLineGap=settings.HOUGH_MAX_GAP_PX)
        segs = self._slope_filter(raw, settings.HOUGH_K_ABS_MIN, settings.HOUGH_K_ABS_MAX,
                                  y_offset=roi_y0) if raw is not None else []

        def x_at(seg: tuple, y: float) -> float:
            k, b, _ = seg
            return (y - b) / k

        def seg_len(seg: tuple) -> float:
            x1, y1, x2, y2 = seg[2]
            return float(np.hypot(x2 - x1, y2 - y1))

        # 每条线外推到误差带两端必须大体在画面附近（±30%），否则是离谱外推
        x_lo, x_hi = -0.3 * w, 1.3 * w
        left_cands = [s for s in segs if s[0] < 0
                      and x_lo <= x_at(s, band_y0) <= x_hi and x_lo <= x_at(s, band_y1) <= x_hi]
        right_cands = [s for s in segs if s[0] > 0
                       and x_lo <= x_at(s, band_y0) <= x_hi and x_lo <= x_at(s, band_y1) <= x_hi]

        ref_center = self._ref_center(w)
        w_min = settings.LANE_PAIR_W_MIN_PX if settings.LANE_PAIR_W_MIN_PX else 110
        w_max = settings.LANE_PAIR_W_MAX_PX if settings.LANE_PAIR_W_MAX_PX else 620

        # ---- 枚举配对：宽度先验 + 支持度 + 连续性打分 ----
        best = None          # (score, lseg, rseg)
        for ls in left_cands:
            for rs in right_cands:
                lx_m, rx_m = x_at(ls, band_mid), x_at(rs, band_mid)
                sep = rx_m - lx_m
                if not (w_min <= sep <= w_max):
                    continue
                mid_m = (lx_m + rx_m) / 2.0
                # 硬闸门：上一帧中心还新鲜时，单帧偏移不许超过 HOUGH_CENTER_JUMP_PX
                fresh = (self._frame_count - self._last_seen) <= settings.HOUGH_HOLD_FRAMES
                if fresh and self._last_center is not None \
                        and abs(mid_m - self._last_center) > settings.HOUGH_CENTER_JUMP_PX:
                    continue
                score = seg_len(ls) + seg_len(rs)
                score -= abs(mid_m - ref_center)          # 离参考中心越近越好
                if best is None or score > best[0]:
                    best = (score, ls, rs)

        self._frame_count += 1
        if best is not None:
            _score, ls, rs = best
            mid_sum = 0.0
            for y in range(band_y0, band_y1):
                mid_sum += (x_at(ls, y) + x_at(rs, y)) / 2.0
            center_x = mid_sum / (band_y1 - band_y0)
            left_x, right_x = x_at(ls, band_mid), x_at(rs, band_mid)
            support = min(len(left_cands) + len(right_cands), 6)
            confidence = min(1.0, settings.HOUGH_FULL_CONF_BASE + 0.03 * support)
            self._last_center = center_x
            self._last_pair_w = right_x - left_x
            self._last_seen = self._frame_count
            self.last_info = {
                "canny": (self.canny_lo, self.canny_hi),
                "segments": len(segs),
                "left_support": len(left_cands), "right_support": len(right_cands),
                "mode": "pair", "ref_center": ref_center,
            }
            return LaneObservation(
                center_x=float(center_x), confidence=float(confidence),
                left_x=int(left_x), right_x=int(right_x),
                valid_rows=band_y1 - band_y0, total_rows=band_y1 - band_y0,
                source="hough",
            )

        # ---- 单侧兜底：不再用图像边缘（会把中点拉飞），用车道宽度先验推中心 ----
        # 选支持度更高的一侧；中心 = 线 ± 半宽（半宽优先用上一帧的实测配对宽度）
        left_best = max(left_cands, key=seg_len) if left_cands else None
        right_best = max(right_cands, key=seg_len) if right_cands else None
        if left_best is not None and (right_best is None
                                      or seg_len(left_best) >= seg_len(right_best)):
            side, on_left = left_best, True
        elif right_best is not None:
            side, on_left = right_best, False
        else:
            side = None

        if side is not None:
            half = (self._last_pair_w / 2.0) if self._last_pair_w \
                else settings.HOUGH_SINGLE_HALF_W_DEFAULT_PX
            mid_sum = 0.0
            for y in range(band_y0, band_y1):
                lx = x_at(side, y)
                # 左线的车道中心在它右侧（+宽），右线的中心在它左侧（−宽）
                rx = lx + (self._last_pair_w if self._last_pair_w
                           else 2 * settings.HOUGH_SINGLE_HALF_W_DEFAULT_PX) \
                    * (1 if on_left else -1)
                mid_sum += (lx + rx) / 2.0
            center_x = mid_sum / (band_y1 - band_y0)
            self._last_center = center_x
            self._last_seen = self._frame_count
            self.last_info = {
                "canny": (self.canny_lo, self.canny_hi),
                "segments": len(segs),
                "left_support": len(left_cands), "right_support": len(right_cands),
                "mode": "single", "ref_center": ref_center,
            }
            return LaneObservation(
                center_x=float(center_x),
                confidence=float(settings.HOUGH_SINGLE_SIDE_CONF),
                left_x=int(x_at(side, band_mid)) if on_left else None,
                right_x=int(x_at(side, band_mid)) if not on_left else None,
                valid_rows=band_y1 - band_y0, total_rows=band_y1 - band_y0,
                source="hough",
            )

        # ---- 什么线都没有 ----
        self.last_info = {
            "canny": (self.canny_lo, self.canny_hi),
            "segments": len(segs), "left_support": 0, "right_support": 0,
            "mode": "none", "ref_center": ref_center,
        }
        return LaneObservation(center_x=self.target_x, confidence=0.0,
                               valid_rows=band_y1 - band_y0,
                               total_rows=band_y1 - band_y0, source="hough")

    def debug_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """返回 Canny 边缘图（调试窗口用；调用 scan() 才会推进自适应阈值）。"""
        h, w = frame_bgr.shape[:2]
        roi_y0, roi_y1 = int(h * settings.HOUGH_ROI_TOP_RATIO), int(h * settings.HOUGH_ROI_BOTTOM_RATIO)
        gray = cv2.cvtColor(frame_bgr[roi_y0:roi_y1], cv2.COLOR_BGR2GRAY)
        return cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0.5, 0.5), self.canny_lo, self.canny_hi)
