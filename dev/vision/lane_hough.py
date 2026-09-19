"""Hough 循线通道：Canny 边缘 + 霍夫直线 + 斜率先验（**不依赖颜色**，巡线主力）。

为什么要有这条通道（2026-09-19 操场实测结论）：
HSV 白线方案在塑胶跑道/暖光下不可靠 —— 白线饱和度 S≈69~125 与红底重叠、
反光斑比白线还亮 40 级。本模块看**灰度突变**（Canny）：线的本质是
"明暗交界的直线"，与颜色无关；反光斑内部没有边缘、形不成直线，天然免疫。

逻辑来源与融合（两轮实车验证的合流）：
- 骨架移植自 2025 届实车跑通的 `oldCode/src/vision/vision.cpp picture()`；
- 2026-09-19 操场第一轮：把骨架并进 LaneScanner.scan() 实测（9 组参数，
  中心稳定 370~378），得出斜率先验/前瞻带/收敛校验/支持度阈值四件套；
- 本模块是第二轮整理：保留上述全部现场结论，补上第一轮缺的三件事——
  ① Canny 自适应**跨帧持久**（第一轮每帧从设置重算，白适应了个寂寞）+ 钳位；
  ② **配对枚举**选线：画面里有多条平行白线（田径跑道到处是线），
    "各侧取最长线段"会配错对、浅斜率线外推后中心飞出画面（实测 cx=874/-10），
    改为枚举左右候选对，用「车道宽度先验 + 透视收敛 + 支持度 + 时序连续性」共同把关；
  ③ **单侧降级**：一侧丢线时用车道宽度先验推中心、置信度降到
    `LANE_HOUGH_SINGLE_SIDE_CONF` 交仲裁层半油门（第一轮直接拒收 →
    一次丢线就停车，有"停止超 20s 判失败"的风险）。

输出与 `lane_scan.LaneScanner` 完全同构的 `LaneObservation`，仲裁/PID 零改动。
"""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

from config import settings
from vision.lane_scan import LaneObservation


def hough_segments(edges: np.ndarray, y_top: int) -> list:
    """HoughLinesP → [(a, b, length, y_lo, y_hi)]，直线按 **x = a·y + b** 参数化。

    用 dx/dy 而不是 oldCode 的 dy/dx：车道线在近处可以很"竖"（dy/dx → ∞ 被
    斜率窗口误杀），dx/dy 参数化对竖直线稳定。斜率窗口 |a| ∈
    [LANE_SLOPE_MIN, LANE_SLOPE_MAX] 一条就能滤掉地面纹理/颗粒/水平纸边
    （车道线在画面里必然是斜的——越远越靠中间）。
    ⚠️ b 用全帧坐标（y_top 是 ROI 上沿）——Hough 段的 y 是 ROI 局部坐标，
    不加偏移的话所有外推都是垃圾（2026-09-19 真图 cx=874/-10 的根因之一）。
    """
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0 * 3,
                            threshold=int(settings.LANE_HOUGH_THRESH),
                            minLineLength=int(settings.LANE_HOUGH_MIN_LEN),
                            maxLineGap=int(settings.LANE_HOUGH_MAX_GAP))
    out = []
    if lines is None:
        return out
    for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
        y1f, y2f = float(y1 + y_top), float(y2 + y_top)
        if abs(y2f - y1f) < 1e-6:
            continue
        a = (float(x2) - float(x1)) / (y2f - y1f)
        if not (settings.LANE_SLOPE_MIN <= abs(a) <= settings.LANE_SLOPE_MAX):
            continue
        b = float(x1) - a * y1f
        length = float(np.hypot(float(x2) - float(x1), y2f - y1f))
        out.append((a, b, length, min(y1f, y2f), max(y1f, y2f)))
    return out


def side_support(segments: list, side: str) -> dict:
    """某一侧的支持度：{'len_sum', 'n', 'segs'}（按斜率符号分组）。

    车道线是一组共线的 Hough 碎段；len_sum 大说明"这一侧真有条线"。
    """
    sign = -1 if side == "left" else +1
    segs = [s for s in segments if (s[0] < 0) == (sign < 0)]
    return {"len_sum": float(sum(s[2] for s in segs)), "n": len(segs), "segs": segs}


class HoughLaneScanner:
    """Canny+Hough 循线器。**有状态**（Canny 阈值跨帧自适应 + 时序连续性）。"""

    def __init__(self,
                 target_x: float = None,
                 img_w: int = None,
                 canny_low: int = None,
                 canny_high: int = None):
        self.target_x = float(settings.TARGET_X if target_x is None else target_x)
        self.img_w = int(settings.IMG_W if img_w is None else img_w)
        self.canny_low = float(settings.LANE_CANNY_LOW if canny_low is None else canny_low)
        self.canny_high = float(settings.LANE_CANNY_HIGH if canny_high is None else canny_high)
        # 时序状态
        self._last_center: Optional[float] = None
        self._last_pair_w: Optional[float] = None
        self._last_seen: int = -10**9
        self._frame_count: int = 0
        self.last_info: dict = {}

    # ------------------------------------------------------------------ 内部
    def _adapt_canny(self, edge_count: int) -> None:
        """参考实现的自适应阈值：边缘太多提阈值、太少降阈值。

        与第一轮实现的差别：**跨帧持久**（第一轮每帧从设置重算，自适应结果
        随帧丢弃，等于没自适应）+ 上下钳位（参考实现没钳位，会漂到 0/255）。
        """
        if not settings.LANE_CANNY_ADAPT:
            return
        if edge_count > settings.LANE_CANNY_TARGET_HI:
            self.canny_low += 2
            self.canny_high += 4
        elif edge_count < settings.LANE_CANNY_TARGET_LO:
            self.canny_low -= 2
            self.canny_high -= 4
        self.canny_low = float(np.clip(self.canny_low,
                                       settings.LANE_CANNY_LOW_MIN, settings.LANE_CANNY_LOW_MAX))
        self.canny_high = float(np.clip(self.canny_high,
                                        settings.LANE_CANNY_HIGH_MIN, settings.LANE_CANNY_HIGH_MAX))

    def _canny(self, frame_bgr: np.ndarray, roi_y0: int, roi_y1: int) -> np.ndarray:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        roi = gray[roi_y0:roi_y1, :]
        blur = cv2.GaussianBlur(roi, (5, 5), 0.5)
        edges = cv2.Canny(blur, int(self.canny_low), int(self.canny_high), 3)
        self._adapt_canny(int(np.count_nonzero(edges)))
        return edges

    def _ref_center(self, img_w: int) -> float:
        fresh = (self._frame_count - self._last_seen) <= settings.LANE_HOUGH_HOLD_FRAMES
        return self._last_center if (fresh and self._last_center is not None) else img_w / 2.0

    # ------------------------------------------------------------------ 接口
    def scan(self, frame_bgr: np.ndarray) -> LaneObservation:
        """对一帧循线，输出与 lane_scan.LaneScanner 同构的观测量。"""
        h, w = frame_bgr.shape[:2]
        roi_y0 = max(0, int(h * settings.LANE_ROI_TOP_RATIO))
        roi_y1 = min(h, h - int(settings.LANE_ROI_BOTTOM_MARGIN))
        if roi_y1 <= roi_y0:
            return LaneObservation(center_x=self.target_x, confidence=0.0, source="hough")

        edges = self._canny(frame_bgr, roi_y0, roi_y1)
        segments = hough_segments(edges, roi_y0)
        left = side_support(segments, "left")
        right = side_support(segments, "right")
        self._frame_count += 1

        # 前瞻带：ROI 上部（看远处，参考实现/第一轮实测同款）
        span = roi_y1 - roi_y0
        y_a = roi_y0
        y_b = roi_y0 + max(1, int(span * settings.LANE_LOOKAHEAD_RATIO))
        y_look = (y_a + y_b) / 2.0

        def x_at(seg: tuple, y: float) -> float:
            return seg[0] * y + seg[1]

        def seg_len(seg: tuple) -> float:
            return seg[2]

        ok_l = left["len_sum"] >= settings.LANE_MIN_SEG_LEN_SUM
        ok_r = right["len_sum"] >= settings.LANE_MIN_SEG_LEN_SUM
        ref_center = self._ref_center(w)
        w_min = settings.LANE_PAIR_W_MIN_PX
        w_max = settings.LANE_PAIR_W_MAX_PX

        # ---- 配对枚举：宽度先验 + 透视收敛 + 连续性 + 支持度打分 ----
        best = None            # (score, lseg, rseg, pair_w, w_top)
        if ok_l and ok_r:
            fresh = (self._frame_count - self._last_seen) <= settings.LANE_HOUGH_HOLD_FRAMES
            for ls in left["segs"]:
                for rs in right["segs"]:
                    x_l, x_r = x_at(ls, y_look), x_at(rs, y_look)
                    pair_w = x_r - x_l
                    if not (w_min <= pair_w <= w_max):
                        continue
                    # 透视收敛校验：远端（ROI 顶）宽度必须明显小于近端，否则配错车道
                    w_top = x_at(rs, roi_y0) - x_at(ls, roi_y0)
                    if w_top >= pair_w * settings.LANE_CONVERGE_MAX_RATIO:
                        continue
                    mid = (x_l + x_r) / 2.0
                    if fresh and self._last_center is not None \
                            and abs(mid - self._last_center) > settings.LANE_HOUGH_CENTER_JUMP_PX:
                        continue
                    score = left["len_sum"] + right["len_sum"] - abs(mid - ref_center)
                    if best is None or score > best[0]:
                        best = (score, ls, rs, pair_w, w_top)

        if best is not None:
            _score, ls, rs, pair_w, w_top = best
            centers = [((x_at(ls, y) + x_at(rs, y)) * 0.5)
                       for y in range(y_a, y_b, max(1, settings.LANE_FIT_ROW_STEP))]
            center_x = float(np.mean(centers))
            x_l, x_r = x_at(ls, y_look), x_at(rs, y_look)
            cover = min(left["len_sum"], right["len_sum"]) / float(2 * max(1.0, y_b - y_a))
            converge = float(np.clip(1.0 - w_top / max(1.0, pair_w), 0.0, 1.0))
            support = float(np.clip(min(left["n"], right["n"]) / 6.0, 0.0, 1.0))
            confidence = float(np.clip(
                cover * (0.45 + 0.35 * converge + 0.20 * support), 0.0, 1.0))
            self._last_center = center_x
            self._last_pair_w = pair_w
            self._last_seen = self._frame_count
            self.last_info = {
                "canny": (int(self.canny_low), int(self.canny_high)),
                "segments": len(segments),
                "left_support": left["n"], "right_support": right["n"],
                "mode": "pair", "ref_center": ref_center,
            }
            return LaneObservation(
                center_x=center_x, confidence=confidence,
                left_x=int(x_l), right_x=int(x_r),
                valid_rows=len(centers), total_rows=max(1, y_b - y_a),
                source="hough",
            )

        # ---- 单侧降级：宽度先验推中心（不用图像边缘兜底——那会把中点拉飞）----
        if settings.LANE_HOUGH_ALLOW_SINGLE and (ok_l or ok_r):
            on_left = ok_l                                   # 两边都行时选支持度高的
            if ok_l and ok_r:
                on_left = left["len_sum"] >= right["len_sum"]
            side = left if on_left else right
            seg = max(side["segs"], key=seg_len)
            pair_w = self._last_pair_w if self._last_pair_w \
                else 2 * settings.LANE_HOUGH_HALF_W_DEFAULT_PX
            centers = []
            for y in range(y_a, y_b, max(1, settings.LANE_FIT_ROW_STEP)):
                x_line = x_at(seg, y)
                centers.append((x_line + (x_line + pair_w * (1 if on_left else -1))) * 0.5)
            center_x = float(np.mean(centers))
            x_line = x_at(seg, y_look)
            self._last_center = center_x
            self._last_seen = self._frame_count
            self.last_info = {
                "canny": (int(self.canny_low), int(self.canny_high)),
                "segments": len(segments),
                "left_support": left["n"], "right_support": right["n"],
                "mode": "single", "ref_center": ref_center,
            }
            return LaneObservation(
                center_x=center_x,
                confidence=float(settings.LANE_HOUGH_SINGLE_SIDE_CONF),
                left_x=int(x_line) if on_left else None,
                right_x=int(x_line) if not on_left else None,
                valid_rows=len(centers), total_rows=max(1, y_b - y_a),
                source="hough",
            )

        # ---- 什么线都没有 ----
        self.last_info = {
            "canny": (int(self.canny_low), int(self.canny_high)),
            "segments": len(segments),
            "left_support": left["n"], "right_support": right["n"],
            "mode": "none", "ref_center": ref_center,
        }
        return LaneObservation(center_x=self.target_x, confidence=0.0,
                               valid_rows=0, total_rows=max(1, y_b - y_a),
                               source="hough")

    def debug_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """返回 Canny 边缘图（调试窗口用；调用 scan() 才会推进自适应阈值）。"""
        h = frame_bgr.shape[0]
        roi_y0 = max(0, int(h * settings.LANE_ROI_TOP_RATIO))
        roi_y1 = min(h, h - int(settings.LANE_ROI_BOTTOM_MARGIN))
        gray = cv2.cvtColor(frame_bgr[roi_y0:roi_y1], cv2.COLOR_BGR2GRAY)
        return cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0.5),
                         int(self.canny_low), int(self.canny_high), 3)
