"""扫线：白线检测 + 逐行取中点 + 纵向加权误差（巡线主力）。

思路来自官方上届 `opencv版本…/code/image.cpp`（逐行从中线向两侧找边界、
中间行权重最大、误差限幅再归一化），但边界检测换成更适合红/蓝跑道 + 白线的
HSV 白线掩膜，并补上了**置信度**输出（官方没有置信度，仲裁层没法用）。

输出与 `control.lane_arbiter` 的约定：`center_x` 单位是像素（640 宽画面），
`confidence` 0~1，由仲裁层决定是否降级。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from config import settings


@dataclass
class LaneObservation:
    """单帧扫线观测量。"""
    center_x: float
    confidence: float
    left_x: Optional[int] = None
    right_x: Optional[int] = None
    valid_rows: int = 0
    total_rows: int = 0
    source: str = "scan"


def white_mask(frame_bgr: np.ndarray,
               roi_top_ratio: float = None,
               roi_bottom_margin: int = None) -> Tuple[np.ndarray, Tuple[int, int]]:
    """返回 (整幅二值掩膜, (roi_y0, roi_y1))。

    白线判据：饱和度低 + 亮度高；亮度阈值**自适应**：
        thr = max(V_MIN, 中位数 + SPLIT × (95分位 − 中位数))
    也就是在"地面/纸面底色"与"更亮的白线"之间自动切一刀。
    （旧公式 max(V_MIN, 0.75×V95) 在地板与打印白线只差 20~30 级亮度时会把整片地板判成白，
     详见 config/settings.py 的 LANE_WHITE_V_SPLIT 注释。）
    """
    top_ratio = settings.LANE_ROI_TOP_RATIO if roi_top_ratio is None else roi_top_ratio
    bottom_margin = (settings.LANE_ROI_BOTTOM_MARGIN
                     if roi_bottom_margin is None else roi_bottom_margin)
    h, w = frame_bgr.shape[:2]
    roi_y0 = max(0, int(h * top_ratio))
    roi_y1 = min(h, h - int(bottom_margin))
    if roi_y1 <= roi_y0:
        return np.zeros((h, w), dtype=np.uint8), (roi_y0, roi_y0)

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    roi_v = v[roi_y0:roi_y1, :]
    if roi_v.size:
        v_med = float(np.percentile(roi_v, 50))
        v_hi = float(np.percentile(roi_v, 95))
        v_thresh = max(settings.LANE_WHITE_V_MIN,
                       v_med + settings.LANE_WHITE_V_SPLIT * (v_hi - v_med))
    else:
        v_thresh = float(settings.LANE_WHITE_V_MIN)

    mask = ((s <= settings.LANE_WHITE_S_MAX) & (v >= v_thresh)).astype(np.uint8) * 255
    mask[:roi_y0, :] = 0
    mask[roi_y1:, :] = 0
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask, (roi_y0, roi_y1)


class LaneScanner:
    """逐行扫线器（无状态，可任意帧调用）。"""

    def __init__(self,
                 target_x: float = None,
                 img_w: int = None,
                 roi_top_ratio: float = None,
                 roi_bottom_margin: int = None,
                 row_step: int = None,
                 weight_peak: float = None,
                 max_error_px: float = None,
                 min_valid_rows: int = None):
        self.target_x = float(settings.TARGET_X if target_x is None else target_x)
        self.img_w = int(settings.IMG_W if img_w is None else img_w)
        self.roi_top_ratio = (settings.LANE_ROI_TOP_RATIO if roi_top_ratio is None else roi_top_ratio)
        self.roi_bottom_margin = (settings.LANE_ROI_BOTTOM_MARGIN
                                  if roi_bottom_margin is None else roi_bottom_margin)
        self.row_step = int(settings.LANE_ROW_STEP if row_step is None else row_step)
        self.weight_peak = float(settings.LANE_WEIGHT_PEAK if weight_peak is None else weight_peak)
        self.max_error_px = float(settings.LANE_MAX_ERROR_PX if max_error_px is None else max_error_px)
        self.min_valid_rows = int(settings.LANE_MIN_VALID_ROWS if min_valid_rows is None else min_valid_rows)

    # ------------------------------------------------------------------ 内部
    @staticmethod
    def _run_width(row: np.ndarray, idx: int) -> int:
        """idx 所在的白色连通段宽度（像素）。"""
        l = idx
        while l > 0 and row[l - 1]:
            l -= 1
        r = idx
        w = row.size
        while r < w - 1 and row[r + 1]:
            r += 1
        return r - l + 1

    @classmethod
    def _pick(cls, row: np.ndarray, candidates: np.ndarray, from_center: bool,
              max_line_w: int) -> Optional[int]:
        """从候选中挑一个"像车道线"的：**从靠近画面中心的一侧往外找**，
        取第一个宽度不超过 max_line_w 的连通段。

        为什么要限宽（2026-09-16 实车教训）：实验室地面/纸边的亮区也会被判成"白"，
        它们是大片连通区（几十~上百像素宽），而车道线只有几~十几像素宽。
        不限宽的话，扫线会锁到大片亮区上，中心值乱跳 → 车左右猛打。
        """
        order = candidates[::-1] if from_center else candidates
        for idx in order:
            if cls._run_width(row, int(idx)) <= max_line_w:
                return int(idx)
        return None

    @staticmethod
    def _row_boundaries(row: np.ndarray, center0: int,
                        margin: int = settings.LANE_EDGE_MARGIN,
                        max_line_w: int = settings.LANE_MAX_LINE_W_PX) -> Tuple[Optional[int], Optional[int]]:
        """在一行里找左右边界。

        `center0` 是期望中心；若该像素本身就是白线（车压线），先跳过它所在的连通段，
        避免把"车下的白线"当成车道边界。
        距画面边缘 `margin` 像素内的白点忽略——那里出现的通常是画面边框/远处墙体等伪影。
        超过 `max_line_w` 的白色连通段不算车道线（那是地面/纸边的亮区，见 _pick 注释）。
        """
        xs = np.flatnonzero(row)
        if xs.size == 0:
            return None, None
        xs = xs[(xs >= margin) & (xs < row.size - margin)]
        if xs.size == 0:
            return None, None

        left_limit = center0 - 1
        right_start = center0 + 1
        if row[center0]:
            l = center0
            while l > 0 and row[l - 1]:
                l -= 1
            r = center0
            w = row.size
            while r < w - 1 and row[r + 1]:
                r += 1
            left_limit = l - 1
            right_start = r + 1

        left = LaneScanner._pick(row, xs[xs <= left_limit], from_center=True, max_line_w=max_line_w)
        right = LaneScanner._pick(row, xs[xs >= right_start], from_center=True, max_line_w=max_line_w)
        return left, right

    # ------------------------------------------------------------------ 接口
    def scan(self, frame_bgr: np.ndarray) -> LaneObservation:
        """对一帧做扫线，返回观测量。

        ⚠️ 关键改动（2026-09-16 实车教训）：**只有左右两侧都找到的行才参与求中心**。
        旧写法照官方实现"缺哪侧就用 0 / w-1 兜底"，结果单侧丢线时行中点被拉飞
        （最远能偏半幅画面），而置信度还有 0.5~0.6 被当成有效 —— 车就会左右猛打、冲出赛道。
        现在：单侧行只用于统计（不参与中心计算），且会拉低置信度。
        """
        h, w = frame_bgr.shape[:2]
        mask, (roi_y0, roi_y1) = white_mask(frame_bgr, self.roi_top_ratio, self.roi_bottom_margin)
        center0 = int(min(max(self.target_x, 0), w - 1))

        rows = list(range(roi_y1 - 1, roi_y0, -self.row_step))
        if not rows:
            return LaneObservation(center_x=self.target_x, confidence=0.0, source="scan")

        span = max(1, roi_y1 - roi_y0)
        weighted_sum = 0.0
        weight_total = 0.0
        left_sum = right_sum = 0.0
        left_n = right_n = 0
        widths = []
        full_rows = partial_rows = 0

        for y in rows:
            left, right = self._row_boundaries(mask[y], center0)
            if left is None and right is None:
                continue
            if left is None or right is None:
                # 单侧缺失：只统计，不参与中心计算（避免"0 / w-1 兜底"把中点拉飞）
                partial_rows += 1
                if left is not None:
                    left_sum += left; left_n += 1
                if right is not None:
                    right_sum += right; right_n += 1
                continue

            full_rows += 1
            widths.append(right - left)
            left_sum += left
            left_n += 1
            right_sum += right
            right_n += 1

            # 纵向权重：ROI 中间行最大（"看得不远不近"）
            u = (y - roi_y0) / float(span)
            weight = 1.0 + (self.weight_peak - 1.0) * float(np.sin(np.pi * u))
            weighted_sum += ((left + right) / 2.0) * weight
            weight_total += weight

        valid_rows = full_rows + partial_rows
        total_rows = len(rows)
        # 双侧齐全的行太少 → 直接判不可信（宁可让仲裁层降级，也不要输出乱跳的中心）
        if full_rows < self.min_valid_rows or weight_total <= 0:
            return LaneObservation(center_x=self.target_x, confidence=0.0,
                                   valid_rows=valid_rows, total_rows=total_rows, source="scan")

        center_x = weighted_sum / weight_total

        # 置信度 = 双侧齐全行占比 × 宽度一致性（宽度越稳定越可信）
        coverage = full_rows / float(total_rows)
        if len(widths) >= 2:
            mean_w = float(np.mean(widths))
            cv = float(np.std(widths)) / mean_w if mean_w > 1e-6 else 1.0
            consistency = float(np.clip(1.0 - cv, 0.0, 1.0))
        else:
            consistency = 0.6
        confidence = float(np.clip(coverage * (0.5 + 0.5 * consistency), 0.0, 1.0))

        return LaneObservation(
            center_x=float(center_x),
            confidence=confidence,
            left_x=int(left_sum / left_n) if left_n else None,
            right_x=int(right_sum / right_n) if right_n else None,
            valid_rows=valid_rows,
            total_rows=total_rows,
            source="scan",
        )

    def debug_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """返回扫线用的白线掩膜（调试窗口用）。"""
        mask, _ = white_mask(frame_bgr, self.roi_top_ratio, self.roi_bottom_margin)
        return mask
