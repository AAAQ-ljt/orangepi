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


def line_like(bw: int, bh: int, area: int,
              min_len: int = None, min_elong: float = None) -> bool:
    """这个连通域"像不像一条车道线"（纯函数，便于单测）。

    线 = **细长**：长度（bbox 长边）远大于等效厚度（面积 / 长度）。
    反光斑 = **块状**：椭圆/团块的长宽比接近 1，细长比只有 1~3。

    2026-09-16 教训：这里原先写的是"厚度 ≤ 18px"的绝对阈值 —— 打印跑道上是能压掉反光，
    但白线在**前视浅角度**下（画面边缘、近处）投影出来的厚度本来就超过 18px，
    于是真线被一起滤掉 → 置信度从 0.6 掉到 0.1、车"看不到车道"完全不动。
    绝对厚度不可靠，**细长比（尺度无关）**才可靠。
    """
    length = max(bw, bh)
    if length < (settings.LANE_LINE_MIN_LEN_PX if min_len is None else min_len):
        return False
    thickness = area / float(max(1, length))
    if thickness > settings.LANE_LINE_MAX_THICK_PX:
        return False        # 次级保险：整片亮区（不是线）靠它挡住
    elong = length / max(1e-6, thickness)
    return elong >= (settings.LANE_LINE_MIN_ELONG if min_elong is None else min_elong)


def shape_filter(mask: np.ndarray) -> np.ndarray:
    """只保留细长（像线）的连通域，去掉块状亮区（地面反光/纸边）。"""
    n, labels, stats, _cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return mask
    keep = np.zeros_like(mask)
    for i in range(1, n):
        _x, _y, bw, bh, area = stats[i]
        if line_like(int(bw), int(bh), int(area)):
            keep[labels == i] = 255
    return keep


def white_mask(frame_bgr: np.ndarray,
               roi_top_ratio: float = None,
               roi_bottom_margin: int = None,
               apply_shape_filter: bool = True) -> Tuple[np.ndarray, Tuple[int, int]]:
    """返回 (整幅二值掩膜, (roi_y0, roi_y1))。

    白线判据：饱和度低 + 亮度高；亮度阈值**自适应**：
        thr = max(V_MIN, 中位数 + SPLIT × (95分位 − 中位数)）
    也就是在"地面/纸面底色"与"更亮的白线"之间自动切一刀。
    （旧公式 max(V_MIN, 0.75×V95) 在地板与打印白线只差 20~30 级亮度时会把整片地板判成白，
     详见 config/settings.py 的 LANE_WHITE_V_SPLIT 注释。）

    `apply_shape_filter=False` 时只做颜色判据（诊断工具要看"过滤前长什么样"）。
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

    if apply_shape_filter:
        mask = shape_filter(mask)
    return mask, (roi_y0, roi_y1)


def _runs(row: np.ndarray, margin: int) -> list:
    """把一行里的白色像素切成连续段，返回 [(起, 止), ...]（去掉贴边伪影）。"""
    xs = np.flatnonzero(row)
    if xs.size == 0:
        return []
    xs = xs[(xs >= margin) & (xs < row.size - margin)]
    if xs.size == 0:
        return []
    runs = []
    start = prev = int(xs[0])
    for x in xs[1:]:
        x = int(x)
        if x != prev + 1:
            runs.append((start, prev))
            start = x
        prev = x
    runs.append((start, prev))
    return runs


def _row_candidates(mask: np.ndarray, y: int, side: str, margin: int,
                    max_line_w: int, center: int) -> list:
    """第 y 行里"像这一侧车道线"的候选 [(内边缘x, 段宽), ...]。"""
    cands = []
    for a, b in _runs(mask[y], margin):
        width = b - a + 1
        if width > max_line_w:
            continue
        inner = b if side == "left" else a          # 内边缘 = 朝画面中心的那一端
        if side == "left" and inner >= center:
            continue
        if side == "right" and inner <= center:
            continue
        cands.append((inner, width))
    return cands


def _trace_from(rows_cands: list, seed_idx: int, seed: tuple,
                max_jump: int, max_miss: int, width_tol: float) -> list:
    """从一个种子出发，向上、向下各走一遍，返回 [(y, 内边缘x, 段宽), ...]（自上而下）。"""
    def walk(step: int):
        out = []
        cur_x, widths, misses = seed[0], [seed[1]], 0
        i = seed_idx + step
        while 0 <= i < len(rows_cands):
            y, cands = rows_cands[i]
            near = [(x, wd) for (x, wd) in cands if abs(x - cur_x) <= max_jump]
            med = float(np.median(widths)) if widths else None
            pick = None
            if near:
                cand = min(near, key=lambda c: abs(c[0] - cur_x))
                if med is None or abs(cand[1] - med) <= width_tol * med:
                    pick = cand
            if pick is None:
                misses += 1
                if misses > max_miss:
                    break
                i += step
                continue
            cur_x, width = pick
            widths.append(width)
            misses = 0
            out.append((y, cur_x, width))
            i += step
        return out

    up = walk(-1)
    down = walk(+1)
    path = list(reversed(up)) + [(rows_cands[seed_idx][0], seed[0], seed[1])] + down
    return path


def trace_boundary(mask: np.ndarray, y_bottom: int, y_top: int, side: str,
                   margin: int = None, max_line_w: int = None,
                   max_jump: int = None, max_miss: int = None,
                   width_tol: float = None,
                   min_len: int = None, seed_step: int = None) -> list:
    """**逐行连续跟踪**一侧车道边界，返回 [(y, 内边缘x, 段宽), ...]（自上而下）。

    为什么不能"每行独立地从画面中心往外找第一个白点"（2026-09-16 实车画面实测）：
    这条打印跑道上白线在赛道**最外侧**（贴着画面边缘），而塑料膜褶皱/反光在画面**中间**——
    独立逐行找的结果是"每一行都锁在中间那团反光上"，中心乱跳。连续跟踪天然区分两者：
    **线是连续、平滑、宽度稳定的**；反光一团一团、宽度忽大忽小、相邻行之间对不上。

    跟踪规则（每一步都要满足）：
    - 内边缘相对上一行位移 ≤ `max_jump`（透视变化是渐变的）；
    - 段宽与已跟踪宽度中位数之差 ≤ `width_tol × 中位数`（反光宽度不稳定）；
    - 允许连续 `max_miss` 行缺失（被光斑/遮挡打断），超过即判跟丢。

    **种子策略（2026-09-19 实车复盘新增）**：先从最下面一行找种子往上跟（正常俯视角度下
    近处的线最清楚）；如果那条路径太短（< `min_len`），再**遍历所有可能的种子行、取最长路径**。
    之所以必须这样：本车下摄角度偏「平」时，画面**最下方只有地面和反光**，两条白线反而在
    **画面中部的远场**才对得上——固定从底部起跟就会"一片空白起步"、什么都跟不到。
    """
    margin = settings.LANE_EDGE_MARGIN if margin is None else margin
    max_line_w = settings.LANE_MAX_LINE_W_PX if max_line_w is None else max_line_w
    max_jump = settings.LANE_TRACK_MAX_JUMP_PX if max_jump is None else max_jump
    max_miss = settings.LANE_TRACK_MAX_MISS if max_miss is None else max_miss
    width_tol = settings.LANE_TRACK_WIDTH_TOL if width_tol is None else width_tol
    min_len = settings.LANE_TRACK_MIN_LEN if min_len is None else min_len
    seed_step = settings.LANE_TRACK_SEED_STEP if seed_step is None else seed_step
    center = mask.shape[1] // 2

    rows_cands = [(y, _row_candidates(mask, y, side, margin, max_line_w, center))
                  for y in range(y_bottom - 1, y_top - 1, -1)]
    rows_cands.reverse()                                   # y 递增，便于两个方向行走

    def seed_index_for(y: int) -> int:
        return max(0, y_bottom - 1 - y)                    # 注意：上面 reverse 过，行号直接映射

    seeds = [(i, cands) for i, (y, cands) in enumerate(rows_cands) if cands]
    if not seeds:
        return []

    # ① 先试"从最下面往上"：正常角度下这就是对的，也最省时间
    first = seeds[0]
    best = _trace_from(rows_cands, first[0], first[1][-1], max_jump, max_miss, width_tol)
    if len(best) >= min_len:
        return best

    # ② 太短 → 遍历（稀疏取样）所有种子，取最长的那条
    for i, cands in seeds[::max(1, seed_step)]:
        for seed in cands:
            path = _trace_from(rows_cands, i, seed, max_jump, max_miss, width_tol)
            if len(path) > len(best):
                best = path
    return best


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

        rows = list(range(roi_y1 - 1, roi_y0, -self.row_step))
        if not rows:
            return LaneObservation(center_x=self.target_x, confidence=0.0, source="scan")

        # 连续跟踪左右边界（不是"每行独立找"——那会锁到画面中间的地面反光上）
        left_map = {y: x for y, x, _ in trace_boundary(mask, roi_y1, roi_y0, "left")}
        right_map = {y: x for y, x, _ in trace_boundary(mask, roi_y1, roi_y0, "right")}

        span = max(1, roi_y1 - roi_y0)
        weighted_sum = 0.0
        weight_total = 0.0
        left_sum = right_sum = 0.0
        left_n = right_n = 0
        widths = []
        full_rows = partial_rows = 0

        for y in rows:
            left = left_map.get(y)
            right = right_map.get(y)
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

            pair_w = right - left
            if not (settings.LANE_PAIR_W_MIN_PX <= pair_w <= settings.LANE_PAIR_W_MAX_PX):
                # 宽度不像"一条 1.22m 的车道" → 这对左右边界不是同一行的两条车道线
                partial_rows += 1
                left_sum += left; left_n += 1
                right_sum += right; right_n += 1
                continue
            full_rows += 1
            widths.append(pair_w)
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
