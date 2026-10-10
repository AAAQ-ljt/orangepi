"""元素颜色检测通道（官方 2026 配方移植，2026-10-09 T4）。

全部为纯函数、无硬件/模型依赖，本地合成图即可单测。四个检测器：

1. `detect_zebra` —— 斑马线：灰度 > ZEBRA_THRESH 二值化 → 连通域面积 > ZEBRA_MIN_AREA
   计数 ≥ ZEBRA_MIN_COUNT（官方 14 号 demo `detectZebra`：车道线只有 1~2 条细边，天然过滤）。
2. `detect_traffic_light` —— 红绿灯：HSV 红/绿双区间面积互比（红 H0-10∪170-180 / 绿 H35-85、
   各自 ≥ TRAFFIC_MIN_AREA、**红>绿 判红 / 绿>红 判绿**，防串色）；可选「横向分区校验」
   （三灯珠左 1/3 红、右 1/3 绿，与方案文档 v1.1 一致）。返回 "red"/"green"/None；
   帧级去抖（连续 N 帧）在调用方（FSM 已有 TRAFFIC_LIGHT_HYSTERESIS_FRAMES 口径）。
3. `detect_parking_yellow` —— 停车区黄色胶布面积 ≥ PARK_YELLOW_MIN_AREA
   （官方 demo `detectYellowZone`；入格细节走几何，本函数只负责"发现停车区"）。
4. `choose_gap` —— 锥桶穿越间隙决策树（穿越火线 demo.cpp:928-1019 移植）：
   输入蓝/红锥桶候选（ConeBox 列表），输出可穿越间隙的中心 x/行 y/宽度；
   · 前景带过滤：只保留 `bottom_y ≥ 最近锥 − CONE_FOREGROUND_MAX_DELTA` 的锥（滤远处杂锥）；
   · 行分组：按 bottom_y 均值距离 ≤ CONE_ROW_TOLERANCE 归组，取最近一组 ≥2 锥；
   · 间隙：相邻锥对宽 ≥ CONE_MIN_GAP_WIDTH，且间隙内无红色阻挡
     （红锥 bottom 落在 `[row_y − RED_ABOVE, row_y + RED_BELOW]` 即视为阻挡）；
   · 选间隙：优先「跨越画面中心」，其次离中心近，再其次更宽（demo `choose_cross_row_pair`）。
5. `classify_ab` —— A/B 标志牌：凸包面积比（convex_hull 面积 / 轮廓面积），< 0.88 判 A
   （有三角缺口的字母），≥ 0.88 判 B（近凸）（2025 newCode doc:2163-2178 配方）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from config import settings
from vision.cone_detect import ConeBox


# ------------------------------------------------------------------ 斑马线
def detect_zebra(frame_bgr: np.ndarray, roi=None, threshold: int = None,
                 min_area: int = None, min_count: int = None,
                 use_shape_gate: bool = None) -> bool:
    """灰度二值化 + 连通域计数（官方 demo detectZebra 配方）。

    2026-10-09 离线实测结论（写入执行单/操场检查单）：
    · 官方数字（阈值 180/面积 200/计数 4）在我们画面上对**远距斑马线**偏严
      （我们的图集里 (ROI0.10, area60, cnt3) 才到 100%）；
    · 但纯"连通域计数"在操场图上有误检风险（白线/反光也被数成条纹，
      离线 logs_pull 误检 58~95%——注意该集无斑马线真值，只是警示）；
    · 因此**默认保持官方纯计数**，形状闸门做成可选开关（见 settings.ZEBRA_USE_SHAPE_GATE）：
      闸门① 条纹须"宽≥1.2×高"（滤细长车道线），闸门② 顶边落在中位数 ±40px 内。
      现场用叠加图（lane_probe 式）标定"该用哪挡"，不要离线拍脑袋。
    """
    h, w = frame_bgr.shape[:2]
    roi = roi or settings.ZEBRA_ROI
    threshold = settings.ZEBRA_THRESH if threshold is None else threshold
    min_area = settings.ZEBRA_MIN_AREA if min_area is None else min_area
    min_count = settings.ZEBRA_MIN_COUNT if min_count is None else min_count
    use_shape_gate = settings.ZEBRA_USE_SHAPE_GATE if use_shape_gate is None else use_shape_gate
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return False
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, int(threshold), 255, cv2.THRESH_BINARY)
    num, _labels, stats, _centroids = cv2.connectedComponentsWithStats(binary, 8)
    boxes = []
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        bw = stats[i, cv2.CC_STAT_WIDTH]
        bh = stats[i, cv2.CC_STAT_HEIGHT]
        if area <= int(min_area):
            continue
        if use_shape_gate and bw < 1.2 * bh:        # 闸门①：条纹是扁矩形（滤细长车道线）
            continue
        boxes.append((stats[i, cv2.CC_STAT_TOP], bw, area))
    if len(boxes) < int(min_count):
        return False
    if use_shape_gate:
        # 闸门②：条纹顶边构成同排（透视下远条纹会略高，允许 ±40px 抖动）
        tops = sorted(b[0] for b in boxes)
        med = float(np.median(tops))
        return sum(1 for t in tops if abs(t - med) <= 40) >= int(min_count)
    return True


# ------------------------------------------------------------------ 红绿灯
def _mask_area(hsv: np.ndarray, low, high) -> Tuple[int, int, int]:
    """返回 (掩膜面积, 左 1/3 面积, 右 1/3 面积)。掩膜与 hsv 同尺寸。"""
    m = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    n = int(m.sum() // 255)
    w = m.shape[1]
    n_l = int(m[:, : w // 3].sum() // 255)
    n_r = int(m[:, 2 * w // 3:].sum() // 255)
    return n, n_l, n_r


def detect_traffic_light(frame_bgr: np.ndarray, roi=None, use_zone: bool = None) -> Optional[str]:
    """HSV 红绿互比判灯态（官方 demo detectTrafficLight + 方案文档分区校验）。

    - 主判据：red ≥ MIN 且 red > green → "red"；green ≥ MIN 且 green > red → "green"；
    - 分区校验（各半区计数）：红区在画面左 1/3、绿区在右 1/3（横向三灯珠）——启用时
      对应的灯色在本侧分区要有面积，否则判 unknown（None）。
    """
    h, w = frame_bgr.shape[:2]
    roi = roi or settings.TRAFFIC_ROI
    use_zone = settings.TRAFFIC_USE_ZONE if use_zone is None else use_zone
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return None
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    r1, r1_l, r1_r = _mask_area(hsv, settings.TRAFFIC_RED_LOW1, settings.TRAFFIC_RED_HIGH1)
    r2, r2_l, r2_r = _mask_area(hsv, settings.TRAFFIC_RED_LOW2, settings.TRAFFIC_RED_HIGH2)
    g, g_l, g_r = _mask_area(hsv, settings.TRAFFIC_GREEN_LOW, settings.TRAFFIC_GREEN_HIGH)
    red = r1 + r2
    green = g
    min_area = settings.TRAFFIC_MIN_AREA
    if red >= min_area and red > green:
        if not use_zone or (r1_l + r2_l) > 0:
            return "red"
        return None
    if green >= min_area and green > red:
        if not use_zone or g_r > 0:
            return "green"
        return None
    return None


# ------------------------------------------------------------------ 停车黄区
def detect_parking_yellow(frame_bgr: np.ndarray, roi=None,
                          hsv_low=None, hsv_high=None, min_area: int = None) -> bool:
    """停车区黄色胶布面积判据（官方 demo detectYellowZone）。"""
    h, w = frame_bgr.shape[:2]
    roi = roi or settings.PARK_ZONE_ROI
    min_area = settings.PARK_YELLOW_MIN_AREA if min_area is None else min_area
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return False
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    low = np.array(hsv_low or settings.PARK_YELLOW_LOW, np.uint8)
    high = np.array(hsv_high or settings.PARK_YELLOW_HIGH, np.uint8)
    m = cv2.inRange(hsv, low, high)
    return int(m.sum() // 255) >= int(min_area)


# ------------------------------------------------------------------ 锥桶间隙决策树
@dataclass
class Gap:
    """一个可穿越间隙（摄像头坐标）。"""
    left_x: float
    right_x: float
    row_y: float
    center_x: float
    width: float
    spans_center: bool


def _group_rows(cones: Sequence[ConeBox], tolerance: int) -> List[List[ConeBox]]:
    """按 bottom_y 与组均值距离 ≤ tolerance 归组；组间按均值 y 从近到远排序。"""
    rows: List[List[ConeBox]] = []
    for c in cones:
        best_i, best_d = -1, 999999
        for i, row in enumerate(rows):
            mean_y = sum(x.bottom_y for x in row) / len(row)
            d = abs(c.bottom_y - mean_y)
            if d <= tolerance and d < best_d:
                best_d, best_i = d, i
        if best_i < 0:
            rows.append([c])
        else:
            rows[best_i].append(c)
            rows[best_i].sort(key=lambda x: x.center_x)
    rows.sort(key=lambda r: -sum(x.bottom_y for x in r) / len(r))
    return rows


def _red_blocks(gap_left: float, gap_right: float, row_y: float,
                red_cones: Sequence[ConeBox], above: int, below: int) -> bool:
    """间隙内是否有红色锥桶阻挡（bottom_y 落在 [row_y−above, row_y+below]）。"""
    for r in red_cones:
        if gap_left < r.center_x < gap_right and -above <= (r.bottom_y - row_y) <= below:
            return True
    return False


def choose_gap(blue_cones: Sequence[ConeBox], red_cones: Sequence[ConeBox],
               img_center: float,
               foreground_delta: int = None, row_tolerance: int = None,
               min_gap_width: int = None, red_above: int = None,
               red_below: int = None) -> Optional[Gap]:
    """穿越火线间隙决策树（demo.cpp:928-1019 + choose_cross_row_pair）。

    返回选中的间隙；无安全间隙返回 None（调用方走"搜索/回中"降级档）。
    输入 ConeBox 均为原图坐标（含 ROI 偏移），红色/蓝色分类由调用方保证。
    """
    foreground_delta = settings.CONE_FOREGROUND_MAX_DELTA if foreground_delta is None else foreground_delta
    row_tolerance = settings.CONE_ROW_TOLERANCE if row_tolerance is None else row_tolerance
    min_gap_width = settings.CONE_MIN_GAP_WIDTH if min_gap_width is None else min_gap_width
    red_above = settings.CONE_RED_BLOCK_ABOVE if red_above is None else red_above
    red_below = settings.CONE_RED_BLOCK_BELOW if red_below is None else red_below

    if not blue_cones:
        return None
    nearest_y = max(c.bottom_y for c in blue_cones)
    foreground = [c for c in blue_cones if c.bottom_y >= nearest_y - foreground_delta]
    if len(foreground) < 2:
        return None

    # 行分组 → 取最近一组 ≥2 锥；没有则退回"跨中心相邻对"（choose_cross_row_pair）
    rows = _group_rows(foreground, row_tolerance)
    pair: Optional[Tuple[ConeBox, ConeBox]] = None
    for row in rows:
        if len(row) >= 2:
            pair = (row[0], row[1])
            break
    if pair is None:
        candidates = []
        sorted_c = sorted(foreground, key=lambda c: c.center_x)
        for i in range(len(sorted_c) - 1):
            a, b = sorted_c[i], sorted_c[i + 1]
            width = b.center_x - a.center_x
            if width < min_gap_width:
                continue
            cx = (a.center_x + b.center_x) / 2
            spans = a.center_x <= img_center <= b.center_x
            candidates.append((a, b, cx, width, spans))
        if not candidates:
            return None
        # 优先级：跨越中心 > 距中心近 > 更宽
        candidates.sort(key=lambda t: (not t[4], abs(t[2] - img_center), -t[3]))
        pair = (candidates[0][0], candidates[0][1])

    left, right = pair
    if right.center_x - left.center_x < min_gap_width:
        return None
    row_y = (left.bottom_y + right.bottom_y) / 2
    if _red_blocks(left.center_x, right.center_x, row_y, red_cones,
                   red_above, red_below):
        return None
    return Gap(left_x=left.center_x, right_x=right.center_x, row_y=row_y,
               center_x=(left.center_x + right.center_x) / 2,
               width=right.center_x - left.center_x,
               spans_center=left.center_x <= img_center <= right.center_x)


# ------------------------------------------------------------------ A/B 标志牌（凸包面积比）
def _normalize_contour(contour) -> np.ndarray:
    return np.asarray(contour, dtype=np.float32).reshape(-1, 1, 2)


def classify_ab(contour: np.ndarray, hull_ratio_thresh: float = None) -> Optional[str]:
    """凸包面积比判 A/B（2025 newCode doc:2163-2178）。

    ratio = contourArea(approx) / convexHullArea；< 0.88 → "A"（有三角缺口），
    ≥ 0.88 → "B"（近凸）。面积过小（< 250）或退化轮廓返回 None。
    """
    thresh = settings.AB_HULL_RATIO if hull_ratio_thresh is None else hull_ratio_thresh
    c = _normalize_contour(contour)
    area = float(cv2.contourArea(c))
    if area < settings.AB_MIN_AREA:
        return None
    hull = cv2.convexHull(c)
    hull_area = float(cv2.contourArea(hull))
    if hull_area <= 0:
        return None
    ratio = area / hull_area
    return "A" if ratio < thresh else "B"


def blue_board_contour(frame_bgr: np.ndarray, roi=None,
                       hsv_low=None, hsv_high=None) -> Optional[np.ndarray]:
    """最大蓝色连通域轮廓（停车区蓝板），供 classify_ab / 接地点几何使用。"""
    from vision.cone_detect import blue_mask
    mask = blue_mask(frame_bgr, roi, hsv_low, hsv_high)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)