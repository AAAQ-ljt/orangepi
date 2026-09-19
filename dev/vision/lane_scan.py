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

    # 开运算去"雪点"：塑胶跑道的颗粒、纸面噪点都是 1~3px 的碎白，
    # 而白线在画面里有 8~20px 宽 —— 3x3 开运算能去掉 90% 碎斑（2026-09-19 操场实测：
    # 4780 → 460 个连通域），且不会伤到线。
    open_px = int(settings.LANE_MASK_OPEN_PX)
    if open_px >= 3:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px)))

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


def _row_runs(mask: np.ndarray, y: int, margin: int, max_line_w: int) -> list:
    """第 y 行里"像线"的候选段：返回 [(段中心x, 段宽), ...]。

    比"取内边缘"更适合拟合：拟合要的是线的位置（中心），配对求车道中心时两条线的
    中心之差才是车道宽度。
    """
    out = []
    for a, b in _runs(mask[y], margin):
        width = b - a + 1
        if width <= max_line_w:
            out.append(((a + b) * 0.5, width))
    return out


def fit_line(points: list) -> tuple:
    """最小二乘拟合 x = a*y + b；返回 (a, b)（点少于 2 个返回 None）。"""
    if len(points) < 2:
        return None
    ys = np.asarray([p[0] for p in points], dtype=np.float64)
    xs = np.asarray([p[1] for p in points], dtype=np.float64)
    if float(np.ptp(ys)) < 1e-6:
        return None
    a, b = np.polyfit(ys, xs, 1)
    return float(a), float(b)


def fit_line_ransac(points: list, tol_px: float = None, iters: int = None,
                    slope_sign: int = 0) -> tuple:
    """RANSAC 直线拟合：返回 (a, b, inliers, points_used)。

    为什么用 RANSAC 而不是"逐行贪心跟踪"（2026-09-19 操场重写）：
    跑道上有**多条**白线（相邻车道），颗粒/断线又会打断线；贪心跟踪一旦跟丢，
    兜底逻辑会挑"最长的那条"——往往是隔壁车道的线，于是中心乱跳（实测在 183~460 之间跳）。
    直线拟合则一次看全局：**车道线在画面里是直线**，用一致性把这几十行点串成一条，
    再用"斜率符号 + 收敛性 + 宽度范围"三条几何先验把隔壁车道和噪声排除。
    """
    tol = float(settings.LANE_FIT_TOL_PX if tol_px is None else tol_px)
    n_iter = int(settings.LANE_FIT_ITERS if iters is None else iters)
    pts = [(float(y), float(x)) for y, x in points]
    if len(pts) < 2:
        return None
    best = None
    rng = np.random.default_rng(0)                     # 固定种子：同一帧结果可复现
    for _ in range(n_iter):
        i, j = rng.choice(len(pts), size=2, replace=False)
        (y1, x1), (y2, x2) = pts[i], pts[j]
        if abs(y1 - y2) < settings.LANE_FIT_MIN_SPAN_PX:
            continue
        a = (x2 - x1) / (y2 - y1)
        b = x1 - a * y1
        if slope_sign and (a > 0) != (slope_sign > 0):
            continue
        inl = [(y, x) for y, x in pts if abs(a * y + b - x) <= tol]
        if best is None or len(inl) > len(best[2]):
            best = (a, b, inl)
    if best is None:
        return None
    a, b, inl = best
    refit = fit_line(inl)                              # 用内点再最小二乘精修一次
    if refit is not None:
        a, b = refit
        inl = [(y, x) for y, x in pts if abs(a * y + b - x) <= tol]
    return a, b, inl


def fit_lane_side(mask: np.ndarray, y_top: int, y_bottom: int, side: str,
                  margin: int = None, max_line_w: int = None) -> dict:
    """拟合一侧车道线，返回 {'a','b','inliers','span','x_top','x_bottom','n'}。

    锚定：左候选取画面中心左侧、右候选取右侧（车在自己的车道里，两侧最近的那条就是我们的线）；
    斜率符号：左线在画面里"越往上越靠中间"→ dx/dy < 0，右线 > 0，用这个把隔壁车道剔除。
    """
    margin = settings.LANE_EDGE_MARGIN if margin is None else margin
    max_line_w = settings.LANE_MAX_LINE_W_PX if max_line_w is None else max_line_w
    center = mask.shape[1] * 0.5
    pts = []
    for y in range(y_top, y_bottom, settings.LANE_FIT_ROW_STEP):
        for x, _w in _row_runs(mask, y, margin, max_line_w):
            if side == "left" and x < center:
                pts.append((y, x))
            elif side == "right" and x > center:
                pts.append((y, x))
    fit = fit_line_ransac(pts, slope_sign=(-1 if side == "left" else +1))
    if fit is None:
        return {"a": None, "b": None, "inliers": [], "span": 0, "x_top": None,
                "x_bottom": None, "n": len(pts)}
    a, b, inl = fit
    ys = [y for y, _x in inl]
    return {
        "a": a, "b": b, "inliers": inl, "span": (max(ys) - min(ys)) if ys else 0,
        "x_top": a * y_top + b, "x_bottom": a * y_bottom + b, "n": len(pts),
    }


def edge_image(frame_bgr: np.ndarray, y_top: int, y_bottom: int):
    """ROI 灰度 → 高斯模糊 → Canny 边缘（阈值自适应）。返回 (edges, (low, high))。

    移植自参考实现（oldCode/src/vision/vision.cpp `picture()`）：
    它不看颜色/亮度阈值，而是看**灰度突变**——白线两侧都是强边缘，
    所以反光、暖光、地面颜色都不影响它。自适应阈值靠"边缘像素总数"回调
    （太多说明噪声多→提高阈值；太少说明对比不足→降低阈值）。
    """
    low = float(settings.LANE_CANNY_LOW)
    high = float(settings.LANE_CANNY_HIGH)
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    roi = gray[y_top:y_bottom, :]
    blur = cv2.GaussianBlur(roi, (5, 5), 0.5)
    edges = cv2.Canny(blur, int(low), int(high), 3)
    if settings.LANE_CANNY_ADAPT:
        n = int(np.count_nonzero(edges))
        if n > settings.LANE_CANNY_TARGET_HI:
            low, high = low + 2, high + 4
        elif n < settings.LANE_CANNY_TARGET_LO:
            low, high = max(10.0, low - 2), max(20.0, high - 4)
    return edges, (low, high)


def hough_lane_segments(edges: np.ndarray, y_top: int) -> list:
    """HoughLinesP → 斜率和截距 (a, b, length, y_lo, y_hi)，a 为 dx/dy（x = a*y + b）。

    只保留**斜率落在物理范围内**的线段：车道线在画面里必然是斜的（既不是水平也不是垂直），
    这一条就能把地面纹理、颗粒、反光边缘全部滤掉（参考实现同款先验）。
    """
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0 * 3,
                            threshold=int(settings.LANE_HOUGH_THRESH),
                            minLineLength=int(settings.LANE_HOUGH_MIN_LEN),
                            maxLineGap=int(settings.LANE_HOUGH_MAX_GAP))
    out = []
    if lines is None:
        return out
    lines = np.asarray(lines).reshape(-1, 4)      # 兼容 (N,1,4) 与 (N,4) 两种返回形状
    for x1, y1, x2, y2 in lines:
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


def lane_side_from_segments(segments: list, side: str) -> dict:
    """按斜率符号分组，取该侧"支持最充分"的一条线。

    参考实现取 |斜率| 最大者；这里取**最长线段**为基准，再用同组中位数校核
    （单条最长线段可能是噪声，但"最长 + 同组一致"就很少错）。
    """
    grp = [seg for seg in segments if (seg[0] < 0) == (side == "left")]
    if not grp:
        return {"a": None, "b": None, "len_sum": 0.0, "n": 0}
    grp.sort(key=lambda t: -t[2])
    a0, b0, _l, _y0, _y1 = grp[0]
    a_med = float(np.median([seg[0] for seg in grp]))
    b_med = float(np.median([seg[1] for seg in grp]))
    # 中位数与最长线段差得太多 → 这一侧不可信
    if abs(a0 - a_med) > settings.LANE_FIT_TOL_PX * 0.01 + 0.5 * abs(a_med):
        return {"a": None, "b": None, "len_sum": 0.0, "n": len(grp)}
    return {"a": a_med, "b": b_med, "len_sum": float(sum(seg[2] for seg in grp)), "n": len(grp)}


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

        # ---- 边缘 + Hough 直线（移植参考实现；见 edge_image/hough_lane_segments 注释）----
        edges, _thr = edge_image(frame_bgr, roi_y0, roi_y1)
        segments = hough_lane_segments(edges, roi_y0)
        left = lane_side_from_segments(segments, "left")
        right = lane_side_from_segments(segments, "right")

        rows = list(range(roi_y1 - 1, roi_y0, -self.row_step))
        total_rows = max(1, len(rows))
        def _reject():
            return LaneObservation(center_x=self.target_x, confidence=0.0,
                                   valid_rows=int(min(left["n"], right["n"])),
                                   total_rows=total_rows, source="scan")

        if (left["a"] is None or right["a"] is None
                or left["len_sum"] < settings.LANE_MIN_SEG_LEN_SUM
                or right["len_sum"] < settings.LANE_MIN_SEG_LEN_SUM):
            return _reject()

        # 取**前瞻带**（ROI 上半部分）上逐行的车道中心再平均 —— 参考实现就是这么做的
        # （它对一排 y 求平均，而不是只看一行）：既平滑，又保留"看远处"的前瞻性。
        span = max(1, roi_y1 - roi_y0)
        y_a = roi_y0
        y_b = roi_y0 + int(span * settings.LANE_LOOKAHEAD_RATIO)
        centers = []
        for y in range(y_a, max(y_a + 1, y_b), max(1, settings.LANE_FIT_ROW_STEP)):
            centers.append(((left["a"] * y + left["b"]) + (right["a"] * y + right["b"])) * 0.5)
        y_look = int((y_a + y_b) * 0.5)
        x_l = left["a"] * y_look + left["b"]
        x_r = right["a"] * y_look + right["b"]
        pair_w = x_r - x_l
        if not (settings.LANE_PAIR_W_MIN_PX <= pair_w <= settings.LANE_PAIR_W_MAX_PX):
            return _reject()

        # 透视收敛校验：越往上（远处）两线应当越靠拢；不收敛说明配错车道了
        w_top = (right["a"] * roi_y0 + right["b"]) - (left["a"] * roi_y0 + left["b"])
        if w_top >= pair_w * settings.LANE_CONVERGE_MAX_RATIO:
            return _reject()

        center_x = float(np.mean(centers)) if centers else (x_l + x_r) * 0.5
        cover = min(left["len_sum"], right["len_sum"]) / float(2 * max(1.0, y_b - y_a))
        converge = float(np.clip(1.0 - w_top / max(1.0, pair_w), 0.0, 1.0))
        confidence = float(np.clip(cover * (0.45 + 0.35 * converge + 0.20 * min(1.0, min(left["n"], right["n"]) / 6.0)),
                                   0.0, 1.0))

        return LaneObservation(
            center_x=float(center_x),
            confidence=confidence,
            left_x=int(x_l),
            right_x=int(x_r),
            valid_rows=int(min(left["n"], right["n"])),
            total_rows=total_rows,
            source="scan",
        )

    def debug_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        """返回扫线用的白线掩膜（调试窗口用）。"""
        mask, _ = white_mask(frame_bgr, self.roi_top_ratio, self.roi_bottom_margin)
        return mask
