"""参考实现版车道线检测（纯视觉，无硬件依赖）—— 严格照 oldCode / newnewCode 移植。

出处（每一处都能对回原文）
--------------------------
oldCode/src/vision/vision.cpp::picture()   检测链路（本模块主体）
    ROI → 灰度 → GaussianBlur(5,5,σ=0.5) → Canny(60,140，按边缘像素数自适应 ±2/±4)
    → HoughLinesP(rho=1, theta=0.05rad, 阈值 50, 最短 30, 最大间隙 5)
    → 每条线段 k = dy/dx，**只保留 0.25 ≤ |k| ≤ 2**（滤地面纹理/颗粒/反光）
    → 按 k 的符号分左右，每侧取 |k| 最大的一条当本侧车道线
    → 在若干行上求左右交点、取中点平均 → center_x
oldCode/src/vision/vision.cpp::getROI()    ROI = (h/20)*10 ~ (h/20)*17，即画面高度的 0.50~0.85
oldCode/src/control/control.cpp            控制（在 control/lane_control.py）
newnewCode/src/lane_follower.py            单侧丢线时的兜底：缺的一侧取固定边界（左 0 / 右 宽-1）
newnewCode/src/config.py                   目标点用**比例**表示；配对宽度闸门

关于"单侧丢线"的一条重要更正（2026-09-21 复核原码发现）
------------------------------------------------------
旧版 `scripts/lane_ref_test.py` 的注释与《循迹脚本交接.md》都写着
"参考实现单侧不给中心"——**这是错的**。原码 `picture()` 里对丢线的一侧
直接代入画面边界（`if (flagl) l = 0;` / `if (flagr) r = frame.cols;`），
读数照样给中心；newnewCode 同款（默认 0 / 宽-1）。
我们上一版把它改成"单侧就不给中心"，等于**主动扔掉了参考实现里唯一能让车
在丢一侧线时继续往前走的机制**，这也是实跑时"一点动就只剩单侧 → 进不了 track
→ 只能探路/停车"的直接原因。

本模块的做法：**在参考实现的形状上，把"猜边界"换成一个物理量——车道半宽**
（由两侧都在时的实测值给出，见 `WidthPrior`），因为"车在自己的车道里"这条
先验永远成立，而"缺的那条线正好在画面边缘"只是巧合。半宽未知时退回
参考实现的边界假设，但把 `quality` 压到 `--min-quality` 之下，让控制层按
"维持/停车"处理（先保误检率，再谈连续性）。

与参考实现的偏离（全部在 `doc/循迹脚本交接.md` §4.4 有登记）
------------------------------------------------------------
1. ROI 默认取 `settings`（= `config/site.yaml` 的现场标定值），不是写死的 0.50~0.85；
2. 目标点用比例 + 起步前自动标定（参考写死 226/320，那是它自己的安装）；
3. 选线加**位置锚定**（左线必须在画面中心左侧），否则会把隔壁车道/横向纹理当选；
4. 单侧兜底用**宽度先验**而不是画面边界（见上）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from config import settings

# ---------------------------------------------------------------- 参考实现常量
CANNY_LOW0, CANNY_HIGH0 = 60, 140            # oldCode/src/config/config.cpp: MIN_YU/MAX_YU
CANNY_EDGE_LO, CANNY_EDGE_HI = 2000, 2500    # 边缘像素数目标区间（自适应）
CANNY_LOW_MIN, CANNY_LOW_MAX = 20.0, 160.0   # 自适应钳位（不钳位会跑飞：见单测）
CANNY_HIGH_MIN, CANNY_HIGH_MAX = 40.0, 320.0
# HoughLinesP 的角分辨率：参考实现字面写的是 0.05 弧度（≈2.87°），**我们沿用 3°（≈0.0524）**。
# 2026-09-21 合成图复验：同一张"车道线被打断 + 横纹干扰"的图，0.05 会让**整条左线消失**
# （角分辨率细 5% 就少累到票），3° 正常。真实赛道的白线只会更碎，所以取粗的那一档。
HOUGH_THETA_RAD = math.pi / 180.0 * 3.0
HOUGH_THRESH, HOUGH_MIN_LEN, HOUGH_MAX_GAP = 50, 30, 5
SLOPE_ABS_MIN, SLOPE_ABS_MAX = 0.25, 2.0     # |dy/dx| 物理范围

# 置信度（quality）档位：数值本身经 CSV 落盘，可直接用实测数据复核
Q_BOTH = 1.0              # 两侧都成对
Q_SINGLE_MEASURED = 0.55  # 单侧 + 半宽来自实测（够过仲裁阈值，但降速）
Q_SINGLE_SEED = 0.25      # 单侧 + 半宽只有先验种子（**低于默认 min-quality → 按维持处理**）
Q_BORDER = 0.35           # 单侧 + 参考实现的"代画面边界"（能用，但中心是被边界"拉"出来的，
                          #   所以只给"略高于阈值"的分数，让控制层降速）
# "支持度"（线段够不够多）的满分线：**单侧只有一条线的线段，不能按双侧的条数要求它**
Q_SUPPORT_FULL = 6        # 双侧：两侧线段合计到这个数记满分
Q_SUPPORT_FULL_SINGLE = 3  # 单侧：一侧线段到这个数就记满分

# 半宽先验的物理范围（全宽 120~700px ← settings.LANE_PAIR_W_MIN/MAX_PX 的一半）
HALF_W_SEED_PX = float(getattr(settings, "LANE_HOUGH_HALF_W_DEFAULT_PX", 240.0))  # 种子值
HALF_W_WINDOW = 15        # 半宽滑动窗口
HALF_W_MIN_SAMPLES = 3    # 实测样本到这个数才认为"半宽来自实测"


# ---------------------------------------------------------------- 参数与读数
@dataclass
class LaneParams:
    """检测参数（命令行 / site.yaml 都能改，不写死）。"""
    roi_top_ratio: float = 0.35          # ROI 上沿（比例）：**只用来拟合直线**
    roi_bottom_ratio: float = 0.58       # ROI 下沿（比例）
    # 前瞻带 = 拿哪些**行**去算中心（绝对行号比例，与 ROI 解耦）。
    # 两个参考实现的比例**互相不一致**，所以这里没有"正确答案"，现场按实测支持行自动定：
    #   oldCode（320×240，见 launch.cpp）：ROI 0.50~0.85，算中心的行 130~230 = **0.54~0.96**（中近处）
    #   newnewCode（640×480）：纵向权重峰值在 0.31~0.43、覆盖 0.11~0.89（中远处）
    # 差异来自**摄像头安装**（谁看得见近处地面），不是算法。默认 0.35~0.48 是本车实拍里
    # "两条白线同时可见"的那一段（y 168~230）。
    look_top_ratio: float = 0.35
    look_bottom_ratio: float = 0.48
    target_ratio: float = 375.0 / 640.0  # 目标点（画面宽度比例）
    pick: str = "steepest"               # steepest=参考实现 / longest=更稳
    slope_min: float = SLOPE_ABS_MIN
    slope_max: float = SLOPE_ABS_MAX
    # 单侧丢线怎么办：
    #   width  = 用车道半宽先验推中心（本车方案，见 WidthPrior）
    #   border = **参考实现原样**：缺的一侧代画面边界（左缺→0，右缺→宽），中心=(可见x+边界)/2
    #   off    = 不给中心（= 上一版的行为，最保守）
    single_mode: str = "width"
    half_w_seed: float = HALF_W_SEED_PX
    pair_w_min: float = float(settings.LANE_PAIR_W_MIN_PX)
    pair_w_max: float = float(settings.LANE_PAIR_W_MAX_PX)

    @classmethod
    def from_settings(cls, **overrides) -> "LaneParams":
        """默认值取现场标定（`config/settings.py` ← `config/site.yaml`）。

        这样"改 ROI"只需要改一处：`lane_ref_test.py`、`lane_probe.py`、
        以后的主程序读的是同一个来源（旧版两套 ROI 是缺陷 #6）。
        """
        p = cls(roi_top_ratio=float(settings.LANE_ROI_TOP_RATIO),
                roi_bottom_ratio=1.0 - float(settings.LANE_ROI_BOTTOM_MARGIN) / float(settings.IMG_H))
        for key, value in overrides.items():
            if value is not None:
                setattr(p, key, value)
        return p


@dataclass
class LaneReading:
    """一帧的车道读数（像素单位；`center_x` 为 None = 本帧没有可信中心）。"""
    left_x: Optional[float] = None
    right_x: Optional[float] = None
    center_x: Optional[float] = None
    error: Optional[float] = None
    n_left: int = 0                     # 左侧支持线段数
    n_right: int = 0
    both_sides: bool = False            # 中心是不是"两侧都真看到"算出来的
    half_width: Optional[float] = None  # 本帧用的半宽（=真值或先验）
    width_measured: bool = False        # 半宽是否来自实测（否则只是种子）
    quality: float = 0.0                # 0~1；低于 --min-quality 时控制层按"维持"处理
    edge_pct: float = 0.0               # 边缘像素占比（诊断）
    canny: Tuple[float, float] = (0.0, 0.0)
    anchored: bool = True               # 选线时是否满足位置锚定（False=只按斜率选的）
    note: str = ""                      # 人看的中文说明（进 CSV）
    tag: str = ""                       # 机器可读的短标签（ASCII，画在叠加图上）：
                                        #   ""（正常双侧）/ 1side-width / 1side-seed /
                                        #   1side-border / 1side-off / no-seg / pair-oor /
                                        #   no-mid / unanchored
    # 本侧代表线的直线方程 (k, b)，x = (y-b)/k（诊断/体检用，不参与控制）
    left_kb: Optional[Tuple[float, float]] = field(default=None, repr=False)
    right_kb: Optional[Tuple[float, float]] = field(default=None, repr=False)

    @property
    def valid(self) -> bool:
        return self.center_x is not None


class WidthPrior:
    """车道半宽先验：两侧都在时实测、丢一侧时拿来推中心。

    "车在自己的车道里"是硬先验，所以"半宽"比"缺的线在画面边缘"可靠得多。
    窗口取中位数：弯道/透视变化下它不是常数，但变化是慢的。
    """

    def __init__(self, seed_px: float = HALF_W_SEED_PX, window: int = HALF_W_WINDOW,
                 min_px: float = 20.0, max_px: float = 500.0) -> None:
        self.seed = float(seed_px)
        self.window = max(1, int(window))
        self.min_px = float(min_px)
        self.max_px = float(max_px)
        self._samples: List[float] = []

    def reset(self) -> None:
        self._samples.clear()

    def update(self, half_w: Optional[float]) -> None:
        """两侧成对时报一次实测半宽（越界的样本不入队：多半是配对错了）。"""
        if half_w is None or not np.isfinite(half_w):
            return
        if not (self.min_px <= float(half_w) <= self.max_px):
            return
        self._samples.append(float(half_w))
        if len(self._samples) > self.window:
            del self._samples[0:len(self._samples) - self.window]

    @property
    def measured(self) -> bool:
        return len(self._samples) >= HALF_W_MIN_SAMPLES

    @property
    def value(self) -> float:
        if self.measured:
            return float(np.median(self._samples))
        return float(self.seed)


# ---------------------------------------------------------------- 工具函数
def _x_at(k: float, b: float, y: float) -> float:
    return (y - b) / k


def x_of_kb(kb: Tuple[float, float], y: float) -> float:
    """直线方程 (k,b) 在行 y 处的 x（x = (y−b)/k）—— 叠加图/诊断共用。"""
    k, b = kb
    return (y - b) / k if abs(k) > 1e-9 else 0.0


def roi_bounds(h: int, p: LaneParams) -> Tuple[int, int]:
    """ROI 的 y 上下沿（像素，已钳位、且保证至少有 2 行）。"""
    y0 = int(round(h * float(p.roi_top_ratio)))
    y1 = int(round(h * float(p.roi_bottom_ratio)))
    y0 = max(0, min(h - 2, y0))
    y1 = max(y0 + 2, min(h, y1))
    return y0, y1


def look_bounds(h: int, p: LaneParams) -> Tuple[int, int]:
    """前瞻带（算中心的行）的 y 上下沿；至少 2 行，且顺序合法。"""
    ya = int(round(h * float(p.look_top_ratio)))
    yb = int(round(h * float(p.look_bottom_ratio)))
    ya = max(0, min(h - 2, ya))
    yb = max(ya + 2, min(h, yb))
    return ya, yb


# ---------------------------------------------------------------- 检测器
class LaneRefDetector:
    """按 oldCode `picture()` 实现的车道检测（纯 CV，不依赖模型）。"""

    def __init__(self, params: Optional[LaneParams] = None, **overrides) -> None:
        if params is None:
            params = LaneParams.from_settings(**overrides)
        else:
            for key, value in overrides.items():
                if value is not None:
                    setattr(params, key, value)
        self.p = params
        self.width_prior = WidthPrior(params.half_w_seed)
        self.canny_low = float(CANNY_LOW0)
        self.canny_high = float(CANNY_HIGH0)

    # 兼容旧调用（`det.target_ratio = ...` / `det.roi_top_ratio` 之类）
    @property
    def roi_top_ratio(self) -> float:
        return self.p.roi_top_ratio

    @property
    def roi_bottom_ratio(self) -> float:
        return self.p.roi_bottom_ratio

    @property
    def target_ratio(self) -> float:
        return self.p.target_ratio

    @target_ratio.setter
    def target_ratio(self, value: float) -> None:
        self.p.target_ratio = float(value)

    def reset(self) -> None:
        """换场地/重新起步时清状态（Canny 自适应 + 半宽窗口）。"""
        self.width_prior.reset()
        self.canny_low = float(CANNY_LOW0)
        self.canny_high = float(CANNY_HIGH0)

    # ------------------------------------------------------ 链路第 1 步：边缘
    def _canny(self, frame_bgr: np.ndarray, y0: int, y1: int) -> Tuple[np.ndarray, float]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        roi = gray[y0:y1, :]
        blur = cv2.GaussianBlur(roi, (5, 5), 0.5)
        edges = cv2.Canny(blur, int(self.canny_low), int(self.canny_high), 3)
        n = int(np.count_nonzero(edges))
        # 参考实现的自适应是"每帧调一步"；我们在同一帧内迭代到位（单帧/低帧率下
        # 每帧一步永远追不上，操场颗粒地面实测边缘占比 34%）。
        # ⚠️ **必须钳位**：无上限地往上调会把车道线本身也滤掉。
        step = 0
        while step < 4 and (n > CANNY_EDGE_HI or n < CANNY_EDGE_LO):
            if n > CANNY_EDGE_HI:
                self.canny_low += 2
                self.canny_high += 4
            else:
                self.canny_low -= 2
                self.canny_high -= 4
            self.canny_low = max(CANNY_LOW_MIN, min(CANNY_LOW_MAX, self.canny_low))
            self.canny_high = max(CANNY_HIGH_MIN, min(CANNY_HIGH_MAX, self.canny_high))
            edges = cv2.Canny(blur, int(self.canny_low), int(self.canny_high), 3)
            n = int(np.count_nonzero(edges))
            step += 1
        return edges, float(np.count_nonzero(edges)) / max(1, edges.size)

    # ------------------------------------------------------ 链路第 2 步：线段
    def _segments(self, edges: np.ndarray, y0: int) -> List[Tuple[float, float, float]]:
        """返回 [(k, b, 长度)]；直线写作 x = (y - b) / k（与参考实现同口径 k=dy/dx）。"""
        lines = cv2.HoughLinesP(edges, 1, HOUGH_THETA_RAD, threshold=HOUGH_THRESH,
                                minLineLength=HOUGH_MIN_LEN, maxLineGap=HOUGH_MAX_GAP)
        out: List[Tuple[float, float, float]] = []
        if lines is None:
            return out
        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
            dy, dx = float(y2 - y1), float(x2 - x1)
            if abs(dx) < 1e-6:
                continue
            k = dy / dx
            if not (self.p.slope_min <= abs(k) <= self.p.slope_max):
                continue
            b = float(y1 + y0) - k * float(x1)   # 用整幅坐标，后面直接代 y
            out.append((k, b, math.hypot(dx, dy)))
        return out

    # ------------------------------------------------------ 链路第 3 步：选线
    def _pick_side(self, segs: Sequence[Tuple[float, float, float]], side: str,
                   y_ref: float, w: int) -> Tuple[Optional[Tuple[float, float]], bool]:
        """每侧选一条代表线。返回 ((k,b), 是否满足位置锚定)。

        参考实现只按"|k| 最大"选（在它的摄像头/赛道下够用），我们的操场画面里
        会选到隔壁车道或颗粒碎线段。这里加**位置锚定**（左线在画面中心左侧）作为
        首选，但**锚定不到时退回参考实现的行为**（只按斜率选）并把中心标成
        `anchored=False` —— 宁可给一个带标记的低质量读数，也不要整帧没有读数。
        """
        same_side = [s for s in segs if (s[0] > 0) == (side == "right")]
        if not same_side:
            return None, True
        anchored = []
        for k, b, length in same_side:
            x_ref = _x_at(k, b, y_ref)
            if side == "left" and x_ref < w * 0.5:
                anchored.append((k, b, length))
            elif side == "right" and x_ref > w * 0.5:
                anchored.append((k, b, length))
        pool, ok = (anchored, True) if anchored else (same_side, False)
        if self.p.pick == "longest":
            pool.sort(key=lambda s: -s[2])
        else:
            pool.sort(key=lambda s: -abs(s[0]))
        k, b, _l = pool[0]
        return (k, b), ok

    # ------------------------------------------------------ 对外：一帧读数
    def detect(self, frame_bgr: np.ndarray, width_prior: Optional[WidthPrior] = None) -> LaneReading:
        prior = width_prior if width_prior is not None else self.width_prior
        h, w = frame_bgr.shape[:2]
        y0, y1 = roi_bounds(h, self.p)
        edges, edge_pct = self._canny(frame_bgr, y0, y1)
        segs = self._segments(edges, y0)

        ya, yb = look_bounds(h, self.p)
        # ★ 一切 x 都在**前瞻带中点**这一行上量（选线锚定、配对宽度、半宽、单侧兜底），
        #   因为这一行就是算中心的那几行的中间 —— 参考实现也是"在 130~230 行上逐行求中点"
        #   （对直线而言，逐行平均 = 在中点取值的解析等价）。
        y_ref = (ya + yb) * 0.5
        left, a_l = self._pick_side(segs, "left", y_ref, w)
        right, a_r = self._pick_side(segs, "right", y_ref, w)

        reading = LaneReading(
            edge_pct=edge_pct, canny=(self.canny_low, self.canny_high),
            n_left=sum(1 for s in segs if s[0] < 0), n_right=sum(1 for s in segs if s[0] > 0),
            anchored=bool(a_l and a_r),
            tag="" if (a_l and a_r) else "unanchored")
        if left is not None:
            reading.left_x = _x_at(*left, y_ref)
            reading.left_kb = left
        if right is not None:
            reading.right_x = _x_at(*right, y_ref)
            reading.right_kb = right

        if left is not None and right is not None:
            if self._paired_ok(left, right, y_ref):
                self._fill_center(reading, left, right, w, h)
                reading.both_sides = True
                reading.half_width = abs(reading.right_x - reading.left_x) * 0.5
                reading.width_measured = prior.measured
                if reading.half_width is not None and \
                        self.p.pair_w_min <= 2 * reading.half_width <= self.p.pair_w_max:
                    prior.update(reading.half_width)
                reading.quality = Q_BOTH * self._support_factor(len(segs))
            else:
                note = (f"配对宽度越界（{abs(reading.right_x - reading.left_x):.0f}px 不在 "
                        f"{self.p.pair_w_min:.0f}~{self.p.pair_w_max:.0f}），按不可信处理")
                return self._finalize_untrusted(reading, prior, note)
            return reading

        if left is None and right is None:
            reading.note = "两侧都没有车道线段"
            reading.tag = "no-seg"
            return reading
        if self.p.single_mode == "off":
            reading.note = "只看到一侧（single_mode=off，不给中心）"
            reading.tag = "1side-off"
            return reading

        if self.p.single_mode == "border":
            # ★ **参考实现原样**（oldCode `picture()`）：缺的一侧代画面边界，
            #   中心 = (可见线 x + 边界) / 2，边界 = 左缺→0 / 右缺→画面宽。
            border = 0.0 if left is None else float(w)
            center = ((reading.left_x if left is not None else reading.right_x) + border) * 0.5
            reading.half_width = (center - reading.left_x) if left is not None \
                else (reading.right_x - center)
            reading.width_measured = prior.measured
            reading.note = f"单侧兜底（参考实现 border：{'左' if left is not None else '右'}线 + 画面边界）"
            reading.tag = "1side-border"
            base = Q_BORDER
        else:
            # ★ 本车方案：用半宽先验把缺的那一侧推出来（比"猜边界"更贴物理）
            half_w = prior.value
            reading.half_width = half_w
            reading.width_measured = prior.measured
            center = (reading.left_x + half_w) if left is not None \
                else (reading.right_x - half_w)
            reading.note = (f"单侧兜底（width：{'左' if left is not None else '右'}线 + 半宽 "
                            f"{half_w:.0f}px{'实测' if prior.measured else '种子'}）")
            reading.tag = "1side-width" if prior.measured else "1side-seed"
            base = Q_SINGLE_MEASURED if prior.measured else Q_SINGLE_SEED
        # 推出来的中心跑到画面外 → 这个先验在这帧不成立，宁可没有读数
        if not (0.0 <= center <= w - 1):
            reading.note = f"单侧兜底推出的中心出画面（{center:.0f}）"
            reading.tag = "oor"
            return reading
        reading.center_x = float(center)
        reading.error = float(center) - w * self.p.target_ratio
        n_side = len([s for s in segs if (s[0] > 0) == (right is not None)])
        reading.quality = base * self._support_factor(n_side, Q_SUPPORT_FULL_SINGLE)
        return reading

    def _finalize_untrusted(self, reading: LaneReading, prior: WidthPrior,
                            note: str) -> LaneReading:
        """两侧都在但配对不可信：保留左右读数、不给中心，但仍让控制层知道"看到了线"。"""
        reading.center_x = None
        reading.error = None
        reading.both_sides = False
        reading.half_width = prior.value
        reading.width_measured = prior.measured
        reading.quality = 0.0
        reading.note = note
        reading.tag = "pair-oor"
        return reading

    def _support_factor(self, n_segs: int, full: int = Q_SUPPORT_FULL) -> float:
        """支持度：线段越多越可信（单侧的 `full` 更小，见 Q_SUPPORT_FULL_SINGLE 的注释）。"""
        return min(1.0, float(n_segs) / float(full))

    def _paired_ok(self, left: Tuple[float, float], right: Tuple[float, float],
                   y_ref: float) -> bool:
        """成对宽度是否落进物理区间（配对宽度闸门，2026-09-19 操场实测加）。"""
        w_pair = _x_at(*right, y_ref) - _x_at(*left, y_ref)
        if w_pair <= 0:
            return False
        return self.p.pair_w_min <= w_pair <= self.p.pair_w_max

    def _fill_center(self, reading: LaneReading, left: Tuple[float, float],
                     right: Tuple[float, float], w: int, h: int) -> None:
        """前瞻带内逐行求左右交点中点再平均（参考实现：它取 130~230 行、共 100 行）。"""
        y_a, y_b = look_bounds(h, self.p)
        mids: List[float] = []
        for y in range(y_a, y_b, 2):
            xl, xr = _x_at(*left, y), _x_at(*right, y)
            if xl < xr:
                mids.append((xl + xr) * 0.5)
        if not mids:
            reading.center_x = None
            reading.error = None
            reading.quality = 0.0
            reading.note = "前瞻带里左右线没有交点（斜率异常）"
            reading.tag = "no-mid"
            return
        reading.center_x = float(np.mean(mids))
        reading.error = reading.center_x - w * self.p.target_ratio

    # ------------------------------------------------------ 对外：镜头视角体检
    def visible_range(self, frame_bgr: np.ndarray, step: int = 8,
                      tol_px: float = 6.0, min_rows: int = 2) -> dict:
        """**两条线同时可见的 y 范围**（用本检测器自己的边缘，不用另一套亮度阈值）。

        做法：在一条**很宽的** ROI（画面 10%~90%）上正常检测一次拿到左右线的直线方程，
        然后逐行检查"两侧都在这条行附近有边缘支持"，取支持行的最小/最大 y。
        判据（沿用 `lane_probe` 的『镜头朝向体检』）：两侧同时可见要延伸到 y ≥ 300
        （画面下半部），且两条线都不贴边（左 < 40 或 右 > 宽−40 就是贴边）。

        返回 dict: ok / y_lo / y_hi / left_x / right_x / rows / text
        """
        h, w = frame_bgr.shape[:2]
        wide = LaneParams(roi_top_ratio=0.10, roi_bottom_ratio=0.90,
                          look_top_ratio=self.p.look_top_ratio,
                          look_bottom_ratio=self.p.look_bottom_ratio,
                          target_ratio=self.p.target_ratio,
                          pick=self.p.pick, slope_min=self.p.slope_min,
                          slope_max=self.p.slope_max, single_mode="off",
                          half_w_seed=self.p.half_w_seed, pair_w_min=self.p.pair_w_min,
                          pair_w_max=self.p.pair_w_max)
        det = LaneRefDetector(wide)          # 独立实例：别污染在线那台的 Canny/半宽状态
        r = det.detect(frame_bgr)
        if r.left_kb is None or r.right_kb is None:
            return {"ok": False, "y_lo": None, "y_hi": None, "left_x": r.left_x,
                    "right_x": r.right_x, "rows": 0,
                    "text": "两侧没有同时成对的直线（先看 ROI 扫描表/逐行白点图）"}
        hits = rows_with_support(frame_bgr, r.left_kb, r.right_kb,
                                 int(0.10 * h), int(0.90 * h), step, tol_px, min_rows)
        if not hits:
            return {"ok": False, "y_lo": None, "y_hi": None, "left_x": r.left_x,
                    "right_x": r.right_x, "rows": 0,
                    "text": "两条线不在同一段高度上同时有边缘（成对性差）"}
        ys = [y for y, _, _ in hits]
        y_lo, y_hi = int(min(ys)), int(max(ys))
        lx = float(np.median([xl for _, xl, _ in hits]))
        rx = float(np.median([xr for _, _, xr in hits]))
        edge_bad = []
        if lx < 40:
            edge_bad.append(f"左线贴左边（x≈{lx:.0f}）")
        if rx > w - 40:
            edge_bad.append(f"右线贴右边（x≈{rx:.0f}）")
        edge_txt = "；".join(edge_bad) if edge_bad else f"两侧都不贴边（左 x≈{lx:.0f} / 右 x≈{rx:.0f}）"
        ok = y_hi >= 300 and not edge_bad
        if ok:
            text = f"✅ 够用：左右同时有边缘的 y 范围 {y_lo}~{y_hi}（已进画面下半部），{edge_txt}"
        else:
            why = []
            if y_hi < 300:
                why.append(f"只看得到远处（最低到 y={y_hi}，下半部 y≥300 没有）→ **把下摄往下压**")
            if edge_bad:
                why.append("线贴画面边缘，车一动就会丢一侧 → 压角度/换广角让线往里收")
            text = f"❌ 不够用：{y_lo}~{y_hi}；" + "；".join(why)
        return {"ok": ok, "y_lo": y_lo, "y_hi": y_hi, "left_x": lx, "right_x": rx,
                "rows": len(hits), "text": text}

    def support_rows(self, frame_bgr: np.ndarray, step: int = 4, tol_px: float = 6.0,
                     min_rows: int = 2) -> List[int]:
        """本帧里"左右线都有边缘支持"的行号（用当前 ROI 拟合出的直线方程）。

        现场用途：**自动定前瞻带**——两条线在哪几行是真的，就把中心算在哪几行，
        而不是让人猜 `--look-top/--look-bottom`。
        """
        h, _w = frame_bgr.shape[:2]
        y0, y1 = roi_bounds(h, self.p)
        edges, _pct = self._canny(frame_bgr, y0, y1)
        segs = self._segments(edges, y0)
        ya, yb = look_bounds(h, self.p)
        left, _ = self._pick_side(segs, "left", (ya + yb) * 0.5, _w)
        right, _ = self._pick_side(segs, "right", (ya + yb) * 0.5, _w)
        if left is None or right is None:
            return []
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        full_edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0.5),
                               int(CANNY_LOW0), int(CANNY_HIGH0), 3)
        hits = rows_with_support(frame_bgr, left, right, 0, h, step, tol_px, min_rows,
                                 edges=full_edges)
        return [y for y, _, _ in hits]


def rows_with_support(frame_bgr: np.ndarray, left_kb: Tuple[float, float],
                      right_kb: Tuple[float, float], y_from: int, y_to: int,
                      step: int = 4, tol_px: float = 6.0, min_rows: int = 2,
                      edges: Optional[np.ndarray] = None,
                      want_stats: bool = False):
    """逐行检查"左右两条拟合线附近是否都有边缘"，返回 [(y, x_left, x_right)]。

    这是"这两条线是真线还是外推出来的"的客观判据：真线在它经过的每一行附近都有边缘点。

    `want_stats=True` 时返回 `(rows, stats)`，stats 里多两个更抗干扰的量：
      checked  —— "两条线都在画面内"的行数（分母：不是整幅画面，避免被无关区域稀释）
      run_max  —— **最长连续支持行**（它经过的最长一段连续证据，是真线的强特征：
                   真车道线给几十行连续支持，反光/碎边只给几行零散支持）
    """
    h, w = frame_bgr.shape[:2]
    if edges is None:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0.5),
                          int(CANNY_LOW0), int(CANNY_HIGH0), 3)
    out: List[Tuple[int, float, float]] = []
    run: List[Tuple[int, float, float]] = []
    checked = 0
    run_max = 0
    step = max(1, step)
    for y in range(max(0, y_from), min(h, y_to), step):
        xl, xr = _x_at(*left_kb, y), _x_at(*right_kb, y)
        ok = False
        if 0 <= xl < w and 0 <= xr < w and xl < xr:
            checked += 1
            sl = edges[y, max(0, int(xl - tol_px)):min(w, int(xl + tol_px) + 1)]
            sr = edges[y, max(0, int(xr - tol_px)):min(w, int(xr + tol_px) + 1)]
            ok = bool(sl.size and sr.size and np.count_nonzero(sl) and np.count_nonzero(sr))
        if ok:
            run.append((y, xl, xr))
        else:
            if len(run) >= min_rows:
                out.extend(run)
            run_max = max(run_max, len(run) * step)
            run = []
    if len(run) >= min_rows:
        out.extend(run)
    run_max = max(run_max, len(run) * step)
    if want_stats:
        return out, {"checked": checked * step, "rows": len(out), "run_max": run_max}
    return out


# ---------------------------------------------------------------- 自动选 ROI / 前瞻带
# ★ 参考实现 oldCode 的带：getROI() = (h/20)*10 ~ (h/20)*17 = 画面高度的 **0.50~0.85**
#   （它的画面是 320×240，见 launch.cpp `capture.set(CAP_PROP_FRAME_WIDTH, 320)`）。
#   这一档永远是候选 —— 万一我们的安装/光照跟它接近，扫描就该选中它，而不是被我们的候选集排除。
REF_BAND = (0.50, 0.85)
# 其余候选覆盖"线只在画面上部"（平视角）到"线一直到画面底部"（镜头压下去之后）的各种情况；
# 现场配置那一档由 band_candidates() 插进来（去重）。
BAND_CANDIDATES: Tuple[Tuple[float, float], ...] = (
    REF_BAND,
    (0.15, 0.95), (0.25, 0.95), (0.35, 0.95), (0.45, 0.95), (0.55, 0.95),
    (0.15, 0.75), (0.25, 0.75), (0.35, 0.75), (0.45, 0.75),
    (0.20, 0.60), (0.30, 0.65), (0.40, 0.75),
)


def band_candidates(base: LaneParams) -> List[Tuple[float, float]]:
    """候选带 = 现场配置那一档 + 上面的通用档（去重、按上沿排序）。"""
    cands = [(round(float(base.roi_top_ratio), 3), round(float(base.roi_bottom_ratio), 3))]
    cands += [c for c in BAND_CANDIDATES if c not in cands]
    return cands


def score_frames(det: LaneRefDetector, frames: Sequence[np.ndarray]) -> dict:
    """在若干帧上跑一台检测器，汇总"这一组参数好不好"的客观指标。

    指标都直接可读，不做加权黑箱：
      paired_frames 有可信中心的帧数（最重要）
      run_max       **最长连续支持行**的中位数（"拟合出的线是一整条还是几段碎边"，
                    2026-09-22 实验室实测后加的：当时所有候选带的支持行占比都只有 0~12%，
                    按占比排名等于抛硬币，而"最长连续支持"能分出真线（几十行）和碎边（几行））
      support_frac  支持行 / "两条线都在画面内"的行（**分母只算线在画面内的行**，
                    不是整幅画面 —— 否则会被无关区域稀释，2026-09-22 修）
      segs_med      每帧支持线段数的中位数（只看数量，带越大人越大 → 只当参考）
      spread_px     有中心那些帧的中心跨距（越小越稳 = 不会来回打舵）
      q_med         质量的帧中位数
    """
    centers, segs, quals, fr, runs, paired = [], [], [], [], [], 0
    for f in frames:
        r = det.detect(f)
        segs.append(r.n_left + r.n_right)
        if r.center_x is not None:
            paired += 1
            centers.append(float(r.center_x))
            quals.append(r.quality)
            # 支持度只统计**配对被接受**的帧：否则会把"被配对宽度闸门否掉的假线"也算成高支持
            # （2026-09-22 实验室实测：出现 80 行支持但 0/10 帧成对的档，排名被它带偏）
            if r.left_kb is not None and r.right_kb is not None:
                h = f.shape[0]
                _hits, st = rows_with_support(f, r.left_kb, r.right_kb,
                                              int(0.05 * h), int(0.95 * h),
                                              step=4, tol_px=6.0, min_rows=2, want_stats=True)
                fr.append(st["rows"] / float(max(1, st["checked"])))
                runs.append(st["run_max"])
    spread = (max(centers) - min(centers)) if len(centers) >= 2 else 0.0
    return {"paired_frames": paired, "n_frames": len(frames),
            "run_max": float(np.median(runs)) if runs else 0.0,
            "support_frac": float(np.mean(fr)) if fr else 0.0,
            "segs_med": float(np.median(segs)) if segs else 0.0,
            "spread_px": spread,
            "q_med": float(np.median(quals)) if quals else 0.0,
            "centers": centers}


def sweep_bands(frames: Sequence[np.ndarray], base: LaneParams,
                candidates: Optional[Sequence[Tuple[float, float]]] = None,
                max_frames: int = 10) -> List[dict]:
    """逐档 (ROI 上沿, ROI 下沿) 跑一遍并打分 —— "该用哪条带"不再靠人眼看表。

    每档用**独立的检测器实例**（Canny 自适应与半宽都是独立状态，不能串）。
    排序键（2026-09-22 依据实验室实测重排）：有中心的帧数 → **最长连续支持行**（证据是不是一整条线）
    → 支持行占比 → 中心跨距（小者优先）。
    平票时保持插入顺序，而插入顺序把**现场配置那一档放在最前** → "没有更好的就沿用现状"。
    为了控制时间，只取前 `max_frames` 帧。
    """
    use = list(frames)[:max(1, int(max_frames))]
    h = frames[0].shape[0] if frames else settings.IMG_H
    out = []
    for top, bottom in (candidates if candidates is not None else band_candidates(base)):
        if top >= bottom - 0.05 or bottom > 1.0:
            continue
        params = LaneParams(**{**base.__dict__, "roi_top_ratio": float(top),
                               "roi_bottom_ratio": float(bottom)})
        det = LaneRefDetector(params)
        st = score_frames(det, use)
        st["roi_top_ratio"], st["roi_bottom_ratio"] = float(top), float(bottom)
        st["y_range"] = (int(h * top), int(h * bottom))
        out.append(st)
    out.sort(key=lambda s: (-s["paired_frames"], -s["run_max"], -s["support_frac"],
                            s["spread_px"]))
    return out


def auto_look_band(frames: Sequence[np.ndarray], params: LaneParams,
                   min_rows: int = 12, clamp: Tuple[float, float] = (0.10, 0.80)
                   ) -> Tuple[Tuple[float, float], List[int], dict]:
    """用**支持行里最长的连续一段**定前瞻带（两条线在哪几行是连续真的，就在哪几行算中心）。

    2026-09-22 实验室实测的教训：原来取"支持行的 15~85 分位"，而支持行的分布常常
    头尾很长（比如 32~360 都有零星支持）→ 前瞻带被拉宽 → 中心里掺进大量外推行，
    静止时中心就在 ±40~90px 间跳（一次自检的『中心跨距』甚至到 166px）。
    改成：把所有支持行合并排序，按"间隙 ≤ 5 行"分成连贯段，**取最长一段的中间
    20%~80%** —— 只在前瞻带里用"一整条连续线的证据"。

    返回 ((top_ratio, bottom_ratio), 支持行样本, 诊断信息)。
    拿不到支持行时返回原值（诊断信息里带 reason，调用方据此告警）。
    """
    det = LaneRefDetector(params)
    seen: List[int] = []
    for f in frames:
        seen.extend(det.support_rows(f))
    if not seen:
        return (params.look_top_ratio, params.look_bottom_ratio), [], {"reason": "no_support"}
    h = frames[0].shape[0]
    seen = sorted(set(seen))

    # 分成连贯段（间隙 ≤ 5 行：support_rows 的步长是 4，容一个漏检行）
    segs: List[List[int]] = []
    cur = [seen[0]]
    for y in seen[1:]:
        if y - cur[-1] <= 5:
            cur.append(y)
        else:
            segs.append(cur)
            cur = [y]
    segs.append(cur)
    longest = max(segs, key=len)
    # 第三个数 = **行数跨度**（不是采样点数：support_rows 的步长是 4，别把 144 行报成 36 行）
    seg_span = longest[-1] - longest[0] + 4
    info = {"longest_seg": (longest[0], longest[-1], seg_span),
            "n_segs": len(segs), "span": (seen[0], seen[-1])}

    i_lo = max(0, int(len(longest) * 0.2))
    i_hi = max(i_lo + 1, int(len(longest) * 0.8))
    y_lo = int(longest[i_lo])
    y_hi = int(longest[i_hi])
    if y_hi - y_lo < min_rows:
        y_mid = (y_lo + y_hi) // 2
        y_lo = max(0, y_mid - min_rows // 2)
        y_hi = min(h, y_lo + min_rows)
    y_lo = int(np.clip(y_lo, int(clamp[0] * h), h - 2))
    y_hi = int(np.clip(y_hi, y_lo + 2, int(clamp[1] * h)))
    return (max(0.0, min(1.0, y_lo / float(h))),
            max(0.0, min(1.0, y_hi / float(h)))), seen, info

