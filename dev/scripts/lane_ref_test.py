#!/usr/bin/env python3
"""循迹测试脚本（参考实现版）—— 严格按 oldCode/newnewCode 的循迹方案，独立自包含。

为什么再写一份
--------------
仓库里已有的两条循迹路（`vision/lane_scan.py` 的逐行跟踪、`vision/lane_hough.py`）
在现场都跑偏过。本脚本**只按参考实现来**，不再掺入我们自己的启发式：

    oldCode/src/vision/vision.cpp::picture()      → 纯 CV 车道线检测
        ROI → 灰度 → GaussianBlur(5,5) → **Canny(60,140)**（按边缘像素数自适应 ±2/±4）
        → **HoughLinesP(1, 3°, 阈值 50, 最短 30, 最大间隙 5)**
        → 每条线段求斜率 k=dy/dx，**只保留 0.25 ≤ |k| ≤ 2**（把地面纹理/颗粒/反光滤掉）
        → 按斜率符号分左右两组，各取 |k| 最大的那条当本侧车道线
        → 在若干行上求左右线交点、取中点平均 → error = 中点均值 − 画面中心

    oldCode/src/control/control.cpp::Control_FollowTrail()   → 控制
        error 直接用**像素**：pid = kp*e + ki*∫e + kd*Δe（kp=0.15, ki=0.01, kd=0.12）
        angle = 90 − pid，限幅 ±15°
        输出再做一次平滑：angle = 0.7*新 + 0.3*旧（抑制舵机抖动）
        误差大时自动减速（|e| < 5 加速 / < 15 正常 / 否则减速）

    newnewCode/src/lane_follower.py               → 借用两点
        · 目标点用**比例**表示（TARGET_X_RATIO），与分辨率无关
        · 纵向权重表（中间行权重最大）——本脚本用等权的"前瞻带"平均，可用 --band 调

用法
----
    # 只看画面与算法读数（不动电机）—— 本地也能跑：--image 指定一张图
    python3 scripts/lane_ref_test.py --no-motor
    python3 scripts/lane_ref_test.py --image /root/dev/logs/xxx.jpg

    # 现场（四轮落地、场地空旷、有人在旁能立刻断电）
    sudo python3 /root/dev/scripts/lane_ref_test.py --allow-motion --max-seconds 120 \
         --log-csv /root/dev/logs/lane_ref.csv

    # 调参（都可用命令行改，不用动代码）
    --kp/--ki/--kd       PID（像素误差口径，默认 0.15/0.01/0.12）
    --limit-deg          舵角限幅（默认 15）
    --target-ratio       目标点（画面宽度比例，默认 0.706 = 参考实现的 226/320）
    --roi-top/--roi-bottom-ratio   ROI（默认 0.50~0.85，同参考实现）
    --band               前瞻带占 ROI 的比例（默认 0.6，看 ROI 上部 = 看远处）

安全
----
摄像头独占（自动停/恢复推流）、电调死区守卫、任何退出路径电调归零 + 恢复远程控制阶段。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np

from config import settings
from control.filters import ErrorFilter
from vision.camera_guard import camera_exclusive, open_camera
from vision.start_gate import StartGate

NEUTRAL_US = 1500.0

# ---------------------------------------------------------------- 参考实现默认参数
CANNY_LOW0, CANNY_HIGH0 = 60, 140          # oldCode/src/config/config.cpp
CANNY_EDGE_LO, CANNY_EDGE_HI = 2000, 2500  # 边缘像素数目标区间（自适应）
CANNY_LOW_MIN, CANNY_LOW_MAX = 20.0, 160.0  # 自适应钳位（跑飞会把线也滤掉）
CANNY_HIGH_MIN, CANNY_HIGH_MAX = 40.0, 320.0
HOUGH_THRESH, HOUGH_MIN_LEN, HOUGH_MAX_GAP = 50, 30, 5
SLOPE_ABS_MIN, SLOPE_ABS_MAX = 0.25, 2.0   # |dy/dx| 物理范围
KP0, KI0, KD0 = 0.15, 0.01, 0.12           # 像素误差口径
INTEGRAL_LIMIT = 50.0
LIMIT_DEG0 = 15.0
SMOOTH0 = 0.7                              # 输出平滑：0.7*新 + 0.3*旧
TARGET_RATIO0 = 375.0 / 640.0              # ★ 操场实测：车摆正时车道中心在画面 375 处
                                           #   （参考实现是 226/320；那是它的摄像头安装）
SPEED_FAST_US, SPEED_SLOW_US = 0.0, -10.0  # 误差小/大时的脉宽增减
                                           # ★ 用户 2026-09-20 实测：循迹 1575 太快 → 基准降到 1560，
                                           #   自适应只减速不提速（参考实现是 +15/-20，见调试清单 §2.6）
SPEED_FLOOR_MARGIN_US = 5.0                # 减速档下限 = 死区 + 这个余量（低于死区车会直接停，
                                           #   基准 1560 时 1560-20=1540 < 1545 正好掉进死区）


@dataclass
class LaneReading:
    """一帧的车道读数（全部像素单位）。"""
    left_x: Optional[float] = None
    right_x: Optional[float] = None
    center_x: Optional[float] = None
    error: Optional[float] = None      # center_x - target_x
    n_left: int = 0                    # 左侧支持线段数
    n_right: int = 0
    mask_pct: float = 0.0              # 边缘像素占比（诊断）
    canny: Tuple[float, float] = (0.0, 0.0)


class LaneRefDetector:
    """按 oldCode 的 picture() 实现的车道检测（纯 CV，不依赖模型）。"""

    def __init__(self, roi_top_ratio: float = 0.50, roi_bottom_ratio: float = 0.85,
                 band_ratio: float = 0.6, target_ratio: float = TARGET_RATIO0,
                 pick: str = "steepest", slope_min: float = SLOPE_ABS_MIN,
                 slope_max: float = SLOPE_ABS_MAX) -> None:
        self.roi_top_ratio = roi_top_ratio
        self.roi_bottom_ratio = roi_bottom_ratio
        self.band_ratio = band_ratio
        self.target_ratio = target_ratio
        self.pick = pick            # steepest（参考实现）| longest（更稳，备选）
        self.slope_min = slope_min
        self.slope_max = slope_max
        self.canny_low = float(CANNY_LOW0)
        self.canny_high = float(CANNY_HIGH0)

    # ---------------------------------------------------------------- 检测
    def _canny(self, frame_bgr: np.ndarray, y0: int, y1: int) -> Tuple[np.ndarray, float]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        roi = gray[y0:y1, :]
        blur = cv2.GaussianBlur(roi, (5, 5), 0.5)
        edges = cv2.Canny(blur, int(self.canny_low), int(self.canny_high), 3)
        n = int(np.count_nonzero(edges))
        # 参考实现的自适应（每帧调一步）；这里**在同一帧内迭代到位**，否则单帧/慢帧率下
        # 阈值永远追不上（操场颗粒地面实测边缘占比 34%，太高）。
        step = 0
        while step < 4 and (n > CANNY_EDGE_HI or n < CANNY_EDGE_LO):
            if n > CANNY_EDGE_HI:
                self.canny_low += 2; self.canny_high += 4
            else:
                self.canny_low -= 2; self.canny_high -= 4
            # ⚠️ **必须钳位**：无上限地往上调会把车道线也滤掉（合成图/弱纹理画面实测会跑飞）
            self.canny_low = max(CANNY_LOW_MIN, min(CANNY_LOW_MAX, self.canny_low))
            self.canny_high = max(CANNY_HIGH_MIN, min(CANNY_HIGH_MAX, self.canny_high))
            edges = cv2.Canny(blur, int(self.canny_low), int(self.canny_high), 3)
            n = int(np.count_nonzero(edges))
            step += 1
        return edges, float(np.count_nonzero(edges)) / max(1, edges.size)

    def _segments(self, edges: np.ndarray, y0: int) -> List[Tuple[float, float, float]]:
        """返回 [(k, b, 长度)]，其中 x = (y - b) / k（与参考实现同口径：k=dy/dx）。"""
        lines = cv2.HoughLinesP(edges, 1, math.pi / 180.0 * 3,
                                threshold=HOUGH_THRESH, minLineLength=HOUGH_MIN_LEN,
                                maxLineGap=HOUGH_MAX_GAP)
        out: List[Tuple[float, float, float]] = []
        if lines is None:
            return out
        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
            dy, dx = float(y2 - y1), float(x2 - x1)
            if abs(dx) < 1e-6:
                continue
            k = dy / dx                                  # 参考实现：k = dy/dx
            if not (self.slope_min <= abs(k) <= self.slope_max):
                continue
            b = float(y1 + y0) - k * float(x1)            # 用整幅坐标，便于后面直接代 y
            length = math.hypot(dx, dy)
            out.append((k, b, length))
        return out

    def _pick_side(self, segs: List[Tuple[float, float, float]], side: str,
                   y_ref: float, w: int) -> Optional[Tuple[float, float]]:
        """每侧选一条线。

        参考实现只按"|k| 最大"选 —— 在它的摄像头/赛道下够用，但在我们的操场画面里
        会选到隔壁车道或颗粒的碎线段（实测左线取到 x=439，整条线是错的）。
        这里加两道**物理上必然成立**的约束（不改参考实现的检测流程）：
          ① 斜率符号：左线 dy/dx<0、右线 dy/dx>0（参考实现同款）；
          ② **位置锚定**：在参考行上，左线必须在画面中心左侧、右线在右侧
             （车在自己的车道里，这条永远成立）。
        再按 `--pick` 在候选里取 steepest（参考实现）或 longest（更稳）的一条。
        """
        cands = []
        for k, b, length in segs:
            if (k > 0) != (side == "right"):
                continue
            x_ref = self._x_at(k, b, y_ref)
            if side == "left" and x_ref >= w * 0.5:
                continue
            if side == "right" and x_ref <= w * 0.5:
                continue
            cands.append((k, b, length))
        if not cands:
            return None
        if self.pick == "longest":
            cands.sort(key=lambda s: -s[2])
        else:
            cands.sort(key=lambda s: -abs(s[0]))
        k, b, _l = cands[0]
        return k, b

    @staticmethod
    def _x_at(k: float, b: float, y: float) -> float:
        return (y - b) / k

    def detect(self, frame_bgr: np.ndarray) -> LaneReading:
        h, w = frame_bgr.shape[:2]
        y0 = int(h * self.roi_top_ratio)
        y1 = int(h * self.roi_bottom_ratio)
        edges, mask_pct = self._canny(frame_bgr, y0, y1)
        segs = self._segments(edges, y0)

        y_ref = (y0 + (y0 + int((y1 - y0) * self.band_ratio))) * 0.5
        left = self._pick_side(segs, "left", y_ref, w)
        right = self._pick_side(segs, "right", y_ref, w)
        reading = LaneReading(mask_pct=mask_pct, canny=(self.canny_low, self.canny_high),
                              n_left=len([s for s in segs if s[0] < 0]),
                              n_right=len([s for s in segs if s[0] > 0]))
        if left is None or right is None:
            # 与参考实现一致：单侧缺失时不给中心（我们不再用 0/宽 兜底，那会把中心拉飞）
            if left is not None:
                reading.left_x = self._x_at(*left, (y0 + y1) * 0.5)
            if right is not None:
                reading.right_x = self._x_at(*right, (y0 + y1) * 0.5)
            return reading

        # 前瞻带：ROI 上部（看远处），逐行求左右交点中点再平均
        y_a = y0
        y_b = y0 + int((y1 - y0) * self.band_ratio)
        mids: List[float] = []
        for y in range(y_a, max(y_a + 1, y_b), 2):
            xl, xr = self._x_at(*left, y), self._x_at(*right, y)
            if xl < xr:
                mids.append((xl + xr) * 0.5)
        if not mids:
            return reading
        reading.left_x = self._x_at(*left, (y_a + y_b) * 0.5)
        reading.right_x = self._x_at(*right, (y_a + y_b) * 0.5)
        reading.center_x = float(np.mean(mids))
        reading.error = reading.center_x - w * self.target_ratio
        return reading


class PidRef:
    """参考实现的 PID（像素误差口径，输出角度增量，带输出平滑）。"""

    def __init__(self, kp: float = KP0, ki: float = KI0, kd: float = KD0,
                 limit_deg: float = LIMIT_DEG0, smooth: float = SMOOTH0,
                 sign: float = 1.0) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit_deg = limit_deg
        self.smooth = smooth
        self.sign = float(sign)      # +1：角度增大=右转（与 settings.STEER_SIGN 同口径）
        self.integral = 0.0
        self.last_error = 0.0
        self.last_angle: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0
        self.last_angle = None

    def step(self, error: float) -> float:
        self.integral += error
        self.integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral))
        pid = self.kp * error + self.ki * self.integral + self.kd * (error - self.last_error)
        self.last_error = error
        # 参考实现是 `90 - pid`；这里接上 settings.STEER_SIGN，让它和 planner 同口径：
        #   sign=+1 表示"角度增大 = 右转" → 90 + pid；sign=-1 → 90 - pid（= 参考实现原式）
        # （2026-09-21 实验室：site.yaml 的 -1 与 bench_test --steer-test 的说明相反，
        #   而 --steer-test 当时被脚本 bug 挡住了没验成 —— 方向必须实测一次。）
        angle = 90.0 + self.sign * pid
        angle = max(90.0 - self.limit_deg, min(90.0 + self.limit_deg, angle))
        if self.last_angle is None:
            self.last_angle = angle
        else:                                                # 输出平滑，抑制抖动
            angle = self.smooth * angle + (1.0 - self.smooth) * self.last_angle
            self.last_angle = angle
        return angle


def adaptive_pulse(error: float, base_us: float,
                   fast: float = SPEED_FAST_US, slow: float = SPEED_SLOW_US) -> float:
    """参考实现的自适应速度：误差小加速、误差大减速（单位 us）。

    减速档有硬下限（死区 + 余量）：基准 1560 时 1560-20=1540 已低于死区 1545，
    那样"减速"会变成"停车"（用户 2026-09-20 把巡线速度降到 1560 后才出现这个风险）。
    """
    e = abs(error)
    if e < 5:
        us = base_us + fast
    elif e < 15:
        us = base_us
    else:
        us = base_us + slow
    return max(us, float(settings.ESC_DEADBAND_US) + SPEED_FLOOR_MARGIN_US)


def main() -> int:
    ap = argparse.ArgumentParser(description="循迹测试（参考实现版，独立自包含）")
    ap.add_argument("--image", default="", help="只分析一张图（本地可跑，不碰硬件）")
    ap.add_argument("--camera", type=int, default=2, help="摄像头（默认 2=下摄）")
    ap.add_argument("--speed-us", type=float, default=1560.0,
                    help="循迹脉宽（默认 1560；死区 1545，调试上限 1600）")
    ap.add_argument("--speed-fast", type=float, default=SPEED_FAST_US,
                    help=f"误差小(<5px)时的脉宽增量（默认 {SPEED_FAST_US:+.0f}，0=不提速）")
    ap.add_argument("--speed-slow", type=float, default=SPEED_SLOW_US,
                    help=f"误差大(≥15px)时的脉宽增量（默认 {SPEED_SLOW_US:+.0f}，"
                         f"下限=死区+{SPEED_FLOOR_MARGIN_US:.0f}us）")
    ap.add_argument("--start-us", type=float, default=1560.0, help="起步/探路脉宽")
    ap.add_argument("--max-seconds", type=float, default=120.0)
    ap.add_argument("--kp", type=float, default=KP0)
    ap.add_argument("--ki", type=float, default=KI0)
    ap.add_argument("--kd", type=float, default=KD0)
    ap.add_argument("--limit-deg", type=float, default=LIMIT_DEG0, help="舵角限幅（默认 15）")
    ap.add_argument("--smooth", type=float, default=SMOOTH0)
    ap.add_argument("--target-ratio", type=float, default=TARGET_RATIO0)
    # ★ ROI 用操场实测的"两条白线同时可见"的那条带（参考实现是 0.50~0.85，
    #   但那个区间里我们下摄的右线已经跑出画面 → 只会找到单侧）
    ap.add_argument("--roi-top", type=float, default=0.35)
    ap.add_argument("--roi-bottom-ratio", type=float, default=0.58)
    ap.add_argument("--band", type=float, default=0.6, help="前瞻带占 ROI 比例")
    ap.add_argument("--pick", choices=("steepest", "longest"), default="steepest",
                    help="每侧选哪条线段：steepest=参考实现；longest=更稳（备选）")
    ap.add_argument("--acquire-s", type=float, default=3.0, help="发车后低速探路找线窗口")
    ap.add_argument("--err-stop-px", type=float, default=60.0,
                    help="跑偏保护：误差绝对值 ≥ 此值（像素）")
    ap.add_argument("--err-stop-s", type=float, default=1.5,
                    help="跑偏保护：连续这么多秒都大误差 → 停车（0 表示关闭）")
    ap.add_argument("--steer-sign", type=float, default=None,
                    help="转向符号：+1=角度增大是右转（与 settings.STEER_SIGN 同口径）；"
                         "默认取 settings.STEER_SIGN（当前 site.yaml 里是 "
                         f"{settings.STEER_SIGN:+.0f}）——方向不对就改这里或 site.yaml")
    ap.add_argument("--no-filter", action="store_true",
                    help="关掉误差滤波（默认开：中位数剔离群 + 越新权重越大，"
                         "实测能压掉 345/367 来回跳）")
    ap.add_argument("--filter-window", type=int, default=None,
                    help=f"滤波窗口（默认 settings.ERROR_FILTER_WINDOW={settings.ERROR_FILTER_WINDOW}）")
    ap.add_argument("--filter-outlier", type=float, default=None,
                    help=f"离群阈值（默认 settings.ERROR_FILTER_OUTLIER={settings.ERROR_FILTER_OUTLIER}）")
    ap.add_argument("--no-motor", action="store_true", help="只跑视觉与决策")
    ap.add_argument("--allow-motion", "--i-know-wheels-are-up", dest="allow_motion",
                    action="store_true", help="确认车可以移动")
    ap.add_argument("--log-csv", default="")
    ap.add_argument("--print-every", type=int, default=3)
    args = ap.parse_args()

    det = LaneRefDetector(roi_top_ratio=args.roi_top, roi_bottom_ratio=args.roi_bottom_ratio,
                          band_ratio=args.band, target_ratio=args.target_ratio, pick=args.pick)

    # ---------------------------------------------------------------- 单图模式（本地可跑）
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[REF] 读不到图片 {args.image}")
            return 1
        r = det.detect(img)
        print(f"[REF] 边缘占比={r.mask_pct * 100:.1f}%  Canny=({r.canny[0]:.0f},{r.canny[1]:.0f})  "
              f"线段 左{r.n_left}/右{r.n_right}")
        print(f"[REF] 左={r.left_x if r.left_x is None else round(r.left_x, 1)} "
              f"右={r.right_x if r.right_x is None else round(r.right_x, 1)} "
              f"中心={r.center_x if r.center_x is None else round(r.center_x, 1)} "
              f"目标={img.shape[1] * args.target_ratio:.0f} "
              f"误差={r.error if r.error is None else round(r.error, 1)}")
        return 0

    use_motor = not args.no_motor
    if use_motor and not args.allow_motion:
        print("[REF] 拒绝运行：这次会驱动电机。确认四轮架空或场地空旷、有人在旁能立即断电，"
              "再加 --allow-motion；只想看读数就加 --no-motor")
        return 2
    if use_motor and min(args.start_us, args.speed_us) < settings.ESC_DEADBAND_US:
        print(f"[REF] 拒绝运行：脉宽低于电调死区 {settings.ESC_DEADBAND_US}us（实测 1540 不转、1545 起转）")
        return 2

    from scripts.bench_common import MotorSession, install_signal_guard, restore_remote_stage
    state = {"stopped": False, "driver": None, "restored": False, "used_motor": use_motor}
    install_signal_guard(state)

    gate = StartGate()
    steer_sign = float(settings.STEER_SIGN if args.steer_sign is None else args.steer_sign)
    pid = PidRef(args.kp, args.ki, args.kd, args.limit_deg, args.smooth, sign=steer_sign)
    err_filter = None if args.no_filter else ErrorFilter(args.filter_window, args.filter_outlier)
    print(f"[REF] 参考实现循迹：kp={args.kp} ki={args.ki} kd={args.kd} 限幅±{args.limit_deg}° "
          f"平滑={args.smooth} 目标比={args.target_ratio:.3f} ROI={args.roi_top}~{args.roi_bottom_ratio} "
          f"选线={args.pick}")
    print(f"[REF] 脉宽 起步={args.start_us:.0f} → 循迹={args.speed_us:.0f}us"
          f"（直道 {args.speed_us + args.speed_fast:.0f} / 大误差 {max(args.speed_us + args.speed_slow, settings.ESC_DEADBAND_US + SPEED_FLOOR_MARGIN_US):.0f}）"
          f"；规则：板在→停；移开→循迹；再见板→停")
    print(f"[REF] 跑偏保护：误差 ≥{args.err_stop_px:.0f}px 持续 {args.err_stop_s:.1f}s 就停车"
          if args.err_stop_s > 0 else "[REF] 跑偏保护：已关闭（--err-stop-s 0）")
    print("[REF] 误差滤波："
          + ("关（--no-filter）" if err_filter is None else
             f"窗口 {err_filter.window} / 离群阈值 {err_filter.outlier:.0f}px（中位数剔除 + 越新权重越大）"))
    print(f"[REF] 转向符号 steer_sign={steer_sign:+.0f}"
          f"（{'+1：角度增大=右转' if steer_sign > 0 else '-1：角度增大=左转'}，"
          f"来自 {'命令行' if args.steer_sign is not None else 'settings/site.yaml'}）"
          f" —— 先用 bench_test.py --steer-test 验方向")

    csv_fh = open(args.log_csv, "w", encoding="utf-8") if args.log_csv else None
    if csv_fh:
        csv_fh.write("t,phase,center_x,error_px,angle_deg,esc_us,left_x,right_x,n_left,n_right,edge_pct\n")

    seen_board = False
    big_err_since = None      # 跑偏保护的计时起点（误差小/无读数时清空）
    steer = 90.0
    rc = 0
    t0 = time.time()
    try:
        with MotorSession(state, enabled=use_motor,
                          speed_us_max=max(args.start_us, args.speed_us)) as pca:
            with camera_exclusive():
                cap = open_camera(args.camera, settings.IMG_W, settings.IMG_H)
                if cap is None:
                    rc = 1
                else:
                    acquire_since = None
                    frames = 0
                    while not state["stopped"] and (time.time() - t0) < args.max_seconds:
                        ok, frame = cap.read()
                        if not ok or frame is None:
                            time.sleep(0.02)
                            continue
                        now = time.time()
                        gs = gate.update(frame)
                        if gs.blocked:
                            seen_board = True
                            acquire_since = None
                        elif seen_board and acquire_since is None:
                            acquire_since = now

                        r = det.detect(frame)
                        driving = seen_board and not gs.blocked
                        exploring = driving and acquire_since is not None and \
                            (now - acquire_since) < args.acquire_s
                        phase = "stop"
                        out_us = NEUTRAL_US
                        angle = 90.0
                        err_c = None                      # 送进控制器的误差（已滤波）
                        if r.error is not None:
                            err_c = r.error if err_filter is None else err_filter.update(r.error)
                        # ★ 跑偏保护：一直大误差说明"锁到的不是本车道"或"转向方向不对"，
                        #   参考实现没有这一层（2026-09-21 实验室：误差恒在 -33px、舵机一直
                        #   往左打到冲出跑道，226 帧里误差从没回到 0 附近）。
                        guard_trip = False
                        if args.err_stop_s > 0 and driving and err_c is not None \
                                and abs(err_c) >= args.err_stop_px:
                            if big_err_since is None:
                                big_err_since = now
                            elif now - big_err_since >= args.err_stop_s:
                                print(f"[REF] ⚠️ 连续 {args.err_stop_s:.1f}s 误差 ≥{args.err_stop_px:.0f}px"
                                      f"（当前 {err_c:+.1f}）→ 判定跑偏/锁错线，停车")
                                state["stopped"] = True
                                guard_trip = True
                        else:
                            big_err_since = None
                        if driving:
                            if err_c is not None:
                                pid_out = pid.step(err_c)
                                steer = pid_out
                                out_us = adaptive_pulse(err_c, args.speed_us,
                                                        args.speed_fast, args.speed_slow)
                                phase = "track"
                            elif exploring:
                                # 探路：没有线也低速往前拱（用中位舵角），窗口结束仍无线就停
                                pid.reset()
                                if err_filter is not None:
                                    err_filter.reset()
                                out_us = args.start_us
                                phase = "explore"
                            else:
                                pid.reset()
                                if err_filter is not None:
                                    err_filter.reset()
                            angle = steer
                        else:
                            pid.reset()
                            if err_filter is not None:
                                err_filter.reset()
                            steer = 90.0
                            angle = 90.0
                        if guard_trip:                    # 触发的这一帧就不再输出动力
                            out_us = NEUTRAL_US
                            angle = 90.0

                        if use_motor and pca is not None:
                            pca.set_steering_angle(angle)
                            pca.write_us(pca.CH_ESC, out_us)

                        frames += 1
                        if frames % max(1, args.print_every) == 0:
                            cx = "-" if r.center_x is None else f"{r.center_x:5.1f}"
                            er = "-" if r.error is None else f"{r.error:+6.1f}"
                            print(f"[REF] {now - t0:6.1f}s 中心={cx} 误差={er} "
                                  f"舵机={angle:5.1f}° 电调={out_us:4.0f}us "
                                  f"线段{ r.n_left:2d}/{r.n_right:2d} 边缘={r.mask_pct * 100:4.1f}% ({phase})")
                        if csv_fh is not None:
                            csv_fh.write("%.3f,%s,%s,%s,%.1f,%.0f,%s,%s,%d,%d,%.1f\n" % (
                                now - t0, phase,
                                "" if r.center_x is None else "%.1f" % r.center_x,
                                "" if r.error is None else "%.1f" % r.error,
                                angle, out_us,
                                "" if r.left_x is None else "%.1f" % r.left_x,
                                "" if r.right_x is None else "%.1f" % r.right_x,
                                r.n_left, r.n_right, r.mask_pct * 100))
                    cap.release()

                    if not seen_board:
                        print("[REF] ⚠️ 全程没见过蓝板：车不会动（这是安全设计）")
                    elif driving and det.detect(frame).error is None:
                        print("[REF] ⚠️ 结束时没有有效车道：检查 ROI/斜率范围，或跑 lane_probe 看画面")
    finally:
        if csv_fh is not None:
            csv_fh.close()
            print(f"[REF] 数据已写入 {args.log_csv}")
        restore_remote_stage(state, "循迹测试结束")
    return rc


if __name__ == "__main__":
    sys.exit(main())
