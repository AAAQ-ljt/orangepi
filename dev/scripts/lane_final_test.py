#!/usr/bin/env python3
"""最终循迹脚本（2026-10-14）—— 摄像头已固定，唯一循迹入口。

一段话讲清它做什么
------------------
把 640×480 的下摄画面里**两条白色车道线**找出来 → 算车道中心 → PD 控制舵机/电调，
**起步前静止自检 → 蓝板边沿发车 → 跟线 → 丢线阶梯 → 全程落盘**。
不碰主程序（vision_main/main.py）、不改任何配置文件；单文件自包含，离线/在线同一套代码。

检测链路（为什么不是照抄官方 Hough，证据在下面）
------------------------------------------------
官方两份参考（2025 opencv 版 `image.cpp::TUxiang_Init3/image_handle2`、2026 室外样例
`双线循迹代码.txt`/`demo(中文注释).cpp`）都是：
    ROI → 灰度 → Canny(60,140) → HoughLinesP → 斜率闸门分左右 → 每侧取一条 → 逐行中点平均
**本素材上这条链路会锁错东西**（2026-10-14 用 `train/opencv-level1/video/frames/2`
全量实测，见 `lane_test/1014_lane_final/` 的对照图与统计）：
  · 摄像头压低后画面 2/3 是**近场路面纹理**：全片 Hough 段绝大多数 |k|<0.45 的近水平
    "刷痕/褶皱"，官方链路每侧"取最陡"在两帧上锁到了这些假结构（f0900：左右两侧
    被选中的是 k=-0.27/x@100=500 与 k=+0.17/x@100=214，两线交叉、宽度越界）；
  · 真白线是**细亮斜线**（宽 2~6px，|k| 实测 0.35~1.2，随车身偏航变化），
    在 y≈0~190 之间；y>200 以下白线已经出画，只剩路面纹理。
所以本脚本把"找线"换成**匹配式脊搜索**（脊 = 亮结构 − 两侧背景），其余全部沿用官方口径：
    ROI 行带 → 灰度 → 顶帽(thin bright ridge) → 脊响应（减 ±12px 旁侧背景）→
    在 (斜率 k, 参考行 x@y_ref) 网格上搜**该侧最强共线脊**（粗搜+精搜两步）→
    左右各一条 → 位置/配对/时域闸门 → 前瞻带逐行中点平均 = center_x → error = center − target
实测判据分离度（同一帧内）：真线脊分 +9~+25（沿线灰度 145~223），冲刷区/刷痕假线 ≤ +6；
扫描全片时 73% 的帧能稳定给出双侧（详情见脚本自带 `--calibrate` 输出与验证报告）。

官方语义里"参考实现照搬"的部分（没改）：单侧丢线**照样给中心**（2025 oldCode 对丢线一侧代
画面边界；本脚本优先用**车道半宽先验**替代边界，退化时用边界并压 quality 分）、
逐行中点平均、自适应速度分档、PD（Ki 默认 0）、输出限幅与平滑、30fps 下不锁死帧率的 dt 折算。

与官方有意偏离（全部登记在此）
------------------------------
1. 找线用脊搜索而不是 Hough（理由/证据见上）；
2. 滑块闸门 |k| 下限 0.25（官方 demo 也写 0.25；本素材真线 |k|≥0.35，留余量）；
3. 配对闸门（宽 240~820px @y_ref + 左右有序）——官方没有，但本素材上不加就会把
   冲刷区的假对当"双侧"（73% 的双侧里假对占多数，实测）；
4. 时域一致性（keep-prev jump-reject，2026-10-11 操场教训）：一侧帧间跳 >60px（15fps 口径，
   按实测 dt 折算）而另一侧 ≤25px → 丢该侧走单侧兜底，且**不清记忆**；
5. 单侧帧默认**回中位直行 + 保速**（2026-10-11 操场实测：跟推算中心会 ±50px 蛇行）。
6. ★ 单侧帧也要过时域门限（2026-10-15，事故复盘）：只剩一侧时那条线会**突然锁到别的
   结构上**——实测 5.34s→5.47s 左线 x@y_ref 从 +163 跳到 −58（一跳 220px），q=0.55
   照样算"有效" → 车带着垃圾中心全速直冲。门限沿用 jump_px（实测合法逐帧位移 p99
   只有 28~42px、垃圾跳 200px+，60px 分得开）；触发 = **丢本帧读数**走丢线阶梯，
   不做任何"修正"（宁可停，也不要带着垃圾读数继续跑）；
7. ★ 单侧帧的"回中位保速"有**时限**（`--single-max-s`，默认 1.2s）：10-11 的用意是
   S 弯里"短暂"丢一侧；连续 1.2s 都只有一侧 = 车已出线/线已出画，继续保速就是直冲
   （2026-10-15 的"冲出赛道"就是这样来的）。超时按丢线停车（可恢复）。

帧率口径（下摄 ≈30fps，自动曝光，亮/暗会 9~30 变）
---------------------------------------------------
控制律全程走 dt：`control/rate.py` 的 `FrameRate` 实测 dt + `PidRef.step(e, dt)` +
滤波窗口/护栏按时间折算。增益按 **15fps 登记口径**给（`control/lane_control.py::gains_for`），
30fps 下逐帧等效自动变成 ki 减半 / kd 加倍 —— **换帧率不用重标 kp/kd**，现场 A/B 表照旧写。
启动时会打印"登记口径 → 逐帧等效"两行，一眼能对。

用法
----
    PY="E:/venvs/smartcar-ultra/Scripts/python.exe"; cd dev && export PYTHONPATH=.

    # ① 离线：单图（不碰硬件）——看检测器到底锁了什么
    "$PY" scripts/lane_final_test.py --image 某张赛道图.jpg

    # ② 离线：跑一段帧目录（1014 素材），出 CSV/叠加图/接触表/四项指标
    "$PY" scripts/lane_final_test.py --frames ../train/opencv-level1/video/frames/2 \
        --out ../train/opencv-level1/lane_test/1014_lane_final/run --every 40 --montage 20

    # ③ 离线：直接跑视频（中文字幕名自动兜底）
    "$PY" scripts/lane_final_test.py --video ../train/opencv-level1/video/true_track_sub_video/2.mp4 --limit 300

    # ④ 离线标定（车在道中/素材）：打印 ROI/前瞻带/宽度/target 候选
    "$PY" scripts/lane_final_test.py --frames <DIR> --calibrate

    # ⑤ 车上：只看读数不动电机（车摆正在车道里，别挡板）
    sudo python3 /root/dev/scripts/lane_final_test.py --no-motor

    # ⑥ 车上：正式跑（场地空旷 + 有人持急停；蓝板挡一下再移开 = 发车）
    sudo python3 /root/dev/scripts/lane_final_test.py --allow-motion --max-seconds 120

现场清单（去操场照这个顺序做）
------------------------------
1. 车摆正在车道里（自检要看你前方的赛道）→ 启动 ⑤/⑥，看启动日志三行：
   「实测帧率」「检测档 ROI/前瞻带」「PID 登记→逐帧等效」；
2. 看到「✅ 起步前自检完成」→ 把蓝板挡到下摄正前方 → 移开 → 发车；
3. 想停：把板挡回下摄正前方（或 Ctrl+C）；跑完看 `logs/lane_<时间戳>/run.csv`；
4. 网页图传全程可用（本脚本只停副摄那一路推流，主摄/网页不受影响）；
5. 读数不对先跑 ①/② 看叠加图，别在现场猜参数。

安全（红线，同 AGENTS.md §1）
-----------------------------
摄像头独占（自动停/恢复副摄推流）、电调死区守卫、任何退出路径电调归零 + 舵机回中 +
恢复推流/opi-control；发车**边沿触发**（先见板 → 再消失，没见过板绝不动车）；
默认 dry-run，动电机必须显式 `--allow-motion`。
"""
from __future__ import annotations

import argparse
import glob
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np

from config import settings
from control.filters import ErrorFilter
from control.lane_control import (BigErrorGuard, LossGuard, NEUTRAL_US, OscillationGuard,
                                  PidRef, adaptive_pulse, floor_pulse, gains_for)
from control.rate import REF_FPS, FrameRate
from vision.camera_guard import camera_exclusive, crop_frame, open_camera
from vision.start_gate import StartGate

# ================================================================ 检测档（1014 素材实测标定）
# 出处：train/opencv-level1/video/true_track_sub_video/2.mp4（3573 帧 @28.9fps，摄像头固定
#       后手推实拍）→ frames/2/；探针与对照图在 train/opencv-level1/lane_test/1014_lane_final/。
# ⚠️ 换摄像头安装角度/高度必须重跑 `--calibrate` 重标这几档（注释里给了实测出处）。
ROI_TOP_PX = 5              # 检测带（找线的行范围）：真线在 y0~190，取到 200 留余量
ROI_BOTTOM_PX = 200
Y_REF_PX = 100              # 直线参数化参考行：x@100 与 k 描述一条线（实测真线在 y100 处清晰）
LOOK_TOP_PX = 24            # 算中心的前瞻带（行）：线在 y0~190 内均可见，取中上段更稳
LOOK_BOTTOM_PX = 165
K_MIN, K_MAX = 0.25, 1.90   # 斜率闸门 |dy/dx|（实测真线 0.35~1.2；0.25 与官方 demo 同值）
XREF_LEFT = (-260.0, 300.0)   # 左线 x@y_ref 搜索范围（允许出画；出画是压低镜头的常态）
XREF_RIGHT = (340.0, 900.0)
TOPHAT_K = 25               # 顶帽核（去除 >25px 的宽亮区，只留"细亮脊"）
TOPHAT_CLIP = 70.0          # 脊响应上限（防过曝区一句话盖过一切）
RIDGE_OFF_PX = 12           # 脊 = 沿线响应 − 两侧 ±12px 背景均值（实测把冲刷区假线从 40 压到 6）
RIDGE_MIN = 7.0             # 判"这条线是真的"的脊分下限（实测真线 9~25，假线 ≤6）
MIN_SAMPLES = 36            # 带内可见采样点下限（行步长 3 → 66 点满；36 ≈ 可见 108px）
PAIR_W_MIN, PAIR_W_MAX = 240.0, 820.0   # 配对宽（@y_ref，实测真对 590~680；留足余量）
EDGE_TOL_PX = 80.0          # 线允许出画的容差（采样越界仍计入分数）
HALF_W_SEED = 300.0         # 半宽先验种子（实测配对宽 ~600 → 半宽 300）
HALF_W_WINDOW = 15          # 半宽滑窗（帧；按帧率折算）
HALF_W_MIN_SAMPLES = 3
# 时域一致性（keep-prev）：登记 15fps 口径，按实测 dt 线性折算（30fps → 门限减半）
JUMP_REJECT_PX = 60.0
JUMP_COHERENT_PX = 25.0
# 置信度分档（与旧 lane_ref 同口径，控制层按 min-quality 决定跟/维持）
Q_BOTH, Q_SINGLE_MEASURED, Q_SINGLE_SEED, Q_BORDER = 1.0, 0.55, 0.25, 0.35
MIN_QUALITY0 = 0.28         # 低于它按丢线处理（< Q_SINGLE_SEED 时进维持/回中）
# ★ target_x 默认值（= 期望的车道中心像素）：
#   1014 素材（车手推、大致居中）双侧帧 center 中位 = 330（p05~p95 272~380）。
#   **权威值以现场静止自检为准**（车摆正在车道里跑 `--calibrate`）；
#   注意老 site.yaml 里的 target_x（320 / 343 / 375）是**旧镜头姿态**下标定的，对本机位作废。
TARGET_X0 = 330.0

# 找线 = 「脊核 → Hough 取种 → 脊分选优 → 局部精修」
#   为什么不是纯网格搜索（2026-10-14 首版踩的坑）：把线参数化成 (k, x@y_ref) 时，
#   "对齐真线"在参数空间里是一条**斜带**——k 差 0.05 就让候选线在带两端偏 ±20px，
#   6px 步长的网格会整条错过真线（合成用例实测：真线候选脊分 69，网格最优只有 5）。
#   改成用 Hough 在**脊核**上取种以后，(k, b) 由真实片段给出、不受网格分辨率影响。
RIDGE_MASK_THR = 18.0       # 脊核阈值（>它才算"亮脊"；实测真线脊 9~25、刷痕 ≤6）
HOUGH_THETA_DEG = 3.0       # 与官方一致（0.05rad 会让碎线整条消失，09-21 结论）
HOUGH_THR = 16              # 票数阈值（脊核比 Canny 边缘稀疏，票数要比官方 50 低）
HOUGH_MIN_LEN = 25          # 最短线段（2025 参考用 25）
HOUGH_MAX_GAP = 20          # 允许断口（白线是虚线/被反光打断）
K_COARSE_STEP = 0.10        # 精修窗口：k ±0.06 / x ±8（围绕 Hough 种子）
X_COARSE_STEP = 8.0
K_FINE_STEP = 0.01
X_FINE_STEP = 2.0
ROW_STEP = 3

# ---- 控制默认值（现场可 CLI 覆盖；都是登记口径 15fps，dt 折算在库里做）
HOLD_S0 = 0.6               # 丢线维持窗口
ACQUIRE_S0 = 3.0            # 发车后"还没见过线"的探路窗口
SINGLE_LIMIT_DEG = 5.0      # 单侧帧舵角限幅（10-11 操场：单侧读数会飘，限小甩不动）
WEAK_LIMIT_DEG = 10.0       # 弱对帧（一侧支持不足）限幅
WEAK_MIN_SAMPLES = 40       # 采样点低于它算"弱对"（满 66）
SINGLE_MAX_S0 = 1.2         # ★ 单侧帧"回中位保速"的时限（秒）；超时按丢线停车（0=关）
ERR_STOP_PX0, ERR_STOP_S0 = 60.0, 1.5
RESUME_FRAMES = 5           # 丢线停车后连续 N 帧有效才自动恢复（防抖）
PRINT_EVERY0 = 10
MAX_SECONDS0 = 120.0


# ================================================================ 读数
@dataclass
class Reading:
    """一帧的车道读数（像素；center_x=None = 本帧没有可信中心）。"""
    center_x: Optional[float] = None
    error: Optional[float] = None
    left_x: Optional[float] = None       # x@y_ref
    right_x: Optional[float] = None
    left_k: Optional[float] = None
    right_k: Optional[float] = None
    score_l: float = 0.0                 # 脊分（诊断/标定用）
    score_r: float = 0.0
    n_l: int = 0                         # 带内可见采样点数（≈可见长度/3）
    n_r: int = 0
    both_sides: bool = False
    half_w: Optional[float] = None
    width_measured: bool = False
    quality: float = 0.0
    tag: str = ""                        # ASCII 短标签（CSV/叠加图；追加式）
    note: str = ""

    @property
    def valid(self) -> bool:
        return self.center_x is not None

    def add_tag(self, t: str) -> None:
        """追加标签（保留已有）——旧版踩过"直接赋值把 jump-reject 吃掉"的坑。"""
        self.tag = f"{self.tag}|{t}" if self.tag else t


class HalfWidthPrior:
    """车道半宽先验：两侧都在时实测、丢一侧时拿来推中心（沿用旧 lane_ref 的方案）。"""

    def __init__(self, seed: float = HALF_W_SEED, window: int = HALF_W_WINDOW,
                 lo: float = 80.0, hi: float = 500.0) -> None:
        self.seed = float(seed)
        self.window = max(1, int(window))
        self.lo, self.hi = float(lo), float(hi)
        self._samples: List[float] = []

    def reset(self) -> None:
        self._samples.clear()

    def update(self, half_w: Optional[float]) -> None:
        if half_w is None or not np.isfinite(half_w) or not (self.lo <= half_w <= self.hi):
            return                       # 越界样本多半是配对错了，不入队
        self._samples.append(float(half_w))
        if len(self._samples) > self.window:
            del self._samples[0:len(self._samples) - self.window]

    @property
    def measured(self) -> bool:
        return len(self._samples) >= HALF_W_MIN_SAMPLES

    @property
    def value(self) -> float:
        return float(np.median(self._samples)) if self.measured else self.seed


# ================================================================ 检测器
class LaneFinalDetector:
    """脊搜索车道线检测（离线/在线共用；见模块 docstring 的链路与证据）。"""

    def __init__(self, target_x: float = 320.0, fps: Optional[float] = None,
                 roi_top: int = ROI_TOP_PX, roi_bottom: int = ROI_BOTTOM_PX,
                 look_top: int = LOOK_TOP_PX, look_bottom: int = LOOK_BOTTOM_PX,
                 y_ref: int = Y_REF_PX, k_min: float = K_MIN, k_max: float = K_MAX,
                 ridge_min: float = RIDGE_MIN, min_samples: int = MIN_SAMPLES,
                 pair_w_min: float = PAIR_W_MIN, pair_w_max: float = PAIR_W_MAX,
                 edge_tol: float = EDGE_TOL_PX, half_w_seed: float = HALF_W_SEED,
                 jump_reject_px: float = JUMP_REJECT_PX,
                 jump_coherent_px: float = JUMP_COHERENT_PX) -> None:
        self.target_x = float(target_x)
        self.y_top, self.y_bot = int(roi_top), int(roi_bottom)
        self.look_top, self.look_bot = int(look_top), int(look_bottom)
        self.y_ref = int(y_ref)
        self.k_min, self.k_max = float(k_min), float(k_max)
        self.ridge_min = float(ridge_min)
        self.min_samples = int(min_samples)
        self.pair_w_min, self.pair_w_max = float(pair_w_min), float(pair_w_max)
        self.edge_tol = float(edge_tol)
        # 半宽窗口按时长语义折算（30fps → 30 帧）
        scale = (float(fps) / REF_FPS) if fps else 1.0
        self.prior = HalfWidthPrior(half_w_seed, window=max(5, int(round(HALF_W_WINDOW * scale))))
        # 时域门限按 dt 折算：登记 15fps 的"每帧位移"→ 当前帧率下的等价位移
        ratio = (REF_FPS / float(fps)) if fps else 1.0
        self.jump_px = float(jump_reject_px) * ratio
        self.coherent_px = float(jump_coherent_px) * ratio
        # ★ 性能（2026-10-15，小车实测 41.7ms/帧 > 33ms 预算）：
        #   灰度/顶帽只算"检测带 ±14 行"（顶帽核 25 → 14 行余量足够），
        #   而不是整帧 480 行 —— 省 2.3 倍的形态学开销，读数口径完全不变。
        # 余量必须 ≥ 顶帽核半径（25//2=12）+ 1：采样行 r 的形态学窗口要落在裁剪区内，
        # 否则边界行的响应与整帧版不同（实测：余量 14 会把双侧率从 82% 打到 77%）
        self._row0 = max(0, self.y_top - 14)
        self._row1 = self.y_bot + 26
        self._prev: Optional[Tuple[float, float]] = None    # 上一次接受的 (left_x, right_x)
        # 每侧最近一次"被接受"的 x@y_ref（双侧/单侧都算）——单侧帧的时域门限用它当参照
        # （2026-10-15：单侧帧原来完全裸奔，出线后一跳 220px 也照收）
        self._side_x: dict = {}
        self.last_attached: Optional[np.ndarray] = None     # 诊断：最近一帧的脊响应图（**只含检测带**）
        self.last_seeds = 0                                  # 诊断：本帧 Hough 种子数

    # -------------------------------------------------- 脊响应
    def _ridge(self, gray: np.ndarray) -> np.ndarray:
        kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (TOPHAT_K, TOPHAT_K))
        R = np.minimum(cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kern).astype(np.float32),
                       TOPHAT_CLIP)
        pad = np.pad(R, ((0, 0), (RIDGE_OFF_PX, RIDGE_OFF_PX)), mode="edge")
        left = pad[:, 0:R.shape[1]]
        right = pad[:, 2 * RIDGE_OFF_PX:2 * RIDGE_OFF_PX + R.shape[1]]
        return R - 0.5 * (left + right)

    # -------------------------------------------------- 候选直线（脊核 → Hough 取种）
    def _score_line(self, resp: np.ndarray, k: float, x_ref: float,
                    ys: np.ndarray) -> Tuple[float, int]:
        """一条候选线的脊分：带内可见采样点的脊响应均值（越像"细亮共线结构"越高）。"""
        w = int(resp.shape[1])
        xs = x_ref + (ys - self.y_ref) / float(k)
        xi = np.rint(xs).astype(np.int32)
        inside = (xi >= -self.edge_tol) & (xi <= w - 1 + self.edge_tol)
        n_in = int(inside.sum())
        if n_in < self.min_samples:
            return 0.0, n_in
        vals = np.where(inside, resp[ys, np.clip(xi, 0, w - 1)], 0.0)
        return float(vals.sum() / max(1, n_in)), n_in

    def _seed_candidates(self, resp: np.ndarray) -> List[Tuple[float, float]]:
        """在脊核上跑 HoughLinesP，返回候选 (k, x@y_ref)（已过斜率闸门）。"""
        # ⚠️ 取种范围必须仍是 y_top~y_bot（2026-10-15 实测：放宽到 0 行会把画面顶部
        #    远端过曝区喂进 Hough，假种子分数压过真线 → 双侧率 82%→75%、jump-reject 翻倍）
        a = max(0, self.y_top - self._row0)
        b = max(a + 1, self.y_bot - self._row0)
        band = np.ascontiguousarray(resp[a:b])
        mask = ((band > RIDGE_MASK_THR).astype(np.uint8)) * 255
        segs = cv2.HoughLinesP(mask, 1, math.radians(HOUGH_THETA_DEG), HOUGH_THR,
                               minLineLength=HOUGH_MIN_LEN, maxLineGap=HOUGH_MAX_GAP)
        out: List[Tuple[float, float]] = []
        if segs is None:
            return out
        for x1, y1, x2, y2 in np.asarray(segs).reshape(-1, 4):
            dx, dy = float(x2 - x1), float(y2 - y1)
            if abs(dx) < 1e-6:
                continue
            k = dy / dx
            if not (self.k_min <= abs(k) <= self.k_max):
                continue
            # 坐标换算：resp 的行号是"带内局部行"（局部 = 绝对 − _row0）；Hough 切片的
            # 起点是 y_top−_row0 → Hough 返回的行号 + y_top 才是整帧行号（两处必须配套，改一个就废）
            y1f = float(y1) + self.y_top
            x_ref = float(x1) + (self.y_ref - y1f) / k
            out.append((float(k), x_ref))
        return out

    def _search_side(self, resp: np.ndarray,
                     seeds: Sequence[Tuple[float, float]], side: str) -> Optional[Tuple[float, float, float, int]]:
        """该侧最强线：Hough 种子里脊分最高者 → 局部精修（k/x 小步爬山）。返回 (score,k,x_ref,n)。"""
        ys = np.arange(self.y_top, self.y_bot, ROW_STEP) - self._row0   # 带内局部行号
        x_lo, x_hi = XREF_LEFT if side == "left" else XREF_RIGHT
        want_pos = side == "right"
        best = (0.0, None, None, 0)
        for k, x_ref in seeds:
            if (k > 0) != want_pos or not (x_lo <= x_ref <= x_hi):
                continue
            sc, n_in = self._score_line(resp, k, x_ref, ys)
            if sc > best[0]:
                best = (sc, k, x_ref, n_in)
        if best[1] is None:
            return None
        # 局部精修：围绕种子（k ±0.06 / x ±8）爬山，抹掉 Hough 的角度量化与片段噪声
        sc, k0, x0, _ = best
        for dx in np.arange(-X_COARSE_STEP, X_COARSE_STEP + 1e-9, X_FINE_STEP):
            for dk in np.arange(-0.06, 0.061, K_FINE_STEP):
                kk, xx = k0 + dk, x0 + dx
                if not (self.k_min <= abs(kk) <= self.k_max) or not (x_lo <= xx <= x_hi):
                    continue
                s2, n2 = self._score_line(resp, kk, xx, ys)
                if s2 > sc:
                    sc, k0, x0 = s2, kk, xx
        if sc < self.ridge_min:
            return None
        _, n_in = self._score_line(resp, k0, x0, ys)
        if n_in < self.min_samples:
            return None
        return float(sc), float(k0), float(x0), int(n_in)

    # -------------------------------------------------- 一帧读数
    def detect(self, frame_bgr: np.ndarray) -> Reading:
        # 只转/只算检测带那几行（见 __init__ 的 _row0/_row1 说明）
        band = frame_bgr[self._row0:min(self._row1, frame_bgr.shape[0])]
        gray = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY)
        resp = self._ridge(gray)
        self.last_attached = resp
        seeds = self._seed_candidates(resp)
        self.last_seeds = len(seeds)
        left = self._search_side(resp, seeds, "left")
        right = self._search_side(resp, seeds, "right")
        r = Reading()
        if left is not None:
            r.score_l, r.left_k, r.left_x, r.n_l = left
        if right is not None:
            r.score_r, r.right_k, r.right_x, r.n_r = right

        if left is None and right is None:
            r.note = "两侧都没搜到线（脊分/可见长度不达标）"
            return self._finish(r, frame_bgr.shape[1])

        # ---- 配对闸门：宽度 + 左右有序；不达标丢低分的那一侧（走单侧兜底）----
        if left is not None and right is not None:
            width = r.right_x - r.left_x
            ordered = r.left_x < r.right_x - 40.0
            if not ordered or not (self.pair_w_min <= width <= self.pair_w_max):
                if r.score_l >= r.score_r:
                    r.add_tag("pair-reject-R")
                    r.note = f"配对越界(宽{width:.0f}/有序{ordered}) → 丢右线"
                    right = None
                    r.right_k = r.right_x = None
                    r.score_r = 0.0
                    r.n_r = 0
                else:
                    r.add_tag("pair-reject-L")
                    r.note = f"配对越界(宽{width:.0f}/有序{ordered}) → 丢左线"
                    left = None
                    r.left_k = r.left_x = None
                    r.score_l = 0.0
                    r.n_l = 0

        # ---- 时域一致性（keep-prev）：只跳一侧、另一侧稳 → 丢跳的那侧 ----
        if left is not None and right is not None and self._prev is not None:
            pl, pr = self._prev
            dl, dr = abs(r.left_x - pl), abs(r.right_x - pr)
            if dl > self.jump_px and dr <= self.coherent_px:
                r.add_tag("jump-reject-L")
                r.note = f"左线帧间跳 {dl:.0f}px 而右线稳({dr:.0f}) → 丢左线"
                left = None
                r.left_k = r.left_x = None
                r.score_l = 0.0
                r.n_l = 0
            elif dr > self.jump_px and dl <= self.coherent_px:
                r.add_tag("jump-reject-R")
                r.note = f"右线帧间跳 {dr:.0f}px 而左线稳({dl:.0f}) → 丢右线"
                right = None
                r.right_k = r.right_x = None
                r.score_r = 0.0
                r.n_r = 0

        # ---- 单侧帧的时域门限（2026-10-15 新增，见模块 docstring 第 6 条）----
        # 只剩一侧时没有"另一侧稳不稳"可对照，就拿**该侧最近一次被接受的 x**当参照：
        # 一跳 200px+ 的读数直接丢（本帧变成"没读数"→ 走丢线阶梯），不试图修正。
        if (left is None) != (right is None):
            side = "L" if left is not None else "R"
            cur = r.left_x if left is not None else r.right_x
            ref = self._side_x.get(side)
            if ref is not None and abs(cur - ref) > self.jump_px:
                r.add_tag(f"1side-jump-{side}")
                r.note = (f"单侧{side}线帧间跳 {abs(cur - ref):.0f}px"
                          f"（>门限 {self.jump_px:.0f}）→ 丢本帧读数（走丢线阶梯）")
                left = right = None
                r.left_k = r.left_x = r.right_k = r.right_x = None
                r.score_l = r.score_r = 0.0
                r.n_l = r.n_r = 0
                return self._finish(r, frame_bgr.shape[1])

        if left is not None and right is not None:
            self._prev = (r.left_x, r.right_x)
            self._side_x["L"], self._side_x["R"] = r.left_x, r.right_x
            r.both_sides = True
            r.half_w = (r.right_x - r.left_x) * 0.5
            r.width_measured = True
            self.prior.update(r.half_w)
            r.quality = Q_BOTH
            r.center_x = self._center_both(r.left_k, r.left_x, r.right_k, r.right_x,
                                           frame_bgr.shape)
            r.error = r.center_x - self.target_x
            return self._finish(r, frame_bgr.shape[1])

        # ---- 单侧兜底：半宽先验（实测/种子）→ 中心；参考实现的"代画面边界"作退化档 ----
        hw = self.prior.value
        r.half_w = hw
        r.width_measured = self.prior.measured
        if left is not None:
            r.center_x = r.left_x + hw
            fill = "width" if r.width_measured else "seed"
        else:
            r.center_x = r.right_x - hw
            fill = "width" if r.width_measured else "seed"
        # 单侧读数被接受 → 记住这一侧的 x（下次单侧帧的时域门限参照）
        self._side_x["L" if left is not None else "R"] = (r.left_x if left is not None
                                                          else r.right_x)
        # 参考实现原样：缺的一侧代画面边界（仅当推算中心离谱时退化用）
        w = frame_bgr.shape[1]
        border_center = ((r.left_x if left is not None else 0.0)
                         + (r.right_x if right is not None else float(w - 1))) * 0.5
        if not (0.0 - self.edge_tol < r.center_x < w - 1 + self.edge_tol):
            r.center_x = border_center
            fill = "border"
        r.add_tag(f"1side-{fill}")
        r.quality = {"width": Q_SINGLE_MEASURED, "seed": Q_SINGLE_SEED,
                     "border": Q_BORDER}[fill]
        if not r.note:
            r.note = f"单侧兜底（{fill}，半宽 {hw:.0f}）"
        r.error = r.center_x - self.target_x
        return self._finish(r, w)

    def _center_both(self, kl: float, xl: float, kr: float, xr: float,
                     shape: Sequence[int]) -> float:
        """前瞻带内逐行中点平均（官方口径：逐行求左右交点取中点再平均）。"""
        mids: List[float] = []
        for y in range(self.look_top, self.look_bot, 2):
            xa = xl + (y - self.y_ref) / kl
            xb = xr + (y - self.y_ref) / kr
            if xa >= xb - 4.0:            # 两线交叉/贴到一起：跳过该行
                continue
            mids.append((xa + xb) * 0.5)
        if len(mids) < 4:                 # 带内几乎没有有效行：退回参考行中点
            return (xl + xr) * 0.5
        return float(np.mean(mids))

    def _finish(self, r: Reading, w: int) -> Reading:
        if r.center_x is not None:
            r.center_x = float(min(max(r.center_x, -self.edge_tol),
                                   w - 1 + self.edge_tol))
        return r

    def reset(self) -> None:
        self.prior.reset()
        self._prev = None
        self._side_x.clear()


# ================================================================ 转向限幅 / 相位机
def apply_steer_cap(center_deg: float, angle: float, both: bool, n_l: int, n_r: int,
                    single_limit: float = SINGLE_LIMIT_DEG,
                    weak_limit: float = WEAK_LIMIT_DEG,
                    weak_min_samples: int = WEAK_MIN_SAMPLES) -> float:
    """单侧/弱对帧单独收紧舵角（2026-09-23 操场：不收紧就会被自己的读数甩出跑道）。"""
    if both:
        if min(n_l, n_r) >= weak_min_samples:
            return angle                     # 双侧且支持充足：不额外限
        lim = weak_limit
    else:
        lim = single_limit
    return float(min(max(angle, center_deg - lim), center_deg + lim))


@dataclass
class ControlOut:
    phase: str
    angle: float
    out_us: float
    err_filt: Optional[float] = None
    note: str = ""
    since_s: float = 0.0          # 距最后一次有效读数的秒数（丢线时长；进 CSV 便于复盘）


class _CameraReader(threading.Thread):
    """后台读帧线程（daemon）：主循环只"取最新一帧"，**读不到帧也能超时退出**。

    为什么必须有（2026-10-15 现场事故）：下摄 UVC 会卡死（已知故障，重启是标准恢复手段），
    此时 `cap.read()` **无限阻塞** —— 主循环既不退出、也不再写电调，车就带着最后的油门前冲，
    而 Ctrl+C 无效（信号只给主循环设标志，循环已经卡在阻塞的 read 里）。
    把 read 挪进 daemon 线程后，主循环改成"看帧的年龄"：
      · 有新帧 → 正常处理（队列只留最新一帧，不攒滞后）；
      · 超过 --read-fail-s 没新帧 → 判相机故障 → 正常走收尾（停车 + 释放摄像头）；
    线程即使永久卡在 read 里也不拦着进程退出（daemon=True）。
    """

    def __init__(self, cap, stop_event: "threading.Event") -> None:
        super().__init__(daemon=True, name="cam-reader")
        self.cap = cap
        self._stop_ev = stop_event          # 不叫 _stop：threading.Thread 内部已有 _stop()，会撞车
        self._lock = threading.Lock()
        self._frame = None
        self._t = 0.0
        self._seq = 0
        self.read_ok = 0
        self.read_fail = 0

    def run(self) -> None:
        while not self._stop_ev.is_set():
            try:
                ok, frame = self.cap.read()
            except Exception:                              # noqa: BLE001
                ok, frame = False, None
            if ok and frame is not None:
                with self._lock:
                    self._frame, self._t, self._seq = frame, time.monotonic(), self._seq + 1
                self.read_ok += 1
            else:
                self.read_fail += 1
                time.sleep(0.05)

    def latest(self):
        with self._lock:
            return self._frame, self._t, self._seq


class LaneRunner:
    """相位机 + 控制律（与硬件无关；在线/离线共用同一条代码路径）。"""

    def __init__(self, det: LaneFinalDetector, rate: FrameRate, center_deg: float,
                 sign: float, kp: float, ki: float, kd: float, limit_deg: float,
                 smooth: float, speed_us: float, start_us: float, single_speed_us: Optional[float],
                 single_hold: bool, single_limit: float, weak_limit: float,
                 min_quality: float, hold_s: float, acquire_s: float,
                 err_stop_px: float, err_stop_s: float, fps_ref: float,
                 use_filter: bool = True, filter_window: Optional[int] = None,
                 filter_outlier_px: Optional[float] = None,
                 observe_only: bool = False,
                 single_max_s: float = SINGLE_MAX_S0) -> None:
        self.det = det
        self.rate = rate
        self.center_deg = float(center_deg)
        self.speed_us, self.start_us = float(speed_us), float(start_us)
        self.single_speed_us = (float(speed_us) if single_speed_us is None
                                else float(single_speed_us))
        self.single_hold = bool(single_hold)
        self.single_limit, self.weak_limit = float(single_limit), float(weak_limit)
        self.single_max_s = float(single_max_s)     # 单侧连续超时（秒；0=关）
        self._single_since: Optional[float] = None  # 连续单侧从哪一刻开始
        self.min_quality = float(min_quality)
        self.acquire_s = float(acquire_s)
        self.pid = PidRef(kp, ki, kd, limit_deg, smooth, sign=sign,
                          center=self.center_deg, ref_fps=fps_ref)
        outlier_px = (filter_outlier_px if filter_outlier_px is not None
                      else float(settings.ERROR_FILTER_OUTLIER) * float(settings.LANE_ERROR_SCALE))
        self.err_filter = (ErrorFilter(filter_window, outlier_px) if use_filter else None)
        self.loss = LossGuard(hold_s)
        self.big_err = BigErrorGuard(err_stop_px, err_stop_s)
        self.osc = OscillationGuard()
        self.stopped = False          # 护栏/丢线超时 → 永久停（护栏）或待恢复（丢线）
        self.stop_reason = ""
        # 离线回放 = 只看不锁：护栏/超时照计数照打印，但不真停车（没有闭环，锁了就白跑）
        self.observe_only = bool(observe_only)
        self.launched_at: Optional[float] = None
        self.last_center: Optional[float] = None
        self._valid_streak = 0        # 丢线停车后的"恢复"计数
        self.guard_trips = 0
        self.frames_ok = 0
        self.frames_total = 0

    # ------------------------------------------------------------ 主步进
    def step(self, now: float, r: Reading, allowed: bool) -> ControlOut:
        self.frames_total += 1
        center = self.center_deg
        if not allowed:                       # 发车许可没给（等板/离线前段）
            self.pid.reset()
            if self.err_filter:
                self.err_filter.reset()
            return ControlOut("stop", center, NEUTRAL_US, None)
        if self.launched_at is None:
            self.launched_at = now

        valid = r.valid and r.quality >= self.min_quality
        ls = self.loss.update(now, valid)
        since_s = 0.0 if valid else (999.0 if ls.since_s == float("inf") else ls.since_s)

        if self.stopped:
            # 护栏触发 = 永久停；丢线超时 = 连续有效帧够多可恢复
            if self.stop_reason == "lost":
                self._valid_streak = self._valid_streak + 1 if valid else 0
                if self._valid_streak >= RESUME_FRAMES:
                    self.stopped = False
                    self.stop_reason = ""
                    self._valid_streak = 0
                    self._single_since = None
                    self.loss.reset()
                    self.pid.reset()
                    if self.err_filter:
                        self.err_filter.reset()
                    print("[LANE] 🔄 读数恢复 → 重新跟线")
                elif self.observe_only:
                    self.stopped = False          # 离线回放：记录事件但不锁死
                else:
                    return ControlOut("lost", center, NEUTRAL_US, None, since_s=since_s)
            else:
                return ControlOut("guard", center, NEUTRAL_US, None, since_s=since_s)

        # ---- 本帧有有效读数 ----
        if ls.phase == "fresh":
            self.frames_ok += 1
            self.last_center = r.center_x
            if r.both_sides:
                self._single_since = None            # 双侧恢复 → 单侧计时清零
            # ★ 单侧帧：默认回中位直行 + 保速（10-11 操场实测：跟推算中心会蛇行）
            if self.single_hold and not r.both_sides:
                if self._single_since is None:
                    self._single_since = now
                held_s = now - self._single_since
                # ★ 时限（2026-10-15）：连续单侧 = 车已出线/线已出画，继续保速就是直冲
                if 0.0 < self.single_max_s <= held_s:
                    self._single_since = None
                    tail = "（离线回放：只记录不停车）" if self.observe_only else ""
                    print(f"[LANE] ⚠️ 单侧持续 {held_s:.1f}s"
                          f"（>门限 {self.single_max_s:.1f}s）→ 按丢线停车"
                          f"（防'出线后回中位保速直冲'）{tail}")
                    if not self.observe_only:
                        self.stopped, self.stop_reason = True, "lost"
                        return ControlOut("lost", center, NEUTRAL_US, None)
                self.pid.reset()
                if r.error is not None:
                    self.pid.prime(r.error)       # 恢复双侧时微分从 0 起跳（防假踢腿）
                if self.err_filter:
                    self.err_filter.reset()
                return ControlOut("single", center, self.single_speed_us, None,
                                  note=f"单侧回中位直行（已 {held_s:.1f}s）")
            err = r.error if r.error is not None else 0.0
            err_filt = self.err_filter.update(err) if self.err_filter is not None else err
            angle = self.pid.step(err_filt, self.rate.dt)
            angle = apply_steer_cap(center, angle, r.both_sides, r.n_l, r.n_r,
                                    self.single_limit, self.weak_limit)
            out_us = adaptive_pulse(err, self.speed_us)
            if not r.both_sides:
                out_us = floor_pulse(self.speed_us)
            phase = "track" if r.both_sides else "single-follow"
            if self._guard_check(now, angle, err_filt, err):
                # ★ 护栏判定"失控"的这一帧就回中位 + 电调归零，不再多跑一帧
                return ControlOut("guard", center, NEUTRAL_US, err_filt, note="护栏触发", since_s=since_s)
            return ControlOut(phase, angle, out_us, err_filt, since_s=since_s)

        # ---- 丢线：维持窗口内沿用上次中心，误差按时间衰减回中位 ----
        if ls.phase == "hold":
            if ls.since_s == float("inf"):        # 发车后还没见过一次线 → 低速探路
                self.pid.reset()
                if self.err_filter:
                    self.err_filter.reset()
                if now - (self.launched_at or now) < self.acquire_s:
                    return ControlOut("acquire", center, self.start_us, None, since_s=since_s)
                self.guard_trips += 1
                if self.observe_only:
                    return ControlOut("acquire", center, self.start_us, None, since_s=since_s)
                self.stopped, self.stop_reason = True, "lost"
                print(f"[LANE] ⚠️ 发车后 {self.acquire_s:.1f}s 没找到线 → 停车"
                      f"（看离线叠加图/--calibrate）")
                return ControlOut("lost", center, NEUTRAL_US, None, since_s=since_s)
            held = (self.last_center - self.det.target_x) if self.last_center is not None else 0.0
            held *= getattr(ls, "decay", 0.0)
            angle = (self.pid.step(held, self.rate.dt) if abs(held) > 1e-6 else center)
            return ControlOut("hold", angle, floor_pulse(self.speed_us), None, since_s=since_s)

        # ---- 超过维持窗口 ----
        self.pid.reset()
        if self.err_filter:
            self.err_filter.reset()
        if self.observe_only:
            return ControlOut("lost", center, NEUTRAL_US, None)
        self.stopped, self.stop_reason = True, "lost"
        print(f"[LANE] ⚠️ 丢线 {ls.since_s:.1f}s 未恢复 → 停车（不带垃圾读数继续跑）")
        return ControlOut("lost", center, NEUTRAL_US, None, since_s=since_s)

    def _guard_check(self, now: float, angle: float, err_filt: Optional[float],
                     err_raw: Optional[float]) -> bool:
        """返回 True = 本帧触发护栏（调用方应立刻回中位停车）。"""
        if self.osc.update(now, angle, self.center_deg):
            self.guard_trips += 1
            if not self.observe_only:
                self.stopped, self.stop_reason = True, "osc"
            print("[LANE] ❌ 检测到画龙（舵角 2s 内大幅来回多次）→ 停车")
            return True
        if self.big_err.update(now, err_filt, err_raw):
            self.guard_trips += 1
            if not self.observe_only:
                self.stopped, self.stop_reason = True, "big-err"
            print(f"[LANE] ❌ 跑偏保护触发（误差持续/高占比 ≥{self.big_err.err_px:.0f}px）→ 停车")
            return True
        return False


# ================================================================ 叠加图 / 记录
def annotate(frame: np.ndarray, r: Optional[Reading], det: LaneFinalDetector,
             phase: str = "", extra: str = "") -> np.ndarray:
    vis = frame.copy()
    h, w = vis.shape[:2]
    for y, color in ((det.y_top, (255, 128, 0)), (det.y_bot, (255, 128, 0)),
                     (det.look_top, (0, 200, 255)), (det.look_bot, (0, 200, 255))):
        cv2.line(vis, (0, int(y)), (w, int(y)), color, 1)
    if r is not None:
        for k, x_ref, color in ((r.left_k, r.left_x, (255, 80, 80)),
                                (r.right_k, r.right_x, (80, 80, 255))):
            if k is None or x_ref is None:
                continue
            for y0, y1 in ((det.y_top, det.y_bot),):
                xa = x_ref + (y0 - det.y_ref) / k
                xb = x_ref + (y1 - det.y_ref) / k
                cv2.line(vis, (int(xa), int(y0)), (int(xb), int(y1)), color, 2)
        cv2.line(vis, (int(det.target_x), 0), (int(det.target_x), h), (0, 200, 0), 1)
        if r.center_x is not None:
            ym = int((det.look_top + det.look_bot) * 0.5)
            cv2.circle(vis, (int(r.center_x), ym), 6, (0, 255, 255), 2)
        kind = "both" if r.both_sides else ("lost" if not r.valid else "single")
        err = f"{r.error:+.0f}" if r.error is not None else "—"
        txt = (f"{phase} {kind} L={r.n_l}({r.score_l:.0f})@{_fmt(r.left_x) or '—'}/k{_fmt(r.left_k,2)} "
               f"R={r.n_r}({r.score_r:.0f})@{_fmt(r.right_x) or '—'}/k{_fmt(r.right_k,2)} "
               f"c={('—' if r.center_x is None else f'{r.center_x:.0f}')} err={err} "
               f"q={r.quality:.2f} {r.tag} {r.note} {extra}")
    else:
        txt = f"{phase} (no reading) {extra}"
    cv2.putText(vis, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(vis, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 255, 60), 1, cv2.LINE_AA)
    return vis


CSV_HEADER = ("t,phase,center_x,error_px,error_filt,target_px,angle_deg,esc_us,"
              "left_x,right_x,left_k,right_k,score_l,score_r,n_l,n_r,quality,both_sides,"
              "half_w,since_valid_s,tag,note,fps\n")


def _fmt(v: Optional[float], nd: int = 1) -> str:
    return "" if v is None else f"{v:.{nd}f}"


class Recorder:
    """会话落盘：CSV + 定时叠加图（ann_*.jpg，可被 diag/ann_live.py 实时推流看）。"""

    def __init__(self, csv_path: str, save_dir: str, every_s: float, max_imgs: int) -> None:
        self.csv_path = csv_path
        self.save_dir = save_dir
        self.every_s, self.max_imgs = float(every_s), int(max_imgs)
        self.n_saved = 0
        self.n_rows = 0
        self._last_save = -1e9
        for d in ({os.path.dirname(csv_path), save_dir} - {""}):
            if d:
                os.makedirs(d, exist_ok=True)
        self._fh = open(csv_path, "w", encoding="utf-8", newline="")
        self._fh.write(CSV_HEADER)

    def wants_save(self, t: float) -> bool:
        return (self.every_s > 0 and (t - self._last_save) >= self.every_s
                and self.n_saved < self.max_imgs)

    def row(self, t: float, phase: str, r: Optional[Reading], err_filt: Optional[float],
            target_x: float, angle: float, out_us: float, fps: float,
            since_s: float = 0.0) -> None:
        r = r or Reading()
        self._fh.write(
            f"{t:.3f},{phase},{_fmt(r.center_x)},{_fmt(r.error)},{_fmt(err_filt)},"
            f"{target_x:.1f},{angle:.1f},{out_us:.0f},{_fmt(r.left_x)},{_fmt(r.right_x)},"
            f"{_fmt(r.left_k, 3)},{_fmt(r.right_k, 3)},{r.score_l:.1f},{r.score_r:.1f},"
            f"{r.n_l},{r.n_r},{r.quality:.2f},{int(r.both_sides)},{_fmt(r.half_w)},"
            f"{since_s:.2f},\"{r.tag}\",\"{r.note}\",{fps:.1f}\n")
        self.n_rows += 1
        # ⚠️ 不每帧 flush：SD 卡上每帧一次落盘是 ms 级开销（2026-10-15 实测把环路从 ~25fps
        #    拖到 ~15fps）；每 30 行（≈1s）落一次，崩溃最多丢 1 秒，收尾 close() 也会落
        if self.n_rows % 30 == 0:
            self._fh.flush()

    def maybe_save(self, t: float, frame: np.ndarray, vis: np.ndarray) -> None:
        if not self.wants_save(t):
            return
        self._last_save = t
        idx = self.n_saved
        self.n_saved += 1
        if self.save_dir:
            cv2.imwrite(os.path.join(self.save_dir, f"ann_{idx:06d}.jpg"), vis,
                        [cv2.IMWRITE_JPEG_QUALITY, 90])
            cv2.imwrite(os.path.join(self.save_dir, f"raw_{idx:06d}.jpg"), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 88])

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


# ================================================================ 离线模式（不碰硬件）
def imread_any(path: str) -> Optional[np.ndarray]:
    img = cv2.imread(path)
    if img is None:                       # 中文路径兜底（cv2.imread 在中文路径下返回 None）
        try:
            img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            img = None
    return img


def list_frames(directory: str) -> List[str]:
    for pat in ("frame_*.jpg", "frame_*.png", "*.jpg", "*.png"):
        found = sorted(glob.glob(os.path.join(directory, pat)))
        if found:
            return found
    return []


def _pick_indices(n: int, args) -> List[int]:
    idx = list(range(n))
    if args.start:
        idx = [i for i in idx if i >= args.start]
    if args.end >= 0:
        idx = [i for i in idx if i <= args.end]
    if args.stride > 1:
        idx = idx[::args.stride]
    if args.limit:
        idx = idx[:args.limit]
    return idx


def frame_source(args) -> Tuple[str, List[Tuple[int, np.ndarray]], float]:
    """把 --frames/--video/--image 读成 [(帧号, 图)] 与素材 fps（用于离线 dt）。

    ★ 统一走 `crop_frame()`（裁掉画面底部的车盖，见 settings.FRAME_CROP_BOTTOM_PX）：
      在线/离线同源，检测档坐标系不变（只切底部，y 原点不动）。
    """
    if args.image:
        img = imread_any(args.image)
        if img is None:
            raise SystemExit(f"[LANE] 读不到图：{args.image}")
        return os.path.basename(args.image), [(0, crop_frame(img))], float(args.fps or REF_FPS)
    if args.frames:
        paths = list_frames(args.frames)
        if not paths:
            raise SystemExit(f"[LANE] 目录里没有帧图：{args.frames}")
        fps = float(args.fps or 0.0)
        if fps <= 0.0:                    # 素材真实帧率：优先读 meta.json（重复帧折算过）
            meta = os.path.join(args.frames, "meta.json")
            try:
                import json
                with open(meta, "r", encoding="utf-8") as fh:
                    fps = float(json.load(fh).get("effective_fps") or 0.0)
            except Exception:
                fps = 0.0
        if fps <= 0.0:
            fps = 30.0
            print("[LANE] ⚠️ 没读到 meta.json 的 effective_fps，离线口径按 30fps（--fps 可指定）")
        out = []
        for i in _pick_indices(len(paths), args):
            img = imread_any(paths[i])
            if img is None:
                print(f"[LANE] ⚠️ 读不到 {paths[i]}（跳过）")
                continue
            out.append((i, crop_frame(img)))
        return os.path.basename(os.path.normpath(args.frames)), out, fps
    if args.video:
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            raise SystemExit(f"[LANE] 打不开视频：{args.video}")
        fps = float(args.fps or cap.get(cv2.CAP_PROP_FPS) or 30.0)
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 10 ** 9
        want = set(_pick_indices(total, args))
        out, i = [], 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i in want:
                out.append((i, crop_frame(frame)))
            i += 1
        cap.release()
        return os.path.basename(args.video), out, fps
    raise SystemExit("[LANE] 要给 --image / --frames / --video（--help 看用法）")


def summarize(rows: List[Reading], centers: List[Optional[float]],
              phases: List[str], scores_dt: List[float]) -> List[str]:
    n = len(rows)
    both = sum(1 for r in rows if r.both_sides)
    single = sum(1 for r in rows if r.valid and not r.both_sides)
    lost = sum(1 for r in rows if not r.valid)
    jr = sum(1 for r in rows if "jump-reject" in r.tag)
    sj = sum(1 for r in rows if "1side-jump" in r.tag)
    pr = sum(1 for r in rows if "pair-reject" in r.tag)
    widths = [r.half_w * 2 for r in rows if r.both_sides and r.half_w]
    ks_l = [r.left_k for r in rows if r.left_k is not None]
    ks_r = [r.right_k for r in rows if r.right_k is not None]
    cs = np.asarray([c for c in centers if c is not None], dtype=float)
    dif = np.abs(np.diff(np.asarray([c for c in centers if c is not None], dtype=float))) \
        if len(cs) >= 2 else np.asarray([0.0])
    runs, s = [], None
    for i, r in enumerate(rows):
        bad = not r.valid
        if bad and s is None:
            s = i
        if not bad and s is not None:
            runs.append(i - s)
            s = None
    if s is not None:
        runs.append(n - s)

    def pct(a, q):
        return float(np.percentile(a, q)) if len(a) else float("nan")

    lines = [
        f"[LANE] {n} 帧 | 双侧 {both}（{both / max(1, n) * 100:.0f}%）| 单侧 {single} | "
        f"丢失 {lost}（{lost / max(1, n) * 100:.0f}%）| 最长连丢 {max(runs) if runs else 0} 帧",
        f"[LANE] 时域拒绝：jump-reject {jr} 帧 / 单侧跳拒绝 {sj} 帧 / pair-reject {pr} 帧",
        f"[LANE] center 中位 {pct(cs, 50):+.1f}  p05~p95 {pct(cs, 5):+.0f}~{pct(cs, 95):+.0f} "
        f"| 帧间|Δcenter| p50 {pct(dif, 50):.1f} p90 {pct(dif, 90):.1f} | "
        f"k 中位 左 {pct(np.asarray(ks_l), 50):+.2f} / 右 {pct(np.asarray(ks_r), 50):+.2f}",
        f"[LANE] 配对宽 p05~p95 {pct(np.asarray(widths), 5):.0f}~{pct(np.asarray(widths), 95):.0f} "
        f"（中位 {pct(np.asarray(widths), 50):.0f}）",
        "[LANE] 相位分布：" + " ".join(f"{p}:{phases.count(p)}" for p in sorted(set(phases))),
    ]
    if scores_dt:
        lines.append(f"[LANE] 单帧检测耗时：p50 {np.percentile(scores_dt, 50) * 1000:.1f}ms "
                     f"p90 {np.percentile(scores_dt, 90) * 1000:.1f}ms "
                     f"（30fps 预算 33ms）")
    return lines


def run_offline(args, det: LaneFinalDetector, center_deg: float, sign: float,
                runner: LaneRunner) -> int:
    tag, items, fps = frame_source(args)
    if not items:
        print("[LANE] 没有可处理的帧")
        return 1
    print(f"[LANE] {tag}: {len(items)} 帧 | 离线口径 fps={fps:.1f}"
          f"（meta.effective_fps；--fps 可覆盖）")
    print(f"[LANE] 检测档：ROI y{det.y_top}~{det.y_bot} | 前瞻带 y{det.look_top}~{det.look_bot} "
          f"| y_ref={det.y_ref} | |k| {det.k_min}~{det.k_max} | 脊下限 {det.ridge_min} "
          f"| 配对宽 {det.pair_w_min:.0f}~{det.pair_w_max:.0f} | target={det.target_x:.0f}")
    if args.calibrate:
        return _calibrate_report(items, det, runner)

    out_dir = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "..", "..", "train", "opencv-level1", "lane_test",
                                       "1014_lane_final", "run")
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    rec = Recorder(os.path.join(out_dir, "run.csv"), os.path.join(out_dir, "frames"),
                   args.save_every_s if args.save_every_s > 0 else 0.0, args.save_max)
    montage_every = args.montage
    thumbs: List[np.ndarray] = []
    sheet_no = 0
    rows: List[Reading] = []
    centers: List[Optional[float]] = []
    phases: List[str] = []
    det_ms: List[float] = []
    for k, (idx, frame) in enumerate(items):
        t_frame = time.perf_counter()
        r = det.detect(frame)
        det_ms.append(time.perf_counter() - t_frame)
        now = k / fps                       # 离线时间轴（秒）
        co = runner.step(now, r, allowed=True)
        rows.append(r)
        centers.append(r.center_x)
        phases.append(co.phase)
        rec.row(now, co.phase, r, co.err_filt, det.target_x, co.angle, co.out_us, fps,
                since_s=co.since_s)
        if args.print_every and k % args.print_every == 0:
            print(f"[LANE] f{idx:06d} {co.phase:12s} L={r.n_l:2d}({r.score_l:4.1f}) "
                  f"R={r.n_r:2d}({r.score_r:4.1f}) c={_fmt(r.center_x) or '—':>5} "
                  f"err={_fmt(r.error) or '—':>6} ang={co.angle:.1f} us={co.out_us:.0f} {r.tag}")
        if args.every and idx % args.every == 0:
            cv2.imwrite(os.path.join(out_dir, f"f_{idx:06d}.jpg"),
                        annotate(frame, r, det, co.phase), [cv2.IMWRITE_JPEG_QUALITY, 90])
        if montage_every and idx % montage_every == 0:
            thumbs.append(cv2.resize(annotate(frame, r, det, co.phase), (320, 240),
                                     interpolation=cv2.INTER_AREA))
            if len(thumbs) == 20:
                _save_sheet(out_dir, sheet_no, thumbs)
                sheet_no += 1
                thumbs = []
    if thumbs:
        _save_sheet(out_dir, sheet_no, thumbs)

    lines = summarize(rows, centers, phases, det_ms)
    if runner.guard_trips:
        lines.append(f"[LANE] 护栏/超时触发：{runner.guard_trips} 次"
                     f"（画龙/跑偏/探路超时；离线复跑用于回归）")
    lines.append(f"[LANE] 输出 → {out_dir}（run.csv / f_*.jpg / sheet_*.jpg）")
    for ln in lines:
        print(ln)
    rec.close()
    with open(os.path.join(out_dir, "summary.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    return 0


def _save_sheet(out_dir: str, sheet_no: int, thumbs: Sequence[np.ndarray]) -> None:
    sheet = np.zeros((4 * 240, 5 * 320, 3), dtype=np.uint8)
    for i, th in enumerate(thumbs[:20]):
        rr, cc = divmod(i, 5)
        sheet[rr * 240:(rr + 1) * 240, cc * 320:(cc + 1) * 320] = th
    cv2.imwrite(os.path.join(out_dir, f"sheet_{sheet_no:03d}.jpg"), sheet,
                [cv2.IMWRITE_JPEG_QUALITY, 88])


def _calibrate_report(items: List[Tuple[int, np.ndarray]], det: LaneFinalDetector,
                      runner: LaneRunner) -> int:
    """标定档：打印本素材/本机位下的检测统计（换镜头/换机位后重跑它）。"""
    rows, centers, phases = [], [], []
    for k, (idx, frame) in enumerate(items):
        r = det.detect(frame)
        co = runner.step(k / 30.0, r, allowed=True)
        rows.append(r)
        centers.append(r.center_x)
        phases.append(co.phase)
    for ln in summarize(rows, centers, phases, []):
        print(ln)
    print("[LANE] —— 标定建议 ——")
    print(f"[LANE] target_x 候选（双侧帧 center 中位）："
          f"{np.nanmedian([c for c, r in zip(centers, rows) if r.both_sides and c is not None]):.0f}"
          f"（**车必须摆正在车道里**；权威值以现场静止自检为准）")
    return 0


# ================================================================ 车上静止标定（--calibrate）
def run_online_calibrate(args, det: LaneFinalDetector) -> int:
    """车**静止摆正在车道里**，采几秒帧出统计 → target_x 候选；默认写回 site.yaml。

    这一步是现场唯一必须做的事：检测档参数是"这台车+这个机位"标定的常量，只有 target_x
    取决于"车摆在哪"（标定时车必须在车道中间、车头对着行进方向）。
    """
    rows: List[Reading] = []
    with camera_exclusive(services=("ffmpeg-stream-sub.service",)):
        cap = open_camera(args.cam, settings.IMG_W, settings.IMG_H, fourcc="MJPG")
        if cap is None:
            print("[LANE] 摄像头打不开（推流是否占着？看 car-mode.sh status）")
            return 1
        t0 = time.monotonic()
        try:
            while (time.monotonic() - t0) < args.calibrate_s:
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.02)
                    continue
                rows.append(det.detect(crop_frame(frame)))
        finally:
            cap.release()
    if not rows:
        print("[LANE] 一帧都没读到")
        return 1
    both = [r for r in rows if r.both_sides and r.center_x is not None]
    cs = np.asarray([r.center_x for r in both], dtype=float)
    ws = np.asarray([r.half_w * 2 for r in both if r.half_w], dtype=float)
    kl = np.asarray([r.left_k for r in both if r.left_k is not None], dtype=float)
    kr = np.asarray([r.right_k for r in both if r.right_k is not None], dtype=float)
    print(f"[LANE] 静止标定：{len(rows)} 帧，双侧 {len(both)}（{len(both) / len(rows) * 100:.0f}%）"
          f"，单侧 {sum(1 for r in rows if r.valid and not r.both_sides)}，"
          f"丢失 {sum(1 for r in rows if not r.valid)}")
    if len(both) < 5:
        print("[LANE] ⚠️ 双侧帧太少（<5）→ 车没摆正在道中 / 镜头没对着赛道，别用这次的结果")
        print("[LANE]    先看叠加图：图片在 /root/dev/img/ 或跑 lane_final_test.py --no-motor 看读数")
        return 1
    med = float(np.median(cs))
    print(f"[LANE] center 中位 **{med:.1f}**（p05~p95 {np.percentile(cs, 5):.1f}~{np.percentile(cs, 95):.1f}；"
          f"跨距 {np.percentile(cs, 95) - np.percentile(cs, 5):.0f}px，越小越说明车没动）")
    if ws.size:
        print(f"[LANE] 配对宽中位 {np.median(ws):.0f}px（半宽 {np.median(ws) / 2:.0f}）"
              f" | k 中位 左 {np.median(kl):+.2f} / 右 {np.median(kr):+.2f}")
    print(f"[LANE] ⇒ 跑车命令直接用：--target-x {med:.0f}")
    if args.save_site:
        try:
            from config import site as _site
            path = _site.save({"target_x": round(med, 1)})
            print(f"[LANE] ✅ 已写回 {path} 的 target_x = {round(med, 1)}（A 路径与以后运行都会用它）")
        except Exception as exc:
            print(f"[LANE] ⚠️ 写 site.yaml 失败（不影响本次打印的结果）：{exc}")
    return 0


# ================================================================ 在线模式（摄像头 + 可选电机）
def run_online(args, det: LaneFinalDetector, center_deg: float, sign: float,
               runner: LaneRunner) -> int:
    from scripts.bench_common import MotorSession, install_signal_guard, restore_remote_stage

    use_motor = bool(args.allow_motion)
    state = {"stopped": False, "driver": None, "restored": False, "used_motor": use_motor}
    install_signal_guard(state)

    # ★ 电调看门狗（2026-10-15 事故修复）：**独立进程**，主控卡死/被杀时由它把电调拉回
    #   1500us。为什么必须有：PCA9685 会一直输出最后一次写进去的脉宽 —— 相机 UVC 卡死让
    #   `cap.read()` 无限阻塞、或进程被 OOM/SIGKILL 杀掉时，车会带着最后的油门前冲，
    #   而 Ctrl+C 完全无效（信号只给主循环设标志）。现场已因此丢过两次车（都要断电）。
    wd = None
    if use_motor and args.watchdog_s > 0:
        try:
            from scripts.esc_watchdog import WatchdogClient
            wd = WatchdogClient(timeout_s=args.watchdog_s)
            wd.start()
        except Exception as exc:                       # noqa: BLE001
            wd = None
            print(f"[LANE] ⚠️ 电调看门狗没起来（{exc}）——跑动期间请人守着、手边留断电开关！")

    gate = None if args.no_start_gate else StartGate()
    rec = Recorder(args.log_csv or os.path.join(args.save_dir, "run.csv"), args.save_dir,
                   args.save_every_s, args.save_max)
    if args.save_every_s > 0:
        print(f"[LANE] 存图：每 {args.save_every_s:.1f}s 一张（最多 {args.save_max} 张 → "
              f"覆盖 {args.save_every_s * args.save_max:.0f}s；不够就调 --save-max）")
    else:
        print("[LANE] 存图：关（--save-every-s 0）")

    # ★ 会话元数据（2026-10-15）：把"这趟用了什么参数 + 为什么停"落进会话目录的 session.json。
    #   为什么：CSV 只记 phase=guard，不记是画龙还是跑偏；stdout 一旦丢（进程被杀/没接 tee）
    #   就查不到原因（现场已遇到）。session.json 落盘即证据，不依赖任何管道。
    sess = {
        "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        "pid": os.getpid(),
        "params": {
            "target_x": det.target_x, "kp": runner.pid.kp, "ki": runner.pid.ki,
            "kd": runner.pid.kd, "limit_deg": runner.pid.limit_deg,
            "smooth": runner.pid.smooth, "speed_us": runner.speed_us,
            "start_us": runner.start_us, "single_max_s": runner.single_max_s,
            "single_hold": runner.single_hold, "hold_s": runner.loss.hold_s,
            "acquire_s": runner.acquire_s, "min_quality": runner.min_quality,
            "watchdog_s": args.watchdog_s, "watchdog": wd is not None, "cam": args.cam,
            "max_seconds": args.max_seconds, "save_every_s": args.save_every_s,
            "save_max": args.save_max, "allow_motion": use_motor,
        },
        "result": None,
    }
    phase_counts: dict = {}

    def _dump_sess() -> None:
        try:
            import json
            with open(os.path.join(args.save_dir, "session.json"), "w", encoding="utf-8") as fh:
                json.dump(sess, fh, ensure_ascii=False, indent=1)
        except Exception:                                  # noqa: BLE001
            pass

    _dump_sess()

    print("[LANE] 最终循迹（自检 → 蓝板边沿发车 → 跟线 → 丢线阶梯 → 落盘）")
    print(f"[LANE] 舵机中位={center_deg:.1f}° 转向符号={sign:+.0f} "
          f"PID kp={runner.pid.kp:.3f} ki={runner.pid.ki:.3f} kd={runner.pid.kd:.3f} "
          f"（15fps 登记口径）限幅±{runner.pid.limit_deg:.0f}° 平滑={runner.pid.smooth}")
    print(f"[LANE] 脉宽：起步 {runner.start_us:.0f} / 循迹 {runner.speed_us:.0f}us"
          f"（大误差 {floor_pulse(runner.speed_us):.0f}；死区 {settings.ESC_DEADBAND_US}）"
          f"；单侧帧 {'回中位直行' if runner.single_hold else '跟推算中心'}"
          f"（时限 {runner.single_max_s:.1f}s 后按丢线停）")
    print(f"[LANE] 丢线阶梯：维持 {runner.loss.hold_s:.1f}s（误差衰减回中位）→ 停车；"
          f"发车后 {runner.acquire_s:.1f}s 没见线按探路低速")
    print(f"[LANE] 落盘：{rec.csv_path}（每 {args.save_every_s:.1f}s 存一帧到 {args.save_dir}）")
    _crop = int(getattr(settings, "FRAME_CROP_LINE_Y", 0) or 0)
    print(f"[LANE] 画面裁剪：保留 y<{_crop}（车盖不入画；闸门/检测/存图同源）"
          if _crop else "[LANE] 画面裁剪：关闭（settings.FRAME_CROP_LINE_Y=0）")
    print("[LANE] ⚠️ 帧率口径：相机请求 640x480@30（亮环境 30fps / 暗环境 AE 长曝光掉到 9~18）；"
          "日志与 CSV 里的 fps 是**本脚本主循环的消费速率**（受 CPU 限制，通常 15~30），"
          "控制律按实测 dt 折算，两者都不影响手感")
    if use_motor:
        print("[LANE] ⚠️ 用法：车摆正在车道里 → 启动 → 看到「自检完成」→ 蓝板挡到下摄正前方"
              "（先看到→再移开=发车）→ 想停就把板挡回去 / Ctrl+C")
    else:
        print("[LANE] dry-run（--no-motor）：只看读数、不动电机；加 --allow-motion 才动车")

    rc = 0
    frames = 0
    last_err = last_ang = last_us = None
    wd_tripped = False
    try:
        with MotorSession(state, enabled=use_motor,
                          speed_us_max=max(runner.start_us, runner.speed_us)) as pca:
            # 只停副摄（巡线用）那一路推流，主摄/网页图传保持在线（10-11 约定）
            with camera_exclusive(services=("ffmpeg-stream-sub.service",)):
                cap = open_camera(args.cam, settings.IMG_W, settings.IMG_H, fourcc="MJPG")
                if cap is None:
                    rc = 1
                else:
                    # ★ 时间基准必须与主循环的 now 同源（time.monotonic）——
                    #   2026-10-14 首版混用 time.time()，t_rel 变成 -1.7e9，
                    #   `t_rel >= --max-seconds` 永远不成立（端到端测试里挂死才发现）。
                    t0 = time.monotonic()
                    last_vis: Optional[np.ndarray] = None
                    allowed = bool(args.no_start_gate)
                    # ★ 读帧挪进 daemon 线程（2026-10-15 事故）：主循环只取最新帧 + 看"帧年龄"，
                    #   这样相机 UVC 卡死时能判死退出，而不是永久阻塞在 cap.read() 里。
                    stop_ev = threading.Event()
                    reader = _CameraReader(cap, stop_ev)
                    reader.start()
                    last_seq = 0
                    while True:
                        frame, t_frame, seq = reader.latest()
                        if seq == last_seq:                 # 还没有新帧
                            now = time.monotonic()
                            if state.get("stopped"):
                                break
                            if t_frame and now - t_frame > args.read_fail_s:
                                print(f"[LANE] ❌ 摄像头 {now - t_frame:.1f}s 没出新帧"
                                      f"（UVC 卡死？）→ 安全退出（看门狗会先把电调归零）")
                                rc = 1
                                break
                            if args.max_seconds > 0 and now - t0 >= args.max_seconds:
                                print(f"[LANE] 到达 --max-seconds {args.max_seconds:.0f}s → 停车退出")
                                break
                            time.sleep(0.005)
                            continue
                        last_seq = seq
                        now = time.monotonic()
                        frame = crop_frame(frame)      # 裁掉底部车盖（闸门/检测/存图同源）
                        if wd is not None:
                            wd.feed()                  # ★ 每帧喂看门狗（/dev/shm utime，微秒级）
                        fps = runner.rate.update_now(now)
                        if runner.err_filter is not None:
                            runner.err_filter.set_fps(fps)
                        if gate is not None and not allowed:
                            st = gate.update(frame, now)
                            allowed = st.released
                        r = det.detect(frame)
                        co = runner.step(now, r, allowed)
                        if use_motor and pca is not None:
                            pca.set_steering_angle(co.angle)
                            pca.write_us(pca.CH_ESC, co.out_us)
                        t_rel = now - t0
                        rec.row(t_rel, co.phase, r, co.err_filt, det.target_x, co.angle,
                                co.out_us, fps, since_s=co.since_s)
                        # ★ 叠加图只在该存图时才画（2026-10-15：每帧全帧绘制在 Pi 上是 ~10ms 级，
                        #   把主循环从 ~25fps 拖到 ~15fps；存图频率本来就是 1s 一张）
                        if rec.wants_save(t_rel):
                            vis = annotate(frame, r, det, co.phase, extra=f"ang={co.angle:.1f} esc={co.out_us:.0f}")
                            last_vis = vis
                            rec.maybe_save(t_rel, frame, vis)
                        frames += 1
                        phase_counts[co.phase] = phase_counts.get(co.phase, 0) + 1
                        last_err, last_ang, last_us = r.error, co.angle, co.out_us
                        if frames % max(1, args.print_every) == 0:
                            print(f"[LANE] t={t_rel:5.1f}s fps={fps:4.1f} {co.phase:12s} "
                                  f"c={_fmt(r.center_x) or '—':>5} err={_fmt(r.error) or '—':>6} "
                                  f"ang={co.angle:.1f} us={co.out_us:.0f} q={r.quality:.2f} {r.tag}")
                        if state.get("stopped"):
                            break
                        if args.max_seconds > 0 and t_rel >= args.max_seconds:
                            print(f"[LANE] 到达 --max-seconds {args.max_seconds:.0f}s → 停车退出")
                            break
                    # 收尾：先请读帧线程停；它若卡在 read() 里（UVC 卡死）就别 release ——
                    # cv2 在并发 read 期间 release 是未定义行为，宁可让进程退出时由内核回收。
                    stop_ev.set()
                    reader.join(timeout=0.3)
                    if reader.is_alive():
                        print("[LANE] ⚠️ 读帧线程卡在 cap.read()（相机没响应）：跳过 release，"
                              "由进程退出回收（副摄推流要等本进程结束才能拿回设备）")
                    else:
                        cap.release()
                    if last_vis is not None and args.save_dir:
                        cv2.imwrite(os.path.join(args.save_dir, "last.jpg"), last_vis)
    except KeyboardInterrupt:
        print("[LANE] Ctrl+C → 安全停车")
    finally:
        rec.close()
        if use_motor:
            restore_remote_stage(state, "run_online 退出")
        if wd is not None:
            wd_code = wd.stop()                        # 电机已安全 → 再解除看门狗
            wd_tripped = wd_code in (2, 3)              # 2/3 = 心跳停过并写了安全值
        # 结果落进 session.json（含着陆原因、相位分布、末帧读数）——复盘不依赖 stdout
        sess["result"] = {
            "ended": time.strftime("%Y-%m-%d %H:%M:%S"), "rc": rc, "frames": frames,
            "frames_ok": runner.frames_ok, "guard_trips": runner.guard_trips,
            "stop_reason": runner.stop_reason or "—", "fps_end": round(runner.rate.fps, 1),
            "phases": dict(sorted(phase_counts.items())),
            "last": {"center_x": runner.last_center, "error_px": last_err,
                     "angle_deg": last_ang, "esc_us": last_us},
            "watchdog_tripped": wd_tripped,
        }
        _dump_sess()
    fps_end = runner.rate.fps
    print(f"[LANE] 结束：{frames} 帧 | 有效跟线 {runner.frames_ok} 帧 "
          f"| 护栏/超时 {runner.guard_trips} 次 | 末帧率 {fps_end:.1f}")
    print(f"[LANE] 末态：停机原因={runner.stop_reason or '—'}"
          f"（osc=画龙 / big-err=跑偏 / lost=丢线）| 相位分布={dict(sorted(phase_counts.items()))}")
    if wd_tripped:
        print("[LANE] ⚠️ 这一趟看门狗触发过（主控卡死/被杀）——务必看它打印的 🔛 那几行！")
    print(f"[LANE] 回实验室看：{rec.csv_path} + session.json"
          + (f" + {args.save_dir}/ann_*.jpg" if args.save_every_s > 0 else ""))
    return rc


# ================================================================ 命令行
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="最终循迹脚本（脊搜索检测 + PD 控制；离线/在线同一套）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    src = ap.add_argument_group("输入（离线二选一；都给 = 按 image→frames→video 顺序）")
    src.add_argument("--image", default="", help="单张图分析")
    src.add_argument("--frames", default="", help="帧目录（frame_*.jpg，按名排序）")
    src.add_argument("--video", default="", help="视频文件（离线跑；fps 取容器/--fps）")
    src.add_argument("--start", type=int, default=0, help="从第几帧开始")
    src.add_argument("--end", type=int, default=-1, help="到第几帧为止（含；-1=到底）")
    src.add_argument("--stride", type=int, default=1, help="隔几帧取一帧")
    src.add_argument("--limit", type=int, default=0, help="最多处理多少帧（0=不限）")
    src.add_argument("--fps", type=float, default=None,
                     help="钉死帧率口径（离线默认读 meta.effective_fps；在线默认实测）")
    src.add_argument("--calibrate", action="store_true",
                     help="只出标定统计：离线(--frames/--video)或**车上静止**(默认)都能用；"
                          "车上模式会把 target_x 写进 site.yaml（--no-save-site 可关）")
    src.add_argument("--calibrate-s", type=float, default=4.0,
                     help="车上静止标定采多少秒（默认 4s，够 60~120 帧）")
    src.add_argument("--no-save-site", dest="save_site", action="store_false", default=True,
                     help="车上 --calibrate 不写回 site.yaml")

    dst = ap.add_argument_group("检测档（1014 素材标定；换机位后重标）")
    dst.add_argument("--roi-top", type=int, default=ROI_TOP_PX)
    dst.add_argument("--roi-bottom", type=int, default=ROI_BOTTOM_PX)
    dst.add_argument("--look-top", type=int, default=LOOK_TOP_PX)
    dst.add_argument("--look-bottom", type=int, default=LOOK_BOTTOM_PX)
    dst.add_argument("--y-ref", type=int, default=Y_REF_PX)
    dst.add_argument("--k-min", type=float, default=K_MIN)
    dst.add_argument("--k-max", type=float, default=K_MAX)
    dst.add_argument("--ridge-min", type=float, default=RIDGE_MIN)
    dst.add_argument("--min-samples", type=int, default=MIN_SAMPLES)
    dst.add_argument("--pair-w-min", type=float, default=PAIR_W_MIN)
    dst.add_argument("--pair-w-max", type=float, default=PAIR_W_MAX)
    dst.add_argument("--edge-tol", type=float, default=EDGE_TOL_PX)
    dst.add_argument("--half-w-seed", type=float, default=HALF_W_SEED)
    dst.add_argument("--jump-reject-px", type=float, default=JUMP_REJECT_PX,
                     help="时域一致性：一侧帧间跳超过它就丢该侧（15fps 登记口径，按 dt 折算）")
    dst.add_argument("--jump-coherent-px", type=float, default=JUMP_COHERENT_PX,
                     help="且另一侧移动不超过它才算\"只跳了一侧\"")

    ctl = ap.add_argument_group("控制（登记口径 15fps；运行按实测 dt 折算）")
    ctl.add_argument("--target-x", type=float, default=None,
                     help="期望中心 x（默认取车端 site.yaml 的现场标定值，没有则 330 = 1014 素材候选；"
                          "**现场先用 --calibrate 标一次**）")
    ctl.add_argument("--speed-us", type=float, default=float(settings.ESC_CREEP_US),
                     help="循迹巡航脉宽（现场高速档用 1580+；低于死区直接拒绝）")
    ctl.add_argument("--start-us", type=float, default=float(settings.ESC_CREEP_US),
                     help="起步/探路脉宽")
    ctl.add_argument("--single-speed-us", type=float, default=None,
                     help="单侧帧脉宽（默认 = 巡航保速）")
    ctl.add_argument("--kp", type=float, default=None)
    ctl.add_argument("--ki", type=float, default=None)
    ctl.add_argument("--kd", type=float, default=None)
    ctl.add_argument("--limit-deg", type=float, default=None)
    ctl.add_argument("--smooth", type=float, default=None)
    ctl.add_argument("--single-limit-deg", type=float, default=SINGLE_LIMIT_DEG)
    ctl.add_argument("--weak-limit-deg", type=float, default=WEAK_LIMIT_DEG)
    ctl.add_argument("--single-hold", dest="single_hold", action="store_true", default=True,
                     help="单侧帧回中位直行（默认开；10-11 操场定案）")
    ctl.add_argument("--no-single-hold", dest="single_hold", action="store_false")
    ctl.add_argument("--single-max-s", type=float, default=SINGLE_MAX_S0,
                     help="单侧连续多久就按丢线停车（秒；0=关。防'出线后回中位保速直冲'）")
    ctl.add_argument("--min-quality", type=float, default=MIN_QUALITY0)
    ctl.add_argument("--hold-s", type=float, default=HOLD_S0, help="丢线维持窗口（秒）")
    ctl.add_argument("--acquire-s", type=float, default=ACQUIRE_S0,
                     help="发车后没见线的探路窗口（秒）")
    ctl.add_argument("--err-stop-px", type=float, default=ERR_STOP_PX0)
    ctl.add_argument("--err-stop-s", type=float, default=ERR_STOP_S0,
                     help="跑偏保护：大误差持续这么久就停车（0=关）")
    ctl.add_argument("--no-filter", action="store_true")
    ctl.add_argument("--filter-window", type=int, default=None)
    ctl.add_argument("--filter-outlier", type=float, default=None,
                     help="离群阈值（像素；默认 settings 值 × 误差单位）")
    ctl.add_argument("--steer-sign", type=float, default=None,
                     help=f"转向符号（默认 settings.STEER_SIGN={settings.STEER_SIGN:+.0f}）")
    ctl.add_argument("--servo-center", type=float, default=None,
                     help=f"舵机中位角（默认 settings.SERVO_CENTER_ANGLE="
                          f"{settings.SERVO_CENTER_ANGLE:.1f}）")

    hw = ap.add_argument_group("硬件/在线")
    hw.add_argument("--cam", type=int, default=2, help="摄像头 index（巡线固定 2=下摄）")
    hw.add_argument("--no-motor", action="store_true", help="只跑视觉，不动电机（默认行为）")
    hw.add_argument("--allow-motion", "--i-know-wheels-are-up", dest="allow_motion",
                    action="store_true", help="确认车可动（真正跑车必须显式给）")
    hw.add_argument("--no-start-gate", action="store_true",
                    help="跳过蓝板边沿发车（实验室/离线用；默认必须见板再消失才走）")
    hw.add_argument("--max-seconds", type=float, default=MAX_SECONDS0)
    hw.add_argument("--read-fail-s", type=float, default=8.0)
    hw.add_argument("--watchdog-s", type=float, default=1.0,
                    help="电调看门狗超时（秒；0=关。主控卡死/被杀时由独立进程强制停车）")

    out = ap.add_argument_group("输出/记录")
    out.add_argument("--out", default="", help="离线产物目录（默认 1014_lane_final/run）")
    out.add_argument("--save-dir", default="",
                     help="在线会话目录（默认 /root/dev/logs/lane_<时间戳>）")
    out.add_argument("--log-csv", default="", help="CSV 路径（默认会话目录/run.csv）")
    out.add_argument("--save-every-s", type=float, default=1.0, help="每 N 秒存一帧（0=不存）")
    out.add_argument("--save-max", type=int, default=0,
                     help="每会话最多存多少张图；0=自动（在线按 --max-seconds 算满全程，不让长趟丢尾巴；离线沿用 400）")
    out.add_argument("--every", type=int, default=0, help="离线：每 N 帧存一张叠加图")
    out.add_argument("--montage", type=int, default=0, help="离线：每 N 帧收进接触表")
    out.add_argument("--print-every", type=int, default=PRINT_EVERY0)
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    offline = bool(args.image or args.frames or args.video)
    if args.allow_motion:
        if min(args.start_us, args.speed_us) < settings.ESC_DEADBAND_US:
            print(f"[LANE] ❌ 脉宽低于电调死区（{settings.ESC_DEADBAND_US}us）："
                  f"start={args.start_us:.0f} speed={args.speed_us:.0f} —— 车不会动，拒绝运行")
            return 2
    else:
        if offline is False:
            print("[LANE] dry-run：--no-motor/默认不动电机；加 --allow-motion 才动车")

    # ---- 参数合成（默认 = 检测档 + settings，CLI 覆盖）
    # target_x：CLI > site.yaml 的现场标定值 > 1014 素材候选（TARGET_X0=330）
    target_x = args.target_x
    if target_x is None:
        try:
            from config import site as _site
            _v = _site.load().get("target_x")
            if _v is not None:
                target_x = float(_v)
                print(f"[LANE] target_x={target_x:.1f}（取自车端 site.yaml 的现场标定值）")
        except Exception as _exc:            # 读不到就用素材候选，绝不因此起不来
            print(f"[LANE] 读 site.yaml 的 target_x 失败（忽略）：{_exc}")
    if target_x is None:
        target_x = TARGET_X0
        print(f"[LANE] target_x={target_x:.0f}（1014 素材候选；**现场请先 --calibrate 重标**）")
    center_deg = float(args.servo_center if args.servo_center is not None
                       else settings.SERVO_CENTER_ANGLE)
    sign = float(args.steer_sign if args.steer_sign is not None else settings.STEER_SIGN)
    kp, ki, kd = gains_for(settings.IMG_W)
    kp = float(args.kp if args.kp is not None else kp)
    ki = float(args.ki if args.ki is not None else ki)
    kd = float(args.kd if args.kd is not None else kd)
    limit_deg = float(args.limit_deg if args.limit_deg is not None else 15.0)
    smooth = float(args.smooth if args.smooth is not None else 0.7)

    rate = FrameRate()
    if args.fps:
        rate.pin(float(args.fps))
    det = LaneFinalDetector(
        target_x=target_x, fps=(float(args.fps) if args.fps else None),
        roi_top=args.roi_top, roi_bottom=args.roi_bottom,
        look_top=args.look_top, look_bottom=args.look_bottom, y_ref=args.y_ref,
        k_min=args.k_min, k_max=args.k_max, ridge_min=args.ridge_min,
        min_samples=args.min_samples, pair_w_min=args.pair_w_min,
        pair_w_max=args.pair_w_max, edge_tol=args.edge_tol, half_w_seed=args.half_w_seed,
        jump_reject_px=args.jump_reject_px, jump_coherent_px=args.jump_coherent_px)
    runner = LaneRunner(
        det, rate, center_deg, sign, kp, ki, kd, limit_deg, smooth,
        args.speed_us, args.start_us, args.single_speed_us, args.single_hold,
        args.single_limit_deg, args.weak_limit_deg, args.min_quality,
        args.hold_s, args.acquire_s, args.err_stop_px, args.err_stop_s, REF_FPS,
        use_filter=not args.no_filter, filter_window=args.filter_window,
        filter_outlier_px=args.filter_outlier, observe_only=offline,
        single_max_s=args.single_max_s)

    print(f"[LANE] 增益（登记 {REF_FPS:.0f}fps）：kp={kp:.3f} ki={ki:.3f} kd={kd:.3f}"
          f"；30fps 逐帧等效 " + "ki=%.3f kd=%.3f" % runner.pid.frame_equivalent(30.0))

    # --save-max 0 = 自动：在线按 --max-seconds 算满全程（0.2s×120s=620 张），
    # 旧默认 400 只覆盖 80s —— 2026-10-15 现场跑 120s 的趟次后半段没图。
    if args.save_max <= 0:
        if (not offline) and args.max_seconds > 0 and args.save_every_s > 0:
            args.save_max = int(args.max_seconds / args.save_every_s) + 20
        else:
            args.save_max = 400

    if args.calibrate and not offline:
        return run_online_calibrate(args, det)

    if offline:
        return run_offline(args, det, center_deg, sign, runner)

    # ---- 在线：会话目录
    if not args.save_dir:
        base = "/root/dev/logs" if os.path.isdir("/root/dev") else os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "logs")
        args.save_dir = os.path.join(base, time.strftime("lane_%Y%m%d_%H%M%S"))
    args.save_dir = os.path.abspath(args.save_dir)
    os.makedirs(args.save_dir, exist_ok=True)
    return run_online(args, det, center_deg, sign, runner)


if __name__ == "__main__":
    sys.exit(main())
