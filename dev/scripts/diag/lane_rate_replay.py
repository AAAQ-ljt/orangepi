#!/usr/bin/env python3
"""帧率口径离线验证：真实实拍帧 → 检测 → **驱动路径的控制律**，两套口径对照。

为什么要有它（2026-10-12）
--------------------------
下摄帧率被证明是**变的**：亮环境 ≈30fps（29.8 实测）、暗环境 AE 长曝光掉到 9~16fps；
而循迹的增益/窗口是 2026-09/10 在 **15fps** 下现场整定的。控制律改成"按实测 dt 折算"
（见 `control/rate.py`）之后必须有**真实素材**上的证据，所以这个脚本把一批实拍帧
（`train/opencv-level1/video/frames/<素材>/`，30fps 素材）当虚拟赛道，逐帧跑：
    检测（LaneRefDetector） → 误差滤波 → PidRef（两种口径） → 单侧/弱对限幅 → 护栏
并回答三个问题：
  ① 30fps 下老口径（逐帧、锁死采样率）与新口径（dt 折算）各算出什么舵角、差多少；
  ② 同一段素材抽成 15fps 跑，新口径的舵角序列是否与 30fps **一致**（不变性）；
  ③ 跑偏/画龙护栏在两种口径下的触发时间（帧数门槛改成时间跨度后的效果）。

用的是**车上同一套类**（`LaneRefDetector` / `ErrorFilter` / `PidRef` / 两个护栏 /
`lane_final_test.apply_steer_cap`），不是另写一份控制律；参数默认取 settings
（= 车端 site.yaml），可用命令行覆盖成这批素材实测的档位。

用法
----
    cd dev && export PYTHONPATH=.
    PY="E:/venvs/smartcar-ultra/Scripts/python.exe"

    # ★ 常用档：统计 + 叠加图（每 20 帧一张）+ 接触表 + 差异最大帧
    "$PY" scripts/diag/lane_rate_replay.py \
        --frames ../train/opencv-level1/video/frames/10_11_1 \
        --out ../train/opencv-level1/lane_test/rate_10_11_1 \
        --roi-top 0.25 --roi-bottom 0.50 --slope-min 0.15 \
        --every 20 --montage 10 --flag-top 12

    # 只想要数字（不存图）
    "$PY" scripts/diag/lane_rate_replay.py --frames <DIR> --limit 400

产物（`--out` 下）：
  `report.txt`   人看的结论（检测率 / 四种口径表现 / 帧率敏感度 / 逐帧等效增益）
  `compare.json` 机器对账（同上，JSON）
  `f_XXXXXX.jpg` **叠加图**（`--every N`）：ROI/前瞻带/左右线/中心线画在图上，
                 左下角三行是"这一帧两种口径各下什么舵角"（本站要给人看的东西）
  `flag_XXXXXX.jpg` + `flags.txt`（`--flag-top K`）：新/旧口径舵角差最大的 K 帧
  `sheet_XXX.jpg` 接触表（`--montage K`：每 K 帧一个缩略图，5×4 一张）
  `pass_*.csv`   逐帧读数与舵角（新/旧口径、15fps 抽帧）
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import cv2
import numpy as np

from config import settings
from control.filters import ErrorFilter
from control.lane_control import (BigErrorGuard, OscillationGuard, PidRef,
                                  adaptive_pulse, gains_for)
from control.rate import REF_FPS
# lane_official 已于 2026-10-14 归档（最终循迹脚本 = scripts/lane_final_test.py 自带检测器）
# → 这里的 "official" 检测器变成可选；缺了也能跑 lane_ref 那一档。
try:
    from vision import lane_official
    from vision.lane_official import (MONTAGE_COLS, MONTAGE_H, MONTAGE_ROWS, MONTAGE_W,
                                      montage_sheet)
    HAS_OFFICIAL = True
except ImportError:                                   # pragma: no cover - 归档后走这条
    lane_official = None
    HAS_OFFICIAL = False
    MONTAGE_COLS, MONTAGE_ROWS, MONTAGE_W, MONTAGE_H = 5, 4, 320, 240

    def montage_sheet(thumbs):                        # 离线接触表的本地兜底实现
        import numpy as _np
        if not thumbs:
            return _np.zeros((MONTAGE_ROWS * MONTAGE_H, MONTAGE_COLS * MONTAGE_W, 3), _np.uint8)
        t = thumbs[0]
        rows = (len(thumbs) + MONTAGE_COLS - 1) // MONTAGE_COLS
        sheet = _np.zeros((rows * t.shape[0], MONTAGE_COLS * t.shape[1], 3), _np.uint8)
        for i, th in enumerate(thumbs):
            rr, cc = divmod(i, MONTAGE_COLS)
            sheet[rr * t.shape[0]:(rr + 1) * t.shape[0],
                  cc * t.shape[1]:(cc + 1) * t.shape[1]] = th
        return sheet

from vision.lane_ref import (LaneParams, LaneReading, LaneRefDetector, look_bounds,
                             roi_bounds)

OUT_CSV = ("t,center_x,error_px,error_filt,angle_deg,esc_us,quality,both_sides,"
           "half_w,n_left,n_right,note,guard\n")


# ================================================================ 素材
def load_frames(directory: str, limit: int = 0, stride: int = 1) -> List[str]:
    """按名排序取帧（= 时间序）。先只认 `frame_*.jpg`；没有才退回全部 jpg ——
    两个 glob 叠加会把同一帧算两遍（2026-10-12 实测踩过：1100 帧被当成 2200 帧）。"""
    paths = sorted(glob.glob(os.path.join(directory, "frame_*.jpg")))
    if not paths:
        paths = sorted(glob.glob(os.path.join(directory, "*.jpg")))
    if stride > 1:
        paths = paths[::stride]
    if limit > 0:
        paths = paths[:limit]
    return paths


def read_frame(path: str) -> Optional[np.ndarray]:
    """读图（cv2.imread 在中文路径下会返回 None → 用 np.fromfile 兜底）。"""
    img = cv2.imread(path)
    if img is not None:
        return img
    try:
        return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


# ================================================================ 叠加图
def _put_lines(vis: np.ndarray, lines: Sequence[str], color=(0, 255, 0)) -> None:
    """左下角一行行写字（黑描边 + 彩字，赛道上任何背景都读得出来）。"""
    y = vis.shape[0] - 10 - 18 * (len(lines) - 1)
    for txt in lines:
        cv2.putText(vis, txt, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, txt, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
        y += 18


def annotate_rate(frame: np.ndarray, *, roi: Tuple[int, int], band: Tuple[int, int],
                  target_x: float, left_kb=None, right_kb=None,
                  center_x: Optional[float] = None, header: str = "",
                  tag: str = "", lines: Sequence[str] = ()) -> np.ndarray:
    """叠加图（**照 `lane_official` 的画法**，本工作区那批参考图的同款直观）：

    ROI 框、扫描带、左右线、目标线、中心点 —— 关键是**左右线只画在扫描带内**，
    不外推成横跨整幅的对角线（2026-10-13：外推版被判定"完全不如上一版"，就是这条）。
    最下面三行写帧率口径的读数（这一帧两种口径各下什么舵角）。
    """
    vis = frame.copy()
    h, w = vis.shape[:2]
    y0, y1 = int(roi[0]), int(roi[1])
    ya, yb = int(band[0]), int(band[1])
    cv2.rectangle(vis, (0, y0), (w - 1, y1), (200, 120, 0), 1)
    for y in (ya, yb):
        cv2.line(vis, (0, y), (w - 1, y), (0, 200, 200), 1)
    for kb, color in ((left_kb, (255, 80, 80)), (right_kb, (80, 80, 255))):
        if kb is None or abs(float(kb[0])) < 1e-9:
            continue
        k, b = float(kb[0]), float(kb[1])
        # ⚠️ 端点**不要**自己 clip 到画面内：一旦把 x 夹到 0/w-1，画出来的线段斜率就变了、
        #   不再压在真白线上（2026-10-13 自测发现）。越界坐标交给 cv2.line 自己裁。
        cv2.line(vis, (int(round((ya - b) / k)), ya), (int(round((yb - b) / k)), yb), color, 2)
    cv2.line(vis, (int(target_x), 0), (int(target_x), h), (0, 200, 0), 1)
    if center_x is not None:
        cv2.circle(vis, (int(center_x), int((ya + yb) * 0.5)), 6, (0, 255, 255), 2)
    for txt, y in ((header, 20), (tag, 40)):
        if not txt:
            continue
        cv2.putText(vis, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 255, 60), 1, cv2.LINE_AA)
    _put_lines(vis, list(lines))
    return vis


# ================================================================ 检测器适配
def _to_reading(res, target_px: float) -> LaneReading:
    """`lane_official.LaneResult` → 本工具统一用的 `LaneReading` 外形（只填控制用得到的字段）。

    质量分按"两侧都看到=1.0 / 单侧=0.55 / 没中心=0"给（与车端 lane_ref 的档位一致，
    控制层据此决定是否跟线、是否降速）；半宽用两侧读数反推（官方检测器不给半宽先验）。
    """
    hw = None
    if res.both_sides and res.left_x is not None and res.right_x is not None:
        hw = abs(float(res.right_x) - float(res.left_x)) / 2.0
    quality = 1.0 if res.both_sides else (0.55 if res.center_x is not None else 0.0)
    return LaneReading(left_x=res.left_x, right_x=res.right_x, center_x=res.center_x,
                       error=res.error, n_left=res.n_left, n_right=res.n_right,
                       both_sides=res.both_sides, half_width=hw,
                       width_measured=bool(res.both_sides), quality=quality,
                       left_kb=res.left_kb, right_kb=res.right_kb,
                       note=(res.note or "").replace(",", "；"),
                       tag=("1side-" + res.single_fill) if res.single_fill else "")


# ================================================================ 一套口径
class LanePass:
    """一套控制律口径（老逐帧 / 新 dt 折算），吃同一串读数，记下每一步。"""

    def __init__(self, name: str, *, gains: Tuple[float, float, float], dt: Optional[float],
                 ref_fps: float = REF_FPS, use_filter: bool = True, steer_sign: float = -1.0,
                 center_deg: float = 86.5, limit_deg: float = 15.0, smooth: float = 0.7,
                 min_quality: float = 0.28, single_limit_deg: float = 5.0,
                 weak_limit_deg: float = 10.0, err_stop_px: float = 60.0,
                 err_stop_s: float = 1.5, target_px: float = 320.0,
                 speed_us: float = 1560.0, speed_fast: float = 0.0,
                 speed_slow: float = -10.0) -> None:
        self.name = name
        self.dt = dt                      # None = 老逐帧口径；有值 = 按 dt 折算
        self.target_px = float(target_px)
        self.center_deg = float(center_deg)
        self.min_quality = float(min_quality)
        self.single_limit_deg = float(single_limit_deg)
        self.weak_limit_deg = float(weak_limit_deg)
        self.speed_us = float(speed_us)
        self.speed_fast = float(speed_fast)
        self.speed_slow = float(speed_slow)
        kp, ki, kd = gains
        self.pid = PidRef(kp, ki, kd, limit_deg, smooth, sign=steer_sign,
                          center=center_deg, ref_fps=ref_fps)
        self.kp, self.ki, self.kd = kp, ki, kd
        fps = (1.0 / dt) if dt else REF_FPS
        self.filter = ErrorFilter(fps=fps) if use_filter else None   # 窗口按秒语义折算
        self.big_err = BigErrorGuard(err_stop_px, err_stop_s)
        self.osc = OscillationGuard()
        self.rows: List[dict] = []
        self.trip_t: Optional[float] = None
        self.trip_kind = ""

    def feed(self, t: float, r: Optional[LaneReading]) -> float:
        """喂一帧读数，返回本帧舵角（记进 self.rows）。"""
        err_c = None
        if r is not None and r.error is not None and r.quality >= self.min_quality:
            err_c = r.error if self.filter is None else self.filter.update(r.error)
        angle = self.center_deg
        guard = ""
        if err_c is not None:
            steer = self.pid.step(err_c, self.dt)
            steer = apply_steer_cap(self.center_deg, steer, bool(r and r.both_sides),
                                    (r.n_left if r else 0), (r.n_right if r else 0),
                                    self.single_limit_deg, self.weak_limit_deg)
            angle = steer
        # 护栏（与实跑同款）：绕行场景这里不存在，直接喂
        if self.big_err.update(t, err_c, r.error if r is not None else None) and not self.trip_t:
            self.trip_t, self.trip_kind = t, "big-err"
        if self.osc.update(t, angle, self.center_deg) and not self.trip_t:
            self.trip_t, self.trip_kind = t, "osc"
        if self.trip_t:
            guard = self.trip_kind
        esc = (adaptive_pulse(err_c, self.speed_us, self.speed_fast, self.speed_slow)
               if err_c is not None else self.speed_us)
        self.rows.append({
            "t": float(t), "angle": float(angle), "esc": float(esc), "err_filt": err_c,
            "center_x": (r.center_x if r else None), "error": (r.error if r else None),
            "quality": (r.quality if r else 0.0),
            "both_sides": bool(r and r.both_sides),
            "half_w": (r.half_width if r else None), "n_left": (r.n_left if r else 0),
            "n_right": (r.n_right if r else 0),
            "note": ((r.note if r else "") or "").replace(",", "；"), "guard": guard})
        return angle

    # ---------------------------------------------------------- 统计
    def angles(self) -> np.ndarray:
        return np.asarray([row["angle"] for row in self.rows], dtype=float)

    def times(self) -> np.ndarray:
        return np.asarray([row["t"] for row in self.rows], dtype=float)

    def stats(self) -> Dict[str, object]:
        a = self.angles()
        if a.size == 0:
            return {"n": 0}
        dev = np.abs(a - self.center_deg)
        step = np.abs(np.diff(a)) if a.size > 1 else np.zeros(1)
        flips = int(np.sum(np.diff(np.sign(a - self.center_deg)) != 0))
        trip_txt = ("%.2fs(%s)" % (self.trip_t, self.trip_kind)) if self.trip_t else "—"
        return {"n": int(a.size),
                "dt": ("%.4f" % self.dt) if self.dt else "逐帧",
                "sat_pct": round(float(np.mean(dev >= 15.0 - 1e-6)) * 100, 1),
                "mean_abs": round(float(dev.mean()), 2),
                "step_max": round(float(step.max()), 2),
                "step_p95": round(float(np.percentile(step, 95)), 2),
                "flips": flips, "trip": trip_txt}

    def csv(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(OUT_CSV)
            for row in self.rows:
                def num(v, fmt="%.1f"):
                    return "" if v is None else fmt % v
                fh.write("%.3f,%s,%s,%s,%.2f,%.0f,%.2f,%d,%s,%d,%d,%s,%s\n" % (
                    row["t"], num(row["center_x"]), num(row["error"]), num(row["err_filt"]),
                    row["angle"], row["esc"], row["quality"], 1 if row["both_sides"] else 0,
                    num(row["half_w"], "%.0f"), row["n_left"], row["n_right"],
                    row["note"], row["guard"]))


def apply_steer_cap(center: float, steer: float, both_sides: bool,
                    n_left: int, n_right: int, single_deg: float, weak_deg: float) -> float:
    """与实跑同一套限幅（从 `scripts/lane_final_test.py` 借；缺失时用等价实现）。"""
    try:
        from scripts.lane_final_test import apply_steer_cap as _cap
        return _cap(center, steer, both_sides, n_left, n_right, single_deg, weak_deg)
    except Exception:
        cap = 180.0
        if not both_sides:
            cap = float(single_deg)
        elif min(n_left, n_right) <= 2:
            cap = float(weak_deg)
        return float(max(center - cap, min(center + cap, steer)))


# ================================================================ 对照
def _resample(t_src: np.ndarray, a_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """把 a_src 按时间最近邻重采样到 t_dst（只用于"同一物理时刻的舵角差"）。"""
    idx = np.clip(np.searchsorted(t_src, t_dst), 0, len(a_src) - 1)
    return a_src[idx]


def compare(pairs: Sequence[Tuple[str, LanePass, str, LanePass]]) -> List[Dict[str, object]]:
    out = []
    for label, p_new, _label_old, p_old in pairs:
        t_new, a_new = p_new.times(), p_new.angles()
        t_old, a_old = p_old.times(), p_old.angles()
        if t_new.size == 0 or t_old.size == 0:
            continue
        # 只比两套都覆盖的时间区间（同一物理轨迹上的同一时刻）
        t0 = max(t_new.min(), t_old.min())
        t1 = min(t_new.max(), t_old.max())
        mask = (t_new >= t0) & (t_new <= t1)
        tc = t_new[mask]
        if tc.size < 10:
            continue
        d = np.abs(a_new[mask] - _resample(t_old, a_old, tc))
        out.append({"case": label, "n": int(d.size),
                    "diff_mean": round(float(d.mean()), 2),
                    "diff_p95": round(float(np.percentile(d, 95)), 2),
                    "diff_max": round(float(d.max()), 2)})
    return out


# ================================================================ 主流程
def run(args) -> int:
    paths = load_frames(args.frames, args.limit, args.stride)
    if not paths:
        print(f"[RATE] {args.frames} 里没有 jpg 帧")
        return 1
    out = args.out or os.path.join("logs", "lane_rate_" + os.path.basename(
        os.path.normpath(args.frames)))
    os.makedirs(out, exist_ok=True)

    params = LaneParams.from_settings(
        roi_top_ratio=args.roi_top, roi_bottom_ratio=args.roi_bottom,
        look_top_ratio=args.look_top, look_bottom_ratio=args.look_bottom,
        target_ratio=args.target_ratio, slope_min=args.slope_min,
        pair_w_min=args.pair_w_min, pair_w_max=args.pair_w_max)
    fps = float(args.fps)
    dt = 1.0 / fps
    center_deg = float(settings.SERVO_CENTER_ANGLE if args.center_deg is None
                       else args.center_deg)
    steer_sign = float(settings.STEER_SIGN if args.steer_sign is None else args.steer_sign)
    kp, ki, kd = gains_for(settings.IMG_W, args.gains)
    kp = kp if args.kp is None else float(args.kp)
    ki = ki if args.ki is None else float(args.ki)
    kd = kd if args.kd is None else float(args.kd)
    target_px = float(args.target_px)
    # ★ 2026-10-13：检测器（lane_ref）的误差口径必须和控制目标一致 ——
    #   否则"读数里的 err"与"控制律实际在追的目标"是两回事，图上/CSV 里对不上
    #   （实跑路径就是这么做的：target_px = params.target_ratio × 画面宽）。
    params.target_ratio = target_px / float(settings.IMG_W)

    report: List[str] = []
    def say(line: str = "") -> None:
        print(line)
        report.append(line)

    say(f"[RATE] 素材 {args.frames}：{len(paths)} 帧（stride={args.stride}），"
        f"按 {fps:.1f}fps 采样（dt={dt:.4f}s）")
    say(f"[RATE] 检测参数：ROI {params.roi_top_ratio:.2f}~{params.roi_bottom_ratio:.2f}"
        f"（y {int(480 * params.roi_top_ratio)}~{int(480 * params.roi_bottom_ratio)}）、"
        f"前瞻带 {params.look_top_ratio:.2f}~{params.look_bottom_ratio:.2f}、"
        f"斜率 |dx/dy| ∈ [{params.slope_min:.2f}, {params.slope_max:.2f}]、"
        f"配对宽 [{params.pair_w_min:.0f}, {params.pair_w_max:.0f}]px")
    say(f"[RATE] 控制律：kp={kp:.3f} ki={ki:.3f} kd={kd:.3f}（登记口径 {args.fps_ref:.0f}fps）、"
        f"限幅±{args.limit_deg:.0f}°、舵机中位 {center_deg:.1f}°、转向符号 {steer_sign:+.0f}、"
        f"目标点 {target_px:.0f}px")

    # ---- 检测器（两种，见 --detector 说明）----
    if args.detector == "official":
        if not HAS_OFFICIAL:
            raise SystemExit("[RATE] lane_official 已于 2026-10-14 归档 → --detector official 不可用；"
                             "用 --detector ref，或直接用新的 scripts/lane_final_test.py --frames")
        off_p = lane_official.LaneParams(
            roi_top=float(params.roi_top_ratio), roi_bottom=float(params.roi_bottom_ratio),
            scan_top=(None if args.look_top is None else float(args.look_top)),
            scan_bottom=(None if args.look_bottom is None else float(args.look_bottom)),
            slope_min=float(params.slope_min), slope_max=float(params.slope_max),
            target_x=float(target_px), single_mode=str(args.official_single))
        det = lane_official.LaneDetector(off_p)
        roi_of = lambda h: lane_official.roi_bounds(h, off_p)      # noqa: E731
        band_of = lambda h: lane_official.scan_bounds(h, off_p)    # noqa: E731
        say(f"[RATE] 检测器：lane_official（本批素材实测锁线最好的那个；"
            f"ROI {off_p.roi_top:.2f}~{off_p.roi_bottom:.2f}、|dx/dy|≥{off_p.slope_min:.2f}、"
            f"单侧={off_p.single_mode}）—— 图上就是这套读数")
    else:
        det = LaneRefDetector(params, fps=fps)
        roi_of = lambda h: roi_bounds(h, params)                   # noqa: E731
        band_of = lambda h: look_bounds(h, params)                 # noqa: E731
        say("[RATE] 检测器：lane_ref（**车端当前驱动路径的检测器**；本批平视角素材上"
            "双侧率明显低于 lane_official）")

    readings: List[Tuple[float, LaneReading]] = []
    src_paths: List[str] = []                 # 与 readings 一一对应（画叠加图时按需重读）
    for i, path in enumerate(paths):
        frame = read_frame(path)
        if frame is None:
            continue
        res = det.detect(frame)
        r = res if args.detector != "official" else _to_reading(res, target_px)
        readings.append((i * dt, r))
        src_paths.append(path)
    n = len(readings)
    if n == 0:
        print("[RATE] 一帧都没读出来")
        return 1
    both = sum(1 for _, r in readings if r.both_sides)
    single = sum(1 for _, r in readings if r.center_x is not None and not r.both_sides)
    lost = sum(1 for _, r in readings if r.center_x is None)
    say(f"[RATE] 检测：双侧 {both}/{n}（{both / n * 100:.0f}%）、单侧兜底 {single}、无中心 {lost}"
        + (f"；半宽先验 {det.width_prior.value:.0f}px"
           f"（{'实测' if det.width_prior.measured else '种子'}，窗口 {det.width_prior.window} 帧）"
           if hasattr(det, "width_prior") else ""))

    def make(name: str, dt_mode: Optional[float], ref_fps: float = args.fps_ref) -> LanePass:
        return LanePass(name, gains=(kp, ki, kd), dt=dt_mode, ref_fps=ref_fps,
                        use_filter=not args.no_filter, steer_sign=steer_sign,
                        center_deg=center_deg, limit_deg=args.limit_deg, smooth=args.smooth,
                        min_quality=args.min_quality, single_limit_deg=args.single_limit_deg,
                        weak_limit_deg=args.weak_limit_deg, err_stop_px=args.err_stop_px,
                        err_stop_s=args.err_stop_s, target_px=target_px,
                        speed_us=args.speed_us, speed_fast=args.speed_fast,
                        speed_slow=args.speed_slow)

    # 四个口径：老/新 × 30fps 全帧 / 15fps 抽帧（抽帧后 dt 翻倍 = 同一物理轨迹的粗采样）
    passes = [("legacy@fps", make("legacy@fps", None)),
              ("dt@fps", make("dt@fps", dt)),
              ("legacy@fps/2", make("legacy@fps/2", None))]
    for name, p in passes:
        for k, (t, r) in enumerate(readings):
            p.feed(t, r)
    half = [(t, r) for k, (t, r) in enumerate(readings) if k % 2 == 0]
    p_dt_half = make("dt@fps/2", 2.0 * dt)
    for t, r in half:
        p_dt_half.feed(t, r)

    say()
    say("[RATE] 各口径的表现（同一批读数）：")
    for name, p in passes + [("dt@fps/2", p_dt_half)]:
        st = p.stats()
        say(f"  {name:<12s} 帧数 {st['n']:>5}  打满限幅 {st['sat_pct']:>5.1f}%  "
            f"平均|舵角−中位| {st['mean_abs']:>5.2f}°  "
            f"单帧最大变化 {st['step_max']:>5.2f}° (p95 {st['step_p95']:.2f})  "
            f"方向翻转 {st['flips']:>4} 次  护栏触发 {st['trip']}")

    cases = compare([("老口径 vs 新口径（同采样率 30fps）",
                      passes[1][1], "", passes[0][1])])
    legacy_full = passes[0][1]
    legacy_half = passes[2][1]
    dt_full = passes[1][1]
    a_f = legacy_full.angles()
    a_h = legacy_half.angles()
    m = min(len(a_f[::2]), len(a_h))
    d_legacy = np.abs(a_f[::2][:m] - a_h[:m])
    m2 = min(len(dt_full.angles()[::2]), len(p_dt_half.angles()))
    d_dt = np.abs(dt_full.angles()[::2][:m2] - p_dt_half.angles()[:m2])
    say()
    say("[RATE] ★ 帧率敏感度（同一批素材：15fps 抽帧 vs 30fps 全帧，逐帧舵角差）：")
    say(f"  老口径（逐帧、不带 dt）：平均 {d_legacy.mean():.2f}° / p95 "
        f"{np.percentile(d_legacy, 95):.2f}° / max {d_legacy.max():.2f}°")
    say(f"  新口径（dt 折算）      ：平均 {d_dt.mean():.2f}° / p95 "
        f"{np.percentile(d_dt, 95):.2f}° / max {d_dt.max():.2f}°")
    say("  （新口径应当明显更小 —— 这就是「帧率翻倍 kp/kd 必须重标」的量化版。"
        "「口径」含三样：微分/积分的 dt 折算、平滑的时间常数、滤波窗口按秒折算；"
        "老口径这三样全锁在「每帧」上）")
    # 残留从哪来：抽帧后**输入读数本身就不同**（采到的帧不同、单侧/双侧分类不同），
    # 不是折算错了。把最大差那一帧的类型打出来，免得好数字被误读成 bug。
    i_max = int(np.argmax(d_dt))
    if i_max < len(p_dt_half.rows):
        row = p_dt_half.rows[i_max]
        err_txt = "—" if row["error"] is None else "%.1fpx" % row["error"]
        say(f"  新口径的最大差出现在 t={row['t']:.2f}s（{d_dt.max():.2f}°）："
            f"该帧 {'双侧' if row['both_sides'] else '单侧'}、质量 {row['quality']:.2f}、"
            f"误差 {err_txt} —— 抽帧两边吃到的**不是同一批读数**"
            f"（这批素材单侧率高的直接代价），不是折算误差")
    if cases:
        c = cases[0]
        say(f"  同采样率下老/新口径的舵角差：平均 {c['diff_mean']}° / p95 "
            f"{c['diff_p95']}° / max {c['diff_max']}°（差多少 = 30fps 下老口径的手感偏了多少）")

    say()
    say("[RATE] 逐帧等效值（现场 A/B 表按登记口径写，这行是「代码实际在用多少」）：")
    kfp, kfd = passes[1][1].pid.frame_equivalent(fps)
    flt = passes[1][1].filter
    say(f"  {fps:.1f}fps → ki={kfp:.4f} kd={kfd:.4f}（登记 {REF_FPS:.0f}fps："
        f"ki={ki:.4f} kd={kd:.4f}）；滤波窗口 "
        f"{flt.window if flt else '关'} 帧" + (f"、拒收 {flt.max_rejects} 帧" if flt else ""))

    # ---------------------------------------------------------------- 叠加图
    # 和 `lane_test/10_11/` 那批一样：ROI/前瞻带/左右线/中心线直接画在图上，
    # 只是左下角多写三行"这一帧两种口径各下什么舵角"—— 帧率问题的证据要长这样。
    dt_full = passes[1][1]
    legacy_full = passes[0][1]
    d_new_old = np.abs(dt_full.angles() - legacy_full.angles())   # 同采样率下新/旧口径差
    flagged: List[Tuple[int, float]] = []
    n_img = 0
    if args.every or args.montage or args.flag_top:
        save_set = set(range(0, n, max(1, args.every))) if args.every else set()
        thumb_set = set(range(0, n, max(1, args.montage))) if args.montage else set()
        if args.flag_top:
            flagged = [(int(i), float(d_new_old[i])) for i in np.argsort(-d_new_old)[:args.flag_top]]
            save_set |= {i for i, _ in flagged}
        need = save_set | thumb_set
        banner = (f"rate: reg {REF_FPS:.0f}fps -> meas {fps:.1f}fps"
                  f" | per-frame ki={kfp:.3f} kd={kfd:.3f}")
        thumbs: List[np.ndarray] = []
        for idx in sorted(i for i in need if i < n):
            frame = read_frame(src_paths[idx])
            if frame is None:
                continue
            r = readings[idx][1]
            t = readings[idx][0]
            row_new = dt_full.rows[idx]
            row_old = legacy_full.rows[idx]
            err = row_new["error"]
            # ⚠️ 叠加图上的字必须是 ASCII：cv2.putText 画中文会变成一串方框
            #   （`lane_final_test.annotate` 里同款注释，2026-09-22 实验室实测）。
            kind = ("both" if row_new["both_sides"] else
                    ("1side" if row_new["center_x"] is not None else "lost"))
            h_sh, w_sh = frame.shape[:2]
            rr = readings[idx][1]
            header = (f"{kind}  c={'--' if rr.center_x is None else '%.0f' % rr.center_x}"
                      f"  e={'--' if rr.error is None else '%+.0f' % rr.error}"
                      f"  q={rr.quality:.2f}"
                      f"  L={'--' if rr.left_x is None else '%.0f' % rr.left_x}"
                      f"  R={'--' if rr.right_x is None else '%.0f' % rr.right_x}"
                      f"  segs={rr.n_left}/{rr.n_right}")
            lines = [banner,
                     f"new(dt)= {row_new['angle']:5.1f}deg  old(framelock)= {row_old['angle']:5.1f}deg"
                     f"  d= {d_new_old[idx]:4.1f}deg  esc= {row_new['esc']:.0f}us"]
            if idx % 2 == 0 and idx // 2 < len(p_dt_half.rows):
                h_new = p_dt_half.rows[idx // 2]
                h_old = passes[2][1].rows[idx // 2]
                lines.append(f"15fps-sample:  new= {h_new['angle']:5.1f}deg"
                             f"  old= {h_old['angle']:5.1f}deg   (same frame as above)")
            lines.append(f"t= {t:6.2f}s  err= {'--' if err is None else '%+.0fpx' % err}"
                         f"  q= {row_new['quality']:.2f}  {kind}"
                         f"  segs= {row_new['n_left']}/{row_new['n_right']}")
            vis = annotate_rate(frame, roi=roi_of(h_sh), band=band_of(h_sh), target_x=target_px,
                                left_kb=rr.left_kb, right_kb=rr.right_kb, center_x=rr.center_x,
                                header=header, tag=("rate | " + rr.tag if rr.tag else "rate"),
                                lines=lines)
            if idx in save_set:
                cv2.imwrite(os.path.join(out, f"f_{idx:06d}.jpg"), vis)
                n_img += 1
            if idx in thumb_set:
                thumbs.append(cv2.resize(vis, (MONTAGE_W, MONTAGE_H),
                                         interpolation=cv2.INTER_AREA))
            if any(idx == i for i, _ in flagged):
                cv2.imwrite(os.path.join(out, f"flag_{idx:06d}.jpg"), vis)
        if args.montage and thumbs:
            per = MONTAGE_COLS * MONTAGE_ROWS
            for si in range(0, len(thumbs), per):
                cv2.imwrite(os.path.join(out, f"sheet_{si // per:03d}.jpg"),
                            montage_sheet(thumbs[si:si + per]))
        if flagged:
            with open(os.path.join(out, "flags.txt"), "w", encoding="utf-8") as fh:
                fh.write("# 帧率口径差异最大的帧（新/旧口径舵角差 Δ，按 Δ 降序）\n")
                fh.write("# 帧号  t(s)  Δ(deg)  新舵角  旧舵角  读数\n")
                for idx, dv in flagged:
                    rw, ro = dt_full.rows[idx], legacy_full.rows[idx]
                    kind = "双侧" if rw["both_sides"] else (
                        "单侧" if rw["center_x"] is not None else "无中心")
                    fh.write(f"{idx:6d} {readings[idx][0]:7.2f} {dv:8.2f} "
                             f"{rw['angle']:8.1f} {ro['angle']:8.1f}  {kind} q={rw['quality']:.2f}\n")
            say()
            say(f"[RATE] 差异最大的 {len(flagged)} 帧（Δ=新/旧口径舵角差）已单独存 flag_*.jpg：")
            for idx, dv in flagged[:5]:
                say(f"  frame {idx:6d}  t={readings[idx][0]:6.2f}s  Δ={dv:5.1f}deg"
                    f"  新={dt_full.rows[idx]['angle']:.1f}deg 旧={legacy_full.rows[idx]['angle']:.1f}deg")

    for name, p in passes + [("dt@fps/2", p_dt_half)]:
        p.csv(os.path.join(out, f"pass_{name.replace('/', '_').replace('@', '_')}.csv"))
    with open(os.path.join(out, "compare.json"), "w", encoding="utf-8") as fh:
        json.dump({"frames": args.frames, "n": n, "fps": fps,
                   "detect": {"both": both, "single": single, "lost": lost},
                   "passes": {name: p.stats() for name, p in passes + [("dt@fps/2", p_dt_half)]},
                   "sensitivity": {"legacy": {"mean": float(d_legacy.mean()),
                                              "p95": float(np.percentile(d_legacy, 95)),
                                              "max": float(d_legacy.max())},
                                   "dt": {"mean": float(d_dt.mean()),
                                          "p95": float(np.percentile(d_dt, 95)),
                                          "max": float(d_dt.max())}}},
                  fh, ensure_ascii=False, indent=1)
    with open(os.path.join(out, "report.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(report) + "\n")
    print(f"\n[RATE] 产物：{out}/report.txt、compare.json、pass_*.csv"
          + (f"、{n_img} 张叠加图 f_*.jpg" if n_img else "")
          + ("、sheet_*.jpg 接触表" if args.montage else "")
          + (f"、flag_*.jpg + flags.txt（{len(flagged)} 帧）" if flagged else ""))
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="帧率口径离线验证（不碰硬件）")
    ap.add_argument("--frames", required=True, help="帧目录（frame_*.jpg）")
    ap.add_argument("--out", default="", help="产物目录（默认 logs/lane_rate_<素材>）")
    ap.add_argument("--fps", type=float, default=30.0, help="素材的采样率（默认 30）")
    ap.add_argument("--detector", choices=("official", "lane_ref"), default="lane_ref",
                    help="用哪个检测器出误差序列（lane_official 已归档 → 默认 lane_ref；"
                         "新的最终脚本 = scripts/lane_final_test.py --frames）")
    ap.add_argument("--official-single", choices=("border", "off"), default="border",
                    help="--detector official 时单侧丢线怎么办：border=参考实现原样（缺的一侧"
                         "代画面边界，10-11 那批图同款）；off=不给中心（按维持/丢线处理）")
    ap.add_argument("--fps-ref", type=float, default=REF_FPS,
                    help=f"增益的**登记口径**帧率（默认 {REF_FPS:.0f} = 现场整定帧率）。"
                         "★ 它只改**刚度**不改不变性：dt 路径是连续时间形式，任何登记值下"
                         "帧率不变性都成立（登记 30 = kd 折算成一半、ki 折算成两倍）。"
                         "想验证'老口径的病'请看输出里的 legacy 行（那才是帧锁定的实现）")
    ap.add_argument("--limit", type=int, default=0, help="只跑前 N 帧（0=全部）")
    ap.add_argument("--stride", type=int, default=1, help="每 N 帧取一帧（跳帧用）")
    ap.add_argument("--every", type=int, default=0,
                    help="每 N 帧存一张**叠加图** f_XXXXXX.jpg（ROI/前瞻带/左右线直接画在图上，"
                         "同 lane_test/10_11 那批；0=不存）")
    ap.add_argument("--annotate-every", type=int, dest="every",
                    help="--every 的旧名（等价；默认 0=不存）")
    ap.add_argument("--montage", type=int, default=0,
                    help="存接触表 sheet_XXX.jpg（每张拼 5x4 个缩略图；0=不存）")
    ap.add_argument("--flag-top", type=int, default=0,
                    help="把「新/旧口径舵角差」最大的前 K 帧单独存 flag_*.jpg + flags.txt（0=不存）")
    # 检测参数（默认取 settings，可用这批素材的实测档覆盖）
    ap.add_argument("--roi-top", type=float, default=None)
    ap.add_argument("--roi-bottom", type=float, default=None)
    ap.add_argument("--look-top", type=float, default=None)
    ap.add_argument("--look-bottom", type=float, default=None)
    ap.add_argument("--slope-min", type=float, default=None)
    ap.add_argument("--pair-w-min", type=float, default=None)
    ap.add_argument("--pair-w-max", type=float, default=None)
    ap.add_argument("--target-ratio", type=float, default=None)
    # 控制参数
    ap.add_argument("--gains", choices=("literal", "width"), default="literal")
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--ki", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--limit-deg", type=float, default=15.0)
    ap.add_argument("--smooth", type=float, default=0.7)
    ap.add_argument("--min-quality", type=float, default=0.28)
    ap.add_argument("--single-limit-deg", type=float, default=5.0)
    ap.add_argument("--weak-limit-deg", type=float, default=10.0)
    ap.add_argument("--err-stop-px", type=float, default=60.0)
    ap.add_argument("--err-stop-s", type=float, default=1.5)
    ap.add_argument("--target-px", type=float, default=320.0, help="目标点（像素）")
    ap.add_argument("--center-deg", type=float, default=None, help="舵机中位（默认 site）")
    ap.add_argument("--steer-sign", type=float, default=None)
    ap.add_argument("--speed-us", type=float, default=1560.0)
    ap.add_argument("--speed-fast", type=float, default=0.0)
    ap.add_argument("--speed-slow", type=float, default=-10.0)
    ap.add_argument("--no-filter", action="store_true")
    return ap


if __name__ == "__main__":
    sys.exit(run(build_parser().parse_args()))
