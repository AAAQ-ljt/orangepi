#!/usr/bin/env python3
"""循迹实跑（参考实现版）—— 面向**操场实测**的唯一入口。

一段话讲清它做什么
------------------
照上届 `oldCode`（纯 CV：Canny + Hough + 斜率先验）移植的循迹，带一套"到场地就能用"的流程：
**起步前自检 → 蓝板发车 → 循迹 → 再见板停车 → 全程落盘**。检测器在 `vision/lane_ref.py`，
诊断探针 `diag/lane_probe.py` 用的是同一个类，两处读数应当一致。

为什么和上一版不一样（上一版实验室跑不通、操场也没成功过）
----------------------------------------------------------
1. **起步前自检是自动的**（`_preflight`）：车静止时扫一遍候选带（哪条带真的锁住两条线）、
   用实测"有边缘支持的行"定前瞻带、再用静止画面标定目标点。**不再让人现场猜 ROI**——
   去一趟操场很贵，猜错一次就白跑一趟；
2. **单侧丢线有兜底**（`vision/lane_ref.py`）：参考实现里丢一侧线时是把画面边界当代替，
   上一版却改成了"单侧不给中心"，等于把唯一能让车继续往前走的机制扔了。现在用
   **车道半宽先验**推中心，并给较低的质量分（控制层据此降速）；
3. **丢线阶梯**（`control/lane_control.py`）：丢线 `--hold-s`(0.6s) 内沿用上一次有效中心
   （误差按时间衰减回中位），超过就停车 —— 对应《具体实施方案》§3.1.4 的仲裁层设计；
4. **全程落盘、回实验室再调**：CSV 多了"滤波后误差 / 目标点 / 质量 / 单双侧 / 半宽"等列，
   另外定时存原图 + 叠加图到一个会话目录（本车没有 X 服务器，事后看图和数字是唯一途径），
   `--replay` 可以在笔记本上拿这些帧重跑检测与控制律，不用为了调参再跑一趟操场。

用法（详见 `doc/实地调试清单.md` §2.6）
--------------------------------------
    # ① 本地：分析一张图（不碰硬件）
    python scripts/lane_ref_test.py --image 某张赛道图.jpg

    # ② 本地：复盘一次实跑的落盘数据（不碰硬件；可换参数重跑检测 + 控制律）
    python scripts/lane_ref_test.py --replay logs/lane_20260922_101500 --kp 0.2 --roi-sweep

    # ③ 车上：只看读数，不动电机
    sudo python3 /root/dev/scripts/lane_ref_test.py --no-motor

    # ④ 车上：正式跑（场地空旷 + 有人能立刻断电）
    sudo python3 /root/dev/scripts/lane_ref_test.py --allow-motion --max-seconds 120

安全（红线，见 AGENTS.md §1 与《循迹脚本交接.md》§8）
----------------------------------------------------
摄像头独占（自动停/恢复两路推流）、电调死区守卫、任何退出路径电调归零 + 舵机回中 +
恢复 `opi-control` 与推流；发车**边沿触发**（先见板 → 再消失），没见过板绝不动车。
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np

from config import settings
from control.filters import ErrorFilter
from control.lane_control import (BigErrorGuard, LIMIT_DEG0, LossGuard, LossState,
                                  NEUTRAL_US, OscillationGuard, PidRef, SPEED_FAST_US, SPEED_SLOW_US,
                                  SPEED_FLOOR_MARGIN_US, adaptive_pulse, floor_pulse,
                                  gains_for)
from vision.camera_guard import camera_exclusive, open_camera
from vision.lane_ref import (REF_BAND, LaneParams, LaneReading, LaneRefDetector, auto_look_band,
                             look_bounds, roi_bounds, sweep_bands, x_of_kb)
from vision.start_gate import StartGate

CSV_HEADER = ("t,phase,center_x,error_px,error_filt,target_px,angle_deg,esc_us,"
              "left_x,right_x,n_left,n_right,quality,both_sides,half_w,width_measured,"
              "anchored,edge_pct,since_valid_s,note\n")
DEFAULT_TARGET_RATIO = 375.0 / 640.0     # 2026-09-19 操场两帧的落点（只在关掉自动标定时才用）
SETUP_FRAMES = 20                        # 起步前自检采几帧
MIN_QUALITY0 = 0.28                      # 与 site.yaml 的 arbiter_conf_thresh 同量级
HOLD_S0 = 0.6                            # 丢线维持窗口（设计文档：0.3~0.8s）
WEAK_RUN_MAX = 20                        # 自检"证据够不够"的门槛：最长连续支持行数低于它
WEAK_SPREAD_PX = 25.0                      # 门槛之二：静止时中心跨距超过它 = 锁的东西在跳
                                            # （2026-09-22 实验室连跑三次 47~166px，目标点跟着漂）
                                         # 就不采纳自检出的 ROI/目标点（2026-09-22 实测加的）
INF = float("inf")

# ★ `--ref-behavior`：把参考实现 oldCode 的**拍摄比例**照搬到我们的 640×480 上。
#   它的画面是 320×240（launch.cpp），所以：
#     画面比例 → ROI 0.50~0.85；算中心的行 130~230 → 0.542~0.958；目标点 = 画面中心 0.5；
#     单侧丢线代画面边界；增益按画面宽度换算（--gains width）。
#   ⚠️ 这是**对照实验**，不是推荐配置：它的镜头看得见近处地面，我们的下摄太平、近处是画外，
#      所以这条 look band（y≈260~460）在本车上多半扫不到线 —— 但"到底是谁的问题"一试就知道。
REF_BEHAVIOR = {
    "roi_top": 0.50,
    "roi_bottom_ratio": 0.85,
    "look_top": 130.0 / 240.0,
    "look_bottom": 230.0 / 240.0,
    "target_ratio": 0.5,
    "single_mode": "border",
    "gains": "width",
    "auto_roi": False,
    "auto_look": False,
    "auto_target": False,
}


# ================================================================ 叠加图
def annotate(frame: np.ndarray, r: Optional[LaneReading], params: LaneParams,
             phase: str, extra: str = "", cones=None) -> np.ndarray:
    """把"算法到底看到了什么"画在图上（存盘用；本车没有 X 服务器，只能事后看）。"""
    vis = frame.copy()
    if cones:
        for c in cones:
            x0, y0, x1, y1 = (int(v) for v in c.xyxy)
            cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 0, 0), 2)
            cv2.putText(vis, f"cone {c.conf:.2f}", (x0, max(0, y0 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
    h, w = vis.shape[:2]
    y0, y1 = roi_bounds(h, params)
    ya, yb = look_bounds(h, params)
    for y, color in ((y0, (255, 128, 0)), (y1, (255, 128, 0)),
                     (ya, (0, 200, 255)), (yb, (0, 200, 255))):
        cv2.line(vis, (0, y), (w, y), color, 1)
    cv2.line(vis, (int(w * params.target_ratio), 0),
             (int(w * params.target_ratio), h), (255, 0, 255), 1)
    if r is not None:
        for kb, color in ((r.left_kb, (255, 80, 0)), (r.right_kb, (0, 0, 255))):
            if kb is None:
                continue
            p1 = (int(np.clip(x_of_kb(kb, 0.05 * h), 0, w - 1)), int(0.05 * h))
            p2 = (int(np.clip(x_of_kb(kb, 0.95 * h), 0, w - 1)), int(0.95 * h))
            cv2.line(vis, p1, p2, color, 2)
        if r.center_x is not None:
            cv2.line(vis, (int(r.center_x), 0), (int(r.center_x), h), (0, 255, 255), 1)
        cv2.putText(vis,
                    f"{phase} c={_fmt(r.center_x, '%.0f')} e={_fmt(r.error, '%+.0f')} "
                    f"q={r.quality:.2f} L={_fmt(r.left_x, '%.0f')} R={_fmt(r.right_x, '%.0f')} "
                    f"segs={r.n_left}/{r.n_right} {'双侧' if r.both_sides else '单侧'}",
                    (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        # 叠加图上只能画 ASCII：`cv2.putText` 画中文会变成一串 ??????
        # （2026-09-22 实验室实测发现），所以这里用读数的 tag，中文说明留在 CSV 里。
        if r.tag:
            cv2.putText(vis, r.tag, (6, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 255), 1, cv2.LINE_AA)
    if extra:
        cv2.putText(vis, extra, (6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 1, cv2.LINE_AA)
    return vis


def _fmt(v, fmt="%.1f") -> str:
    return "-" if v is None else fmt % v


def _sided_text(r) -> str:
    """状态行里的"双侧/单侧/无"——没读数时写"单侧"会误导人（2026-09-22 实验室实测发现）。"""
    if r is None or r.center_x is None:
        return "无"
    return "双侧" if r.both_sides else "单侧"


class Recorder:
    """CSV + 定时存图 + 结束统计（把复盘要用的东西一次留住）。"""

    def __init__(self, csv_path: str, save_dir: str, save_every_s: float,
                 save_max: int, annotate_on: bool = True) -> None:
        self.csv_path = csv_path
        self.save_dir = save_dir
        self.save_every_s = float(save_every_s)
        self.save_max = int(save_max)
        self.annotate_on = annotate_on
        self._fh = None
        self._n_saved = 0
        self._next_save = 0.0
        self.phases: Dict[str, int] = {}
        self.errors: List[float] = []
        self.errors_filt: List[float] = []
        self.qualities: List[float] = []
        self.n_frames = 0
        self.n_both = 0
        self.n_single = 0
        self.n_no_center = 0
        # ★ 顺序要紧：**先建目录再开文件**。CSV 默认就写在会话目录里
        #   （`<save_dir>/run.csv`），旧版先 open 后 makedirs → 车上一跑就
        #   FileNotFoundError（2026-09-22 实验室实测；本地集成测试因为用了已存在的
        #   临时目录而没抓到 —— 所以这里必须能容忍"目录还不存在"）。
        if save_dir:
            try:
                os.makedirs(os.path.join(save_dir, "setup"), exist_ok=True)
            except OSError as exc:
                print(f"[REF] ⚠️ 建会话目录失败（{exc}）→ 本次不落盘，车照常跑")
                self.save_dir = save_dir = ""
        if csv_path:
            try:
                parent = os.path.dirname(os.path.abspath(csv_path))
                if parent:
                    os.makedirs(parent, exist_ok=True)
                self._fh = open(csv_path, "w", encoding="utf-8")
                self._fh.write(CSV_HEADER)
            except OSError as exc:
                print(f"[REF] ⚠️ 打不开 CSV {csv_path}（{exc}）→ 本次不写 CSV，车照常跑")
                self._fh = None
                self.csv_path = ""

    def row(self, t: float, phase: str, r: Optional[LaneReading],
            error_filt: Optional[float], target_px: float, angle: float,
            esc_us: float, since_valid: float) -> None:
        self.n_frames += 1
        self.phases[phase] = self.phases.get(phase, 0) + 1
        if r is None or r.center_x is None:
            self.n_no_center += 1
        elif r.both_sides:
            self.n_both += 1
        else:
            self.n_single += 1
        if r is not None:
            self.qualities.append(r.quality)
            if r.error is not None:
                self.errors.append(r.error)
        if error_filt is not None:
            self.errors_filt.append(error_filt)
        if self._fh is None:
            return

        def num(v, fmt="%.1f"):
            return "" if v is None else fmt % v

        self._fh.write("%.3f,%s,%s,%s,%s,%s,%.1f,%.0f,%s,%s,%d,%d,%.2f,%d,%s,%d,%d,%.1f,%.2f,%s\n" % (
            t, phase, num(r.center_x if r else None), num(r.error if r else None),
            num(error_filt), "%.1f" % target_px, angle, esc_us,
            num(r.left_x if r else None), num(r.right_x if r else None),
            (r.n_left if r else 0), (r.n_right if r else 0), (r.quality if r else 0.0),
            (1 if (r and r.both_sides) else 0),
            num(r.half_width if r else None, "%.0f"),
            (1 if (r and r.width_measured) else 0), (1 if (r and r.anchored) else 0),
            (r.edge_pct * 100 if r else 0.0),
            (0.0 if since_valid == INF else since_valid),
            ((r.note if r else "") or "").replace(",", "；")))

    def wants_save(self, t: float) -> bool:
        """这一帧要不要存图（先问再画叠加图 —— 画图也有成本，别每帧白画）。"""
        return bool(self.save_dir and self.save_every_s > 0
                    and self._n_saved < self.save_max and t >= self._next_save)

    def maybe_save(self, t: float, frame: np.ndarray, vis: np.ndarray, tag: str = "") -> None:
        """定时存一帧（默认 1s 一张）。⚠️ 不在**每帧**写盘：`cv2.imwrite` 是阻塞 I/O。"""
        if not self.wants_save(t):
            return
        self._next_save = t + self.save_every_s
        idx = self._n_saved
        self._n_saved += 1
        name = f"{idx:04d}{('_' + tag) if tag else ''}.jpg"
        try:
            cv2.imwrite(os.path.join(self.save_dir, "frame_" + name), frame)
            if self.annotate_on:
                cv2.imwrite(os.path.join(self.save_dir, "ann_" + name), vis)
        except Exception as exc:                       # 存图失败不该打断跑车
            print(f"[REF] ⚠️ 存图失败：{exc}")

    def save_setup(self, frames: List[np.ndarray], det: LaneRefDetector,
                   params: LaneParams) -> None:
        """起步前那几帧全存下来（它们是"该用哪条带"的证据，最有复盘价值）。"""
        if not self.save_dir:
            return
        for i, f in enumerate(frames):
            try:
                cv2.imwrite(os.path.join(self.save_dir, "setup", f"frame_{i:02d}.jpg"), f)
                if self.annotate_on:
                    cv2.imwrite(os.path.join(self.save_dir, "setup", f"ann_{i:02d}.jpg"),
                                annotate(f, det.detect(f), params, "setup"))
            except Exception as exc:
                print(f"[REF] ⚠️ 存 setup 图失败：{exc}")
                return

    def summary(self) -> dict:
        def stats(vals):
            if not vals:
                return {}
            a = np.asarray(vals, dtype=float)
            return {"n": int(a.size), "mean": float(a.mean()), "median": float(np.median(a)),
                    "p05": float(np.percentile(a, 5)), "p95": float(np.percentile(a, 95)),
                    "min": float(a.min()), "max": float(a.max()),
                    "mean_abs": float(np.abs(a).mean())}
        return {"n_frames": self.n_frames, "phases": dict(self.phases),
                "n_both_sides": self.n_both, "n_single_side": self.n_single,
                "n_no_center": self.n_no_center,
                "error_px": stats(self.errors), "error_filt": stats(self.errors_filt),
                "quality": stats(self.qualities),
                "saved_frames": self._n_saved, "save_dir": self.save_dir,
                "csv": self.csv_path}

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None


def _print_summary(s: dict, driven: bool) -> None:
    print("\n[REF] ===== 本次统计 =====")
    n = max(1, s["n_frames"])
    print(f"[REF] 帧数 {s['n_frames']}；阶段分布 "
          + " ".join(f"{k}={v}" for k, v in sorted(s["phases"].items())))
    ok_n = s["n_both_sides"] + s["n_single_side"]
    print(f"[REF] 有中心 {ok_n}/{s['n_frames']}（双侧 {s['n_both_sides']}、"
          f"单侧兜底 {s['n_single_side']}、无 {s['n_no_center']}）→ 可用率 {ok_n / n * 100:.0f}%")
    for key, label in (("error_px", "原始误差"), ("error_filt", "滤波后误差")):
        st = s[key]
        if st:
            print(f"[REF] {label}：中位 {st['median']:+.1f}px  平均|e| {st['mean_abs']:.1f}px  "
                  f"p05~p95 {st['p05']:+.0f}~{st['p95']:+.0f}  极值 {st['min']:+.0f}~{st['max']:+.0f}")
    if s["quality"]:
        print(f"[REF] 质量：中位 {s['quality']['median']:.2f}（低于 --min-quality 控制层不跟线）")
    if s["save_dir"]:
        print(f"[REF] 落盘：{s['saved_frames']} 张图 → {s['save_dir']}；CSV → {s['csv']}")
        print(f"[REF] 拉回本地复盘：python scripts/ssh_get.py -r {s['save_dir']} .")
        print("[REF] 本地重跑：python scripts/lane_ref_test.py --replay <本地目录> --roi-sweep")
    if driven and ok_n == 0:
        print("[REF] ⚠️ 全程没有一次可信读数 → 先解决【看得到线】：压镜头角度 / 调带"
              "（lane_roi_*，下一轮自检会自己选）/ 看 setup 目录里的原图确认线是否进画面")


# ================================================================ 起步前自检
def _preflight(frames: List[np.ndarray], args, params: LaneParams) -> Tuple[LaneParams, Optional[float]]:
    """车静止时把【该用哪条带 / 前瞻带在哪 / 目标点多少 / 镜头够不够】一次算完。

    这是上场地时唯一需要人工配合的动作：**别挡板，让镜头看赛道**。
    返回 (最终参数, 标定出的目标点)。
    """
    if not frames:
        return params, None
    weak = False        # 自检证据够不够（不够就不采纳自检出的目标点，见 WEAK_RUN_MAX）
    if not args.auto_roi:
        print(f"[REF] ① 候选带扫描：已关闭（--no-auto-roi），用配置的 "
              f"{params.roi_top_ratio:.2f}~{params.roi_bottom_ratio:.2f}")
    else:
        print(f"[REF] ① 候选带扫描（{len(frames)} 帧静止画面，每档独立检测器；约 5~10 秒）")
        print("[REF]    ROI上  ROI下   y范围      有中心  连续支持  支持行占比  中心跨距  质量")
        table = sweep_bands(frames, params)
        best = None
        for row in table:
            mark = ""
            is_cfg = (abs(row["roi_top_ratio"] - params.roi_top_ratio) < 1e-6
                      and abs(row["roi_bottom_ratio"] - params.roi_bottom_ratio) < 1e-6)
            is_ref = (abs(row["roi_top_ratio"] - REF_BAND[0]) < 1e-6
                      and abs(row["roi_bottom_ratio"] - REF_BAND[1]) < 1e-6)
            if best is None and row["paired_frames"] > 0:
                best, mark = row, " ←采用"
            elif is_cfg:
                mark = " ←当前配置"
            elif is_ref:
                mark = " ←参考实现那一档"
            print(f"[REF]    {row['roi_top_ratio']:.2f}  {row['roi_bottom_ratio']:.2f}   "
                  f"{row['y_range'][0]:>3}~{row['y_range'][1]:<3}   "
                  f"{row['paired_frames']:>2}/{row['n_frames']:<2}   "
                  f"{row['run_max']:>5.0f}行  "
                  f"{row['support_frac'] * 100:>7.0f}%   "
                  f"{row['spread_px']:>5.0f}px   {row['q_med']:.2f}{mark}")
        # ★ 证据太弱时不采纳任何"自检出来的值"：锁到的可能不是一整条连续白线
        #   （碎边/反光/别的边也能被拟合出线）。宁可用上一次现场标定的目标点，
        #   也不要照着一个错目标把车放出去跑（2026-09-22 实验室实测踩过：
        #   车摆偏时自检把目标标成 417/291，真值约 341，车就照着错目标走）。
        #   ⚠️ 两个分支都必须置位 weak（这里踩过一次：只置了 else 分支 → 警告打印了、
        #   但目标点照样被采纳并写回 site.yaml）。
        if best is None:
            weak = True
            print("[REF]    ❌ 所有候选带都锁不到两条线 → 保留当前配置；先解决画面里有没有线"
                  "（看 setup 目录的原图，或跑 lane_probe 的镜头朝向体检）")
        elif best["run_max"] < WEAK_RUN_MAX:
            weak = True
            print(f"[REF]    ⚠️ 最好的一档也只有 {best['run_max']:.0f} 行连续支持（< "
                  f"{WEAK_RUN_MAX}）→ 证据不足：**保留 site.yaml 的带与目标点**"
                  f"（不采纳自检值、不写回 site.yaml）。请重新摆正车（车头朝赛道、尽量居中）"
                  f"再跑一次；每次都这样就是镜头视角问题（先把下摄往下压）")
        else:
            params.roi_top_ratio = best["roi_top_ratio"]
            params.roi_bottom_ratio = best["roi_bottom_ratio"]
            print(f"[REF]    → 采用带 {params.roi_top_ratio:.2f}~{params.roi_bottom_ratio:.2f}"
                  f"（y {best['y_range'][0]}~{best['y_range'][1]}，"
                  f"{best['paired_frames']}/{best['n_frames']} 帧成对，"
                  f"最长连续支持 {best['run_max']:.0f} 行）")

    if args.auto_look:
        look, rows, info = auto_look_band(frames, params)
        if rows:
            params.look_top_ratio, params.look_bottom_ratio = look
            h = frames[0].shape[0]
            lo, hi, n = info["longest_seg"]
            print(f"[REF] ② 自动前瞻带：支持行 {min(rows)}~{max(rows)}，"
                  f"最长连续段 y {lo}~{hi}（{n} 行）→ 用它中间 20%~80%（y "
                  f"{int(look[0] * h)}~{int(look[1] * h)}）算中心（look "
                  f"{look[0]:.2f}~{look[1]:.2f}）")
            if n < 40:
                print(f"[REF]    ⚠️ 最长连续段只有 {n} 行 —— 线不是一整段，中心会跳；"
                      f"建议重新摆车/压镜头")
        else:
            print("[REF] ② 自动前瞻带：没有找到两条线都有边缘支持的行 → 用配置的 "
                  f"{params.look_top_ratio:.2f}~{params.look_bottom_ratio:.2f}"
                  f"（要手改就是 --look-top/--look-bottom）")
    else:
        print(f"[REF] ② 自动前瞻带：已关闭（--no-auto-look），用 "
              f"{params.look_top_ratio:.2f}~{params.look_bottom_ratio:.2f}")

    det = LaneRefDetector(params)
    centers: List[float] = []
    for f in frames:
        r = det.detect(f)
        if r.center_x is not None and r.quality >= args.min_quality:
            centers.append(float(r.center_x))
    target = None
    if centers and args.auto_target and not weak:
        target = float(np.median(centers))
        # ★ 必须先记下"自检开始前"的目标点：执行下面这行会把 params.target_ratio 覆盖成自检值，
        #   在它**之后**再取 configured 会拿到自检值本身 → "不采纳"形同虚设、还照样写回
        #   site.yaml（2026-09-22 实测 191511：提示"不采纳"，目标点 352.3 还是被用了/写了）。
        configured = params.target_ratio * frames[0].shape[1]
        det.p.target_ratio = target / float(frames[0].shape[1])
        spread = max(centers) - min(centers)
        print(f"[REF] ③ 自动标定目标点：{target:.1f}px（{len(centers)} 帧中位，跨距 "
              f"{spread:.0f}px，半宽先验 {det.width_prior.value:.0f}px"
              f"{'实测' if det.width_prior.measured else '种子'}）")
        if abs(target - configured) > 60:
            print(f"[REF]    ⚠️ 与配置里的目标点（{configured:.0f}px）差了 "
                  f"{abs(target - configured):.0f}px —— 要么这次自检时车没摆正，要么现场视角变了。"
                  f"**先确认车是不是真的在车道正中**，再决定要不要用它")
        # ★ 静止时中心跨距大 = 车没摆正 / 锁的东西在跳 → 这帧数标定的目标点不可信，
        #   照用的话车会对着一个错目标跑（2026-09-22 实验室连跑三次：跨距 47~166px，
        #   目标点跟着每跑一次变一次 318→348→354）。当作"证据弱"处理：不采纳、也不写 site.yaml。
        if spread > WEAK_SPREAD_PX:
            weak = True
            target = None
            det.p.target_ratio = configured / float(frames[0].shape[1])   # 还原
            print(f"[REF]    ⚠️ 静止时中心就跳了 {spread:.0f}px（> {WEAK_SPREAD_PX:.0f}）"
                  f"→ **不采纳这个目标点**、不写回 site.yaml（保留旧值）。"
                  f"车没摆正 / 线不成一整段 / 两条线选来选去 —— 重新摆车再跑一次自检")
    elif not args.auto_target:
        print(f"[REF] ③ 自动标定：已关闭（--no-auto-target），目标点 "
              f"{params.target_ratio * frames[0].shape[1]:.0f}px")
    elif weak:
        print(f"[REF] ③ 自动标定：自检证据弱（{_weak_reason(args, params)}）→ 用配置里的目标点 "
              f"{params.target_ratio * frames[0].shape[1]:.0f}px")
    else:
        print("[REF] ③ ❌ 自动标定没成（没有质量达标的帧）→ 用配置里的目标点 "
              f"{params.target_ratio * frames[0].shape[1]:.0f}px")

    aim = det.visible_range(frames[-1])
    print(f"[REF] ④ 镜头朝向体检：{aim['text']}")
    if not aim["ok"]:
        print("[REF]    （体检不通过也还能跑：带扫描如果锁到了线，说明线在画面里，"
              "只是没进下半部。要根治就是机械压角度/换广角）")
    return params, target, not weak


def apply_steer_cap(center: float, steer: float, both_sides: bool,
                     n_left: int, n_right: int,
                     single_limit_deg: float = 5.0, weak_limit_deg: float = 10.0) -> float:
    """对不可靠读数的舵角输出做额外限幅：

      · 单侧兜底帧（中心是"可见线+半宽"推的）→ ±single_limit_deg（±5°）
      · 双侧但在的一侧线段 ≤2 的"弱对"帧 → ±weak_limit_deg（±10°）
        2026-09-23 操场实测：弱对帧的误差一帧可跳 50~140px（占大跳变的 60%），
        不限幅的话 kp+kd 会把这种假误差放大成满舵，车被自己的读数甩动。
      双侧且两侧都有 ≥3 段的帧 → 不额外限幅（正常跟线）。
    返回限幅后的舵角。"""
    if both_sides and min(n_left, n_right) >= 3:
        return steer
    cap = single_limit_deg if not both_sides else weak_limit_deg
    if abs(steer - center) > cap:
        return center + np.sign(steer - center) * cap
    return steer


def guard_trip_text(err_px: float, hold_s: float,
                     err_c: Optional[float], err_raw: Optional[float]) -> str:
    """跑偏保护触发时的打印：**先区分触发原因**再报数。

    2026-09-22 用户反复遇到"车已经修正了还是停"——真正的机理是：**目标点错 → 车在车道正中
    也读出恒大的误差 → 保护把"追不上错目标"当成了"跑偏"**。为了以后一眼分得清，这里区分：
      · err_c（滤波后、质量够的）也超标 → "判定跑偏/锁错线"（读数可信、车真的偏了）；
      · 只有原始误差超标（ERR_C 为 None 或不足 40px）→ "读数持续不可信"（翻车/单侧兜底垃圾），
        保护按"宁可停"兜住它而不是当跑偏。
    """
    base = (f"⚠️ 连续 {hold_s:.1f}s 读数 ≥{err_px:.0f}px"
            f"（滤波后 {_fmt(err_c, '%+.1f')}，原始 {_fmt(err_raw, '%+.1f')}）")
    if err_c is not None and abs(float(err_c)) >= err_px:
        return (f"[REF] {base} → **判定跑偏/锁错线**：读数可信但车真的偏了/锁错线，停车。"
                f"若每次都是同一方向，先查目标点（site.yaml 的 target_x）与转向符号")
    return (f"[REF] {base} → **读数持续不可信**（原始误差很大但滤波后值没跟上/质量不够）"
            f"→ 保护按『宁可停』兜住，停车。这是信号质量/锁翻车的问题，不是跑偏")


def _weak_reason(args, params: LaneParams) -> str:
    """自检为什么被判定为"证据弱"（打印给操作员的理由）。"""
    return (f"最长连续支持 < {WEAK_RUN_MAX} 行 或 静止中心跨距 > {WEAK_SPREAD_PX:.0f}px"
            f"（本次用 带 {params.roi_top_ratio:.2f}~{params.roi_bottom_ratio:.2f}、"
            f"目标 {params.target_ratio * 640:.0f}px）")


def _write_site(args, params: LaneParams, target_px: Optional[float]) -> None:
    """把自检结果写回 `config/site.yaml`（车端本地、不进 git）——下次跑车直接用。"""
    if not args.save_site:
        print("[REF] ⑤ --no-save-site：自检结果只在本次生效")
        return
    from config import site
    vals = {"lane_roi_top_ratio": round(float(params.roi_top_ratio), 3),
            "lane_roi_bottom_margin": int(round((1.0 - float(params.roi_bottom_ratio))
                                                * float(settings.IMG_H)))}
    if target_px is not None:
        vals["target_x"] = round(float(target_px), 1)
    try:
        path = site.save(vals)
        print(f"[REF] ⑤ 已写入 {path}：{vals}（改错了就把这几行删掉）")
    except Exception as exc:
        print(f"[REF] ⚠️ 写 site.yaml 失败（本次仍用自检值）：{exc}")


# ================================================================ 离线模式
def analyze_image(path: str, params: LaneParams, args=None) -> int:
    img = cv2.imread(path)
    if img is None:
        # Windows 本地验证时中文路径 cv2.imread 读不了 → 用 fromfile 兜底
        img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        print(f"[REF] 读不到图片 {path}")
        return 1
    det = LaneRefDetector(params)
    r = det.detect(img)
    h, w = img.shape[:2]
    print(f"[REF] {path}  ({w}x{h})")
    print(f"[REF] 带 {params.roi_top_ratio:.2f}~{params.roi_bottom_ratio:.2f} "
          f"(y {roi_bounds(h, params)[0]}~{roi_bounds(h, params)[1]})  "
          f"前瞻 y {look_bounds(h, params)[0]}~{look_bounds(h, params)[1]}")
    print(f"[REF] 边缘占比={r.edge_pct * 100:.1f}%  Canny=({r.canny[0]:.0f},{r.canny[1]:.0f})  "
          f"线段 左{r.n_left}/右{r.n_right}  锚定={r.anchored}")
    print(f"[REF] 左={_fmt(r.left_x)} 右={_fmt(r.right_x)} 中心={_fmt(r.center_x)} "
          f"目标={w * params.target_ratio:.0f} 误差={_fmt(r.error)}")
    print(f"[REF] 质量={r.quality:.2f} 双侧={r.both_sides} "
          f"半宽={_fmt(r.half_width, '%.0f')}px{'实测' if r.width_measured else '种子'}"
          f"{'  ' + r.note if r.note else ''}")
    print(f"[REF] 镜头朝向体检：{det.visible_range(img)['text']}")
    # 锥桶检测（单图验证；hsv 不依赖权重，model 需要车上有 rknn 权重）
    if args is not None and getattr(args, "cone_method", "off") != "off":
        from vision.cone_detect import ConeDetector
        cone_det = ConeDetector(method=args.cone_method, model_path=args.cone_model or None)
        cones = cone_det.detect(img)
        if cones:
            for c in cones:
                print(f"[REF] 锥桶({args.cone_method}): x={c.center_x:.0f} y底={c.bottom_y:.0f} "
                      f"{c.width:.0f}x{c.height:.0f} conf={c.conf:.2f}")
        elif cone_det.method == "hsv":
            # 部署 ROI 没检出，但全图可能检出（比如训练图是手持朝前拍、锥桶在上半画面）：
            # 提示“锥桶在 ROI 外”，区分“检测器不行”和“锥桶没进部署区域”
            from vision.cone_detect import detect_cones_hsv
            all_cones = detect_cones_hsv(img, roi=(0.0, 1.0, 0.0, 1.0))
            if all_cones:
                print(f"[REF] 锥桶: 部署 ROI 内无；全图检出 "
                      f"{[f'x={c.center_x:.0f} y={c.xyxy[1]:.0f}~{c.xyxy[3]:.0f}' for c in all_cones[:3]]}"
                      f" —— 锥桶在 ROI 外（现场把锥桶放进画面中下部再测）")
            else:
                print("[REF] 锥桶: 无（hsv 通道全图也没检出）")
        else:
            print("[REF] 锥桶: 无（model 通道）")
    return 0


def replay(directory: str, params: LaneParams, args, steer_sign: float,
           gains: Tuple[float, float, float]) -> int:
    """拿落盘的帧在本地重跑检测 + 控制律 —— 不用再去一趟操场就能调参。"""
    setup = sorted(glob.glob(os.path.join(directory, "setup", "frame_*.jpg")))
    files = sorted(glob.glob(os.path.join(directory, "frame_*.jpg")))
    paths = setup or files
    if not paths:
        print(f"[REF] {directory} 里没有 frame_*.jpg（给会话目录，不是给它下面的 setup/）")
        return 1
    frames = [im for im in (cv2.imread(p) for p in paths) if im is not None]
    if not frames:
        print(f"[REF] {directory} 里的图都读不出来（文件坏了？）")
        return 1
    print(f"[REF] 复盘 {len(frames)} 帧（{'起步前自检帧' if setup else '实跑帧'}）："
          f"{os.path.basename(os.path.normpath(directory))}")
    if args.roi_sweep:
        print("[REF] 候选带扫描（用这些真实帧重算）：")
        print("[REF]    ROI上  ROI下   y范围      有中心  连续支持  支持行占比  中心跨距  质量")
        for row in sweep_bands(frames, params):
            print(f"[REF]    {row['roi_top_ratio']:.2f}  {row['roi_bottom_ratio']:.2f}   "
                  f"{row['y_range'][0]:>3}~{row['y_range'][1]:<3}   "
                  f"{row['paired_frames']:>2}/{row['n_frames']:<2}   "
                  f"{row['run_max']:>5.0f}行  "
                  f"{row['support_frac'] * 100:>7.0f}%   "
                  f"{row['spread_px']:>5.0f}px   {row['q_med']:.2f}")
    det = LaneRefDetector(params)
    err_filter = None if args.no_filter else ErrorFilter(args.filter_window, args.filter_outlier)
    kp, ki, kd = gains
    pid = PidRef(kp, ki, kd, args.limit_deg, args.smooth, sign=steer_sign)
    rows = []
    for f in frames:
        r = det.detect(f)
        ef = err_filter.update(r.error) if (err_filter is not None and r.error is not None) \
            else r.error
        angle = pid.step(ef) if (ef is not None and abs(ef) < args.err_stop_px) else 90.0
        rows.append((r, ef, angle))
    both = sum(1 for r, _, _ in rows if r.both_sides)
    single = sum(1 for r, _, _ in rows if r.center_x is not None and not r.both_sides)
    print(f"[REF] 检测：{both + single}/{len(rows)} 帧有中心（双侧 {both}、单侧兜底 {single}）")
    errs = [r.error for r, _, _ in rows if r.error is not None]
    if errs:
        a = np.asarray(errs)
        print(f"[REF] 原始误差：中位 {np.median(a):+.1f}px  p05~p95 {np.percentile(a, 5):+.0f}~"
              f"{np.percentile(a, 95):+.0f}  极值 {a.min():+.0f}~{a.max():+.0f}")
    angles = np.asarray([ang for _, _, ang in rows], dtype=float)
    if angles.size:
        sat = float(np.mean(np.abs(angles - 90.0) >= args.limit_deg - 1e-6)) * 100
        step = np.abs(np.diff(angles)) if angles.size > 1 else np.zeros(1)
        flips = int(np.sum(np.diff(np.sign(angles - 90.0)) != 0))
        print(f"[REF] 控制律（kp={kp} ki={ki} kd={kd} 限幅±{args.limit_deg}°）："
              f"平均|舵角−90|={np.mean(np.abs(angles - 90.0)):.1f}°  打满限幅 {sat:.0f}%  "
              f"单帧最大变化 {step.max():.1f}°  方向翻转 {flips} 次")
        print("[REF]   （翻转多=抖动 → 加 kd 或降 kp；一直打满/越修越偏 → 目标点或转向方向不对）")
    return 0


# ================================================================ 实跑
def main() -> int:
    ap = argparse.ArgumentParser(
        description="循迹实跑（参考实现版；起步前自动自检 + 全程落盘）")
    ap.add_argument("--image", default="", help="只分析一张图（本地可跑，不碰硬件）")
    ap.add_argument("--replay", default="", help="复盘一个会话目录里的帧（本地可跑，不碰硬件）")
    ap.add_argument("--roi-sweep", action="store_true", help="复盘时顺带重算候选带表")
    ap.add_argument("--camera", type=int, default=2, help="摄像头（默认 2=下摄，巡线固定这一路）")
    ap.add_argument("--speed-us", type=float, default=float(settings.ESC_CREEP_US),
                    help=f"循迹脉宽（默认 {settings.ESC_CREEP_US}；死区 "
                         f"{settings.ESC_DEADBAND_US}，调试上限 {settings.ESC_DEBUG_MAX_US}）")
    ap.add_argument("--speed-fast", type=float, default=SPEED_FAST_US,
                    help=f"误差小(<5px)时的脉宽增量（默认 {SPEED_FAST_US:+.0f}，0=不提速）")
    ap.add_argument("--speed-slow", type=float, default=SPEED_SLOW_US,
                    help=f"误差大(≥15px)时的脉宽增量（默认 {SPEED_SLOW_US:+.0f}，"
                         f"下限=死区+{SPEED_FLOOR_MARGIN_US:.0f}us）")
    ap.add_argument("--start-us", type=float, default=float(settings.ESC_CREEP_US),
                    help="起步/探路脉宽")
    ap.add_argument("--max-seconds", type=float, default=120.0, help="整段硬上限")
    ap.add_argument("--kp", type=float, default=None,
                    help="PID 比例项（不给我就按 --gains 从参考实现推，见该参数说明）")
    ap.add_argument("--ki", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None, help="抖动/画龙 → 加大到 0.3")
    ap.add_argument("--gains", choices=("literal", "width"), default=None,
                    help="参考实现的增益是**按 320 宽画面**（oldCode 是 320×240）整定的："
                         "literal=直接抄它的数字 0.15/0.01/0.12（默认，现场调参表按这套写）；"
                         "width=按画面宽度换算成 640 口径 0.075/0.005/0.06"
                         "（'同一个物理横偏 → 同一个舵角'）。")
    ap.add_argument("--limit-deg", type=float, default=LIMIT_DEG0, help="舵角限幅（默认 15°）")
    ap.add_argument("--smooth", type=float, default=0.7, help="输出平滑：0.7*新+0.3*旧")
    ap.add_argument("--target-ratio", type=float, default=None,
                    help=f"目标点比例（只在 --no-auto-target 时生效；默认 {DEFAULT_TARGET_RATIO:.3f}）")
    ap.add_argument("--auto-target", dest="auto_target", action="store_true", default=None,
                    help="起步前自动标定目标点（默认开）")
    ap.add_argument("--no-auto-target", dest="auto_target", action="store_false")
    ap.add_argument("--roi-top", type=float, default=None,
                    help="ROI 上沿比例（默认取 settings ← site.yaml；一般不用手给，自检会选）")
    ap.add_argument("--roi-bottom-ratio", type=float, default=None,
                    help="ROI 下沿比例（默认 1 − LANE_ROI_BOTTOM_MARGIN/480）")
    ap.add_argument("--look-top", type=float, default=None, help="前瞻带上沿比例（算中心的行）")
    ap.add_argument("--look-bottom", type=float, default=None, help="前瞻带下沿比例")
    ap.add_argument("--auto-roi", dest="auto_roi", action="store_true", default=None,
                    help="起步前扫候选带并采用最好的一档（默认开）")
    ap.add_argument("--no-auto-roi", dest="auto_roi", action="store_false")
    ap.add_argument("--auto-look", dest="auto_look", action="store_true", default=None,
                    help="用实测【有边缘支持的行】定前瞻带（默认开）")
    ap.add_argument("--no-auto-look", dest="auto_look", action="store_false")
    ap.add_argument("--ref-behavior", action="store_true",
                    help="★对照实验：按参考实现 oldCode 的原样跑（它 320×240：ROI 0.50~0.85、"
                         "算中心的行 130~230、目标点=画面中心、单侧代画面边界、增益按画面换算），"
                         "并把自检（自动选带/前瞻带/目标点）关掉。用来回答'到底是我们的改动、"
                         "还是镜头视角不行'")
    ap.add_argument("--save-site", dest="save_site", action="store_true", default=True,
                    help="把自检结果写回 config/site.yaml（默认开：下次跑车直接用）")
    ap.add_argument("--no-save-site", dest="save_site", action="store_false")
    ap.add_argument("--pick", choices=("steepest", "longest"), default="steepest",
                    help="每侧选哪条线段：steepest=参考实现；longest=更稳（备选）")
    ap.add_argument("--single-mode", choices=("width", "border", "off"), default="width",
                    help="单侧丢线怎么办：width=用车道半宽先验推中心（默认，本车方案）；"
                         "border=**参考实现原样**（缺的一侧代画面边界）；off=不给中心")
    ap.add_argument("--no-single", dest="single_mode", action="store_const", const="off",
                    help="等价于 --single-mode off（最保守：单侧就不跟线）")
    ap.add_argument("--half-w", type=float, default=None,
                    help=f"车道半宽种子（像素，默认 settings.LANE_HOUGH_HALF_W_DEFAULT_PX="
                         f"{settings.LANE_HOUGH_HALF_W_DEFAULT_PX}）；两侧都在时会自动更新为实测值")
    ap.add_argument("--min-quality", type=float, default=MIN_QUALITY0,
                    help=f"质量低于它按丢线处理（默认 {MIN_QUALITY0}）")
    ap.add_argument("--acquire-s", type=float, default=3.0,
                    help="发车后【还没见过一次线】的低速探路窗口（秒）")
    ap.add_argument("--hold-s", type=float, default=HOLD_S0,
                    help=f"丢线维持窗口（默认 {HOLD_S0}s：沿用上次中心、误差衰减回中位）")
    ap.add_argument("--single-limit-deg", type=float, default=5.0,
                    help="**单侧兜底帧**的舵角限幅（默认 ±5°）：运动时只靠一侧线推出来的中心"
                         "经常是错的，限幅小它就没法把车甩起来（2026-09-23 操场 S 弯主振荡源）")
    ap.add_argument("--weak-limit-deg", type=float, default=10.0,
                    help="**弱对帧**（双侧但在的一侧 ≤2 段）的舵角限幅（默认 ±10°）：这类帧的"
                         "中心一跳 50~140px（占大跳变的 60%），限小一点就不至于被自己的读数甩动")
    ap.add_argument("--err-stop-px", type=float, default=60.0, help="跑偏保护：误差阈值（像素）")
    ap.add_argument("--err-stop-s", type=float, default=1.5,
                    help="跑偏保护：连续这么多秒大误差就停车（0=关闭）")
    ap.add_argument("--steer-sign", type=float, default=None,
                    help=f"转向符号：+1=角度增大是右转；默认取 settings（当前 "
                         f"{settings.STEER_SIGN:+.0f}）。先跑 bench_test --steer-test 实测方向")
    ap.add_argument("--no-filter", action="store_true", help="关掉误差滤波")
    ap.add_argument("--filter-window", type=int, default=None)
    ap.add_argument("--filter-outlier", type=float, default=None,
                    help=f"离群阈值（**像素**，默认 "
                         f"{settings.ERROR_FILTER_OUTLIER * settings.LANE_ERROR_SCALE:.0f}px）")
    ap.add_argument("--no-motor", action="store_true", help="只跑视觉与决策，不碰电机")
    ap.add_argument("--allow-motion", "--i-know-wheels-are-up", dest="allow_motion",
                    action="store_true", help="确认车可以移动")
    ap.add_argument("--log-csv", default="", help="CSV 路径（默认写进会话目录）")
    ap.add_argument("--save-dir", default="",
                    help="会话目录（默认 /root/dev/logs/lane_<时间戳>；存图用 --save-every-s 0 关）")
    ap.add_argument("--save-every-s", type=float, default=1.0,
                    help="每隔多少秒存一帧原图+叠加图（0=不存图；写盘是阻塞 I/O，别调太小）")
    ap.add_argument("--save-max", type=int, default=400, help="一次最多存多少帧（防塞满卡）")
    ap.add_argument("--setup-frames", type=int, default=SETUP_FRAMES,
                    help=f"起步前自检采几帧（默认 {SETUP_FRAMES}）")
    ap.add_argument("--setup-timeout-s", type=float, default=15.0,
                    help="等自检画面的上限（秒）；没凑够就先用手上的帧")
    ap.add_argument("--read-fail-s", type=float, default=8.0,
                    help="摄像头连续读失败超过这么久就报错退出（默认 8s）——"
                         "相机掉线/被抢走时不要傻等到 --max-seconds")
    ap.add_argument("--print-every", type=int, default=3)
    # ---- 锥桶检测与 S 型避让（P1-3 雏形；默认关，不影响纯循迹复测）----
    ap.add_argument("--cone-method", choices=("hsv", "model", "off"), default="off",
                    help="锥桶检测通道：hsv=蓝色连通域（参考实现同源，本地可验、不依赖权重）；"
                         "model=RKNN car4cls 检测（车上有权重时更抗环境误检，权重路径用 "
                         "--cone-model 给）；off=关闭（默认）")
    ap.add_argument("--cone-model", default="",
                    help="--cone-method model 时的 RKNN 权重路径（默认取 "
                         "/root/dev/models/ 下的 car4cls 权重）")
    ap.add_argument("--cone-avoid", action="store_true",
                    help="开启锥桶 S 型避让（检测到锥桶 → 左绕→回正→右绕→回 track）")
    ap.add_argument("--cone-stop", action="store_true",
                    help="实验室模式：检测到锥桶就停车（锥桶移开可继续前进），不做绕行")
    ap.add_argument("--cone-avoid-deg", type=float, default=None,
                    help=f"绕行舵角偏离中位的幅度（默认 settings.CONE_AVOID_DEG="
                         f"{settings.CONE_AVOID_DEG:.0f}°）")
    ap.add_argument("--cone-avoid-s", type=float, default=None,
                    help=f"单段绕行时长秒（默认 settings.CONE_AVOID_S={settings.CONE_AVOID_S:.1f}）")
    ap.add_argument("--cone-resume-s", type=float, default=None,
                    help=f"两段绕行间的回正直行秒（默认 settings.CONE_SEGUE_S="
                         f"{settings.CONE_SEGUE_S:.1f}）")
    ap.add_argument("--cone-first-dir", choices=("left", "right"), default="left",
                    help="S 型第一段往哪边绕（默认 left：左绕→回正→右绕；"
                         "第二个锥桶在另一边时不用改，两段方向相反即成 S）")
    args = ap.parse_args()

    # ---- 参数来源：命令行 > --ref-behavior 预设 > settings/site.yaml（ROI/前瞻带只有一处定义）
    def opt(name: str, default):
        """命令行优先；没给就看 --ref-behavior 预设；再没有才用默认。"""
        value = getattr(args, name)
        if value is not None:
            return value
        return REF_BEHAVIOR.get(name, default) if args.ref_behavior else default

    for name in ("auto_roi", "auto_look", "auto_target"):
        setattr(args, name, bool(opt(name, True)))     # 后面的自检读 args.* 判断
    single_mode = str(opt("single_mode", "width"))
    gains_mode = str(opt("gains", "literal"))
    target_ratio = float(opt("target_ratio", DEFAULT_TARGET_RATIO))
    half_w = (args.half_w if args.half_w is not None
              else float(settings.LANE_HOUGH_HALF_W_DEFAULT_PX))
    params = LaneParams.from_settings(
        roi_top_ratio=opt("roi_top", None), roi_bottom_ratio=opt("roi_bottom_ratio", None),
        look_top_ratio=opt("look_top", None), look_bottom_ratio=opt("look_bottom", None),
        target_ratio=target_ratio, pick=args.pick, single_mode=single_mode,
        half_w_seed=half_w)
    args.target_ratio = target_ratio
    steer_sign = float(settings.STEER_SIGN if args.steer_sign is None else args.steer_sign)
    # PID 增益：参考实现是按 320 宽画面整定的（oldCode 是 320×240）→ 口径要说清楚
    g_kp, g_ki, g_kd = gains_for(settings.IMG_W, gains_mode)
    kp = g_kp if args.kp is None else float(args.kp)
    ki = g_ki if args.ki is None else float(args.ki)
    kd = g_kd if args.kd is None else float(args.kd)
    if args.ref_behavior:
        print("[REF] ★ --ref-behavior：按参考实现原样跑（它的拍摄比例照搬到我们 640×480）")
        print(f"[REF]   ROI {params.roi_top_ratio:.2f}~{params.roi_bottom_ratio:.2f}、"
              f"算中心的行 {params.look_top_ratio * 480:.0f}~{params.look_bottom_ratio * 480:.0f}"
              f"（≈y 260~460）、目标点 {target_ratio * 640:.0f}px（画面中心）、"
              f"单侧代画面边界、PID {kp:.3f}/{ki:.3f}/{kd:.3f}")
        print("[REF]   这是**对照实验**：我们的下摄太平，近处是画外，这条 look band 多半扫不到线。"
              "跑完看自检/CSV 就能分清是'我们的改动'还是'视角'的问题")

    if args.replay:
        return replay(args.replay, params, args, steer_sign, (kp, ki, kd))
    if args.image:
        return analyze_image(args.image, params, args)

    use_motor = not args.no_motor
    if use_motor and not args.allow_motion:
        print("[REF] 拒绝运行：这次会驱动电机。确认四轮架空或场地空旷、有人在旁能立即断电，"
              "再加 --allow-motion；只想看读数就加 --no-motor")
        return 2
    if use_motor and min(args.start_us, args.speed_us) < settings.ESC_DEADBAND_US:
        print(f"[REF] 拒绝运行：脉宽低于电调死区 {settings.ESC_DEADBAND_US}us"
              f"（实测 1540 不转、{settings.ESC_DEADBAND_US} 起转）")
        return 2

    save_dir = args.save_dir
    if not save_dir:
        base = ("/root/dev/logs" if os.path.isdir("/root/dev/logs")
                else os.path.join(os.getcwd(), "logs"))
        save_dir = os.path.join(base, "lane_" + time.strftime("%Y%m%d_%H%M%S"))
    save_dir = os.path.abspath(save_dir)
    csv_path = args.log_csv or os.path.join(save_dir, "run.csv")

    from scripts.bench_common import MotorSession, install_signal_guard, restore_remote_stage
    state = {"stopped": False, "driver": None, "restored": False, "used_motor": use_motor}
    install_signal_guard(state)

    center_deg = float(settings.SERVO_CENTER_ANGLE)   # ★ 舵机中位（=机械直行角，现场标定）
    gate = StartGate()
    pid = PidRef(kp, ki, kd, args.limit_deg, args.smooth, sign=steer_sign,
                 center=center_deg)
    # ★ 滤波的离群阈值单位是**误差单位（1 单位 = 4px）**，而本脚本的误差是**像素**。
    #   上一版把 settings 的 15 直接当像素用 → 任何 >15px 的变化都被当离群点丢掉 →
    #   滤波后误差长期卡在旧值上，车对真实偏差不响应（实测读数在 345/367 间跳时尤其致命）。
    outlier_px = (args.filter_outlier if args.filter_outlier is not None
                  else float(settings.ERROR_FILTER_OUTLIER) * float(settings.LANE_ERROR_SCALE))
    err_filter = None if args.no_filter else ErrorFilter(args.filter_window, outlier_px)
    loss = LossGuard(args.hold_s)
    big_err = BigErrorGuard(args.err_stop_px, args.err_stop_s)
    osc = OscillationGuard()          # 画龙（S 弯）检测：舵角来回打满幅 → 停车

    print("[REF] 参考实现版循迹（起步前自检 → 板发车 → 循迹 → 再见板停车）")
    print(f"[REF] PID kp={kp:.3f} ki={ki:.3f} kd={kd:.3f} 限幅±{args.limit_deg}° "
          f"舵机中位={center_deg:.1f}°(servo_center_angle) 平滑={args.smooth} "
          f"目标比={params.target_ratio:.3f} 选线={args.pick} "
          f"转向符号={steer_sign:+.0f}"
          f"（{'+1 角度增大=右转' if steer_sign > 0 else '−1 角度增大=左转'}）")
    print(f"[REF] 脉宽 起步={args.start_us:.0f} → 循迹={args.speed_us:.0f}us"
          f"（直道 {args.speed_us + args.speed_fast:.0f} / 大误差 "
          f"{max(args.speed_us + args.speed_slow, float(settings.ESC_DEADBAND_US) + SPEED_FLOOR_MARGIN_US):.0f}"
          f" / 丢线维持 {floor_pulse(args.speed_us, args.speed_slow):.0f}）；"
          f"死区 {settings.ESC_DEADBAND_US}")
    print(f"[REF] 丢线阶梯：质量<{args.min_quality:.2f} 或没有读数 → 维持 {args.hold_s:.1f}s"
          f"（沿用上次中心、误差衰减回中位）→ 还没回来就停车；"
          f"发车后 {args.acquire_s:.1f}s 内一次没见过线按【低速探路】处理")
    print("[REF] 跑偏保护：滤波后误差 ≥%.0fpx 持续 %.1fs 就停车" % (args.err_stop_px, args.err_stop_s)
          if args.err_stop_s > 0 else "[REF] 跑偏保护：已关闭（--err-stop-s 0）")
    print("[REF] 误差滤波：" + ("关（--no-filter）" if err_filter is None else
          f"窗口 {err_filter.window} / 离群阈值 {outlier_px:.0f}px"))
    print(f"[REF] 落盘：{csv_path}"
          + (f"；每 {args.save_every_s:.1f}s 存一帧到 {save_dir}（上限 {args.save_max} 张）"
             if args.save_every_s > 0 else "；不存图"))
    print("[REF] ⚠️ 用法：① 车摆正在车道里、先别挡板（自检要看你前方的赛道）→ "
          "② 看到“✅ 起步前自检完成”→ ③ 把蓝板挡到下摄正前方 → ④ 移开板发车 → "
          "⑤ 再挡回板立刻停")

    rec = Recorder(csv_path, save_dir, args.save_every_s, args.save_max)
    rc = 0
    t0 = time.time()
    frames = 0
    n_track = 0
    try:
        with MotorSession(state, enabled=use_motor,
                          speed_us_max=max(args.start_us, args.speed_us)) as pca:
            with camera_exclusive():
                cap = open_camera(args.camera, settings.IMG_W, settings.IMG_H)
                if cap is None:
                    rc = 1
                else:
                    det = LaneRefDetector(params)
                    # ---- 锥桶检测（P1-3；hsv 本地可跑，model 用车端 RKNN car4cls）----
                    cone_det = None
                    cone_tracker = None
                    if args.cone_method != "off":
                        from vision.cone_detect import ConeDetector, ConeTracker
                        method = args.cone_method
                        model_path = args.cone_model
                        if method == "model" and not model_path:
                            import glob as _glob
                            cands = sorted(_glob.glob("/root/dev/models/*car4cls*.rknn")
                                           + _glob.glob("/root/dev/models/*4cls*.rknn"))
                            model_path = cands[0] if cands else ""
                            if not model_path:
                                print("[REF] ⚠️ --cone-method model 但没找到 /root/dev/models/"
                                      " 下的 car4cls 权重（--cone-model 可显式指定）→ 回退 hsv")
                                method = "hsv"
                        cone_det = ConeDetector(method=method,
                                                model_path=(model_path or None))
                        cone_tracker = ConeTracker()
                        print(f"[REF] 锥桶检测：{method} 通道"
                              + (f"（{model_path}）" if model_path else "")
                              + (" + S 型避让开" if args.cone_avoid else
                                 (" + 停车模式开" if args.cone_stop else "（--cone-avoid/--cone-stop 开启后才会响应）")))
                    cone_phase = None            # None/"avoid1"/"resume"/"avoid2"
                    cone_phase_t0: Optional[float] = None
                    cone_stopped = False         # --cone-stop：检测到锥桶后停车（实验室模式）
                    avoid_deg = (settings.CONE_AVOID_DEG if args.cone_avoid_deg is None
                                 else float(args.cone_avoid_deg))
                    avoid_s = (settings.CONE_AVOID_S if args.cone_avoid_s is None
                               else float(args.cone_avoid_s))
                    resume_s = (settings.CONE_SEGUE_S if args.cone_resume_s is None
                                else float(args.cone_resume_s))
                    seen_board = False
                    preflight_done = False
                    station_ok = True          # 自检通过才能发车（2026-09-23 操场实测加）
                    launch_t: Optional[float] = None
                    target_px: Optional[float] = None
                    setup_frames: List[np.ndarray] = []
                    setup_deadline = time.time() + args.setup_timeout_s
                    warned_early_board = False
                    steer = center_deg
                    last_center: Optional[float] = None
                    n_center_frames = 0
                    target_w = float(settings.IMG_W)
                    read_fail_since: Optional[float] = None
                    read_fail_warned = False

                    while not state["stopped"] and (time.time() - t0) < args.max_seconds:
                        ok, frame = cap.read()
                        if not ok or frame is None:
                            # 摄像头掉线/被别的进程抢走时**别傻等**：读失败到上限就报错收尾
                            # （安全层照常执行，只是不再空转）
                            if read_fail_since is None:
                                read_fail_since = time.time()
                            elif not read_fail_warned and (time.time() - read_fail_since) >= 2.0:
                                read_fail_warned = True
                                print("[REF] ⚠️ 摄像头连续读失败（画面丢了）…若持续到 "
                                      f"{args.read_fail_s:.0f}s 就报错退出")
                            if (time.time() - read_fail_since) >= args.read_fail_s:
                                print(f"[REF] ❌ 摄像头 {args.read_fail_s:.0f}s 没有给出画面 → 退出"
                                      f"（检查 /dev/video{args.camera} 是否被推流/别的进程占用）")
                                rc = 1
                                break
                            time.sleep(0.02)
                            continue
                        read_fail_since = None
                        read_fail_warned = False
                        now = time.time()
                        target_w = float(frame.shape[1])
                        gs = gate.update(frame)

                        if gs.blocked:
                            if not seen_board:
                                seen_board = True
                                if not preflight_done and len(setup_frames) < 3 \
                                        and not warned_early_board:
                                    warned_early_board = True
                                    print("[REF] ⚠️ 板来得太快：**自检被跳过**，本次用系统里的"
                                          "旧带/旧目标点（很可能不准）→ 车开出去的话跑偏保护"
                                          "会在第一时间兜住。真要测就 Ctrl-C 重跑一次："
                                          "启动后**先别挡板**，等自检四步打印完再挡板")
                            elif launch_t is not None:
                                # ★ 车正在跟线时"再见板" = 立即停车；把本次行程的计时/状态清掉，
                                #   否则移开板重新发车时会拿旧的"最后有效读数时间"判丢线 → 一发车就停
                                launch_t = None
                                pid.reset()
                                if err_filter is not None:
                                    err_filter.reset()
                                loss.reset()
                                big_err.reset()
                                print("[REF] 🛑 再见板 → 停车（本次行程结束；移开板可重新发车）")
                        elif not seen_board and not preflight_done \
                                and len(setup_frames) < args.setup_frames \
                                and now < setup_deadline:
                            setup_frames.append(frame.copy())     # 起步前自检窗口
                            continue

                        if not seen_board and not preflight_done:
                            preflight_done = True
                            if len(setup_frames) >= 3:
                                params, target_px, station_ok = _preflight(setup_frames, args, params)
                                if not station_ok:
                                    print("[REF] 🚫 自检证据不足 → **本次不会发车**；"
                                          "请重新摆正车（车头朝赛道、尽量居中）后 Ctrl-C 重跑。"
                                          "强行发车只会立刻触发保护，没有意义")
                                det = LaneRefDetector(params)
                                if target_px is None:
                                    target_px = params.target_ratio * target_w
                                det.width_prior.reset()
                                for f in setup_frames:      # 暖机：半宽先验 + Canny 自适应
                                    det.detect(f)
                                print(f"[REF]   目标点本次用 {target_px:.1f}px；半宽先验 "
                                      f"{det.width_prior.value:.0f}px"
                                      f"{'实测' if det.width_prior.measured else '种子'}")
                                rec.save_setup(setup_frames, det, params)
                                _write_site(args, params, target_px)
                            else:
                                print(f"[REF] ⚠️ 自检只有 {len(setup_frames)} 帧（板挡得太早）→ "
                                      f"用配置：带 {params.roi_top_ratio:.2f}~"
                                      f"{params.roi_bottom_ratio:.2f}，目标点 "
                                      f"{params.target_ratio * target_w:.0f}px")
                            print("[REF] ✅ 起步前自检完成 —— 现在把蓝板挡到下摄正前方"
                                  "（板在=不动；移开=发车）")

                        r = det.detect(frame)
                        if r.center_x is not None and r.quality >= args.min_quality:
                            n_center_frames += 1

                        # ---- 锥桶检测：每帧更新防抖状态（只有检测器在才跑）----
                        cone_present = False
                        if cone_tracker is not None:
                            cones = cone_det.detect(frame) if cone_det is not None else []
                            cone_present = cone_tracker.update(cones)
                            if cones:
                                c0 = cones[0]
                                if frames % max(1, args.print_every) == 0:
                                    print(f"[REF] 锥桶 {len(cones)} 个：x={c0.center_x:.0f} "
                                          f"yb={c0.bottom_y:.0f} {c0.width:.0f}x{c0.height:.0f} "
                                          f"conf={c0.conf:.2f} 防抖={'有' if cone_present else '积累中'}")

                        # ---- 锥桶停车模式（实验室先验检测；锥桶移开可继续前进）----
                        # 与"再见板停车"同款设计：不置 stopped（那样会退出整个循环），
                        # 而是让 driving=False 停车；锥桶移开（防抖消失）后自动恢复前进。
                        if args.cone_stop and cone_tracker is not None:
                            if cone_present and not cone_stopped:
                                cone_stopped = True
                                launch_t = None
                                pid.reset()
                                if err_filter is not None:
                                    err_filter.reset()
                                loss.reset()
                                big_err.reset()
                                print("[REF] 🛑 检测到锥桶 → 停车（锥桶移开可继续前进）")
                            elif not cone_present and cone_stopped:
                                cone_stopped = False
                                print("[REF] ▶️ 锥桶已移开 → 继续前进")

                        driving = seen_board and not gs.blocked and station_ok \
                            and not cone_stopped
                        if not station_ok and seen_board and not gs.blocked                                 and (frames % 30 == 0):
                            print("[REF] 🚫 自检未通过（证据不足），保持停车不动。"
                                  "重新摆车后 Ctrl-C 重跑")
                        if driving and launch_t is None:
                            launch_t = now
                            pid.reset()
                            if err_filter is not None:
                                err_filter.reset()
                            loss.reset()
                            big_err.reset()
                            print(f"[REF] 🚗 发车（目标点 {params.target_ratio * target_w:.0f}px，"
                                  f"带 {params.roi_top_ratio:.2f}~{params.roi_bottom_ratio:.2f}）")

                        # 原始误差（这一帧检测器给的）与送进控制器的滤波后误差都留着：
                        # 跑偏保护要同时看（只盯滤波值会被"滤波冻结"骗过去，见 filters.py 说明）
                        err_raw = r.error if r.center_x is not None else None
                        err_c = None
                        if r.error is not None and r.quality >= args.min_quality:
                            err_c = r.error if err_filter is None else err_filter.update(r.error)

                        ls = loss.update(now, err_c is not None) if driving \
                            else LossState("idle", 0.0, 1.0)
                        trip = bool(driving and args.err_stop_s > 0
                                    and big_err.update(now, err_c, raw_error=err_raw))
                        if trip:
                            # ⚠️ err_c 可能为 None（读数质量不够时不进控制器），
                            #   而原始误差照样能触发保护 → 这里**不能**直接格式化成 float
                            #   （2026-09-22 落地实测崩过：TypeError: NoneType.__format__）
                            print(guard_trip_text(args.err_stop_px, args.err_stop_s,
                                                  err_c, err_raw))
                            state["stopped"] = True

                        # ---- S 型避让状态机（driving 且开启时才推进；放在 trip 之后）----
                        if cone_tracker is not None and args.cone_avoid and driving \
                                and not trip and not state["stopped"]:
                            now_c = time.time()
                            if cone_phase is None:
                                if cone_present:
                                    cone_phase = "avoid1"
                                    cone_phase_t0 = now_c
                                    print(f"[REF] 🚩 检测到锥桶 → S 型避让开始"
                                          f"（{args.cone_first_dir}绕 {avoid_s:.1f}s）")
                            elif cone_phase == "avoid1":
                                if now_c - cone_phase_t0 >= avoid_s:
                                    cone_phase = "resume"
                                    cone_phase_t0 = now_c
                                    print(f"[REF] → 回正直行 {resume_s:.1f}s")
                            elif cone_phase == "resume":
                                if now_c - cone_phase_t0 >= resume_s:
                                    cone_phase = "avoid2"
                                    cone_phase_t0 = now_c
                                    print(f"[REF] → 反向绕 {avoid_s:.1f}s")
                            elif cone_phase == "avoid2":
                                if now_c - cone_phase_t0 >= avoid_s:
                                    cone_phase = None
                                    cone_phase_t0 = None
                                    cone_tracker.reset()
                                    print("[REF] → 避让结束，回到循迹")

                        phase, out_us, angle = "stop", NEUTRAL_US, 90.0
                        if trip:
                            phase = "guard"
                        elif cone_phase is not None and args.cone_avoid and driving \
                                and not state["stopped"]:
                            # ---- S 型绕行：开环动作（参考实现 avoidFromLeft/Right 的参数化版）----
                            # 绕行是**故意**的大幅转向：跑偏保护/画龙检测对它豁免（否则一绕就误触发停车）。
                            # 安全兜底：绕行时油门降到 CONE_AVOID_US 低速；任何时刻再见板 → driving=False
                            # 直接进上面的停车分支；--max-seconds 总上限仍然生效。
                            sign = steer_sign                          # +1=角度大右转 / -1=角度大左转
                            first = -1.0 if args.cone_first_dir == "left" else 1.0
                            if cone_phase == "avoid1":
                                angle = center_deg + first * sign * avoid_deg    # 先向 first 侧
                            elif cone_phase == "avoid2":
                                angle = center_deg - first * sign * avoid_deg    # 再向另一侧
                            else:                                          # resume：回中直行找下一个
                                angle = center_deg
                            steer = angle
                            out_us = settings.CONE_AVOID_US
                            phase = cone_phase
                        elif driving:
                            if ls.phase == "fresh":
                                steer = pid.step(err_c)
                                # ★ 单侧/弱对帧的中心不可靠（实测一跳 50~140px），
                                #   单独收窄它们的舵角：还能走，但甩不动
                                steer = apply_steer_cap(center_deg, steer, r.both_sides,
                                                        r.n_left, r.n_right,
                                                        args.single_limit_deg,
                                                        args.weak_limit_deg)
                                angle = steer
                                out_us = adaptive_pulse(err_c, args.speed_us,
                                                        args.speed_fast, args.speed_slow)
                                if not r.both_sides:
                                    out_us = floor_pulse(args.speed_us, args.speed_slow)
                                phase = "track"
                                last_center = r.center_x
                                n_track += 1
                            elif ls.phase == "hold" and ls.since_s == INF:
                                # 发车后还没见过一次线：低速探路（舵角回中位，往前拱找线）
                                pid.reset()
                                if err_filter is not None:
                                    err_filter.reset()
                                if (now - (launch_t or now)) < args.acquire_s:
                                    steer, angle, out_us, phase = center_deg, center_deg, args.start_us, "acquire"
                                else:
                                    phase = "lost"
                                    state["stopped"] = True
                                    print(f"[REF] ⚠️ 探路 {args.acquire_s:.1f}s 也没找到线 → 停车"
                                          f"（看 setup 目录的原图 / 跑 lane_probe）")
                            elif ls.phase == "hold":
                                # 丢线维持：沿用上次中心，误差按时间衰减回中位（越久越回正）
                                held = (last_center - params.target_ratio * target_w) \
                                    if last_center is not None else 0.0
                                angle = pid.step(held * ls.decay) if abs(held) > 1e-6 else 90.0
                                steer = angle
                                out_us = floor_pulse(args.speed_us, args.speed_slow)
                                phase = "hold"
                            else:
                                pid.reset()
                                if err_filter is not None:
                                    err_filter.reset()
                                phase = "lost"
                                state["stopped"] = True
                                print(f"[REF] ⚠️ 丢线 {ls.since_s:.1f}s 没恢复 → 停车"
                                      f"（保守：不带垃圾读数继续跑）")

                        # ★ 画龙（S 弯）检测：这一帧的舵角已经算完，统计它是否在窗口内来回打满幅
                        #   （2026-09-23 操场实测：舵机 75°↔105° 来回甩、误差 ±100~300 翻符号，
                        #    连续计时型跑偏保护永远凑不满 → 用拐角次数直接抓）
                        if driving and not trip and not state["stopped"]:
                            if osc.update(now, angle, center_deg):
                                trip = True
                                print("[REF] ❌ 检测到画龙（S 弯）：舵机 2s 内大幅来回摆多次"
                                      " → 控制失稳，停车（先查单侧兜底 / kp 太高 / 平滑太小）")
                        if trip:
                            state["stopped"] = True

                        if use_motor and pca is not None:
                            pca.set_steering_angle(angle)
                            pca.write_us(pca.CH_ESC, out_us)

                        frames += 1
                        t_rel = now - t0
                        rec.row(t_rel, phase, r, err_c, params.target_ratio * target_w,
                                angle, out_us, ls.since_s)
                        if rec.wants_save(t_rel):          # 画叠加图只在真要存的时候做
                            rec.maybe_save(t_rel, frame,
                                           annotate(frame, r, params, phase,
                                                    f"esc={out_us:.0f} t={t_rel:.0f}s",
                                                    cones=(cones if cone_tracker is not None else None)),
                                           phase)
                        if frames % max(1, args.print_every) == 0:
                            print(f"[REF] {t_rel:6.1f}s 中心={_fmt(r.center_x, '%5.1f')} "
                                  f"误差={_fmt(r.error, '%+6.1f')} 滤波后={_fmt(err_c, '%+6.1f')} "
                                  f"舵机={angle:5.1f}° 电调={out_us:4.0f}us "
                                  f"质量={r.quality:.2f} {_sided_text(r)} "
                                  f"线段{r.n_left:2d}/{r.n_right:2d} "
                                  f"边缘={r.edge_pct * 100:4.1f}% ({phase})")
                    cap.release()

                    if not seen_board:
                        print("[REF] ⚠️ 全程没见过蓝板：车不会动（这是安全设计）")
                        print("[REF]    板要摆在下摄正前方、够大够近；先跑 lane_probe 看蓝色占比")
                    elif n_center_frames == 0:
                        print("[REF] ⚠️ 全程没有质量达标的读数 → 看 setup 目录的图 + "
                              "lane_probe 的镜头朝向体检")
                    else:
                        print(f"[REF] 本次有效读数 {n_center_frames} 帧，其中跟线 {n_track} 帧")
    finally:
        rec.close()
        summary = rec.summary()
        _print_summary(summary, driven=state.get("used_motor", False))
        try:
            with open(os.path.join(rec.save_dir or save_dir, "summary.json"),
                      "w", encoding="utf-8") as fh:
                json.dump(summary, fh, ensure_ascii=False, indent=1)
        except Exception as exc:
            print(f"[REF] ⚠️ 写 summary.json 失败：{exc}")
        restore_remote_stage(state, "循迹结束")
    return rc


if __name__ == "__main__":
    sys.exit(main())
