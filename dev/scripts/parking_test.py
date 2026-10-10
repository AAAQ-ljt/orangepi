#!/usr/bin/env python3
"""停车位实跑（P1-4 雏形，2026-10-10）：蓝板发车 → 循迹 → 黄区发现 → 判左右 → 逼近 → 停车。

流程（单项可测，每段独立参数）：
    wait_start（StartGate 边沿，板在=停/移开=走）
    → track（lane_ref 循迹，丢线保护）
    → park_approach（黄区连续 PARK_HYSTERESIS 帧发现 → 降速 PARK_APPROACH_US）
    → park_align（判左右 + 7 帧锁定 → 朝目标半格低速修正）
    → park_stop（蓝板够近 → 停车确认 → done）

判左右（开发文档 §2.6 几何先验法；本届规则只有"挡板遮挡两格"，无 A/B 概念）：
    黄区外框（一个实例）水平中点 mid_x 视为停车区分割参考；蓝板**底边中点** board_x
    为遮挡位置。board_x < mid_x → 板在左 → 停右半格；反之停左半格。
    目标点 = mid_x ± 黄区宽/4（俯视中线近似，未接 IPM 的现场简化版；容差/比例现场标定）。
    连续 LockVote 帧同侧 → 锁定不再切换（保守：不确定时停在分流点前）。

安全框架镜像 lane_ref_test / centroid_follow（bench_common 四点样式）：
    --allow-motion 必给、MotorSession（opi-control 让出/恢复）、camera_exclusive、
    install_signal_guard（Ctrl+C/TERM/HUP → 恢复手动阶段）、CSV+会话目录落盘、
    --image 本地无硬件验证。

用法：
    本地验证:  PYTHONPATH=dev python scripts/parking_test.py --image 某停车区图.jpg
    车上实跑:  sudo python3 scripts/parking_test.py --allow-motion --save-dir run_park_1010 \\
                 --max-seconds 120
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import settings                                   # noqa: E402
from control.lane_control import NEUTRAL_US, PidRef            # noqa: E402
from vision.camera_guard import camera_exclusive, open_camera  # noqa: E402
from vision.color_elements import detect_parking_yellow, parking_yellow_box  # noqa: E402
from vision.cone_detect import blue_mask                      # noqa: E402
from vision.lane_ref import LaneParams, LaneRefDetector       # noqa: E402
from vision.start_gate import StartGate                       # noqa: E402

# ------------------------------------------------------------------ 停车纯函数（可单测）
SIDE_TOL_RATIO = 0.15          # board_x 与 mid_x 的偏差需超过 黄区宽×15% 才算"明确一侧"
LOCK_FRAMES = 7                # 连续几帧同侧才锁定（开发文档 §2.6：多帧投票+锁定后不切换）
ARRIVED_W_RATIO = 0.55         # 蓝板宽度占画面比例 ≥ 此值 → 够近，停车


def side_verdict(board_x: float, mid_x: float, span: float,
                 tol_ratio: float = SIDE_TOL_RATIO) -> str:
    """蓝板在哪侧："left"/"right"/"center"（center=压中线附近，不投票，继续观察）。

    board_x < mid_x − span×tol_ratio → 板在左（停右半格）；反之在右；中间带判 center。
    """
    tol = abs(span) * tol_ratio
    if board_x < mid_x - tol:
        return "left"
    if board_x > mid_x + tol:
        return "right"
    return "center"


def park_target_x(side: str, mid_x: float, span: float) -> float:
    """目标半格的横坐标（画面像素）：右半格 ≈ 中点 + 宽/4，左半格 ≈ 中点 − 宽/4。"""
    if side == "right":
        return mid_x + abs(span) / 4.0
    if side == "left":
        return mid_x - abs(span) / 4.0
    return mid_x                      # center：先对准中线，等判出侧别再偏


def arrived_verdict(board_w: float, frame_w: float,
                    ratio: float = ARRIVED_W_RATIO) -> bool:
    """蓝板宽度（占画面比例）足够大 → 已近停车位入口。"""
    return frame_w > 0 and board_w >= frame_w * ratio


class LockVote:
    """多帧投票锁定：连续 need 帧同侧 → 锁定；center 或反向 → 清零重来；锁定后不切换。"""

    def __init__(self, need: int = LOCK_FRAMES) -> None:
        self.need = need
        self.side: str | None = None
        self._votes = 0
        self._last: str | None = None

    def update(self, side: str) -> str | None:
        if self.side is not None:
            return self.side                  # 已锁定：不再切换（保守，防近距误判）
        if side == "center":
            self._votes = 0
            self._last = None
            return None
        if side == self._last:
            self._votes += 1
        else:
            self._votes = 1
            self._last = side
        if self._votes >= self.need:
            self.side = side
        return self.side

    def reset(self) -> None:
        self.side = None
        self._votes = 0
        self._last = None


# ------------------------------------------------------------------ 图像辅助
def board_box(frame_bgr: np.ndarray, roi=None) -> tuple | None:
    """蓝色掩膜最大连通域的 bbox (x, y, w, h)；返回底边中点与宽度。"""
    h, w = frame_bgr.shape[:2]
    roi = roi or settings.PARK_ZONE_ROI
    mask = blue_mask(frame_bgr, roi=roi)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    x, y, bw, bh = cv2.boundingRect(max(contours, key=cv2.contourArea))
    if bw * bh < 400:                     # 太小的蓝块（噪点）不算板
        return None
    return (x, y, bw, bh)


def probe_parking(frame_bgr: np.ndarray, roi=None) -> dict:
    """单帧全量读数（--image 与 CSV 共用），全部像素坐标、纯函数。"""
    h, w = frame_bgr.shape[:2]
    yellow = detect_parking_yellow(frame_bgr, roi=roi)
    box = parking_yellow_box(frame_bgr, roi=roi)
    board = board_box(frame_bgr, roi=roi)
    r = {"yellow": yellow,
         "mid_x": None, "span": None, "yellow_w": None, "yellow_bottom": None,
         "board_x": None, "board_w": None}
    if box is not None:
        x, y, bw, bh = box
        r["mid_x"] = float(x + bw / 2)
        r["span"] = float(bw)
        r["yellow_w"] = float(bw)
        r["yellow_bottom"] = float(y + bh)
    if board is not None:
        x, y, bw, bh = board
        r["board_x"] = float(x + bw / 2)      # 底边中点（接地线）
        r["board_w"] = float(bw)
    if r["mid_x"] is not None and r["board_x"] is not None:
        r["side"] = side_verdict(r["board_x"], r["mid_x"], r["span"] or 1.0)
    return r


# ------------------------------------------------------------------ 主程序
def main() -> int:
    ap = argparse.ArgumentParser(description="停车位实跑：发车→循迹→黄区→判左右→停车")
    ap.add_argument("--camera", type=int, default=2, help="下摄（发车+循迹+停车同一路）")
    ap.add_argument("--speed-us", type=int, default=int(settings.ESC_CREEP_US),
                    help=f"循迹巡航脉宽（默认 {settings.ESC_CREEP_US}，死区 {settings.ESC_DEADBAND_US}）")
    ap.add_argument("--approach-us", type=int, default=1548,
                    help="黄区发现后的接近脉宽（默认 1548：>死区且比巡航慢）")
    ap.add_argument("--kp", type=float, default=0.15, help="循迹 KP")
    ap.add_argument("--kd", type=float, default=0.12, help="循迹 KD（Ki≡0）")
    ap.add_argument("--lost-frames", type=int, default=18,
                    help="循迹期连续无中心多少帧停车（默认 18）")
    ap.add_argument("--park-hyst", type=int, default=3,
                    help="黄区连续检出多少帧进入停车段（默认 3）")
    ap.add_argument("--max-seconds", type=int, default=120)
    ap.add_argument("--save-dir", default="", help="会话目录（CSV/叠加图）")
    ap.add_argument("--allow-motion", action="store_true", help="必给：才允许驱动电机")
    ap.add_argument("--image", default=None, help="本地单图分析模式（无硬件）")
    args = ap.parse_args()

    if args.image:
        img = cv2.imdecode(np.fromfile(args.image, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print(f"cannot read {args.image}")
            return 1
        print("[PARK] " + json.dumps(probe_parking(img), ensure_ascii=False))
        return 0

    if not args.allow_motion:
        print("安全门：必须显式给 --allow-motion（四轮架空或场地空旷）")
        return 2
    if min(args.speed_us, args.approach_us) < settings.ESC_DEADBAND_US:
        print(f"安全门：脉宽低于死区 {settings.ESC_DEADBAND_US}us，拒绝")
        return 2
    if args.approach_us >= args.speed_us:
        print(f"安全门：approach-us {args.approach_us} 应小于 speed-us {args.speed_us}"
              f"（接近要更慢）")
        return 2

    from scripts.bench_common import (MotorSession, install_signal_guard,
                                      restore_remote_stage)
    state = {"stopped": False, "driver": None, "restored": False, "used_motor": True}
    install_signal_guard(state)

    params = LaneParams.from_settings()
    det = LaneRefDetector(params)
    gate = StartGate()
    pid = PidRef(args.kp, 0.0, args.kd, 15.0, 0.7, sign=float(settings.STEER_SIGN))
    center_deg = float(settings.SERVO_CENTER_ANGLE)

    rc = 0
    csv_fh = None
    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)
        csv_fh = open(os.path.join(args.save_dir, "run.csv"), "w", encoding="utf-8")
        csv_fh.write("t,phase,center_x,quality,side,mid_x,span,board_x,board_w,angle,esc_us\n")

    lost = 0
    yellow_streak = 0
    vote = LockVote()
    lock_since: float | None = None
    t0 = time.monotonic()
    try:
        with MotorSession(state, enabled=True,
                          speed_us_max=max(args.speed_us, settings.ESC_DEBUG_MAX_US)) as pca:
            with camera_exclusive():
                cap = open_camera(args.camera, settings.IMG_W, settings.IMG_H)
                if cap is None:
                    rc = 1
                else:
                    released = False
                    phase = "wait_start"
                    side: str | None = None
                    hold = 0.0
                    while (not state["stopped"]
                           and time.monotonic() - t0 < args.max_seconds):
                        ok, frame = cap.read()
                        if not ok or frame is None:
                            time.sleep(0.05)
                            continue
                        now = time.monotonic()
                        h, w = frame.shape[:2]

                        gs = gate.update(frame)
                        if gs.released and not released:
                            released = True
                            print("[PARK] ✅ 发车（板移开）")

                        r = det.detect(frame)
                        p = probe_parking(frame)

                        if not released:
                            phase, angle, out_us = "wait_start", center_deg, NEUTRAL_US
                            lost = 0
                        else:
                            # ---- 停车段判定（优先于循迹）----
                            if p["yellow"]:
                                yellow_streak += 1
                            else:
                                yellow_streak = 0
                            if phase != "park_stop" and yellow_streak >= args.park_hyst:
                                if phase in ("track", "wait_start"):
                                    phase = "park_approach"
                                    pid.reset()
                                    print("[PARK] 🟡 检测到停车区（连续 "
                                          f"{args.park_hyst} 帧）→ 降速接近")
                            if phase == "park_approach":
                                if p["mid_x"] is not None and p["board_x"] is not None:
                                    side = vote.update(side_verdict(p["board_x"], p["mid_x"],
                                                                    p["span"] or 1.0))
                                    if side is not None and lock_since is None:
                                        lock_since = now
                                        if side == "right":
                                            print("[PARK] 🔒 锁定：板在左 → 停**右**半格")
                                        else:
                                            print("[PARK] 🔒 锁定：板在右 → 停**左**半格")
                            if phase == "park_approach" and side is not None \
                                    and p["board_w"] is not None \
                                    and arrived_verdict(p["board_w"], w):
                                phase = "park_stop"
                                hold = now
                                print(f"[PARK] 🅿️ 已到停车位入口（板宽 {p['board_w']:.0f}px/"
                                      f"{w}）→ 停车确认（停{'右' if side == 'right' else '左'}半格）")

                        # ---- 输出决策 ----
                        if phase in ("wait_start", "park_stop"):
                            angle, out_us = center_deg, NEUTRAL_US
                        elif phase == "park_approach":
                            if side is None:
                                target = p["mid_x"] if p["mid_x"] is not None else float(w / 2)
                            else:
                                target = park_target_x(side, p["mid_x"] or float(w / 2),
                                                       p["span"] or 1.0)
                            err = target - w / 2
                            offset = max(-10.0, min(10.0, 0.8 * err / 4.0))
                            angle = center_deg + offset
                            out_us = int(args.approach_us)
                        elif r.center_x is not None and r.quality >= 0.28:
                            lost = 0
                            angle = pid.step(float(r.error))   # PidRef：像素口径、输出=含中位角度
                            out_us = int(args.speed_us)
                        else:
                            lost += 1
                            if lost >= args.lost_frames:
                                if phase == "track":
                                    print(f"[PARK] ⚠️ 连续 {args.lost_frames} 帧无车道 → 停车")
                                    phase = "lost"
                                angle, out_us = center_deg, NEUTRAL_US
                            else:
                                angle, out_us = center_deg, int(args.approach_us)

                        pca.set_steering_angle(angle)
                        pca.write_us(pca.CH_ESC, out_us)

                        if csv_fh is not None:
                            csv_fh.write("%.2f,%s,%s,%.2f,%s,%s,%s,%s,%s,%.1f,%d\n" % (
                                now - t0, phase,
                                "" if r.center_x is None else round(r.center_x, 1),
                                r.quality,
                                side, "" if p["mid_x"] is None else round(p["mid_x"], 1),
                                "" if p["span"] is None else round(p["span"], 1),
                                "" if p["board_x"] is None else round(p["board_x"], 1),
                                "" if p["board_w"] is None else round(p["board_w"], 1),
                                angle, out_us))
                        if int(now - t0) % 5 == 0 or phase != "track":
                            print(f"[PARK] t={now - t0:.0f}s phase={phase} "
                                  f"center={'' if r.center_x is None else round(r.center_x, 1)} "
                                  f"mid={p['mid_x']} board_x={p['board_x']} "
                                  f"side={side} esc={out_us}")
                        if phase == "park_stop" and now - hold >= 3.0:
                            print(f"[PARK] ✅ 停车完成（{'右' if side == 'right' else '左'}半格）"
                                  f"，等待人工确认/扫码；Ctrl+C 恢复手动模式")
                            break
                    cap.release()
    finally:
        restore_remote_stage(state, "停车测试结束")
        if csv_fh is not None:
            csv_fh.close()
        if args.save_dir:
            try:
                with open(os.path.join(args.save_dir, "summary.json"), "w",
                          encoding="utf-8") as fh:
                    json.dump({"speed_us": args.speed_us, "approach_us": args.approach_us,
                               "kp": args.kp, "kd": args.kd}, fh)
            except OSError as exc:
                print(f"[PARK] ⚠️ 写 summary.json 失败：{exc}")
    print("[PARK] 结束，已安全停车并恢复手动模式")
    return rc


if __name__ == "__main__":
    sys.exit(main())