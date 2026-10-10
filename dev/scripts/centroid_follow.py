#!/usr/bin/env python3
"""质心法循迹实跑脚本（官方 4 方案之 ②单线质心 / ③双线质心，2026-10-09 为选型新增）。

对照 `scripts/lane_ref_test.py`（=方案④ 边缘法）。算法（官方 line_follow.cpp 同款）：
    ROI 裁剪 → 灰度 → 二值化（阈值可调）→ ROI 内全部前景像素**全局质心**
    → 误差 = 质心x − target → PID → 舵机 = 中位 + offset → 电调脉宽。

选型口径（--scheme）：
    --scheme 2 → KP=0.6, KD=0.1（单线质心 line_follow2.cpp 口径）
    --scheme 3 → KP=7.0, KD=3.0（双线质心 9 号案例口径；官方代码是 `SERVO_MID − pid_out`——
                本脚本 PID 符号沿 A 路径口径，若现场方向反了用 `--sign -1` 翻转，不用改代码）
双线质心在本车视角下 ≈ ROI 内两条白线前景都在 → 质心落在两线中间（官方注释原话）。

安全框架镜像 lane_ref_test：--allow-motion 必给、MotorSession（opi-control 让出/恢复）、
camera_exclusive、StartGate 发车边沿（板在=停/移开=走/放回=停）、丢线阶梯（连续 N 帧
无质心 → 停车）、CSV+会话目录落盘。--image 供本地无硬件验证。

用法：
    本地验证:    PYTHONPATH=dev python scripts/centroid_follow.py --image 某图.jpg
    车上选型:    sudo python3 scripts/centroid_follow.py --allow-motion --scheme 3 \
                 --save-dir run_1010_c3 --max-seconds 90
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import settings                                   # noqa: E402
from control.pid import PID                                   # noqa: E402
from vision.camera_guard import camera_exclusive, open_camera  # noqa: E402
from vision.start_gate import StartGate                       # noqa: E402


# ------------------------------------------------------------------ 质心检测
def centroid_center(frame_bgr: np.ndarray, roi, threshold: int = 185,
                    invert: bool = False, min_white: int = 30):
    """官方质心法：ROI 内前景像素质心。返回 (center_x, n_white) 或 (None, n)。"""
    h, w = frame_bgr.shape[:2]
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return None, 0
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    mode = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
    _, binary = cv2.threshold(gray, int(threshold), 255, mode)
    pts = cv2.findNonZero(binary)
    n = 0 if pts is None else len(pts)
    if n < min_white:                       # 官方 WHITE_MAXNUM_STOP=30
        return None, n
    xy = pts.reshape(-1, 2)
    return float(xy[:, 0].mean()) + x0, n


GAINS = {"2": (0.6, 0.1), "3": (7.0, 3.0)}   # (KP, KD)：官方 line_follow2 / 9 号案例


def main() -> int:
    ap = argparse.ArgumentParser(description="质心法循迹（方案②③选型）")
    ap.add_argument("--scheme", default="2", choices=["2", "3"], help="质心法口径（KP/KD）")
    ap.add_argument("--kp", type=float, default=None, help="覆盖 KP")
    ap.add_argument("--kd", type=float, default=None, help="覆盖 KD（官方 Ki≡0）")
    ap.add_argument("--threshold", type=int, default=190, help="二值化阈值（②=185/③=190）")
    ap.add_argument("--invert", action="store_true", help="黑线模式（THRESH_BINARY_INV，官方①）")
    ap.add_argument("--roi", default="0.00,1.00,0.25,0.95",
                    help="ROI (x0, x1, y0, y1) 比例——默认贴车上 site.yaml 视野")
    ap.add_argument("--target-x", type=float, default=None, help="期望质心 x（默认 settings.TARGET_X）")
    ap.add_argument("--sign", type=float, default=None,
                    help="PID 误差符号：默认沿 STEER_SIGN；官方③反号时用 --sign -1")
    ap.add_argument("--speed-us", type=int, default=None, help="巡航脉宽（默认 ESC_CREEP_US）")
    ap.add_argument("--camera", type=int, default=2)
    ap.add_argument("--max-seconds", type=int, default=120)
    ap.add_argument("--lost-frames", type=int, default=15, help="连续无质心多少帧停车")
    ap.add_argument("--save-dir", default="", help="会话目录（CSV/叠加图）")
    ap.add_argument("--allow-motion", action="store_true", help="必给：才允许驱动电机")
    ap.add_argument("--image", default=None, help="本地单图分析模式（无硬件）")
    args = ap.parse_args()

    kp, kd = GAINS[args.scheme]
    kp = args.kp if args.kp is not None else kp
    kd = args.kd if args.kd is not None else kd
    roi = tuple(float(v) for v in args.roi.split(","))
    sign = float(settings.STEER_SIGN if args.sign is None else args.sign)
    target_x = float(settings.TARGET_X if args.target_x is None else args.target_x)
    cruise_us = int(settings.ESC_CREEP_US if args.speed_us is None else args.speed_us)

    # ---------------- 单图分析（本地验证）----------------
    if args.image:
        img = cv2.imdecode(np.fromfile(args.image, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print(f"cannot read {args.image}")
            return 1
        cx, n = centroid_center(img, roi, args.threshold, args.invert)
        print(f"[image] scheme={args.scheme} roi={roi} threshold={args.threshold} "
              f"center={cx} n_white={n} target={target_x}")
        return 0

    # ---------------- 车上模式 ----------------
    if not args.allow_motion:
        print("安全门：必须显式给 --allow-motion（四轮架空或场地空旷）")
        return 2
    if cruise_us < settings.ESC_DEADBAND_US:
        print(f"安全门：speed-us {cruise_us} < 死区 {settings.ESC_DEADBAND_US}，拒绝")
        return 2

    from scripts.bench_common import (MotorSession, install_signal_guard,
                                      restore_remote_stage)
    # ★ 退出保护（参考 scripts/lane_ref_test.py 的同款结构，2026-10-09 对齐）：
    #   state 必须是**同一个 dict**：install_signal_guard 写 stopped（Ctrl+C/信号→置位），
    #   主循环读它退出，MotorSession/restore_remote_stage 读写 driver/used_motor/restored。
    #   四字段预置，避免任何路径读到缺失键。
    state = {"stopped": False, "driver": None, "restored": False, "used_motor": True}
    install_signal_guard(state)
    gate = StartGate()
    # 单摄像头循环：发车判定与循迹用同一路画面（真车上发车看板用主摄的话，
    # 加 --gate-camera 分摄的任务在 vision_main 双摄链路里做，这里先单摄闭环）
    pid = PID(kp=kp, ki=0.0, kd=kd, out_limit=15.0, integral_limit=50.0)

    rc = 0
    writer = csv_fh = None
    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)
        csv_fh = open(os.path.join(args.save_dir, "run.csv"), "w", newline="")
        writer = csv.writer(csv_fh)
        writer.writerow(["t", "phase", "center_x", "error_px", "angle_deg", "esc_us",
                         "n_white", "board_blocked", "note"])

    print(f"[CF] scheme={args.scheme} KP={kp} KD={kd} sign={sign} target={target_x} "
          f"cruise={cruise_us}us roi={roi} threshold={args.threshold}")
    t0 = time.monotonic()
    released = False
    lost = 0
    try:
        with MotorSession(state, enabled=True,
                          speed_us_max=max(cruise_us, settings.ESC_DEBUG_MAX_US)) as pca:
            with camera_exclusive():
                cap = open_camera(args.camera, settings.IMG_W, settings.IMG_H)
                if cap is None:
                    print("[CF] 摄像头打不开")
                    rc = 1
                else:
                    pca.write_us(pca.CH_ESC, settings.ESC_STOP_US)
                    # Ctrl+C（install_signal_guard 置 stopped）→ 循环立即退出 → 恢复链收尾
                    while (not state.get("stopped")
                           and time.monotonic() - t0 < args.max_seconds):
                        ok, frame = cap.read()
                        if not ok or frame is None:
                            time.sleep(0.05)
                            continue
                        st = gate.update(frame) if not released else None
                        if st is not None and st.released:
                            released = True
                            print("[CF] ✅ 发车（板移开）")
                        center_x, n = centroid_center(frame, roi, args.threshold, args.invert)
                        if center_x is None:
                            lost += 1
                        else:
                            lost = 0

                        if not released:
                            phase, angle, out_us = "wait_start", settings.SERVO_CENTER_ANGLE, \
                                settings.ESC_STOP_US
                        elif lost >= args.lost_frames:
                            phase, angle, out_us = "lost", settings.SERVO_CENTER_ANGLE, \
                                settings.ESC_STOP_US
                        elif center_x is not None:
                            err = center_x - target_x
                            offset = pid.update(sign * err / 4.0, 1.0 / 30.0)
                            angle = float(settings.SERVO_CENTER_ANGLE) + offset
                            phase, out_us = "track", cruise_us
                        else:
                            # 还没到丢线阈值：维持上一次舵角、减速蠕动
                            phase, angle, out_us = "hold", None, max(
                                settings.ESC_STOP_US, cruise_us - 10)
                        if angle is not None:
                            pca.set_steering_angle(angle)
                        pca.write_us(pca.CH_ESC, out_us)

                        if writer is not None:
                            writer.writerow([round(time.monotonic() - t0, 2), phase,
                                             None if center_x is None else round(center_x, 1),
                                             None if center_x is None else round(center_x - target_x, 1),
                                             "" if angle is None else round(angle, 1),
                                             out_us, n,
                                             bool(st is not None and st.blocked), phase])
                        if int(time.monotonic() - t0) % 5 == 0:
                            print(f"[CF] t={time.monotonic()-t0:.0f}s phase={phase} "
                                  f"center={center_x} n={n}")
                    cap.release()
    finally:
        # ★ 退出保护（与 lane_ref_test 完全同构）：安全停车与远程控制阶段恢复
        #   由 restore_remote_stage 统一做（电调归零/舵机回中/释放 PCA9685/重启
        #   opi-control/验证推流）；finally 显式调用一次 = 与 MotorSession.__exit__
        #   双保险（幂等），没走到那里也有 install_signal_guard 的 atexit 兜底。
        restore_remote_stage(state, "质心循迹结束")
        if csv_fh is not None:
            csv_fh.close()
        if args.save_dir:
            try:
                with open(os.path.join(args.save_dir, "summary.json"), "w") as fh:
                    json.dump({"scheme": args.scheme, "kp": kp, "kd": kd,
                               "threshold": args.threshold, "roi": list(roi),
                               "target_x": target_x, "cruise_us": cruise_us}, fh)
            except OSError as exc:
                print(f"[CF] ⚠️ 写 summary.json 失败：{exc}")
    print("[CF] 结束，已安全停车并恢复")
    return rc


if __name__ == "__main__":
    sys.exit(main())