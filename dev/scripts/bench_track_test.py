#!/usr/bin/env python3
"""扫线循迹台架测试：**见板→停；板移开→扫线循迹（转向用真 PID，电调用设定脉宽）；再见板→停**。

和 `bench_board_test.py` 的区别：那一个只验证"检测+电调"，这一个跑**真实的循迹链路**
（`LaneScanner` 扫线 → 误差滤波 → PID → 舵机；电调用固定脉宽），用来验证"识别到板→移开→循迹→再识别→停"整条流程。

⚠️ 两点必须知道：
1. "见板就停"是**台架专用**行为（比赛里发车后再看到蓝板是正常的，停车区挡板就是同一块蓝板）；
2. 循迹用的是**下摄（video2）**——板在车前时下摄也看得到（实测面积 0.638），所以本脚本单摄即可，不做切换。

用法（车上）：
    # ① 只看检测与扫线读数，不动电机
    sudo python3 /root/dev/scripts/bench_track_test.py --no-motor

    # ② 正式台架（四轮架空）：见板→停 / 板移开→循迹 / 再见板→停
    sudo python3 /root/dev/scripts/bench_track_test.py --speed-us 1550 --i-know-wheels-are-up

    # ③ 带预览窗口（需要 X11）：能同时看到扫线中线与板的判据
    sudo python3 /root/dev/scripts/bench_track_test.py --show --no-motor

    # ④ 台架上想强制跑（忽略"无有效车道"的保护）：
    sudo python3 /root/dev/scripts/bench_track_test.py --force-run --speed-us 1534 --i-know-wheels-are-up
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from config import settings
from control.lane_arbiter import LaneArbiter
from control.planner import Planner
from scripts.bench_common import NEUTRAL_US, MotorSession, install_signal_guard
from vision.camera_guard import camera_exclusive
from vision.lane_scan import LaneScanner
from vision.start_gate import StartGate


def decide_track_output(blocked: bool, seen_board: bool, lane_stop: bool,
                        force_run: bool, speed_us: float,
                        neutral_us: float = NEUTRAL_US) -> tuple:
    """台架规则（纯函数，便于单测）：返回 (电调脉宽, 原因)。

    没见板 → 中位（安全不变量）；见板 → 中位；车道失效且没开 force_run → 中位；否则 → speed_us。
    """
    if not seen_board:
        return neutral_us, "未见过板"
    if blocked:
        return neutral_us, "板在"
    if lane_stop and not force_run:
        return neutral_us, "无有效车道"
    return speed_us, "循迹中"


def main() -> int:
    ap = argparse.ArgumentParser(description="扫线循迹台架测试（见板→停 / 板开→循迹 / 再见板→停）")
    ap.add_argument("--camera", type=int, default=2, help="摄像头：2=下摄（默认，循迹用），0=云台主摄")
    ap.add_argument("--speed-us", type=float, default=1550.0, help="循迹时的电调脉宽（默认 1550）")
    ap.add_argument("--max-seconds", type=float, default=180.0)
    ap.add_argument("--max-steer", type=float, default=None, help="转向角限幅（默认用 settings 的 30°）")
    ap.add_argument("--target-x", type=float, default=None, help="期望车道中心（默认 settings.TARGET_X）")
    ap.add_argument("--force-run", action="store_true",
                    help="忽略'无有效车道'的保护，强制给油（台架观察用，比赛绝不能用）")
    ap.add_argument("--show", action="store_true", help="显示预览窗口（需要 X11）")
    ap.add_argument("--no-motor", action="store_true", help="只跑视觉与决策，不输出动力")
    ap.add_argument("--i-know-wheels-are-up", action="store_true")
    ap.add_argument("--width", type=int, default=settings.IMG_W)
    ap.add_argument("--height", type=int, default=settings.IMG_H)
    ap.add_argument("--print-every", type=int, default=5)
    args = ap.parse_args()

    use_motor = not args.no_motor
    if use_motor and not args.i_know_wheels_are_up:
        print("[BENCH] 拒绝运行：请确认四轮已架空并加 --i-know-wheels-are-up（或先用 --no-motor）")
        return 2

    state = {"stopped": False, "driver": None, "restored": False, "used_motor": use_motor}
    install_signal_guard(state)

    scanner = LaneScanner(target_x=args.target_x)
    gate = StartGate()
    arbiter = LaneArbiter(target_x=args.target_x)
    planner = Planner(target_x=args.target_x, max_steer=args.max_steer)
    steer_limit = float(args.max_steer or settings.LANE_STEER_LIMIT_DEG)
    steering = float(settings.SERVO_CENTER_ANGLE)
    seen_board = False

    print(f"[BENCH] 摄像头={args.camera}  速度={args.speed_us:.0f}us  "
          f"电机={'开' if use_motor else '关(只跑视觉/决策)'}  force_run={args.force_run}")
    print("[BENCH] 规则：没见板→中位；见板→中位；板移开→扫线循迹；再见板→立即停")

    rc = 0
    try:
        with MotorSession(state, enabled=use_motor, speed_us_max=args.speed_us) as pca:
            with camera_exclusive():
                cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    print("[BENCH] 摄像头打不开")
                    rc = 1
                else:
                    t0 = time.time()
                    last_t = t0
                    frames = 0
                    try:
                        while not state["stopped"] and (time.time() - t0) < args.max_seconds:
                            ok, frame = cap.read()
                            if not ok or frame is None:
                                time.sleep(0.03)
                                continue
                            now = time.time()
                            dt = max(1e-3, now - last_t)
                            last_t = now

                            # ---- 感知：板 + 扫线 ----
                            gs = gate.update(frame)
                            if gs.blocked:
                                seen_board = True
                            obs = scanner.scan(frame)
                            arb = arbiter.update(obs.center_x, obs.confidence, now)

                            out_us, why = decide_track_output(
                                gs.blocked, seen_board, arb.should_stop, args.force_run, args.speed_us)

                            # ---- 控制：真 PID + 转向限速（停车时缓慢回中，并清 PID 状态避免踢腿）----
                            if out_us <= NEUTRAL_US:
                                planner.reset()
                                target_steer = float(settings.SERVO_CENTER_ANGLE)
                            else:
                                target_steer = float(settings.SERVO_CENTER_ANGLE) + \
                                    planner.steering_offset(arb.center_x, dt)
                                target_steer = max(settings.SERVO_CENTER_ANGLE - steer_limit,
                                                   min(settings.SERVO_CENTER_ANGLE + steer_limit,
                                                       target_steer))
                            max_delta = settings.STEERING_SLEW_DEG_PER_S * dt
                            delta = max(-max_delta, min(max_delta, target_steer - steering))
                            steering += delta

                            if use_motor and pca is not None:
                                pca.set_steering_angle(steering)
                                pca.write_us(pca.CH_ESC, out_us)

                            frames += 1
                            if frames % max(1, args.print_every) == 0:
                                print(f"[BENCH] {now - t0:6.1f}s  面积={gs.metrics.area_ratio:.3f} "
                                      f"有板={int(gs.blocked)} 武装={int(seen_board)}  "
                                      f"车道中心={obs.center_x:5.1f} 置信={obs.confidence:.2f} "
                                      f"→  舵机={steering:5.1f}°  电调={out_us:.0f}us  ({why})")

                            if args.show:
                                disp = frame.copy()
                                roi_y0 = int(frame.shape[0] * settings.LANE_ROI_TOP_RATIO)
                                roi_y1 = frame.shape[0] - settings.LANE_ROI_BOTTOM_MARGIN
                                cv2.line(disp, (int(obs.center_x), roi_y0), (int(obs.center_x), roi_y1),
                                         (0, 255, 0), 2)
                                color = (0, 0, 255) if gs.blocked else (0, 255, 0)
                                cv2.putText(disp, f"board={gs.blocked} area={gs.metrics.area_ratio:.3f} "
                                                  f"conf={obs.confidence:.2f} steer={steering:.0f} "
                                                  f"esc={out_us:.0f} ({why})",
                                            (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                                cv2.imshow("bench: track + board", disp)
                                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                                    break
                    finally:
                        cap.release()
    except Exception as exc:
        print(f"[BENCH] ⚠️ 异常：{exc}")
        rc = 1
    finally:
        if args.show:
            cv2.destroyAllWindows()
    return rc


if __name__ == "__main__":
    sys.exit(main())
