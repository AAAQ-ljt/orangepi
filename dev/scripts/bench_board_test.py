#!/usr/bin/env python3
"""蓝板台架测试：**板在 → 停；板移开 → 跑（默认 1550us）；板放回 → 停**。

⚠️ 为什么这段逻辑只在测试程序里，不进主程序：
   比赛语义是"确认蓝板 → 板移开 → 发车"，**发车之后**再看到蓝板是正常的
   （停车区的挡板就是同一块蓝板）。主程序若按"看到蓝板就停"来跑，
   车会在停车区前把自己停住 —— 所以"板放回即停"只属于台架调试。

安全约束（都写在代码里，不是靠人记）：
   1. **没见过蓝板，绝不输出动力**（与主程序同一条红线，复用 StartGate 的武装状态）；
   2. 默认 `--no-motor` 之外，必须显式 `--i-know-wheels-are-up` 才会真正驱动电机；
   3. `--max-seconds` 到时自动停（默认 180s）；
   4. Ctrl+C / 退出 → 电调归零、舵机回中、恢复 opi-control 与推流；
   5. 运行时把云台复位到中位（否则云台角度会让板看不见，测试不可复现）。

用法（车上，四轮必须架空）：
    # 只看检测、不动电机（先确认能稳定识别到板）
    sudo python3 /root/dev/scripts/bench_board_test.py --camera 0 --no-motor

    # 正式台架：板在→停，板开→跑 1550us，板回→停
    sudo python3 /root/dev/scripts/bench_board_test.py --camera 0 --speed-us 1550 \
         --i-know-wheels-are-up

    # 带预览窗口（需要 X11 / 车上显示器）
    sudo python3 /root/dev/scripts/bench_board_test.py --camera 0 --show --no-motor
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time

# 允许直接 `sudo python3 /root/dev/scripts/bench_board_test.py` 运行（不必手动设 PYTHONPATH）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from config import settings
from vision.camera_guard import camera_exclusive
from vision.start_gate import StartGate

NEUTRAL_US = 1500.0          # 电调中位（停止）
PAN_CENTER, TILT_CENTER = 90, 90


def decide_output(blocked: bool, seen_board: bool, speed_us: float,
                  neutral_us: float = NEUTRAL_US) -> float:
    """台架规则（纯函数，便于单测）：
    没见过板 → 中位；有板 → 中位；没板 → speed_us。
    """
    if not seen_board or blocked:
        return neutral_us
    return speed_us


def main() -> int:
    ap = argparse.ArgumentParser(description="蓝板台架测试（板在→停 / 板开→跑 / 板回→停）")
    ap.add_argument("--camera", type=int, default=0,
                    help="看蓝板的摄像头：0=云台主摄（默认），2=下摄")
    ap.add_argument("--speed-us", type=float, default=1550.0, help="放行后的电调脉宽（默认 1550）")
    ap.add_argument("--max-seconds", type=float, default=180.0, help="最长运行时间，到时自动停")
    ap.add_argument("--show", action="store_true", help="显示预览窗口（需要 X11）")
    ap.add_argument("--no-motor", action="store_true", help="只测检测，不输出任何动力")
    ap.add_argument("--i-know-wheels-are-up", action="store_true",
                    help="确认四轮已架空（不加这个且没加 --no-motor 时拒绝运行）")
    ap.add_argument("--width", type=int, default=settings.IMG_W)
    ap.add_argument("--height", type=int, default=settings.IMG_H)
    ap.add_argument("--print-every", type=int, default=5, help="每 N 帧打印一行")
    args = ap.parse_args()

    use_motor = not args.no_motor
    if use_motor and not args.i_know_wheels_are_up:
        print("[BENCH] 拒绝运行：请确认四轮已架空，并加 --i-know-wheels-are-up（或先用 --no-motor 只测检测）")
        return 2

    pca = None
    driver = None
    if use_motor:
        import subprocess
        from control.driver import Driver
        print("[BENCH] 停止 opi-control（释放 PCA9685），准备输出动力")
        subprocess.run(["systemctl", "stop", "opi-control.service"], check=False)
        time.sleep(1.5)
        driver = Driver(real=True, esc_max_us=int(max(args.speed_us, settings.ESC_DEBUG_MAX_US)))
        driver.arm()
        pca = driver.pca
        # 云台复位到中位：否则云台朝向会让板看不见，测试不可复现
        pca.set_pan_angle(PAN_CENTER)
        pca.set_tilt_angle(TILT_CENTER)
        pca.set_esc_percent(0)

    gate = StartGate()          # 与主程序同一套检测（阈值同在 config/settings.py）
    seen_board = False
    stopped = False

    def _cleanup(*_a):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    print(f"[BENCH] 摄像头={args.camera}  速度={args.speed_us:.0f}us  电机={'开' if use_motor else '关(只测检测)'}")
    print("[BENCH] 规则：没见板→中位1500us（绝不动）；见板→中位；板移开→跑；板放回→停")
    try:
        with camera_exclusive():
            cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                print("[BENCH] 摄像头打不开")
                return 1
            t0 = time.time()
            frames = 0
            out_us = NEUTRAL_US
            try:
                while not stopped and (time.time() - t0) < args.max_seconds:
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        time.sleep(0.05)
                        continue
                    st = gate.update(frame)
                    if st.blocked:
                        seen_board = True
                    out_us = decide_output(st.blocked, seen_board, args.speed_us)
                    if use_motor:
                        pca.write_us(pca.CH_ESC, out_us)

                    frames += 1
                    if frames % max(1, args.print_every) == 0:
                        el = time.time() - t0
                        print(f"[BENCH] {el:6.1f}s  面积={st.metrics.area_ratio:.3f} "
                              f"主导度={st.metrics.dominance:5.1f}  有板={int(st.blocked)} "
                              f"武装={int(seen_board)}  →  电调={out_us:.0f}us "
                              f"({'停' if out_us <= NEUTRAL_US else '跑'})")

                    if args.show:
                        disp = frame.copy()
                        color = (0, 0, 255) if st.blocked else (0, 255, 0)
                        cv2.putText(disp, f"board={st.blocked} area={st.metrics.area_ratio:.3f} "
                                          f"dom={st.metrics.dominance:.0f} esc={out_us:.0f}us",
                                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        cv2.imshow("bench: blue board", disp)
                        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                            break
            finally:
                cap.release()
    finally:
        if driver is not None:
            driver.safe_stop()
            driver.shutdown()
            import subprocess
            print("[BENCH] 电调归零、恢复 opi-control")
            subprocess.run(["systemctl", "start", "opi-control.service"], check=False)
        if args.show:
            cv2.destroyAllWindows()
        print("[BENCH] 结束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
