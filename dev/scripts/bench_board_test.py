#!/usr/bin/env python3
"""蓝板台架测试：**板在 → 停；板移开 → 跑（默认 1550us）；板放回 → 停**。

⚠️ 为什么这段逻辑只在测试程序里，不进主程序：
   比赛语义是"确认蓝板 → 板移开 → 发车"，**发车之后**再看到蓝板是正常的
   （停车区的挡板就是同一块蓝板）。主程序若按"看到蓝板就停"来跑，
   车会在停车区前把自己停住 —— 所以"板放回即停"只属于台架调试。

安全层见 `scripts/bench_common.py`（唯一实现处，所有台架脚本共用）：
    · 只有驱动电机时才停 opi-control；--no-motor 完全不碰它
    · 摄像头用 camera_exclusive：进则停推流、出则必恢复
    · 没见过蓝板绝不输出动力（decide_output 纯函数，有单测）
    · 不加 --i-know-wheels-are-up 拒绝驱动电机；--max-seconds 到时自动停
    · **任何退出路径**（Ctrl+C / kill / 关会话 SIGHUP / 异常 / 正常）都归零并恢复远程控制阶段，
      且恢复后逐项验证打印 ✅/❌

用法（车上，四轮必须架空）：
    sudo python3 /root/dev/scripts/bench_board_test.py --camera 0 --no-motor   # 只测检测
    sudo python3 /root/dev/scripts/bench_board_test.py --camera 0 --speed-us 1550 \
         --i-know-wheels-are-up                                                # 正式台架
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from config import settings
from scripts.bench_common import NEUTRAL_US, MotorSession, install_signal_guard
from vision.camera_guard import camera_exclusive
from vision.start_gate import StartGate


def decide_output(blocked: bool, seen_board: bool, speed_us: float,
                  neutral_us: float = NEUTRAL_US) -> float:
    """台架规则（纯函数，便于单测）：没见过板 → 中位；有板 → 中位；没板 → speed_us。"""
    if not seen_board or blocked:
        return neutral_us
    return speed_us


def main() -> int:
    ap = argparse.ArgumentParser(description="蓝板台架测试（板在→停 / 板开→跑 / 板回→停）")
    ap.add_argument("--camera", type=int, default=0, help="看蓝板的摄像头：0=云台主摄（默认），2=下摄")
    ap.add_argument("--speed-us", type=float, default=1550.0, help="放行后的电调脉宽（默认 1550）")
    ap.add_argument("--max-seconds", type=float, default=180.0, help="最长运行时间，到时自动停")
    ap.add_argument("--show", action="store_true", help="显示预览窗口（需要 X11）")
    ap.add_argument("--no-motor", action="store_true", help="只测检测，不输出任何动力（不碰 opi-control）")
    ap.add_argument("--i-know-wheels-are-up", action="store_true",
                    help="确认四轮已架空（不加这个且没加 --no-motor 时拒绝运行）")
    ap.add_argument("--width", type=int, default=settings.IMG_W)
    ap.add_argument("--height", type=int, default=settings.IMG_H)
    ap.add_argument("--print-every", type=int, default=5, help="每 N 帧打印一行")
    args = ap.parse_args()

    use_motor = not args.no_motor
    if use_motor and not args.i_know_wheels_are_up:
        print("[BENCH] 拒绝运行：请确认四轮已架空，并加 --i-know-wheels-are-up"
              "（或先用 --no-motor 只测检测）")
        return 2

    state = {"stopped": False, "driver": None, "restored": False, "used_motor": use_motor}
    install_signal_guard(state)

    gate = StartGate()          # 与主程序同一套检测（阈值同在 config/settings.py）
    seen_board = False

    print(f"[BENCH] 摄像头={args.camera}  速度={args.speed_us:.0f}us  "
          f"电机={'开（已停 opi-control，退出自动恢复）' if use_motor else '关(只测检测，不碰 opi-control)'}")
    print("[BENCH] 规则：没见板→中位1500us（绝不动）；见板→中位；板移开→跑；板放回→停")

    rc = 0
    try:
        with MotorSession(state, enabled=use_motor, speed_us_max=args.speed_us) as pca:
            with camera_exclusive():
                cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    print("[BENCH] 摄像头打不开（推流是否已释放？）")
                    rc = 1
                else:
                    t0 = time.time()
                    frames = 0
                    try:
                        while not state["stopped"] and (time.time() - t0) < args.max_seconds:
                            ok, frame = cap.read()
                            if not ok or frame is None:
                                time.sleep(0.05)
                                continue
                            st = gate.update(frame)
                            if st.blocked:
                                seen_board = True
                            out_us = decide_output(st.blocked, seen_board, args.speed_us)
                            if use_motor and pca is not None:
                                pca.write_us(pca.CH_ESC, out_us)

                            frames += 1
                            if frames % max(1, args.print_every) == 0:
                                print(f"[BENCH] {time.time() - t0:6.1f}s  "
                                      f"面积={st.metrics.area_ratio:.3f} 主导度={st.metrics.dominance:5.1f}  "
                                      f"有板={int(st.blocked)} 武装={int(seen_board)}  →  "
                                      f"电调={out_us:.0f}us ({'停' if out_us <= NEUTRAL_US else '跑'})")

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
    except Exception as exc:
        print(f"[BENCH] ⚠️ 异常：{exc}")
        rc = 1
    finally:
        if args.show:
            cv2.destroyAllWindows()
    return rc


if __name__ == "__main__":
    sys.exit(main())
