#!/usr/bin/env python3
"""蓝板台架测试：**板在 → 停；板移开 → 跑（默认 1550us）；板放回 → 停**。

⚠️ 为什么这段逻辑只在测试程序里，不进主程序：
   比赛语义是"确认蓝板 → 板移开 → 发车"，**发车之后**再看到蓝板是正常的
   （停车区的挡板就是同一块蓝板）。主程序若按"看到蓝板就停"来跑，
   车会在停车区前把自己停住 —— 所以"板放回即停"只属于台架调试。

════════════════════════════ 安全设计（逐条对应"上电即远程控制阶段"的限制）════════════════════════════
| 约束 | 做法 |
|---|---|
| 不许抢 PCA9685 | 只有**驱动电机**时才停 `opi-control.service`；`--no-motor` 完全不碰它 |
| 不许占用摄像头而不还 | 用 `camera_exclusive()` 上下文：进则停 ffmpeg 推流，出则必恢复（含异常路径） |
| 没见过蓝板绝不能动 | `decide_output()`：未武装 → 中位 1500us（与主程序同一条红线，有单测） |
| 不能被误用来跑车 | 不加 `--i-know-wheels-are-up` 拒绝驱动电机；另有 `--no-motor` 纯检测模式 |
| 不能无限跑 | `--max-seconds`（默认 180s）到时自动停 |
| **任何退出路径都要归零** | Ctrl+C / kill / 关会话(SIGHUP) / 异常 / 正常结束 → 全部走同一个 `_restore()`，且用 `atexit` 兜底、幂等 |
| **退出后必须回到远程控制阶段** | `_restore()` 会重启 `opi-control` 并**逐一验证**（opi-control / ffmpeg 两路推流），打印 ✅/❌ |

用法（车上）：
    # ① 只测检测、不动电机（此时完全不碰 opi-control，远程控制阶段不受影响）
    sudo python3 /root/dev/scripts/bench_board_test.py --camera 0 --no-motor

    # ② 正式台架（四轮必须架空）：板在→停，板开→跑 1550us，板回→停
    sudo python3 /root/dev/scripts/bench_board_test.py --camera 0 --speed-us 1550 \
         --i-know-wheels-are-up

    # ③ 只验证"退出后能否恢复远程控制阶段"（速度用中位，电机不会转）
    sudo python3 /root/dev/scripts/bench_board_test.py --speed-us 1500 \
         --i-know-wheels-are-up --max-seconds 10
"""
from __future__ import annotations

import argparse
import atexit
import os
import signal
import subprocess
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
STREAM_SERVICES = ["ffmpeg-stream.service", "ffmpeg-stream-sub.service"]

# 模块级状态：信号处理与 atexit 都要能碰到
_STATE = {"driver": None, "restored": False, "stopped": False, "used_motor": False}


def _svc_active(name: str) -> bool:
    return subprocess.run(["systemctl", "is-active", "--quiet", name],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0


def _restore(reason: str = "") -> None:
    """停电机 + 恢复远程控制阶段，并**验证**恢复结果（幂等，任何退出路径都调它）。"""
    if _STATE["restored"]:
        return
    _STATE["restored"] = True
    print(f"\n[BENCH] 收尾{f'（{reason}）' if reason else ''}：")
    driver = _STATE["driver"]
    if driver is not None:
        try:
            driver.safe_stop()      # 电调 1500us、舵机回中
            driver.shutdown()       # 释放 PCA9685
            print("[BENCH]   电调已归零、舵机回中、PCA9685 已释放")
        except Exception as exc:    # 收尾阶段不能再抛异常
            print(f"[BENCH]   ⚠️ 释放驱动异常：{exc}")
    if _STATE["used_motor"]:
        subprocess.run(["systemctl", "start", "opi-control.service"], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        checks = [("opi-control.service", "远程控制（网页摇杆/键盘）")]
        checks += [(s, "图传推流") for s in STREAM_SERVICES]
        all_ok = True
        for svc, desc in checks:
            ok = _svc_active(svc)
            all_ok &= ok
            print(f"[BENCH]   {'✅' if ok else '❌'} {svc:28s} {desc} = {'active' if ok else '未启动'}")
        if all_ok:
            print("[BENCH] ✅ 已恢复**远程控制阶段**（可以正常遥控/看图传）")
        else:
            print("[BENCH] ⚠️ 有服务没起来，请手动执行："
                  "systemctl start opi-control ffmpeg-stream ffmpeg-stream-sub")
    else:
        # --no-motor：全程没停过 opi-control，只需确认推流回来了
        for svc in STREAM_SERVICES:
            ok = _svc_active(svc)
            print(f"[BENCH]   {'✅' if ok else '❌'} {svc:28s} 图传推流 = {'active' if ok else '未启动'}")
        print("[BENCH] ✅ 未改动远程控制阶段（本次只读摄像头）")


def _on_signal(signum, _frame):
    """Ctrl+C / kill / 关会话：只置标志，由循环退出后统一收尾（不在信号里做重活）。"""
    print(f"\n[BENCH] 收到信号 {signum}，准备收尾…")
    _STATE["stopped"] = True


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

    _STATE["used_motor"] = use_motor
    # 任何退出路径（含 atexit 兜底）都走同一个收尾
    atexit.register(_restore, "atexit")
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT, signal.SIGABRT):
        try:
            signal.signal(sig, _on_signal)
        except (ValueError, OSError):
            pass
    if hasattr(signal, "SIGPIPE"):
        try:
            signal.signal(signal.SIGPIPE, signal.SIG_IGN)   # 串口/管道断开不要直接杀进程
        except (ValueError, OSError):
            pass

    if use_motor:
        from control.driver import Driver
        print("[BENCH] 停止 opi-control（暂时让出 PCA9685；退出时会自动恢复）")
        subprocess.run(["systemctl", "stop", "opi-control.service"], check=False)
        time.sleep(1.5)
        driver = Driver(real=True, esc_max_us=int(max(args.speed_us, settings.ESC_DEBUG_MAX_US)))
        driver.arm()
        _STATE["driver"] = driver
        pca = driver.pca
        # 云台复位到中位：否则云台朝向会让板看不见，测试不可复现
        pca.set_pan_angle(PAN_CENTER)
        pca.set_tilt_angle(TILT_CENTER)
        pca.set_esc_percent(0)          # 先给中位，电调完成解锁
        time.sleep(1.0)
    else:
        pca = None

    gate = StartGate()          # 与主程序同一套检测（阈值同在 config/settings.py）
    seen_board = False

    print(f"[BENCH] 摄像头={args.camera}  速度={args.speed_us:.0f}us  "
          f"电机={'开（已停 opi-control，退出自动恢复）' if use_motor else '关(只测检测，不碰 opi-control)'}")
    print("[BENCH] 规则：没见板→中位1500us（绝不动）；见板→中位；板移开→跑；板放回→停")
    rc = 0
    try:
        # camera_exclusive：进则停 ffmpeg 推流，出则必恢复
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
                    while not _STATE["stopped"] and (time.time() - t0) < args.max_seconds:
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
    except Exception as exc:
        print(f"[BENCH] ⚠️ 异常：{exc}")
        rc = 1
    finally:
        if args.show:
            cv2.destroyAllWindows()
        _restore("正常结束")        # 幂等；atexit 再调也无害
    return rc


if __name__ == "__main__":
    sys.exit(main())
