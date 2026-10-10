#!/usr/bin/env python3
"""2026 智能车控制端入口。

默认 dry-run（不碰硬件）；显式加 --real 才会驱动 PCA9685。
运动前必须加 --arm 解锁，否则即使 real 模式也不会输出动力。
调试阶段默认把电调最大脉宽限制在 1600us，避免速度过快。
注意：1500us 停、**≈1545us 才起转**（2026-09-16 架空实测），调试脉宽的可用下限是 1545 而不是 1500。

安全交接（★ 与台架脚本同一套流程，见 `scripts/bench_common.py`）：`--real` 上场前
**先停 `opi-control`（手动遥控）让出 PCA9685**（两边同时写 I2C 会把舵机/电调变成
"两个主人交替驱动"）；退出时（Ctrl+C / TERM / HUP / 异常 / 关会话）电调归零、舵机
回中、释放 PCA9685，并把 opi-control 交还、逐项验证——**任何退出路径都不留
"既不能遥控、也不在自驾"的中间态**。dry-run 完全不碰服务与硬件。
"""
from __future__ import annotations

import argparse
import atexit
import os
import signal
import sys
import time

# 支持任意 cwd 直跑（car-mode.sh 用 PYTHONPATH，这里再兜一层）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import settings
from control.controller import Controller
from control.driver import Driver


def main() -> int:
    parser = argparse.ArgumentParser(description="2026 智能车控制端")
    parser.add_argument("--real", action="store_true",
                        help="真实硬件模式（默认 dry-run，不驱动电机）")
    parser.add_argument("--arm", action="store_true",
                        help="解锁动力输出（必须与 --real 同时使用才会动）")
    parser.add_argument("--port", type=int, default=settings.UDP_PORT,
                        help="UDP 监听端口")
    parser.add_argument("--target-x", type=float, default=settings.TARGET_X,
                        help="期望车道中心 x")
    parser.add_argument("--max-us", type=int, default=settings.ESC_DEBUG_MAX_US,
                        help=f"电调最大脉宽，调试默认 {settings.ESC_DEBUG_MAX_US}us（死区 {settings.ESC_DEADBAND_US}us）")
    parser.add_argument("--zebra-seconds", type=float, default=settings.ZEBRA_STOP_SECONDS,
                        help="斑马线停车时长（规则要求 10s；调试可调小，但比赛必须 ≥10）")
    args = parser.parse_args()

    if args.real:
        print(f"[MAIN] REAL mode, hardware enabled, ESC max = {args.max_us}us")
    else:
        print("[MAIN] DRY-RUN mode, no hardware movement")

    # ★ 退出保护：state 是信号处理/atexit/收尾共用的同一个 dict（字段与
    #   scripts/bench_common.py 预置的四键对齐，任何路径都不会读到缺失键）。
    state = {"stopped": False, "driver": None, "restored": False, "used_motor": False}
    restore_remote_stage = None
    if args.real:
        # ★ 手动阶段交接（与台架脚本同一套安全层）：真跑前让出 PCA9685。
        #   谁停谁恢复——退出时把 opi-control 交还，不然车留在"既不能遥控、
        #   也不在自驾"的中间态（网页摇杆/键盘全部失灵）。
        #   推流不归这里管：control 端不碰摄像头（vision 进程有自己的镜头
        #   独占守卫 camera_exclusive；car-mode.sh 也兜了一层）。
        from scripts.bench_common import restore_remote_stage as _restore, svc_stop
        restore_remote_stage = _restore
        print("[MAIN] 退出手动遥控阶段：停 opi-control（让出 PCA9685；退出时自动交还）")
        svc_stop("opi-control.service")
        time.sleep(1.5)
        state["used_motor"] = True
        atexit.register(restore_remote_stage, state, "atexit")

    driver = None
    controller = None
    try:
        driver = Driver(real=args.real, esc_max_us=args.max_us)
        state["driver"] = driver
        if args.real and args.arm:
            driver.arm()
            print("[MAIN] ARMED, motor/servo output enabled")
        elif args.real:
            print("[MAIN] NOT ARMED, motor/servo output disabled (use --arm)")

        controller = Controller(driver, port=args.port, target_x=args.target_x,
                                zebra_stop_seconds=args.zebra_seconds)

        def _signal_handler(_signum, _frame):
            print("\n[MAIN] signal received, stopping safely")
            state["stopped"] = True
            controller.stop()

        # AGENTS.md §1.1.4：所有退出路径都必须安全停车 —— 统一收敛到 _signal_handler
        # （2026-09-23 代码审查 B5：原来只捕 INT/TERM，ABRT（Python 内部致命错误）与
        # QUIT（某些 kill 组合）不经过它，controller.shutdown() 就不会执行）
        # 注：Windows 开发机的 Python 没有 SIGQUIT/SIGHUP（AttributeError），用 getattr 兼容。
        # 2026-10-10 补：HUP（SSH 关会话——断线也要安全停车 + 交还遥控）与 SIGPIPE 忽略。
        for _sig in (signal.SIGINT, signal.SIGTERM, signal.SIGABRT,
                     getattr(signal, "SIGQUIT", None), getattr(signal, "SIGHUP", None)):
            if _sig is not None:
                signal.signal(_sig, _signal_handler)
        if hasattr(signal, "SIGPIPE"):
            signal.signal(signal.SIGPIPE, signal.SIG_IGN)

        try:
            controller.run_forever()
        except KeyboardInterrupt:
            controller.stop()
    finally:
        # ★ 收尾（幂等）：real = 电调归零/舵机回中/释放 PCA9685 + 交还 opi-control
        #   并逐项验证；dry-run = 只清理 mock（不碰任何服务与硬件）。
        if args.real and restore_remote_stage is not None:
            restore_remote_stage(state, "控制端退出")
        elif driver is not None:
            driver.shutdown()
        print("[MAIN] exited cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
