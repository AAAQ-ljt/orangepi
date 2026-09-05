#!/usr/bin/env python3
"""2026 智能车控制端入口。

默认 dry-run（不碰硬件）；显式加 --real 才会驱动 PCA9685。
调试阶段默认把电调最大脉宽限制在 1540us，避免速度过快。
"""
from __future__ import annotations

import argparse
import signal
import sys

from config import settings
from control.controller import Controller
from control.driver import Driver


def main() -> int:
    parser = argparse.ArgumentParser(description="2026 智能车控制端")
    parser.add_argument("--real", action="store_true",
                        help="真实硬件模式（默认 dry-run，不驱动电机）")
    parser.add_argument("--port", type=int, default=settings.UDP_PORT,
                        help="UDP 监听端口")
    parser.add_argument("--target-x", type=float, default=settings.TARGET_X,
                        help="期望车道中心 x")
    parser.add_argument("--max-us", type=int, default=settings.ESC_DEBUG_MAX_US,
                        help="电调最大脉宽，调试默认 1540us")
    parser.add_argument("--zebra-seconds", type=float, default=10.0,
                        help="斑马线停车时长，调试可调小")
    args = parser.parse_args()

    if args.real:
        print(f"[MAIN] REAL mode, hardware enabled, ESC max = {args.max_us}us")
    else:
        print("[MAIN] DRY-RUN mode, no hardware movement")

    driver = Driver(real=args.real, esc_max_us=args.max_us)
    controller = Controller(driver, port=args.port, target_x=args.target_x,
                            zebra_stop_seconds=args.zebra_seconds)

    def _signal_handler(_signum, _frame):
        print("\n[MAIN] signal received, stopping safely")
        controller.stop()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        controller.run_forever()
    except KeyboardInterrupt:
        controller.stop()
    finally:
        driver.shutdown()
        print("[MAIN] exited cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
