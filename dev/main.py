#!/usr/bin/env python3
"""2026 智能车控制端入口。

默认 dry-run（不碰硬件）；显式加 --real 才会驱动 PCA9685。
"""
from __future__ import annotations

import argparse
import signal
import sys

from control.controller import Controller
from control.driver import Driver


def main() -> int:
    parser = argparse.ArgumentParser(description="2026 智能车控制端")
    parser.add_argument("--real", action="store_true",
                        help="真实硬件模式（默认 dry-run，不驱动电机）")
    parser.add_argument("--port", type=int, default=5000,
                        help="UDP 监听端口")
    parser.add_argument("--target-x", type=float, default=320.0,
                        help="期望车道中心 x")
    args = parser.parse_args()

    if args.real:
        print("[MAIN] REAL mode, hardware enabled")
    else:
        print("[MAIN] DRY-RUN mode, no hardware movement")

    driver = Driver(real=args.real)
    controller = Controller(driver, port=args.port, target_x=args.target_x)

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
