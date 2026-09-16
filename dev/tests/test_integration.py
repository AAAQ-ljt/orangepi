"""端到端 dry-run 集成测试：启动控制端，用 UDP 发送模拟感知消息。

安全：不带 --real 时不会驱动硬件。
跑法：
    cd dev && PYTHONPATH=. python tests/test_integration.py
"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time

from common.protocol import PerceptionMessage


def send_message(port: int, **kwargs) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    msg = PerceptionMessage(**kwargs).to_dict()
    sock.sendto(json.dumps(msg).encode("utf-8"), ("127.0.0.1", port))
    sock.close()


def main() -> int:
    dev = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ)
    env["PYTHONPATH"] = dev
    port = 5055

    proc = subprocess.Popen(
        [sys.executable, "-u", os.path.join(dev, "main.py"), "--port", str(port)],
        cwd=dev,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        time.sleep(1.2)

        # 1) 只报"无遮挡"：绝不允许发车（安全红线）
        for _ in range(6):
            send_message(port, board_blocked=False, center_x=320, lane_confidence=1.0)
            time.sleep(0.05)

        # 2) 先出现挡板，再移开：满足边沿触发后才允许进 TRACKING
        send_message(port, board_blocked=True, center_x=320, lane_confidence=1.0)
        time.sleep(0.05)
        for _ in range(8):
            send_message(port, board_blocked=False, center_x=320, lane_confidence=1.0)
            time.sleep(0.05)

        # 3) 连续斑马线（满足防抖）→ ZEBRA_STOP
        for _ in range(4):
            send_message(port, board_blocked=False, center_x=330, lane_confidence=1.0,
                         has_zebra_crossing=True)
            time.sleep(0.05)
        time.sleep(0.3)
    finally:
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate(timeout=5)

    print(out)
    assert "WAIT_START -> TRACKING" in out, "满足边沿触发后 FSM 应进入 TRACKING"
    assert "ZEBRA_STOP" in out, "连续斑马线应进入 ZEBRA_STOP"

    # 安全断言：第一阶段（只报无遮挡）不许出现任何发车
    first_idx = out.find("[FSM] WAIT_START -> TRACKING")
    assert first_idx > 0, "未发生状态切换"
    assert "ZEBRA" not in out[:first_idx], "发车前不应触发斑马线逻辑"
    print("INTEGRATION TEST PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
