"""端到端 dry-run 集成测试：启动控制端，用 UDP 发送模拟感知消息。

安全：--dry-run 默认不驱动硬件。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import socket

from common.protocol import PerceptionMessage


def send_message(port: int, **kwargs):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    msg = PerceptionMessage(**kwargs).to_dict()
    sock.sendto(json.dumps(msg).encode("utf-8"), ("127.0.0.1", port))
    sock.close()


def main() -> int:
    dev = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ)
    env["PYTHONPATH"] = dev

    proc = subprocess.Popen(
        [sys.executable, "-u", os.path.join(dev, "main.py"), "--port", "5055"],
        cwd=dev,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        time.sleep(1.2)
        send_message(5055, is_barrier=True, center_x=320)
        time.sleep(0.2)
        send_message(5055, is_barrier=False, center_x=320)
        time.sleep(0.2)
        send_message(5055, center_x=300)
        time.sleep(0.2)
        send_message(5055, center_x=340)
        time.sleep(0.2)
        send_message(5055, has_zebra_crossing=True, center_x=330)
        time.sleep(0.5)
    finally:
        proc.terminate()
        try:
            out, _ = proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate(timeout=5)

    print(out)
    assert "WAIT_START -> TRACKING" in out, "FSM did not transition"
    assert "ZEBRA_STOP" in out, "FSM did not enter ZEBRA_STOP"
    print("INTEGRATION TEST PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
