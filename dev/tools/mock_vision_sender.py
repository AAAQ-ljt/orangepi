"""模拟视觉端：向控制端发送测试用 PerceptionMessage。

用于在完全没有真实摄像头/模型的情况下验证控制端流程。
用法：
    python tools/mock_vision_sender.py --target 127.0.0.1 --port 5000 --duration 60
"""
from __future__ import annotations

import argparse
import json
import socket
import time

from common.protocol import PerceptionMessage


def build_message(**kwargs) -> dict:
    msg = PerceptionMessage(**kwargs)
    return msg.to_dict()


def main() -> int:
    parser = argparse.ArgumentParser(description="Mock vision sender")
    parser.add_argument("--target", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.time()

    print(f"[MOCK] sending to {args.target}:{args.port} for {args.duration}s")

    # 简单脚本序列，方便观察 FSM 切换
    script = [
        # (delay, message_kwargs)
        (1.0, dict(is_barrier=True, center_x=320)),
        (1.0, dict(is_barrier=False, center_x=320)),          # 发车
        (3.0, dict(center_x=300)),
        (3.0, dict(center_x=340)),
        (2.0, dict(center_x=330, has_zebra_crossing=True)),   # 斑马线
        (12.0, dict(center_x=330)),                            # 斑马线结束
        (2.0, dict(center_x=320, traffic_light_state="red")), # 红灯
        (4.0, dict(center_x=320, traffic_light_state="green")),# 绿灯
        (2.0, dict(center_x=320, blue_cone_count=1)),          # 锥桶
        (3.0, dict(center_x=320, blue_cone_count=0)),
        (2.0, dict(center_x=320, has_sign_a=True)),            # 停车
        (3.0, dict(center_x=320, has_sign_a=True)),
    ]

    try:
        for delay, kwargs in script:
            if time.time() - start >= args.duration:
                break
            time.sleep(delay)
            data = json.dumps(build_message(**kwargs)).encode("utf-8")
            sock.sendto(data, (args.target, args.port))
            print(f"[MOCK] {kwargs}")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
