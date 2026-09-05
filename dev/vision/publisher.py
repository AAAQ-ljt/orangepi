"""UDP 发送封装。"""
from __future__ import annotations

import json
import socket
from typing import Any


class UDPPublisher:
    def __init__(self, host: str = "127.0.0.1", port: int = 9101):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, obj: Any) -> None:
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.sock.sendto(data, (self.host, self.port))

    def close(self) -> None:
        self.sock.close()
