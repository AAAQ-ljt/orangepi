"""UDP 接收端：接收视觉 JSON。"""
from __future__ import annotations

import json
import socket
from typing import Optional

from common.protocol import PerceptionMessage


class UDPServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 5000, timeout: float = 0.5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(timeout)

    def recv(self) -> Optional[PerceptionMessage]:
        try:
            data, _addr = self.sock.recvfrom(65535)
        except socket.timeout:
            return None
        except OSError:
            return None
        try:
            payload = json.loads(data.decode("utf-8"))
            return PerceptionMessage.from_dict(payload)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
