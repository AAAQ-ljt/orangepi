"""IMU 硬件抽象骨架。"""
from __future__ import annotations


class IMU:
    def __init__(self, device: str = "/dev/ttyS0", baudrate: int = 9600):
        self.device = device
        self.baudrate = baudrate

    def init(self) -> None:
        # TODO: 初始化串口
        pass

    def read_yaw(self) -> float:
        # TODO: 读取航向角
        return 0.0

    def close(self) -> None:
        pass
