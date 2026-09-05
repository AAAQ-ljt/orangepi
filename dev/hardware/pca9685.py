"""PCA9685 硬件抽象骨架。"""
from __future__ import annotations


class PCA9685:
    def __init__(self, bus: int = 5, address: int = 0x40):
        self.bus = bus
        self.address = address

    def init(self) -> None:
        # TODO: 初始化 I2C
        pass

    def set_pwm(self, channel: int, us: int) -> None:
        # TODO: 设置通道脉宽
        pass

    def release(self) -> None:
        # TODO: 释放 PCA9685
        pass
