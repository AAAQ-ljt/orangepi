"""底层驱动骨架：后续对接 PCA9685。"""
from __future__ import annotations


class Driver:
    def __init__(self):
        self.armed = False

    def arm(self) -> None:
        # TODO: 电调解锁时序
        self.armed = True

    def set_steering(self, value: float) -> None:
        # TODO: 映射到舵机 PWM
        pass

    def set_throttle(self, value: float) -> None:
        # TODO: 映射到电调 PWM
        pass

    def safe_stop(self) -> None:
        # TODO: 电机归零 + 舵机回中 + 释放 PCA9685
        pass
