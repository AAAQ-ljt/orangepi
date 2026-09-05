"""控制端驱动：把 ControlTarget 写入真实/模拟硬件。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from hardware.mock import MockPCA9685
from hardware.pca9685 import PCA9685


@dataclass
class ControlTarget:
    steering: float = 90.0       # 角度 0~180，默认中位 90
    throttle: float = 0.0        # -100~100
    pan: Optional[float] = None
    tilt: Optional[float] = None


class Driver:
    def __init__(self, pca=None, real: bool = False):
        self.real = real
        if real:
            self.pca = pca if pca is not None else PCA9685()
            self.pca.init()
        else:
            self.pca = pca if pca is not None else MockPCA9685()
            self.pca.init()
        self.armed = False

    def arm(self) -> None:
        self.armed = True

    def disarm(self) -> None:
        self.armed = False
        self.safe_stop()

    def execute(self, target: ControlTarget) -> None:
        if not self.armed:
            # 未解锁时只允许安全停止
            self.pca.set_steering_angle(90)
            self.pca.set_esc_percent(0)
            return

        self.pca.set_steering_angle(target.steering)
        self.pca.set_esc_percent(target.throttle)
        if target.pan is not None:
            self.pca.set_pan_angle(target.pan)
        if target.tilt is not None:
            self.pca.set_tilt_angle(target.tilt)

    def safe_stop(self) -> None:
        self.pca.center_all()
        if hasattr(self.pca, "cleaned"):
            self.pca.cleaned = True

    def shutdown(self) -> None:
        self.disarm()
        self.pca.cleanup()
