"""Mock 硬件：用于本地联调，不碰真实小车。"""
from __future__ import annotations

from typing import Dict


class MockPCA9685:
    def __init__(self):
        self.channels: Dict[int, float] = {}
        self.cleaned = False

    def init(self) -> None:
        self.center_all()

    def write_us(self, channel: int, pulse_us: float) -> None:
        self.channels[channel] = pulse_us

    def set_steering_angle(self, angle: float) -> None:
        self.write_us(0, angle)

    def set_esc_percent(self, percent: float, **kwargs) -> None:
        self.write_us(1, percent)

    def set_pan_angle(self, angle: float) -> None:
        self.write_us(2, angle)

    def set_tilt_angle(self, angle: float) -> None:
        self.write_us(3, angle)

    def center_all(self) -> None:
        self.set_steering_angle(90)
        self.set_pan_angle(90)
        self.set_tilt_angle(90)
        self.set_esc_percent(0)

    def cleanup(self) -> None:
        self.center_all()
        self.cleaned = True
