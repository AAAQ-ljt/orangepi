"""PCA9685 硬件抽象，使用 adafruit_extended_bus 指定香橙派 I2C 总线。"""
from __future__ import annotations

from typing import Optional

try:
    from adafruit_extended_bus import ExtendedI2C as I2C
    from adafruit_pca9685 import PCA9685 as _PCA9685
    _HAS_HW = True
except Exception:  # pragma: no cover - 本地无硬件时允许导入
    _HAS_HW = False


class PCA9685:
    """封装 PCA9685：转向舵机、电调、云台。"""

    CH_STEERING = 0
    CH_ESC = 1
    CH_PAN = 2
    CH_TILT = 3

    def __init__(self, bus_id: int = 5, address: int = 0x40, freq: int = 50):
        if not _HAS_HW:
            raise RuntimeError("PCA9685 hardware libraries not available")
        self.bus_id = bus_id
        self.address = address
        self.freq = freq
        self._i2c: Optional[I2C] = None
        self._pca: Optional[_PCA9685] = None

    def init(self) -> None:
        if self._pca is not None:
            return
        self._i2c = I2C(self.bus_id)
        self._pca = _PCA9685(self._i2c, address=self.address)
        self._pca.frequency = self.freq
        self.center_all()

    def _ensure(self):
        if self._pca is None:
            raise RuntimeError("PCA9685 not initialized")

    def _us_to_duty(self, pulse_us: float) -> int:
        period_us = 20000.0  # 50Hz
        duty = int((pulse_us / period_us) * 65535)
        return max(0, min(65535, duty))

    def write_us(self, channel: int, pulse_us: float) -> None:
        self._ensure()
        self._pca.channels[channel].duty_cycle = self._us_to_duty(pulse_us)

    def angle_to_us(self, angle: float) -> float:
        angle = max(0.0, min(180.0, float(angle)))
        return 500.0 + (angle / 180.0) * 2000.0

    def set_steering_angle(self, angle: float) -> None:
        self.write_us(self.CH_STEERING, self.angle_to_us(angle))

    def set_esc_percent(self, percent: float, stop_us: int = 1500,
                        min_us: int = 1400, max_us: int = 2000) -> None:
        percent = max(-100.0, min(100.0, float(percent)))
        if percent == 0:
            pulse = float(stop_us)
        elif percent > 0:
            pulse = stop_us + (percent / 100.0) * (max_us - stop_us)
        else:
            pulse = stop_us + (percent / 100.0) * (stop_us - min_us)
        self.write_us(self.CH_ESC, pulse)

    def set_pan_angle(self, angle: float) -> None:
        self.write_us(self.CH_PAN, self.angle_to_us(angle))

    def set_tilt_angle(self, angle: float) -> None:
        self.write_us(self.CH_TILT, self.angle_to_us(angle))

    def center_all(self) -> None:
        if self._pca is None:
            return
        self.set_steering_angle(90)
        self.set_pan_angle(90)
        self.set_tilt_angle(90)
        self.set_esc_percent(0)

    def cleanup(self) -> None:
        if self._pca is None:
            return
        self.center_all()
        self._pca.deinit()
        self._pca = None
        self._i2c = None
