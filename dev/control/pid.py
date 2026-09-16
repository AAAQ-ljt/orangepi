"""位置式 PID（带 dt、输出限幅、积分限幅、微分低通）。

与官方参考实现的差别（那些是它的坑，不要照抄）：
- **带 dt**：积分与微分都按时间归一，控制频率变化时行为一致；
- **积分限幅**：防积分饱和；
- **微分低通**：官方直接对含噪误差求差分，噪声 × kd 被放大；
- **reset() 清状态**：官方 set_PID 不重置，切状态时输出会踢一下。
"""
from __future__ import annotations

from typing import Optional

from config import settings


class PID:
    def __init__(self,
                 kp: float,
                 ki: float = 0.0,
                 kd: float = 0.0,
                 out_limit: Optional[float] = None,
                 integral_limit: Optional[float] = None,
                 d_alpha: float = None):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = None if out_limit is None else float(out_limit)
        self.integral_limit = None if integral_limit is None else float(integral_limit)
        self.d_alpha = float(settings.LANE_DERIV_ALPHA if d_alpha is None else d_alpha)
        self._integral = 0.0
        self._prev_error: Optional[float] = None
        self._prev_d = 0.0
        self._last_dt = 1.0 / 30.0

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = None
        self._prev_d = 0.0

    def update(self, error: float, dt: float) -> float:
        error = float(error)
        dt = float(dt)
        if dt <= 1e-6:
            dt = self._last_dt          # 时间戳异常时不产生除零/爆炸
        self._last_dt = dt

        # 积分（限幅）
        self._integral += error * dt
        if self.integral_limit is not None:
            self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))

        # 微分（低通）
        if self._prev_error is None:
            raw_d = 0.0
        else:
            raw_d = (error - self._prev_error) / dt
        d = self.d_alpha * raw_d + (1.0 - self.d_alpha) * self._prev_d
        self._prev_d = d
        self._prev_error = error

        out = self.kp * error + self.ki * self._integral + self.kd * d
        if self.out_limit is not None:
            out = max(-self.out_limit, min(self.out_limit, out))
        return out
