"""规划器：把观测量（center_x + 油门比例）转成 ControlTarget。

链路：center_x → 误差（像素，限幅）→ 误差滤波（离群剔除）→ PID → 转向角增量 → 叠加中位。
"哪些状态该走"由 FSM 决定、"降级时油门打几折"由仲裁层决定，见 controller.py。
"""
from __future__ import annotations

from typing import Optional

from config import settings
from common.protocol import PerceptionMessage
from control.driver import ControlTarget
from control.filters import ErrorFilter
from control.pid import PID


class Planner:
    def __init__(self,
                 target_x: float = None,
                 max_steer: float = None,
                 cruise_throttle: float = None,
                 pid: Optional[PID] = None,
                 error_filter: Optional[ErrorFilter] = None):
        self.target_x = float(settings.TARGET_X if target_x is None else target_x)
        self.max_steer = float(settings.LANE_STEER_LIMIT_DEG if max_steer is None else max_steer)
        self.cruise_throttle = float(settings.CRUISE_THROTTLE
                                     if cruise_throttle is None else cruise_throttle)
        self.pid = pid if pid is not None else PID(
            kp=settings.LANE_KP,
            ki=settings.LANE_KI,
            kd=settings.LANE_KD,
            out_limit=settings.LANE_STEER_LIMIT_DEG,
            integral_limit=settings.LANE_INTEGRAL_LIMIT,
        )
        self.error_filter = error_filter if error_filter is not None else ErrorFilter()

    def reset(self) -> None:
        """切换状态 / 重新发车时调用，清 PID 与滤波器状态（否则会输出"踢腿"）。"""
        self.pid.reset()
        self.error_filter.reset()

    def steering_offset(self, center_x: float, dt: float) -> float:
        """由横向观测量算出转向角增量（度）。"""
        error_px = max(-settings.LANE_MAX_ERROR_PX,
                       min(settings.LANE_MAX_ERROR_PX, float(center_x) - self.target_x))
        error_units = error_px / float(settings.LANE_ERROR_SCALE)
        filtered = self.error_filter.update(error_units)
        offset = self.pid.update(filtered, dt)
        return max(-self.max_steer, min(self.max_steer, offset))

    def plan(self,
             msg: PerceptionMessage,
             center_x: Optional[float] = None,
             throttle_scale: float = 1.0,
             should_stop: bool = False,
             dt: float = 1.0 / 30.0) -> ControlTarget:
        target = ControlTarget(steering=float(settings.SERVO_CENTER_ANGLE), throttle=0.0)

        x = msg.center_x if center_x is None else center_x
        if x is not None:
            target.steering = float(settings.SERVO_CENTER_ANGLE) + self.steering_offset(x, dt)

        if should_stop:
            target.throttle = 0.0
            return target

        scale = float(throttle_scale)
        if msg.blue_cone_count > 0 or msg.yellow_cone_count > 0:
            scale *= settings.CONE_THROTTLE_SCALE
        target.throttle = self.cruise_throttle * scale
        return target
