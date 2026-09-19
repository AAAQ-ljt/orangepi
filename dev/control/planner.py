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
        self._last_error_units: Optional[float] = None

    def reset(self) -> None:
        """切换状态 / 重新发车时调用，清 PID 与滤波器状态（否则会输出"踢腿"）。"""
        self.pid.reset()
        self.error_filter.reset()
        self._last_error_units = None

    def steering_offset(self, center_x: float, dt: float) -> float:
        """由横向观测量算出转向角增量（度）。

        转向符号由 `settings.STEER_SIGN` 决定（+1：角度增大=右转）。实车若发现"越修越偏"，
        说明符号反了 —— 把 `steer_sign: -1` 写进 `config/site.yaml` 即可，不用改代码。
        """
        error_px = max(-settings.LANE_MAX_ERROR_PX,
                       min(settings.LANE_MAX_ERROR_PX, float(center_x) - self.target_x))
        error_units = error_px / float(settings.LANE_ERROR_SCALE) * float(settings.STEER_SIGN)
        self._last_error_units = error_units
        filtered = self.error_filter.update(error_units)
        offset = self.pid.update(filtered, dt)
        return max(-self.max_steer, min(self.max_steer, offset))

    def _adaptive_throttle_scale(self) -> float:
        """按横向误差大小调油门（oldCode Control_FollowTrail 的自适应速度策略）。

        误差小（直道）→ 提速；误差大（弯道/正在纠偏）→ 减速保稳定。
        没有有效观测量时恒为 1.0。
        """
        err = self._last_error_units
        if err is None:
            return 1.0
        a = abs(err)
        if a < settings.LANE_ERR_FAST_UNITS:
            return float(settings.THROTTLE_FAST_SCALE)
        if a > settings.LANE_ERR_SLOW_UNITS:
            return float(settings.THROTTLE_SLOW_SCALE)
        return 1.0

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

        scale = float(throttle_scale) * self._adaptive_throttle_scale()
        if msg.blue_cone_count > 0 or msg.yellow_cone_count > 0:
            scale *= settings.CONE_THROTTLE_SCALE
        target.throttle = self.cruise_throttle * scale
        return target
