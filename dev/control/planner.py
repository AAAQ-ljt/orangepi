"""规划器：把观测量（center_x + 油门比例）转成 ControlTarget。

链路：center_x → 误差（像素，限幅）→ 误差滤波（离群剔除）→ PID → 转向角增量 → 叠加中位。
"哪些状态该走"由 FSM 决定、"降级时油门打几折"由仲裁层决定，见 controller.py。

护栏（2026-10-09 从 lane_ref_test 迁入 A 路径）：
- 单侧/弱对帧收紧转向限幅（LANE_SINGLE/WEAK_LIMIT_DEG，与 B 路径同口径）；
- 画龙（OscillationGuard）与跑偏（BigErrorGuard）触发 → 停车。
"""
from __future__ import annotations

from typing import Optional

from config import settings
from common.protocol import PerceptionMessage
from control.driver import ControlTarget
from control.filters import ErrorFilter
from control.lane_control import BigErrorGuard, OscillationGuard
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
        # 护栏（与 lane_ref_test 的 OscillationGuard/BigErrorGuard 同参同源）
        self._osc_guard = OscillationGuard()
        self._err_guard = BigErrorGuard()
        self._now = 0.0                      # 用 dt 累计的内部时钟（单测友好）
        self.guard_tripped = False           # 本帧护栏是否要求停车（供日志/测试断言）

    def reset(self) -> None:
        """切换状态 / 重新发车时调用，清 PID 与滤波器状态（否则会输出"踢腿"）。"""
        self.pid.reset()
        self.error_filter.reset()
        self._last_error_units = None
        self._osc_guard.reset()
        self._err_guard.reset()
        self._now = 0.0
        self.guard_tripped = False

    def _error_px(self, center_x: float) -> float:
        return max(-settings.LANE_MAX_ERROR_PX,
                   min(settings.LANE_MAX_ERROR_PX, float(center_x) - self.target_x))

    def steering_offset(self, center_x: float, dt: float) -> float:
        """由横向观测量算出转向角增量（度）。

        转向符号由 `settings.STEER_SIGN` 决定（+1：角度增大=右转）。实车若发现"越修越偏"，
        说明符号反了 —— 把 `steer_sign: -1` 写进 `config/site.yaml` 即可，不用改代码。
        """
        error_units = self._error_px(center_x) / float(settings.LANE_ERROR_SCALE) \
            * float(settings.STEER_SIGN)
        self._last_error_units = error_units
        filtered = self.error_filter.update(error_units)
        offset = self.pid.update(filtered, dt)
        return max(-self.max_steer, min(self.max_steer, offset))

    def steer_cap_deg(self, msg: PerceptionMessage) -> Optional[float]:
        """单侧/弱对帧的转向限幅（度）；正常双边帧返回 None（用全局限幅）。

        单侧：左右只有一侧有线 → 半宽兜底的运动中不可靠，限 ±5°；
        弱对：双侧都有但一侧支持线段 ≤ LANE_WEAK_MIN_SEGS → 限 ±10°。
        """
        single = (msg.left_x is None) != (msg.right_x is None)
        if single:
            return float(settings.LANE_SINGLE_LIMIT_DEG)
        if (msg.left_x is not None and msg.right_x is not None
                and msg.lane_n_left is not None and msg.lane_n_right is not None
                and min(msg.lane_n_left, msg.lane_n_right) <= settings.LANE_WEAK_MIN_SEGS):
            return float(settings.LANE_WEAK_LIMIT_DEG)
        return None

    def _adaptive_throttle_scale(self, throttle_scale: float) -> float:
        """按横向误差大小调油门（oldCode Control_FollowTrail 的自适应速度策略）。

        误差小（直道）→ 提速；误差大（弯道/正在纠偏）→ 减速保稳定。
        没有有效观测量时恒为 1.0。

        ⚠️ 只在仲裁层**未降级**（throttle_scale >= 1.0）时生效：
        降级路径（如半油门 50% ≈ 1550us）本来就在电调死区（1545us）边缘，
        再乘 0.85 → 42.5% ≈ 1542us 掉进死区，车直接停死（会被判"停止超 20s"）。
        """
        if throttle_scale < 1.0:
            return 1.0
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
            offset = self.steering_offset(x, dt)
            cap = self.steer_cap_deg(msg)
            if cap is not None:
                offset = max(-cap, min(cap, offset))
            target.steering = float(settings.SERVO_CENTER_ANGLE) + offset

        # ---- 护栏（dt 累计时钟；触发 → 回中位停车）----
        stop = bool(should_stop)
        if x is not None:
            self._now += max(1e-3, dt)
            raw_px = self._error_px(x)
            if self._osc_guard.update(self._now, target.steering,
                                      float(settings.SERVO_CENTER_ANGLE)):
                print("[CTRL] 护栏：画龙失稳，停车")
                stop = True
            elif self._err_guard.update(self._now, self._last_error_units * settings.LANE_ERROR_SCALE,
                                        raw_px):
                print("[CTRL] 护栏：误差过大/振荡，停车")
                stop = True
        self.guard_tripped = stop and not bool(should_stop)

        if stop:
            target.steering = float(settings.SERVO_CENTER_ANGLE)
            target.throttle = 0.0
            return target

        scale = float(throttle_scale) * self._adaptive_throttle_scale(float(throttle_scale))
        if msg.blue_cone_count > 0 or msg.yellow_cone_count > 0:
            scale *= settings.CONE_THROTTLE_SCALE
        target.throttle = self.cruise_throttle * scale
        return target
