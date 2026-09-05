"""规划器：根据感知消息生成 ControlTarget。"""
from __future__ import annotations

from common.protocol import PerceptionMessage
from control.driver import ControlTarget


class Planner:
    def __init__(self, target_x: float = 320.0,
                 max_steer: float = 30.0,
                 cruise_throttle: float = 25.0):
        self.target_x = target_x
        self.max_steer = max_steer
        self.cruise_throttle = cruise_throttle

    def plan(self, msg: PerceptionMessage) -> ControlTarget:
        steering = 90.0
        throttle = 0.0

        if msg.center_x is not None:
            error = msg.center_x - self.target_x
            # error -> steering offset，限制在 ±max_steer
            offset = max(-self.max_steer, min(self.max_steer, -error * 0.15))
            steering = 90.0 + offset

        if msg.is_barrier:
            throttle = 0.0
        elif msg.has_zebra_crossing:
            throttle = 0.0
        elif msg.traffic_light_state == "red":
            throttle = 0.0
        elif msg.has_sign_a or msg.has_sign_b:
            throttle = 0.0
        elif msg.blue_cone_count > 0:
            # 有锥桶先降速，后续再细化绕障
            throttle = self.cruise_throttle * 0.4
        else:
            throttle = self.cruise_throttle

        return ControlTarget(steering=steering, throttle=throttle)
