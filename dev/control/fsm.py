"""FSM 状态机：2026 比赛任务链。"""
from __future__ import annotations

import time
from enum import Enum, auto

from common.protocol import PerceptionMessage


class State(Enum):
    WAIT_START = auto()
    TRACKING = auto()
    ZEBRA_STOP = auto()
    TRAFFIC_LIGHT = auto()
    AVOID_CONE = auto()
    PARKING = auto()
    DONE = auto()


class FSM:
    def __init__(self, zebra_stop_seconds: float = 10.0):
        self.zebra_stop_seconds = zebra_stop_seconds
        self.state = State.WAIT_START
        self._state_started = time.monotonic()

    def _set(self, state: State) -> None:
        if state != self.state:
            print(f"[FSM] {self.state.name} -> {state.name}")
            self.state = state
            self._state_started = time.monotonic()

    def update(self, msg: PerceptionMessage) -> State:
        now = time.monotonic()

        if self.state == State.WAIT_START:
            # 蓝色挡板消失后发车
            if not msg.is_barrier:
                self._set(State.TRACKING)

        elif self.state == State.TRACKING:
            if msg.has_zebra_crossing:
                self._set(State.ZEBRA_STOP)
            elif msg.traffic_light_state == "red":
                self._set(State.TRAFFIC_LIGHT)
            elif msg.blue_cone_count > 0:
                self._set(State.AVOID_CONE)
            elif msg.has_sign_a or msg.has_sign_b:
                self._set(State.PARKING)

        elif self.state == State.ZEBRA_STOP:
            if now - self._state_started >= self.zebra_stop_seconds:
                self._set(State.TRACKING)

        elif self.state == State.TRAFFIC_LIGHT:
            if msg.traffic_light_state == "green":
                self._set(State.TRACKING)

        elif self.state == State.AVOID_CONE:
            if msg.blue_cone_count == 0:
                self._set(State.TRACKING)

        elif self.state == State.PARKING:
            # 简单处理：停车后短暂停留进入 DONE
            if now - self._state_started >= 2.0:
                self._set(State.DONE)

        return self.state
