"""FSM 状态机：2026 比赛任务链，带防抖和计时。"""
from __future__ import annotations

import time
from enum import Enum, auto

from config import settings
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

        # 防抖计数器
        self._zebra_frames = 0
        self._cone_frames = 0
        self._sign_frames = 0
        self._red_frames = 0
        self._green_frames = 0

    def _set(self, state: State) -> None:
        if state != self.state:
            print(f"[FSM] {self.state.name} -> {state.name}")
            self.state = state
            self._state_started = time.monotonic()
            # 切换状态时清空防抖计数
            self._zebra_frames = 0
            self._cone_frames = 0
            self._sign_frames = 0
            self._red_frames = 0
            self._green_frames = 0

    def update(self, msg: PerceptionMessage) -> State:
        now = time.monotonic()

        # 更新防抖计数
        if msg.has_zebra_crossing:
            self._zebra_frames += 1
        else:
            self._zebra_frames = 0

        if msg.blue_cone_count > 0:
            self._cone_frames += 1
        else:
            self._cone_frames = 0

        if msg.has_sign_a or msg.has_sign_b:
            self._sign_frames += 1
        else:
            self._sign_frames = 0

        if msg.traffic_light_state == "red":
            self._red_frames += 1
            self._green_frames = 0
        elif msg.traffic_light_state == "green":
            self._green_frames += 1
            self._red_frames = 0
        else:
            self._red_frames = 0
            self._green_frames = 0

        if self.state == State.WAIT_START:
            # 蓝色挡板消失后发车
            if not msg.is_barrier:
                self._set(State.TRACKING)

        elif self.state == State.TRACKING:
            if self._zebra_frames >= settings.ZEBRA_HYSTERESIS_FRAMES:
                self._set(State.ZEBRA_STOP)
            elif self._red_frames >= settings.TRAFFIC_LIGHT_HYSTERESIS_FRAMES:
                self._set(State.TRAFFIC_LIGHT)
            elif self._cone_frames >= settings.CONE_HYSTERESIS_FRAMES:
                self._set(State.AVOID_CONE)
            elif self._sign_frames >= settings.SIGN_HYSTERESIS_FRAMES:
                self._set(State.PARKING)

        elif self.state == State.ZEBRA_STOP:
            if now - self._state_started >= self.zebra_stop_seconds:
                self._set(State.TRACKING)

        elif self.state == State.TRAFFIC_LIGHT:
            if self._green_frames >= settings.TRAFFIC_LIGHT_HYSTERESIS_FRAMES:
                self._set(State.TRACKING)

        elif self.state == State.AVOID_CONE:
            if self._cone_frames == 0 and now - self._state_started >= 0.5:
                self._set(State.TRACKING)

        elif self.state == State.PARKING:
            if now - self._state_started >= 2.0:
                self._set(State.DONE)

        return self.state
