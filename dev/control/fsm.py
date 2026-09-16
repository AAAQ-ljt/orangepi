"""FSM 状态机：2026 比赛任务链，带防抖和计时。

发车判据（主方案 §3.1.7）：
    必须**边沿触发** —— 先确认"有遮挡"（`board_blocked=True`），
    再等"遮挡消失"连续 N 帧，才进入 TRACKING。
    绝不允许"当前无遮挡就发车"（上电即冲）。

本文件当前只实现到"能安全跑通巡线 + 斑马线停车 + 红绿灯等待"，
锥桶绕行与停车入库属于 P1-3 / P1-4（见 doc/执行路线图.md）。
"""
from __future__ import annotations

import time
from enum import Enum, auto
from typing import Optional

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
    def __init__(self, zebra_stop_seconds: float = None,
                 start_release_frames: int = None):
        self.zebra_stop_seconds = (settings.ZEBRA_STOP_SECONDS
                                   if zebra_stop_seconds is None else zebra_stop_seconds)
        self.start_release_frames = (settings.START_RELEASE_FRAMES
                                     if start_release_frames is None else start_release_frames)
        self.state = State.WAIT_START
        self._state_started = time.monotonic()

        # 发车边沿检测（控制端自行复核，不盲信视觉端的闩锁）
        self._saw_board = False
        self._start_clear_frames = 0

        # 防抖计数器
        self._zebra_frames = 0
        self._cone_frames = 0
        self._sign_frames = 0
        self._red_frames = 0
        self._green_frames = 0

    # ------------------------------------------------------------------ 内部
    def _set(self, state: State) -> None:
        if state != self.state:
            print(f"[FSM] {self.state.name} -> {state.name}")
            self.state = state
            self._state_started = time.monotonic()
            self._zebra_frames = 0
            self._cone_frames = 0
            self._sign_frames = 0
            self._red_frames = 0
            self._green_frames = 0

    def _update_start_edge(self, msg: PerceptionMessage) -> bool:
        """发车边沿检测：先见板 → 再连续多帧不见板。"""
        if msg.board_blocked:
            self._saw_board = True
            self._start_clear_frames = 0
            return False
        if not self._saw_board:
            return False
        self._start_clear_frames += 1
        if self._start_clear_frames >= self.start_release_frames:
            print(f"[FSM] 发车条件满足（挡板已移开，连续 {self._start_clear_frames} 帧无遮挡）")
            return True
        return False

    # ------------------------------------------------------------------ 接口
    def update(self, msg: PerceptionMessage) -> State:
        now = time.monotonic()

        # 更新防抖计数
        self._zebra_frames = self._zebra_frames + 1 if msg.has_zebra_crossing else 0
        self._cone_frames = self._cone_frames + 1 if msg.blue_cone_count > 0 else 0
        self._sign_frames = self._sign_frames + 1 if (msg.has_sign_a or msg.has_sign_b) else 0

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
            if self._update_start_edge(msg):
                self._set(State.TRACKING)

        elif self.state == State.TRACKING:
            if self._zebra_frames >= settings.ZEBRA_HYSTERESIS_FRAMES:
                self._set(State.ZEBRA_STOP)
            elif self._cone_frames >= settings.CONE_HYSTERESIS_FRAMES:
                self._set(State.AVOID_CONE)
            elif self._sign_frames >= settings.SIGN_HYSTERESIS_FRAMES:
                self._set(State.PARKING)

        elif self.state == State.ZEBRA_STOP:
            if now - self._state_started >= self.zebra_stop_seconds:
                self._set(State.TRACKING)

        elif self.state == State.TRAFFIC_LIGHT:
            # 禁止单帧决策：连续 N 帧绿灯才放行（红灯/灭灯/不确定一律等）
            if self._green_frames >= settings.TRAFFIC_LIGHT_HYSTERESIS_FRAMES:
                self._set(State.TRACKING)

        elif self.state == State.AVOID_CONE:
            if self._cone_frames == 0 and now - self._state_started >= 0.5:
                self._set(State.TRACKING)

        elif self.state == State.PARKING:
            if now - self._state_started >= 2.0:
                self._set(State.DONE)

        return self.state

    # ---------------------------------------------------------- 供调试/测试
    def enter_traffic_light(self) -> None:
        """由光电传感器触发进入红绿灯等待（P1-3 接入 GPIO 后调用）。"""
        self._set(State.TRAFFIC_LIGHT)
