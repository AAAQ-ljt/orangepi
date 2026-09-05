"""FSM 状态机骨架。"""
from __future__ import annotations

from enum import Enum, auto


class State(Enum):
    WAIT_START = auto()
    TRACKING = auto()
    ZEBRA_STOP = auto()
    TRAFFIC_LIGHT = auto()
    AVOID_CONE = auto()
    PARKING = auto()
    DONE = auto()


class FSM:
    def __init__(self):
        self.state = State.WAIT_START

    def transition(self, event: str) -> None:
        # TODO: 实现状态转移表
        raise NotImplementedError("FSM.transition() 待实现")

    def current(self) -> State:
        return self.state
