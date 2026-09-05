"""控制端主控制器：UDP -> FSM -> Planner -> Driver。"""
from __future__ import annotations

import threading
from typing import Optional

from common.protocol import PerceptionMessage
from control.driver import Driver
from control.fsm import FSM, State
from control.planner import Planner
from control.udp_server import UDPServer


class Controller:
    def __init__(self, driver: Driver, port: int = 5000,
                 target_x: float = 320.0,
                 zebra_stop_seconds: float = 10.0):
        self.driver = driver
        self.port = port
        self.fsm = FSM(zebra_stop_seconds=zebra_stop_seconds)
        self.planner = Planner(target_x=target_x)
        self.server = UDPServer(port=port)
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def handle_message(self, msg: Optional[PerceptionMessage]) -> None:
        if msg is None:
            return
        state = self.fsm.update(msg)
        target = self.planner.plan(msg)

        # 未完成前只在 TRACKING 才允许油门；安全状态一律停车
        if state not in (State.TRACKING, State.AVOID_CONE):
            target.throttle = 0.0

        self.driver.execute(target)

    def run_forever(self) -> None:
        print(f"[CTRL] UDP listening on 0.0.0.0:{self.port}")
        while not self._stop.is_set():
            msg = self.server.recv()
            if msg is not None:
                self.handle_message(msg)
        self.shutdown()

    def shutdown(self) -> None:
        self.driver.safe_stop()
        self.server.close()
