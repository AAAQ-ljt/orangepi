"""控制端主控制器：UDP -> FSM -> Planner -> Driver，带断连保护、转向平滑和语音播报。"""
from __future__ import annotations

import threading
import time
from typing import Optional

from config import settings
from common.protocol import PerceptionMessage
from control.driver import Driver, ControlTarget
from control.fsm import FSM, State
from control.planner import Planner
from control.udp_server import UDPServer
from hardware.audio import AudioPlayer


class Controller:
    def __init__(self, driver: Driver, port: int = 5000,
                 target_x: float = 320.0,
                 zebra_stop_seconds: float = 10.0,
                 failsafe_timeout: float = settings.FAILSAFE_TIMEOUT,
                 audio_path: str = "/root/dev/voice/aa.mp3"):
        self.driver = driver
        self.port = port
        self.fsm = FSM(zebra_stop_seconds=zebra_stop_seconds)
        self.planner = Planner(target_x=target_x)
        self.server = UDPServer(port=port)
        self.audio = AudioPlayer(audio_path=audio_path)
        self._stop = threading.Event()
        self._last_msg_time = time.monotonic()
        self._smoothed_steering = 90.0
        self._failsafe_timeout = failsafe_timeout
        self._prev_state: Optional[State] = None

    def stop(self) -> None:
        self._stop.set()

    def _apply_failsafe(self) -> None:
        if time.monotonic() - self._last_msg_time > self._failsafe_timeout:
            self.driver.safe_stop()

    def _on_state_change(self, state: State) -> None:
        if state != self._prev_state:
            if state == State.ZEBRA_STOP:
                print("[AUDIO] playing team voice")
                self.audio.play()
        self._prev_state = state

    def handle_message(self, msg: Optional[PerceptionMessage]) -> None:
        if msg is None:
            return
        self._last_msg_time = time.monotonic()

        state = self.fsm.update(msg)
        self._on_state_change(state)
        target = self.planner.plan(msg)

        # 非行驶状态一律停车
        if state not in (State.TRACKING, State.AVOID_CONE):
            target.throttle = 0.0

        # 转向平滑
        alpha = settings.STEERING_SMOOTHING
        self._smoothed_steering += alpha * (target.steering - self._smoothed_steering)
        target.steering = self._smoothed_steering

        self.driver.execute(target)

    def run_forever(self) -> None:
        print(f"[CTRL] UDP listening on 0.0.0.0:{self.port}")
        while not self._stop.is_set():
            msg = self.server.recv()
            if msg is not None:
                self.handle_message(msg)
            else:
                self._apply_failsafe()
        self.shutdown()

    def shutdown(self) -> None:
        self.driver.safe_stop()
        self.server.close()
