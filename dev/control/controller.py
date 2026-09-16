"""控制端主控制器：UDP -> 仲裁 -> FSM -> Planner -> Driver，带断连保护、转向限速和语音播报。"""
from __future__ import annotations

import threading
import time
from typing import Optional

from config import settings
from common.protocol import PerceptionMessage
from control.driver import Driver
from control.fsm import FSM, State
from control.lane_arbiter import LaneArbiter
from control.planner import Planner
from control.udp_server import UDPServer
from hardware.audio import AudioPlayer

# 允许输出动力的状态（其余状态一律电调归零）
DRIVING_STATES = (State.TRACKING, State.AVOID_CONE)


class Controller:
    def __init__(self, driver: Driver, port: int = None,
                 target_x: float = None,
                 zebra_stop_seconds: float = None,
                 failsafe_timeout: float = None,
                 audio_path: str = "/root/dev/voice/aa.mp3"):
        self.driver = driver
        self.port = settings.UDP_PORT if port is None else port
        self.fsm = FSM(zebra_stop_seconds=zebra_stop_seconds)
        self.arbiter = LaneArbiter(target_x=target_x)
        self.planner = Planner(target_x=target_x)
        self.server = UDPServer(port=self.port)
        self.audio = AudioPlayer(audio_path=audio_path)
        self._stop = threading.Event()
        self._last_msg_time = time.monotonic()
        self._last_loop_time = time.monotonic()
        self._smoothed_steering = float(settings.SERVO_CENTER_ANGLE)
        self._failsafe_timeout = (settings.FAILSAFE_TIMEOUT
                                  if failsafe_timeout is None else failsafe_timeout)
        self._prev_state: Optional[State] = None

    # ------------------------------------------------------------------ 生命周期
    def stop(self) -> None:
        self._stop.set()

    def _apply_failsafe(self) -> None:
        if time.monotonic() - self._last_msg_time > self._failsafe_timeout:
            self.driver.safe_stop()

    def _on_state_change(self, state: State) -> None:
        if state != self._prev_state:
            if state == State.ZEBRA_STOP:
                print("[AUDIO] 播报队名（后台播放，不阻塞控制环）")
                self.audio.play()
            if state == State.PARKING:
                print("[CTRL] 进入停车状态（车位判定与入库控制在 P1-4 接入）")
            # 状态切换时清 PID/滤波状态，避免输出"踢腿"
            self.planner.reset()
        self._prev_state = state

    # ------------------------------------------------------------------ 主处理
    def handle_message(self, msg: Optional[PerceptionMessage]) -> None:
        if msg is None:
            return
        now = time.monotonic()
        self._last_msg_time = now
        dt = max(1e-3, now - self._last_loop_time)
        self._last_loop_time = now

        state = self.fsm.update(msg)
        self._on_state_change(state)

        arb = self.arbiter.update(msg.center_x, msg.lane_confidence, now)
        target = self.planner.plan(msg, center_x=arb.center_x,
                                   throttle_scale=arb.throttle_scale,
                                   should_stop=arb.should_stop,
                                   dt=dt)

        # 非行驶状态一律停车
        if state not in DRIVING_STATES:
            target.throttle = 0.0

        # 转向限速（度/秒），防止从 0 直接跳满舵
        max_delta = settings.STEERING_SLEW_DEG_PER_S * dt
        delta = max(-max_delta, min(max_delta, target.steering - self._smoothed_steering))
        self._smoothed_steering += delta
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
        self.audio.stop()
        self.driver.safe_stop()
        self.server.close()
