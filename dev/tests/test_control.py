"""控制端单元测试：使用 Mock 硬件，不碰真实小车。"""
from __future__ import annotations

import time

from common.protocol import PerceptionMessage
from control.driver import Driver
from control.fsm import State
from control.planner import Planner
from hardware.mock import MockPCA9685


def test_fsm_start():
    from control.fsm import FSM
    fsm = FSM()
    assert fsm.state == State.WAIT_START
    fsm.update(PerceptionMessage(is_barrier=False))
    assert fsm.state == State.TRACKING


def test_planner_stops_on_zebra():
    planner = Planner()
    target = planner.plan(PerceptionMessage(has_zebra_crossing=True, center_x=320))
    assert target.throttle == 0.0


def test_driver_dry_run_safe_stop():
    mock = MockPCA9685()
    driver = Driver(pca=mock, real=False)
    driver.arm()
    driver.execute(Planner().plan(PerceptionMessage(center_x=320)))
    driver.safe_stop()
    assert mock.channels[1] == 0  # 电调应归零


if __name__ == "__main__":
    test_fsm_start()
    test_planner_stops_on_zebra()
    test_driver_dry_run_safe_stop()
    print("all tests passed")
