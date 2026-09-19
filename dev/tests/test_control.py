"""控制端单元测试：使用 Mock 硬件，不碰真实小车。

跑法：
    cd dev && PYTHONPATH=. python tests/test_control.py
"""
from __future__ import annotations

from config import settings
from common.protocol import PerceptionMessage
from control.controller import Controller
from control.driver import Driver
from control.fsm import FSM, State
from control.planner import Planner
from hardware.mock import MockPCA9685


# --------------------------------------------------------------------- 发车
def test_fsm_never_starts_without_board():
    """安全红线：没见过挡板就绝不允许进入 TRACKING（防"上电即冲"）。"""
    fsm = FSM()
    assert fsm.state == State.WAIT_START
    for _ in range(30):
        fsm.update(PerceptionMessage(board_blocked=False, center_x=320))
    assert fsm.state == State.WAIT_START, "没有'先见板'的沿，不允许发车"


def test_fsm_starts_after_board_removed():
    fsm = FSM(start_release_frames=5)
    for _ in range(3):
        fsm.update(PerceptionMessage(board_blocked=True, center_x=320))
    assert fsm.state == State.WAIT_START, "仅见板还不能走"
    for i in range(4):
        fsm.update(PerceptionMessage(board_blocked=False, center_x=320))
        assert fsm.state == State.WAIT_START, f"去抖未满足（{i+1}/5）不应发车"
    fsm.update(PerceptionMessage(board_blocked=False, center_x=320))
    assert fsm.state == State.TRACKING


def test_fsm_board_flicker_resets_release_counter():
    fsm = FSM(start_release_frames=3)
    fsm.update(PerceptionMessage(board_blocked=True))
    fsm.update(PerceptionMessage(board_blocked=False))
    fsm.update(PerceptionMessage(board_blocked=False))
    fsm.update(PerceptionMessage(board_blocked=True))     # 又闪一下
    fsm.update(PerceptionMessage(board_blocked=False))
    fsm.update(PerceptionMessage(board_blocked=False))
    assert fsm.state == State.WAIT_START, "挡板闪回应清零计数"
    fsm.update(PerceptionMessage(board_blocked=False))
    assert fsm.state == State.TRACKING


# --------------------------------------------------------------------- 规划
def test_planner_steers_toward_lane_center():
    planner = Planner(target_x=320.0)
    # 车道中心出现在画面右侧 → 车偏左 → 修正方向由 settings.STEER_SIGN 决定
    # （site.yaml 可标定 steer_sign: -1——实车"角度增大=左转"，见 config/site.py），
    # 所以这里断言"偏移方向与符号一致"，不写死 >90/<90。
    sign = float(settings.STEER_SIGN)
    right = planner.plan(PerceptionMessage(center_x=400.0), dt=0.05)
    assert (right.steering - 90.0) * sign > 0, \
        f"center_x>target 应沿 STEER_SIGN({sign:+g}) 方向修正，实际 {right.steering}"
    planner.reset()
    left = planner.plan(PerceptionMessage(center_x=240.0), dt=0.05)
    assert (left.steering - 90.0) * sign < 0, \
        f"center_x<target 应反向修正，实际 {left.steering}"


def test_planner_center_x_no_error():
    planner = Planner(target_x=320.0)
    target = planner.plan(PerceptionMessage(center_x=320.0), dt=0.05)
    assert abs(target.steering - 90.0) < 1e-6


def test_planner_respects_stop_and_scale():
    planner = Planner(cruise_throttle=100.0)
    assert planner.plan(PerceptionMessage(center_x=320.0), should_stop=True).throttle == 0.0
    half = planner.plan(PerceptionMessage(center_x=320.0), throttle_scale=0.5)
    assert abs(half.throttle - 50.0) < 1e-6
    cone = planner.plan(PerceptionMessage(center_x=320.0, blue_cone_count=1))
    assert cone.throttle < 100.0, "见锥桶应降速"


# --------------------------------------------------------------------- 控制器
def _controller(**kwargs) -> Controller:
    mock = MockPCA9685()
    driver = Driver(pca=mock, real=False)
    driver.arm()
    return Controller(driver, port=0, audio_path="nonexistent.mp3", **kwargs)


def test_controller_drives_only_in_driving_states():
    ctrl = _controller()
    try:
        # 未发车：不给动力
        ctrl.handle_message(PerceptionMessage(center_x=320, lane_confidence=1.0))
        assert ctrl.driver.pca.channels[1] == 0.0, "WAIT_START 期间电调必须为 0"

        # 发车
        ctrl.handle_message(PerceptionMessage(board_blocked=True, center_x=320, lane_confidence=1.0))
        for _ in range(5):
            ctrl.handle_message(PerceptionMessage(board_blocked=False, center_x=320,
                                                  lane_confidence=1.0))
        assert ctrl.fsm.state == State.TRACKING
        assert ctrl.driver.pca.channels[1] > 0.0, "TRACKING 应给动力"

        # 斑马线：停车
        for _ in range(3):
            ctrl.handle_message(PerceptionMessage(center_x=320, lane_confidence=1.0,
                                                  has_zebra_crossing=True))
        assert ctrl.fsm.state == State.ZEBRA_STOP
        assert ctrl.driver.pca.channels[1] == 0.0, "ZEBRA_STOP 必须停车"
    finally:
        ctrl.shutdown()


def test_controller_failsafe_when_no_message():
    ctrl = _controller(failsafe_timeout=0.0)
    try:
        ctrl.handle_message(PerceptionMessage(center_x=320, lane_confidence=1.0))
        ctrl._apply_failsafe()          # 模拟断连超时
        assert ctrl.driver.pca.channels[1] == 0.0
    finally:
        ctrl.shutdown()


def test_driver_not_armed_stays_safe():
    mock = MockPCA9685()
    driver = Driver(pca=mock, real=False)     # 未 arm
    driver.execute(Planner().plan(PerceptionMessage(center_x=400.0)))
    assert mock.channels[1] == 0.0, "未解锁时不允许输出动力"
    assert mock.channels[0] == 90.0, "未解锁时舵机应回中"


if __name__ == "__main__":
    test_fsm_never_starts_without_board()
    test_fsm_starts_after_board_removed()
    test_fsm_board_flicker_resets_release_counter()
    test_planner_steers_toward_lane_center()
    test_planner_center_x_no_error()
    test_planner_respects_stop_and_scale()
    test_controller_drives_only_in_driving_states()
    test_controller_failsafe_when_no_message()
    test_driver_not_armed_stays_safe()
    print("test_control: all passed")
