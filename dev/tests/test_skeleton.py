"""基础骨架测试：确认包可导入。"""
from common.protocol import VisionMessage
from control.fsm import FSM, State
from vision.publisher import UDPPublisher


def test_protocol():
    msg = VisionMessage(timestamp=1.0, lane_center=0.1, lane_visible=True)
    assert msg.lane_center == 0.1


def test_fsm_initial_state():
    fsm = FSM()
    assert fsm.current() == State.WAIT_START
