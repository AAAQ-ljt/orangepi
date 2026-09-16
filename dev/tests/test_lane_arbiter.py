"""仲裁层（LaneArbiter）单元测试：置信度调度 / 降级 / 停车。

跑法：
    cd dev && PYTHONPATH=. python tests/test_lane_arbiter.py
"""
from __future__ import annotations

from control.lane_arbiter import LaneArbiter


def test_high_confidence_uses_scan():
    arb = LaneArbiter()
    out = arb.update(center_x=300.0, confidence=0.9, now=0.0)
    assert out.source == "scan"
    assert out.center_x == 300.0
    assert out.throttle_scale == 1.0
    assert not out.degraded and not out.should_stop


def test_low_confidence_holds_then_degrades():
    arb = LaneArbiter(conf_thresh=0.35, low_conf_frames=3, hold_s=0.8, stop_s=1.5)
    arb.update(center_x=300.0, confidence=0.9, now=0.0)

    # 前 2 帧低置信度：还不到降级阈值，仍保持正常油门
    for i, t in enumerate((0.1, 0.2)):
        out = arb.update(center_x=None, confidence=0.0, now=t)
        assert out.source == "hold"
        assert out.center_x == 300.0, "应沿用最近一次有效中心"
        assert not out.degraded, f"第 {i+1} 帧不应提前降级"
        assert out.throttle_scale == 1.0

    # 第 3 帧：达到降级阈值 → 降油门
    out = arb.update(center_x=None, confidence=0.0, now=0.3)
    assert out.degraded and out.throttle_scale < 1.0
    assert not out.should_stop


def test_long_loss_requires_stop():
    arb = LaneArbiter(hold_s=0.5, stop_s=1.0)
    arb.update(center_x=300.0, confidence=0.9, now=0.0)
    out = arb.update(center_x=None, confidence=0.0, now=1.5)
    assert out.should_stop, "超过 stop_s 仍无有效信息 → 必须要求停车"
    assert out.throttle_scale == 0.0
    assert out.held_s >= 1.4


def test_never_had_valid_value_stops_immediately():
    arb = LaneArbiter()
    out = arb.update(center_x=None, confidence=0.0, now=0.0)
    assert out.source == "none"
    assert out.should_stop and out.throttle_scale == 0.0


def test_recovery_after_loss():
    arb = LaneArbiter(stop_s=1.0)
    arb.update(center_x=300.0, confidence=0.9, now=0.0)
    arb.update(center_x=None, confidence=0.0, now=2.0)
    out = arb.update(center_x=280.0, confidence=0.8, now=2.1)
    assert out.source == "scan" and out.center_x == 280.0
    assert not out.degraded and not out.should_stop
    assert out.throttle_scale == 1.0


if __name__ == "__main__":
    test_high_confidence_uses_scan()
    test_low_confidence_holds_then_degrades()
    test_long_loss_requires_stop()
    test_never_had_valid_value_stops_immediately()
    test_recovery_after_loss()
    print("test_lane_arbiter: all passed")
