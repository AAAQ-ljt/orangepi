"""UDP 消息契约测试：兼容旧字段、容忍缺失/null、新字段往返。

跑法：
    cd dev && PYTHONPATH=. python tests/test_protocol.py
"""
from __future__ import annotations

from common.protocol import PerceptionMessage


def test_legacy_is_barrier_maps_to_board_blocked():
    """旧视觉端发 is_barrier=True 时，语义应等同 board_blocked。"""
    msg = PerceptionMessage.from_dict({"is_barrier": True, "center_x": 300})
    assert msg.board_blocked is True
    assert msg.is_barrier is True

    msg2 = PerceptionMessage.from_dict({"is_barrier": False, "center_x": 300})
    assert msg2.board_blocked is False


def test_legacy_zebra_field():
    msg = PerceptionMessage.from_dict({"is_rxd_flag": True, "left_x": 100, "right_x": 500})
    assert msg.has_zebra_crossing is True
    assert msg.center_x == 300.0, "缺 center_x 时应由左右线求平均兜底"


def test_missing_and_null_fields_are_tolerated():
    msg = PerceptionMessage.from_dict({"center_x": None, "left_x": None, "right_x": None,
                                       "board_blocked": None, "lane_confidence": None})
    assert msg.center_x is None
    assert msg.board_blocked is False
    assert msg.lane_confidence == 0.0, "无 center_x 时置信度应为 0"


def test_lane_confidence_default_when_absent():
    """旧视觉端不给置信度：有 center_x 就按可信处理（避免整车因缺字段瘫住）。"""
    assert PerceptionMessage.from_dict({"center_x": 320}).lane_confidence == 1.0
    assert PerceptionMessage.from_dict({"left_x": 10, "right_x": 630}).lane_confidence == 1.0


def test_roundtrip_new_fields():
    original = PerceptionMessage(timestamp=1.5, center_x=333.0, lane_confidence=0.75,
                                 board_blocked=True, start_released=False,
                                 traffic_light_state="red", blue_cone_count=2)
    data = original.to_dict()
    restored = PerceptionMessage.from_dict(data)
    assert restored.center_x == 333.0
    assert abs(restored.lane_confidence - 0.75) < 1e-9
    assert restored.board_blocked is True
    assert restored.traffic_light_state == "red"
    assert restored.blue_cone_count == 2


def test_traffic_light_alias_and_case():
    assert PerceptionMessage.from_dict({"traffic_light": "RED"}).traffic_light_state == "red"
    assert PerceptionMessage.from_dict({"traffic_light_state": " green "}).traffic_light_state == "green"


if __name__ == "__main__":
    test_legacy_is_barrier_maps_to_board_blocked()
    test_legacy_zebra_field()
    test_missing_and_null_fields_are_tolerated()
    test_lane_confidence_default_when_absent()
    test_roundtrip_new_fields()
    test_traffic_light_alias_and_case()
    print("test_protocol: all passed")
