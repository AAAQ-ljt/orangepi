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
                                 traffic_light_state="red", blue_cone_count=2,
                                 elements=[{"name": "zebra", "conf": 0.9}])
    data = original.to_dict()
    restored = PerceptionMessage.from_dict(data)
    assert restored.center_x == 333.0
    assert abs(restored.lane_confidence - 0.75) < 1e-9
    assert restored.board_blocked is True
    assert restored.traffic_light_state == "red"
    assert restored.blue_cone_count == 2
    # 2026-09-23 代码审查 B1：to_dict 曾漏掉 elements → 元素列表永远不过 UDP
    assert restored.elements == [{"name": "zebra", "conf": 0.9}]


def test_elements_survive_udp_roundtrip():
    """元素列表是 vision→control 的唯一通道，必须随 to_dict/from_dict 完整往返。"""
    elems = [{"name": "cone", "color": "blue", "conf": 0.81, "x": 100, "y": 200},
             {"name": "parking_area", "conf": 0.55}]
    msg = PerceptionMessage(elements=elems)
    assert msg.to_dict()["elements"] == elems
    assert PerceptionMessage.from_dict(msg.to_dict()).elements == elems


def test_elements_tolerate_missing_or_junk():
    assert PerceptionMessage.from_dict({}).elements == []
    assert PerceptionMessage.from_dict({"elements": None}).elements == []
    assert PerceptionMessage.from_dict({"elements": [{"name": 1}, "junk", None]}).elements \
        == [{"name": 1}]


def test_traffic_light_alias_and_case():
    assert PerceptionMessage.from_dict({"traffic_light": "RED"}).traffic_light_state == "red"
    assert PerceptionMessage.from_dict({"traffic_light_state": " green "}).traffic_light_state == "green"


if __name__ == "__main__":
    test_legacy_is_barrier_maps_to_board_blocked()
    test_legacy_zebra_field()
    test_missing_and_null_fields_are_tolerated()
    test_lane_confidence_default_when_absent()
    test_roundtrip_new_fields()
    test_elements_survive_udp_roundtrip()
    test_elements_tolerate_missing_or_junk()
    test_traffic_light_alias_and_case()
    print("test_protocol: all passed")
