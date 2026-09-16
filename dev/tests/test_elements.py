"""元素层与模型可插拔（profile）单元测试：合成张量，不碰硬件/模型文件。

跑法：
    cd dev && PYTHONPATH=. python tests/test_elements.py
"""
from __future__ import annotations

import numpy as np

from vision.elements import (Element, color_of, load_profile, nearest, pick)
from vision.postprocess import parse_elements, postprocess


# --------------------------------------------------------------------- profile
def test_profile_legacy_mapping():
    prof = load_profile("legacy")
    assert prof.num_classes == 9
    assert prof.element_name(8) == "zebra"          # zebra_crossing
    assert prof.element_name(3) == "cone"           # obstacle_cone_blue
    assert prof.element_name(5) == "parking_sign"   # parking_sign_A
    assert prof.element_name(2) is None             # left_lane：不用（扫线负责车道）


def test_profile_smartcar2026_mapping():
    prof = load_profile("smartcar2026")
    assert prof.num_classes == 8
    assert prof.element_name(0) == "blue_board"
    assert prof.element_name(1) == "zebra"          # crosswalk
    assert prof.element_name(2) == "light"          # traffic_light_off（本阶段不消费）
    assert prof.element_name(7) == "parking_area"


def test_color_hint():
    assert color_of("obstacle_cone_blue") == "blue"
    assert color_of("obstacle_cone_red") == "red"
    assert color_of("obstacle_cone_yellow") == "yellow"
    assert color_of("traffic_light_green") == "green"
    assert color_of("traffic_light_off") == "off"
    assert color_of("zebra_crossing") is None


# --------------------------------------------------------------------- 解码
def _fake_outputs(nc: int, entries):
    """造一个 (1, 4+nc, N) 的假检测输出。entries: [(cx,cy,w,h,class_id,score)]"""
    n = max(1, len(entries))
    det = np.zeros((1, 4 + nc, n), dtype=np.float32)
    for i, (cx, cy, w, h, cid, score) in enumerate(entries):
        det[0, 0, i], det[0, 1, i], det[0, 2, i], det[0, 3, i] = cx, cy, w, h
        det[0, 4 + cid, i] = score
    return [det]


def test_parse_elements_legacy():
    prof = load_profile("legacy")
    outs = _fake_outputs(9, [(320, 300, 120, 60, 8, 0.9),      # zebra
                             (100, 400, 30, 40, 3, 0.7)])      # blue cone
    elements = parse_elements(outs, prof)
    names = sorted(e.name for e in elements)
    assert names == ["cone", "zebra"], f"实际 {names}"
    cone = [e for e in elements if e.name == "cone"][0]
    assert cone.color == "blue" and abs(cone.conf - 0.7) < 1e-6


def test_parse_elements_smartcar2026():
    prof = load_profile("smartcar2026")
    outs = _fake_outputs(8, [(320, 300, 200, 80, 1, 0.85),     # crosswalk → zebra
                             (500, 420, 40, 50, 5, 0.66),      # obstacle_cone_blue
                             (200, 380, 60, 60, 0, 0.55)])     # blue_board
    elements = parse_elements(outs, prof)
    names = sorted(e.name for e in elements)
    assert names == ["blue_board", "cone", "zebra"], f"实际 {names}"


def test_low_confidence_filtered():
    prof = load_profile("smartcar2026")
    outs = _fake_outputs(8, [(320, 300, 200, 80, 1, 0.10)])
    assert parse_elements(outs, prof, conf_threshold=0.25) == []
    assert len(parse_elements(outs, prof, conf_threshold=0.05)) == 1


def test_nms_suppresses_duplicates():
    prof = load_profile("smartcar2026")
    outs = _fake_outputs(8, [(320, 300, 100, 50, 5, 0.9),
                             (322, 302, 100, 50, 5, 0.8)])     # 几乎重合
    assert len(parse_elements(outs, prof)) == 1, "重叠框应被 NMS 抑制"


def test_postprocess_default_profile_is_legacy():
    """不给 profile 时的默认行为保持兼容（上一届模型）。"""
    outs = _fake_outputs(9, [(320, 300, 120, 60, 8, 0.9)])
    elements = postprocess(outs)
    assert [e.name for e in elements] == ["zebra"]


# --------------------------------------------------------------------- 几何
def test_element_geometry_uses_bottom_line():
    e = Element(name="zebra", conf=0.9, xyxy=(100.0, 200.0, 300.0, 260.0))
    assert e.center_x == 200.0
    assert e.bottom_y == 260.0, "距离估计要用接地线（框底边），不是几何中心"
    assert e.width == 200.0 and e.height == 60.0


def test_pick_and_nearest():
    near = Element(name="cone", conf=0.8, xyxy=(100, 380, 140, 430))
    far = Element(name="cone", conf=0.9, xyxy=(300, 220, 340, 250))
    zebra = Element(name="zebra", conf=0.7, xyxy=(0, 300, 640, 340))
    els = [far, near, zebra]
    assert len(pick(els, "cone")) == 2
    assert nearest(els, "cone") is near, "底边更低 = 离车更近"
    assert nearest(els) is near


if __name__ == "__main__":
    test_profile_legacy_mapping()
    test_profile_smartcar2026_mapping()
    test_color_hint()
    test_parse_elements_legacy()
    test_parse_elements_smartcar2026()
    test_low_confidence_filtered()
    test_nms_suppresses_duplicates()
    test_postprocess_default_profile_is_legacy()
    test_element_geometry_uses_bottom_line()
    test_pick_and_nearest()
    print("test_elements: all passed")
