"""元素颜色检测器单测（2026-10-09 T4）：全是合成图/构造对象，无硬件依赖。

覆盖：斑马线连通域计数、红绿灯 HSV 互比（含分区校验）、停车黄区面积、
锥桶间隙决策树（前景带/行分组/红色阻挡/跨中心优先）、A/B 凸包面积比。
跑法：cd dev && PYTHONPATH=. python tests/test_color_elements.py
"""
from __future__ import annotations

import numpy as np
import cv2

from vision.color_elements import (classify_ab, choose_gap, detect_parking_yellow,
                                   detect_traffic_light, detect_zebra,
                                   parking_yellow_box)
from vision.cone_detect import ConeBox


def _cone(x0, y0, x1, y1) -> ConeBox:
    return ConeBox(xyxy=(float(x0), float(y0), float(x1), float(y1)),
                   area_ratio=0.01, conf=0.9)


# ------------------------------------------------------------------ 斑马线
def _zebra_frame(n_stripes: int, frame: np.ndarray = None, spacing: int = 24) -> np.ndarray:
    if frame is None:
        frame = np.full((480, 640, 3), (60, 60, 60), dtype=np.uint8)
    for i in range(n_stripes):
        y = 300 + i * spacing
        cv2.rectangle(frame, (200, y), (440, y + 10), (255, 255, 255), -1)
    return frame


def test_zebra_detects_four_plus_stripes():
    assert detect_zebra(_zebra_frame(5)), "5 条白条纹应判斑马线"
    assert detect_zebra(_zebra_frame(4)), "4 条应达标"


def test_zebra_rejects_few_stripes():
    assert not detect_zebra(_zebra_frame(2)), "2 条不能判斑马线"
    assert not detect_zebra(_zebra_frame(0)), "无条纹不能判"


def test_zebra_shape_gate_filters_lane_lines():
    """形状闸门开启时：细长（车道线状）白条不算条纹。"""
    img = np.full((480, 640, 3), (60, 60, 60), dtype=np.uint8)
    for i in range(5):
        cv2.rectangle(img, (300, 300 + i * 10), (306, 360 + i * 10), (255, 255, 255), -1)  # 竖长条
    assert not detect_zebra(img, use_shape_gate=True), "5 根竖长条（车道线状）应被闸门滤掉"
    assert detect_zebra(_zebra_frame(5, spacing=18), use_shape_gate=True), \
        "扁矩形条纹（间距 18px，顶边跨度 72px 落在 ±40 内）应通过闸门"


# ------------------------------------------------------------------ 红绿灯
def _light_frame(color: str, radius: int = 30, x: int = 320, y: int = 120) -> np.ndarray:
    img = np.full((480, 640, 3), (30, 30, 40), dtype=np.uint8)
    bgr = {"red": (0, 0, 255), "green": (0, 255, 0)}[color]
    cv2.circle(img, (x, y), radius, bgr, -1)
    return img


def test_light_red_green_basic():
    """纯 HSV 互比（不开分区校验）：红/绿独立可判。"""
    assert detect_traffic_light(_light_frame("red"), use_zone=False) == "red"
    assert detect_traffic_light(_light_frame("green"), use_zone=False) == "green"
    assert detect_traffic_light(np.full((480, 640, 3), (30, 30, 40), np.uint8),
                                use_zone=False) is None


def test_light_zone_check():
    """分区校验：灯珠在左 1/3 → 只有红色能过；在右 1/3 → 只有绿色能过。"""
    assert detect_traffic_light(_light_frame("red", x=100, y=120), use_zone=True) == "red"
    assert detect_traffic_light(_light_frame("green", x=100, y=120), use_zone=True) is None, \
        "绿灯珠在左 1/3 不满足分区（三灯珠横排右 1/3 才是绿）"
    assert detect_traffic_light(_light_frame("green", x=540, y=120), use_zone=True) == "green"
    assert detect_traffic_light(_light_frame("red", x=540, y=120), use_zone=True) is None


# ------------------------------------------------------------------ 停车黄区
def test_parking_yellow_area():
    img = np.full((480, 640, 3), (60, 60, 60), dtype=np.uint8)
    cv2.rectangle(img, (100, 300), (500, 360), (0, 200, 220), -1)   # BGR 黄
    assert detect_parking_yellow(img), "400*60=24000px 黄区应检出"
    img2 = np.full((480, 640, 3), (60, 60, 60), dtype=np.uint8)
    cv2.rectangle(img2, (100, 300), (120, 340), (0, 200, 220), -1)  # 20*40=800px 太小
    assert not detect_parking_yellow(img2)


def test_parking_yellow_box_geometry():
    """parking_yellow_box 返回最大黄区外框（原图坐标），判左右几何用。"""
    img = np.full((480, 640, 3), (60, 60, 60), dtype=np.uint8)
    cv2.rectangle(img, (100, 300), (500, 360), (0, 200, 220), -1)
    box = parking_yellow_box(img)
    assert box is not None
    x, y, w, h = box
    assert (x, y) == (100, 300) and w == 401 and h == 61, f"bbox={box}"
    assert parking_yellow_box(np.full((480, 640, 3), (60, 60, 60), np.uint8)) is None


# ------------------------------------------------------------------ 锥桶间隙决策树
def test_gap_basic_pair():
    """两蓝锥相距 100px、跨画面中心 → 选中该间隙。"""
    blue = [_cone(200, 300, 220, 360), _cone(360, 300, 380, 360)]
    gap = choose_gap(blue, [], img_center=320.0)
    assert gap is not None
    assert abs(gap.center_x - 290.0) < 1.0
    assert gap.spans_center


def test_gap_foreground_filter_ignores_far_cone():
    """远处杂锥（bottom_y 低于最近锥 145px 以上）不参与配对。"""
    blue = [_cone(200, 300, 220, 360),            # 近
            _cone(500, 100, 520, 150),            # 远（typo 距离 210px）
            _cone(380, 300, 400, 360)]            # 近
    gap = choose_gap(blue, [], img_center=320.0)
    assert gap is not None
    assert gap.right_x < 400, "配对应只用近处两个"


def test_gap_red_blocker_kills_gap():
    """红色锥桶落在间隙内（bottom 与行 y 差在 [−12,+40]）→ 该间隙不安全。"""
    blue = [_cone(200, 300, 220, 360), _cone(380, 300, 400, 360)]   # 行 y = 360
    red = [_cone(280, 340, 300, 355)]             # bottom=355，dy=−5 ∈ [−12,+40]
    assert choose_gap(blue, red, img_center=320.0) is None
    red_far = [_cone(280, 180, 300, 220)]         # bottom=220，dy=−140（太远，不算阻挡）
    gap = choose_gap(blue, red_far, img_center=320.0)
    assert gap is not None


def test_gap_narrow_gap_rejected():
    blue = [_cone(300, 300, 315, 360), _cone(330, 300, 345, 360)]   # 15px 间隙
    assert choose_gap(blue, [], img_center=320.0) is None


# ------------------------------------------------------------------ A/B 凸包面积比
def _contour_of(points):
    return np.array(points, dtype=np.int32).reshape(-1, 1, 2)


def test_ab_classify_convex_vs_concave():
    # B：凸多边形（正方形 + 顶部小凸起仍是凸）
    square = [(100, 100), (200, 100), (200, 200), (100, 200)]
    assert classify_ab(_contour_of(square)) == "B", "正方形近凸 → B"
    # A：凹形（矩形右缘切掉一个大三角缺口 → 面积显著小于凸包）
    concave = [(100, 100), (200, 100), (200, 180), (150, 150), (100, 180)]
    assert classify_ab(concave) == "A", "有三角缺口 → A"


def test_ab_rejects_tiny():
    tiny = [(10, 10), (20, 10), (20, 20), (10, 20)]
    assert classify_ab(tiny) is None, "面积过小不判"


# ------------------------------------------------------------------ 汇总
if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"test_color_elements: all passed ({len(fns)})")