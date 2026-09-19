"""Hough 循线单元测试：合成赛道图，不碰摄像头/模型。

覆盖 oldCode picture() 移植时最要紧的行为：
  - 双线可见 → center_x 指向两线中点；
  - 赛道整体偏移 → center_x 跟着偏；
  - 单侧丢线 → 边缘兜底给出"朝丢线一侧修"的偏差，置信度降级；
  - 无线 → 置信度 0；
  - 近水平的斑马线/纸边（|k| 太小）不参与；
  - Canny 阈值跨帧自适应有界。

跑法：
    cd dev && PYTHONPATH=. python tests/test_lane_hough.py
"""
from __future__ import annotations

import numpy as np

from config import settings
from vision.lane_hough import HoughLaneScanner

W, H = 640, 480
BG = 60        # 跑道底色亮度（暗）
LINE = 230     # 白线亮度


def make_frame(left_x_at_bottom: float, right_x_at_bottom: float,
               slope: float = 0.0, keep_left: bool = True,
               keep_right: bool = True, extra_horiz: bool = False) -> np.ndarray:
    """画一个梯形赛道：两条白线从底边向上收敛（slope=每行向中心收多少 px）。

    keep_left/right=False 时抹掉那一侧的线（模拟丢线）。
    extra_horiz=True 时加几条近水平的亮带（模拟斑马线/纸边，|k|≈0，应当被滤掉）。
    """
    frame = np.full((H, W, 3), BG, dtype=np.uint8)
    y_top, y_bot = int(H * settings.HOUGH_ROI_TOP_RATIO), int(H * settings.HOUGH_ROI_BOTTOM_RATIO)
    for y in range(y_top, y_bot):
        converge = slope * (y_bot - y)
        lx = left_x_at_bottom + converge
        rx = right_x_at_bottom - converge
        for cx, on in ((lx, keep_left), (rx, keep_right)):
            if on:
                x0, x1 = int(cx) - 3, int(cx) + 3
                if 0 <= x0 and x1 < W:
                    frame[y, x0:x1] = LINE
    if extra_horiz:
        for y0 in (int(H * 0.6), int(H * 0.75)):
            frame[y0:y0 + 6, 60:W - 60] = LINE
    return frame


def test_both_lines_centered():
    s = HoughLaneScanner()
    obs = s.scan(make_frame(160, 480, slope=0.7))
    assert obs.source == "hough"
    assert obs.confidence >= settings.HOUGH_FULL_CONF_BASE, f"双侧齐全置信度应高，实际 {obs.confidence}"
    assert abs(obs.center_x - 320.0) <= 6.0, f"居中赛道 center_x 应≈320，实际 {obs.center_x}"
    assert obs.left_x is not None and obs.right_x is not None


def test_track_shifted_right():
    s = HoughLaneScanner()
    obs = s.scan(make_frame(260, 580, slope=0.7))          # 中心在 420，整体右偏 100px
    assert abs(obs.center_x - 420.0) <= 8.0, f"center_x 应≈420，实际 {obs.center_x}"
    assert obs.center_x > 320.0


def test_single_side_degrades_confidence():
    s = HoughLaneScanner()
    obs = s.scan(make_frame(160, 480, slope=0.7, keep_right=False))   # 丢右线
    assert obs.confidence == settings.HOUGH_SINGLE_SIDE_CONF
    # 右线丢失 → 中点 = 左线与右边缘的平均 > 320 → 误差为正 → 朝右修（找回右线）
    assert obs.center_x > 320.0, f"丢右线应向右兜底，实际 {obs.center_x}"
    assert 320.0 < obs.center_x < 480.0, f"兜底不应拉飞到边缘，实际 {obs.center_x}"


def test_no_lines_zero_confidence():
    s = HoughLaneScanner()
    frame = np.full((H, W, 3), BG, dtype=np.uint8)
    frame[H // 2 - 20:H // 2 + 20, :] = 200     # 一整条横带，也不该形成斜线
    obs = s.scan(frame)
    assert obs.confidence == 0.0
    assert obs.center_x == s.target_x


def test_horizontal_stripes_ignored():
    s = HoughLaneScanner()
    obs = s.scan(make_frame(160, 480, slope=0.7, extra_horiz=True))
    assert abs(obs.center_x - 320.0) <= 6.0, f"斑马线不应干扰 center_x，实际 {obs.center_x}"
    assert obs.confidence >= settings.HOUGH_FULL_CONF_BASE


def test_canny_adaptation_is_bounded():
    s = HoughLaneScanner()
    lo0, hi0 = s.canny_lo, s.canny_hi
    # 连续喂"边缘很多"的帧 → 阈值上升但有上界
    for _ in range(30):
        s.scan(make_frame(160, 480, slope=0.7))
    assert s.canny_hi <= settings.HOUGH_CANNY_HI_LIMIT[1]
    assert s.canny_lo >= settings.HOUGH_CANNY_LO_LIMIT[0]
    assert (s.canny_lo, s.canny_hi) != (lo0, hi0), "边缘多的帧应推高阈值"


def test_curved_converging_lines():
    s = HoughLaneScanner()
    # 透视收敛（远窄近宽）：center 仍应在两线中间
    obs = s.scan(make_frame(100, 540, slope=0.9))
    assert abs(obs.center_x - 320.0) <= 10.0, f"收敛赛道中心应≈320，实际 {obs.center_x}"


if __name__ == "__main__":
    test_both_lines_centered()
    test_track_shifted_right()
    test_single_side_degrades_confidence()
    test_no_lines_zero_confidence()
    test_horizontal_stripes_ignored()
    test_canny_adaptation_is_bounded()
    test_curved_converging_lines()
    print("test_lane_hough: all passed")
