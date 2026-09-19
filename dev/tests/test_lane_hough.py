"""Hough 循线通道单元测试：合成赛道图，不碰摄像头/模型。

对应 vision/lane_hough.py v2（oldCode 骨架 + 操场第一轮现场结论 + 配对枚举升级）。
测试图用**透视斜线**——车道线在画面里必然是斜的（越远越靠中间），
竖直/水平线会被斜率先验滤掉，测不到真东西（与 test_lane_scan 同一教训）。

跑法：
    cd dev && PYTHONPATH=. python tests/test_lane_hough.py
"""
from __future__ import annotations

import numpy as np

from config import settings
from vision.lane_hough import HoughLaneScanner

W, H = 640, 480
BG = 60          # 红棕跑道暗色
LINE = 235       # 白线


def make_frame(shift: float = 0.0, keep_left: bool = True, keep_right: bool = True,
               diverge: bool = False, extra_horiz: bool = False,
               noise: bool = False) -> np.ndarray:
    """画一个梯形赛道：两条白线近处分、远处靠中间（透视收敛）。

    shift: 整体平移（+ = 赛道在画面右侧）；
    keep_left/right=False 抹掉那一侧（模拟丢线）；
    diverge=True 把右线画成"越远越宽"（不收敛，配对必须拒绝它）；
    extra_horiz=True 加近水平亮带（斑马线/纸边，斜率先验必须滤掉）。
    """
    frame = np.full((H, W, 3), BG, dtype=np.uint8)
    y_top = int(H * settings.LANE_ROI_TOP_RATIO)          # 与默认 ROI(0.35) 一致
    y_bot = H - 16

    def draw(x_bot: float, x_top: float):
        for y in range(y_top, y_bot):
            t = (y - y_top) / float(max(1, y_bot - y_top))
            x = int(round(x_top + (x_bot - x_top) * t))
            frame[y, max(0, x - 3):x + 4] = LINE

    if keep_left:
        draw(60 + shift, 250 + shift)                      # 近处在左、远处靠中间 → a<0
    if keep_right:
        if diverge:
            draw(700 + shift, 520 + shift)                 # 远端更宽 → 不收敛
        else:
            draw(580 + shift, 390 + shift)                 # a>0
    if extra_horiz:
        for y0 in (int(H * 0.5), int(H * 0.6)):
            frame[y0:y0 + 6, 60:W - 60] = LINE
    if noise:
        rng = np.random.default_rng(0)
        for _ in range(400):
            y = int(rng.integers(y_top, y_bot))
            x = int(rng.integers(0, W))
            frame[y, x:x + 2] = LINE
    return frame


def test_pair_centered():
    s = HoughLaneScanner()
    obs = s.scan(make_frame())
    assert obs.source == "hough"
    assert s.last_info["mode"] == "pair"
    assert obs.confidence > 0.3, f"双线齐全置信度应高，实际 {obs.confidence}"
    assert abs(obs.center_x - 320.0) <= 12.0, f"居中赛道 center_x 应≈320，实际 {obs.center_x}"


def test_pair_shifted_right():
    s = HoughLaneScanner()
    obs = s.scan(make_frame(shift=100))
    assert s.last_info["mode"] == "pair"
    assert abs(obs.center_x - 420.0) <= 15.0, f"center_x 应≈420，实际 {obs.center_x}"


def test_single_side_degrades():
    s = HoughLaneScanner()
    s.scan(make_frame())                                   # 先建立 _last_pair_w（≈183）
    obs = s.scan(make_frame(keep_right=False))             # 丢右线
    assert s.last_info["mode"] == "single"
    assert obs.confidence == settings.LANE_HOUGH_SINGLE_SIDE_CONF
    # 中心 = 左线 + 上一帧实测半宽 → 应回到真实车道中心附近
    assert abs(obs.center_x - 320.0) <= 15.0, f"宽度先验兜底应≈320，实际 {obs.center_x}"
    assert obs.right_x is None and obs.left_x is not None


def test_single_side_without_history_uses_default():
    s = HoughLaneScanner()
    obs = s.scan(make_frame(keep_right=False))
    assert s.last_info["mode"] == "single"
    # 无历史帧 → 用默认半宽，方向必须正确（中心在左线右侧）
    assert obs.center_x > 200.0


def test_no_lines_zero_confidence():
    s = HoughLaneScanner()
    frame = np.full((H, W, 3), BG, dtype=np.uint8)
    frame[H // 2 - 20:H // 2 + 20, :] = 200                # 一条横带也不该形成斜线
    obs = s.scan(frame)
    assert s.last_info["mode"] == "none"
    assert obs.confidence == 0.0
    assert obs.center_x == s.target_x


def test_horizontal_stripes_ignored():
    s = HoughLaneScanner()
    obs = s.scan(make_frame(extra_horiz=True))
    assert s.last_info["mode"] == "pair"
    assert abs(obs.center_x - 320.0) <= 12.0, f"斑马线不应干扰中心，实际 {obs.center_x}"


def test_grain_noise_robust():
    s = HoughLaneScanner()
    obs = s.scan(make_frame(noise=True))
    assert s.last_info["mode"] == "pair"
    assert abs(obs.center_x - 320.0) <= 12.0, f"颗粒噪点不应干扰中心，实际 {obs.center_x}"


def test_diverging_pair_rejected():
    """右线画成"越远越宽"→ 收敛校验必须拒绝这个配对 → 落到单侧兜底。"""
    s = HoughLaneScanner()
    obs = s.scan(make_frame(diverge=True))
    assert s.last_info["mode"] == "single", f"不收敛的配对应被拒收，实际 {s.last_info['mode']}"
    assert obs.confidence == settings.LANE_HOUGH_SINGLE_SIDE_CONF


def test_canny_adapt_persists_and_bounded():
    s = HoughLaneScanner()
    lo0, hi0 = s.canny_low, s.canny_high
    for _ in range(40):
        s.scan(make_frame(noise=True))
    assert (s.canny_low, s.canny_high) != (lo0, hi0), "边缘多的帧应推高阈值"
    assert settings.LANE_CANNY_LOW_MIN <= s.canny_low <= settings.LANE_CANNY_LOW_MAX
    assert settings.LANE_CANNY_HIGH_MIN <= s.canny_high <= settings.LANE_CANNY_HIGH_MAX


def test_continuity_gate_rejects_jump():
    """连续性闸门：上一帧中心新鲜时，单帧偏移超过 LANE_HOUGH_CENTER_JUMP_PX 的配对被拒。"""
    s = HoughLaneScanner()
    s.scan(make_frame())                                   # 建立 last_center≈320
    obs = s.scan(make_frame(shift=250))                    # 突然跳 250px
    # 配对被跳变闸门拒收 → 单侧兜底或 none；绝不能直接输出跳变后的中心
    if s.last_info["mode"] == "pair":
        assert abs(obs.center_x - s.target_x) <= settings.LANE_HOUGH_CENTER_JUMP_PX + 15
    else:
        assert obs.confidence <= settings.LANE_HOUGH_SINGLE_SIDE_CONF


if __name__ == "__main__":
    test_pair_centered()
    test_pair_shifted_right()
    test_single_side_degrades()
    test_single_side_without_history_uses_default()
    test_no_lines_zero_confidence()
    test_horizontal_stripes_ignored()
    test_grain_noise_robust()
    test_diverging_pair_rejected()
    test_canny_adapt_persists_and_bounded()
    test_continuity_gate_rejects_jump()
    print("test_lane_hough: all passed")
