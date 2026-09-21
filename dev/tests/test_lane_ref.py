"""参考实现版循迹（scripts/lane_ref_test.py）单元测试：合成图 + 实测参数，不碰硬件。

覆盖要点：
  1. 对称车道的中心应≈目标点、误差≈0；
  2. 车道整体右移 → 中心跟着右移（方向不能反）；
  3. **斜率先验**：横向纹理（近水平线）必须被滤掉，不能当真车道线；
  4. 只画一侧 → 不给中心（不做 0/宽度兜底）；
  5. PID：误差为正时按参考实现 `angle = 90 - pid` 输出（角度变小）。

跑法：
    cd dev && PYTHONPATH=. python tests/test_lane_ref.py
"""
from __future__ import annotations

import numpy as np

from config import settings
from scripts.lane_ref_test import (SPEED_FAST_US, SPEED_FLOOR_MARGIN_US,
                                  LaneRefDetector, PidRef, adaptive_pulse)

W, H = 640, 480


def _lane_image(shift: int = 0, left: bool = True, right: bool = True) -> np.ndarray:
    """红棕底 + 两条**斜**白线（透视），默认中心在 375（= 操场实测目标点）。"""
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    frame[:, :, 2], frame[:, :, 1], frame[:, :, 0] = 120, 45, 35
    y_top, y_bot = int(H * 0.30), int(H * 0.60)

    def draw(x_bot: float, x_top: float, thickness: int = 8):
        half = thickness // 2
        for y in range(y_top, y_bot):
            t = (y - y_top) / float(max(1, y_bot - y_top))
            x = int(round(x_top + (x_bot - x_top) * t))
            frame[y, max(0, x - half):x + half + 1] = 225

    if left:
        draw(160 + shift, 300 + shift)      # 近处 160 → 远处 300（越远越靠中间）
    if right:
        draw(590 + shift, 450 + shift)      # 中心 = (160+590)/2 = 375 = 操场实测目标点
    return frame


def test_centered_lane_gives_near_zero_error():
    det = LaneRefDetector()
    r = det.detect(_lane_image())
    assert r.center_x is not None, "两条线都在，应该给出中心"
    assert abs(r.error) < 25.0, f"对称摆放时误差应接近 0，实际 {r.error}"
    assert r.n_left >= 1 and r.n_right >= 1


def test_shifted_lane_moves_error_same_direction():
    """车道整体右移 → 中心右移、误差变大（正误差 = 车在目标左侧）。"""
    det = LaneRefDetector()
    base = det.detect(_lane_image())
    shifted = det.detect(_lane_image(shift=20))     # 现场量级：偏移 20px 已足够验证方向
    assert shifted.center_x is not None
    assert shifted.center_x > base.center_x + 10, "整体右移后中心应变大（方向不能反）"


def test_horizontal_texture_is_not_a_lane_line():
    """近水平的横向纹理（|dy/dx| ≈ 0.26）不能当选成车道线：斜率先验 + 位置锚定要挡住。"""
    frame = _lane_image()
    for y in range(int(H * 0.35), int(H * 0.55), 12):     # 画一堆横线（模拟颗粒/横纹）
        frame[y:y + 3, :] = 200
    det = LaneRefDetector()
    r = det.detect(frame)
    assert r.center_x is not None, "真车道线还在，应该仍能找到"
    assert abs(r.center_x - 375.0) < 40.0, f"横纹不该把中心带跑，实际 {r.center_x}"


def test_single_side_gives_no_center():
    det = LaneRefDetector()
    r = det.detect(_lane_image(right=False))
    assert r.center_x is None and r.error is None, "只看到一侧时不给中心"
    assert r.left_x is not None, "看到的那一侧要报出来（便于诊断）"


def test_pid_matches_reference_convention():
    """转向符号与 settings.STEER_SIGN 同口径：
       sign=-1（参考实现原式 `90 - pid`）→ 正误差输出 <90°；
       sign=+1（"角度增大=右转"）→ 正误差输出 >90°。
    """
    pid = PidRef(kp=0.15, ki=0.01, kd=0.12, limit_deg=15, smooth=1.0, sign=-1.0)
    a_pos = pid.step(+40.0)
    assert a_pos < 90.0, f"sign=-1 时正误差应输出 <90°，实际 {a_pos}"
    pid.reset()
    a_neg = pid.step(-40.0)
    assert a_neg > 90.0, f"sign=-1 时负误差应输出 >90°，实际 {a_neg}"

    pid2 = PidRef(kp=0.15, ki=0.01, kd=0.12, limit_deg=15, smooth=1.0, sign=+1.0)
    assert pid2.step(+40.0) > 90.0, "sign=+1 时必须与 sign=-1 反向（否则改 site.yaml 没效果）"


def test_angle_limited_and_smoothed():
    pid = PidRef(kp=0.15, ki=0.01, kd=0.12, limit_deg=15, smooth=0.7)
    for _ in range(30):
        a = pid.step(300.0)
    assert 90.0 - 15.0 - 1e-6 <= a <= 90.0 + 15.0 + 1e-6, f"必须限幅在 ±15°，实际 {a}"


def test_adaptive_pulse_direction():
    """自适应速度：默认只减速不提速，且减速档不得掉进电调死区。

    用户 2026-09-20：巡线 1575 太快 → 基准 1560（参考实现是 +15/-20）。
    基准降到 1560 后，"误差大 -20" 会算出 1540 < 死区 1545 → 车会从"减速"变"停车"，
    所以减速档必须有硬下限。
    """
    base = 1560.0
    assert adaptive_pulse(1.0, base) == base          # 默认增量 0：直道不提速
    assert adaptive_pulse(8.0, base) == base
    assert adaptive_pulse(40.0, base) == base - 10.0  # 大误差减速 10us
    floor = float(settings.ESC_DEADBAND_US) + SPEED_FLOOR_MARGIN_US
    assert adaptive_pulse(40.0, base, SPEED_FAST_US, -40.0) == floor, "减速档不得低于死区"
    assert adaptive_pulse(40.0, 1546.0, SPEED_FAST_US, -20.0) == floor


if __name__ == "__main__":
    test_centered_lane_gives_near_zero_error()
    test_shifted_lane_moves_error_same_direction()
    test_horizontal_texture_is_not_a_lane_line()
    test_single_side_gives_no_center()
    test_pid_matches_reference_convention()
    test_angle_limited_and_smoothed()
    test_adaptive_pulse_direction()
    print("test_lane_ref: all passed")
