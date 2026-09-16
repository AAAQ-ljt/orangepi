"""发车遮挡检测（StartGate）单元测试：合成图，不碰硬件。

跑法：
    cd dev && PYTHONPATH=. python tests/test_start_gate.py
"""
from __future__ import annotations

import numpy as np

from vision.start_gate import StartGate, blue_ratio, edge_density


def _blue_frame(h: int = 480, w: int = 640) -> np.ndarray:
    """整幅纯蓝（BGR 的蓝色通道满值）——模拟挡板占据画面。"""
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :, 0] = 220        # B
    frame[:, :, 1] = 80         # G
    frame[:, :, 2] = 40         # R
    return frame


def _track_frame(h: int = 480, w: int = 640) -> np.ndarray:
    """红棕跑道 + 白线——模拟挡板已移开。"""
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :, 2] = 130        # R
    frame[:, :, 1] = 40
    frame[:, :, 0] = 30
    frame[h // 2:, 180:188] = 235      # 左侧白线
    frame[h // 2:, 450:458] = 235      # 右侧白线
    return frame


def test_blue_ratio_discriminates():
    blue = blue_ratio(_blue_frame())
    track = blue_ratio(_track_frame())
    assert blue > 0.9, f"整幅蓝应接近 1.0，实际 {blue:.3f}"
    assert track < 0.02, f"跑道不应判成蓝色，实际 {track:.3f}"
    assert edge_density(_blue_frame()) < 0.01, "纯色图边缘密度应接近 0"


def test_no_arming_without_board():
    """最关键的安全用例：从未见过挡板 → 绝不发车（防'上电即冲'）。"""
    gate = StartGate()
    state = None
    for _ in range(20):
        state = gate.update(_track_frame())
    assert state is not None
    assert not state.armed, "没见过挡板不应武装"
    assert not state.released, "没见过挡板绝不能发车"
    assert not state.blocked


def test_edge_trigger_requires_arm_then_clear():
    gate = StartGate(arm_frames=3, release_frames=5)

    # 先连续看到挡板 → 武装
    for i in range(2):
        state = gate.update(_blue_frame())
        assert not state.armed, f"第 {i+1} 帧不应提前武装"
    state = gate.update(_blue_frame())
    assert state.armed and not state.released, "连续 3 帧后应武装但未放行"

    # 遮挡消失，但去抖未满足 → 仍不放行
    for i in range(4):
        state = gate.update(_track_frame())
        assert not state.released, f"第 {i+1} 帧无遮挡不应提前放行"
    state = gate.update(_track_frame())
    assert state.released, "连续 5 帧无遮挡后应放行"


def test_release_is_latched_and_flash_resets_counter():
    gate = StartGate(arm_frames=2, release_frames=3)
    gate.update(_blue_frame())
    gate.update(_blue_frame())
    gate.update(_track_frame())
    gate.update(_track_frame())
    gate.update(_blue_frame())        # 中途又闪一下挡板 → 计数清零
    assert not gate.update(_track_frame()).released
    gate.update(_track_frame())
    state = gate.update(_track_frame())
    assert state.released, "重新连续 3 帧后应放行"
    # 闩锁：之后即使又检测到蓝色也保持已放行
    assert gate.update(_blue_frame()).released


def test_timeout_only_warns():
    gate = StartGate(timeout_s=0.0)
    state = gate.update(_track_frame(), now=time_now())
    assert state.timed_out, "超时应置位（仅告警）"
    assert not state.released, "超时绝不能自动发车"


def time_now() -> float:
    import time
    return time.time()


if __name__ == "__main__":
    test_blue_ratio_discriminates()
    test_no_arming_without_board()
    test_edge_trigger_requires_arm_then_clear()
    test_release_is_latched_and_flash_resets_counter()
    test_timeout_only_warns()
    print("test_start_gate: all passed")
