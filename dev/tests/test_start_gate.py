"""发车遮挡检测（StartGate）单元测试：合成图，不碰硬件。

重点覆盖**两种放置时序**（用户 2026-09-16 明确要求）：
  A. 程序启动前板已放好   → 启动即确认，板移走才发车
  B. 程序启动后板才放上   → 启动后绝不发车，放上并移走后发车

跑法：
    cd dev && PYTHONPATH=. python tests/test_start_gate.py
"""
from __future__ import annotations

import numpy as np

from vision.start_gate import StartGate, blue_metrics, blue_ratio, edge_density

W, H = 640, 480


def _blue_frame(h: int = H, w: int = W) -> np.ndarray:
    """整幅纯蓝（模拟挡板占据画面）。"""
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :, 0] = 220        # B
    frame[:, :, 1] = 80         # G
    frame[:, :, 2] = 40         # R
    return frame


def _small_blue_frame(size: int = 70) -> np.ndarray:
    """画面里只有一小块蓝（模拟板在远处：面积小但主导度高）。"""
    frame = _track_frame()
    cy, cx = H // 2, W // 2
    frame[cy:cy + size, cx:cx + size] = (220, 80, 40)
    return frame


def _track_frame(h: int = H, w: int = W) -> np.ndarray:
    """红棕跑道 + 白线（模拟挡板已移开）。"""
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :, 2] = 130        # R
    frame[:, :, 1] = 40
    frame[:, :, 0] = 30
    frame[h // 2:, 180:188] = 235
    frame[h // 2:, 450:458] = 235
    return frame


# --------------------------------------------------------------------- 判据
def test_metrics_discriminate():
    mb = blue_metrics(_blue_frame())
    mt = blue_metrics(_track_frame())
    assert mb.area_ratio > 0.5, f"整幅蓝的面积占比应接近 1，实际 {mb.area_ratio:.3f}"
    assert mb.dominance > 50, f"整幅蓝的主导度应很高，实际 {mb.dominance:.1f}"
    assert mt.area_ratio < 0.02, f"跑道不应判成蓝色，实际 {mt.area_ratio:.3f}"
    assert mt.dominance < 20, f"跑道主导度应很低，实际 {mt.dominance:.1f}"
    assert blue_ratio(_blue_frame()) > 0.9          # 兼容接口
    assert edge_density(_blue_frame()) < 0.01


# --------------------------------------------------------------------- 时序 A
def test_a_board_present_before_start():
    """板先放好：启动即确认 → 中途不动 → 移走后发车。"""
    gate = StartGate(arm_frames=3, release_frames=5, verbose=False)
    for _ in range(3):
        st = gate.update(_blue_frame())
    assert st.armed and not st.released, "启动时就该确认'见到板'"
    for i in range(4):
        st = gate.update(_track_frame())
        assert not st.released, f"去抖未满足（{i + 1}/5）不应发车"
    assert gate.update(_track_frame()).released, "连续 5 帧无遮挡应放行"


# --------------------------------------------------------------------- 时序 B
def test_b_board_placed_later():
    """板后放：启动后 20 帧内绝不能发车（安全红线）。"""
    gate = StartGate(arm_frames=3, release_frames=3, verbose=False)
    for _ in range(20):
        st = gate.update(_track_frame())
        assert not st.armed, "没见到板就不该武装"
        assert not st.released, "没见到板绝不能发车"
    for _ in range(3):
        st = gate.update(_blue_frame())
    assert st.armed, "板放上后应确认"
    for _ in range(3):
        st = gate.update(_track_frame())
    assert st.released, "板移走后应发车"


# ------------------------------------------------------------- 距离/光照鲁棒
def test_dominance_cue_works_when_board_is_small():
    """板在远处（面积小）时靠主导度判据也能确认。"""
    gate = StartGate(area_thresh=0.9, dominance_thresh=30.0,
                     area_thresh_low=0.0005, arm_frames=2, verbose=False)
    st = None
    for _ in range(2):
        st = gate.update(_small_blue_frame())
    assert st is not None and st.armed, f"小面积蓝块应靠主导度确认，读数 {st.metrics.summary}"


def test_dominance_cue_covers_dark_board():
    """暗光/阴影下的蓝板：亮度低于 HSV 掩膜的 V 下限（掩膜失效），但通道差判据仍能兜住。

    这是两路判据互补的真实场景 —— 掩膜是"S 与 V 都要够"的硬门槛，
    而 B-max(R,G) 的通道差对亮度不敏感（板在阴影里照样有蓝色主导度）。
    """
    frame = _blue_frame().copy()
    frame[:, :, :] = (55, 15, 10)            # 暗蓝：V=55 低于 V 下限 60，掩膜取不到
    gate = StartGate(area_thresh=0.06, area_thresh_low=0.0,
                     dominance_thresh=30.0, arm_frames=2, verbose=False)
    st = gate.update(frame)
    assert st.metrics.area_ratio < 0.06, "暗蓝不该被 HSV 掩膜算成面积"
    assert st.blocked, f"应靠主导度判据确认，读数 {st.metrics.summary}"


# --------------------------------------------------- 车盖边缘回归（2026-10-15 现场事故）
COVER_BGR = (87, 72, 56)      # 实测：车盖上沿的"暗蓝灰"像素（HSV≈(105, 91, 87)）


def _cover_edge_frame(h: int = H, w: int = W, y0: int = 405) -> np.ndarray:
    """红棕跑道 + 底部一条"车盖上沿"暗蓝灰带（2026-10-15 实拍特征）。

    这条带会**过**蓝掩膜（S=91≥80、V=87≥60、H≈105 ∈ [95,135]），
    所以老判据"面积≥0.06 就武装"会被它骗开（当天两次无板发车）。
    """
    frame = _track_frame(h, w)
    frame[y0:h, :] = COVER_BGR
    return frame


def test_cover_edge_band_is_not_a_board():
    """2026-10-15 事故回归：车盖上沿的暗蓝灰带不是蓝板，绝不能武装/发车。

    两个层次都要钉（任一被改回去就红）：
      · B（判据）：把 ROI 手工设回旧档 (0.15,0.85,0.20,0.95)，让蓝带落进 ROI ——
        此时面积 0.13 ≥ 0.06 但主导度只有 15 < 35，必须**不武装**；
      · A（区域）：用 settings 默认 ROI（底边 0.80）时蓝带根本不在 ROI 内。
    """
    frame = _cover_edge_frame()
    old_roi = (0.15, 0.85, 0.20, 0.95)

    # —— B：蓝带在 ROI 内，靠"面积路径也要主导度"挡住
    m_old = blue_metrics(frame, roi=old_roi)
    assert m_old.area_ratio >= 0.06, f"用例前提：旧 ROI 下面积必须够大（{m_old.area_ratio:.3f}）"
    assert m_old.dominance < 35.0, f"用例前提：车盖带主导度必须低（{m_old.dominance:.1f}）"
    gate_b = StartGate(roi=old_roi, arm_frames=3, verbose=False)
    st = None
    for _ in range(5):
        st = gate_b.update(frame)
    assert not st.armed, f"车盖边缘带被当成蓝板了！读数 {st.metrics.summary}"
    assert not st.released

    # —— A：settings 默认 ROI（2026-10-15 起底边 0.80）下蓝带不入 ROI
    from config import settings
    m_new = blue_metrics(frame, roi=settings.START_GATE_ROI)
    assert m_new.area_ratio < 0.06, f"默认 ROI 下不该看到车盖带（{m_new.area_ratio:.3f}）"
    gate_a = StartGate(arm_frames=3, verbose=False)
    for _ in range(5):
        st = gate_a.update(frame)
    assert not st.armed, "默认 ROI 下车盖带不该武装闸门"

    # —— 反向验证：真板放上去照样武装（别把判据改死）
    board = frame.copy()
    board[200:300, 150:450] = (220, 80, 40)          # 一块亮饱和蓝板
    gate_c = StartGate(arm_frames=2, verbose=False)
    st_c = None
    for _ in range(3):
        st_c = gate_c.update(board)
    assert st_c.armed, f"真板必须仍能武装，读数 {st_c.metrics.summary}"


# --------------------------------------------------------------------- 其它
def test_flicker_resets_release_counter():
    gate = StartGate(arm_frames=2, release_frames=3, verbose=False)
    gate.update(_blue_frame())
    gate.update(_blue_frame())
    gate.update(_track_frame())
    gate.update(_track_frame())
    gate.update(_blue_frame())        # 中途闪回一下
    assert not gate.update(_track_frame()).released
    gate.update(_track_frame())
    assert gate.update(_track_frame()).released, "重新连续 3 帧后应放行"
    assert gate.update(_blue_frame()).released, "放行是闩锁，之后保持"


def test_timeout_only_warns_never_releases():
    gate = StartGate(timeout_s=0.0, verbose=False)
    st = gate.update(_track_frame(), now=1000.0)
    assert st.timed_out and not st.released, "超时只能告警，绝不自动发车"


def test_bench_rule_never_runs_without_board():
    """台架脚本的安全红线：没见过板绝不给动力（规则见 scripts/bench_test.py）。"""
    from scripts.bench_test import NEUTRAL_US, SPEED_US_DEFAULT, START_US_DEFAULT, decide
    d = decide(blocked=False, seen_board=False, lane_ok=True, aligned=True,
               force_run=False, speed_us=SPEED_US_DEFAULT, start_us=START_US_DEFAULT)
    assert d.out_us == NEUTRAL_US
    d2 = decide(blocked=True, seen_board=True, lane_ok=True, aligned=True,
                force_run=False, speed_us=SPEED_US_DEFAULT, start_us=START_US_DEFAULT)
    assert d2.out_us == NEUTRAL_US
    d3 = decide(blocked=False, seen_board=True, lane_ok=True, aligned=True,
                force_run=False, speed_us=SPEED_US_DEFAULT, start_us=START_US_DEFAULT)
    assert d3.out_us == SPEED_US_DEFAULT


def test_reset_clears_state():
    gate = StartGate(arm_frames=2, release_frames=2, verbose=False)
    gate.update(_blue_frame())
    gate.update(_blue_frame())
    gate.reset()
    st = gate.update(_track_frame())
    assert not st.armed and not st.released, "复位后必须回到'没见过板'状态"


if __name__ == "__main__":
    test_metrics_discriminate()
    test_a_board_present_before_start()
    test_b_board_placed_later()
    test_dominance_cue_works_when_board_is_small()
    test_dominance_cue_covers_dark_board()
    test_cover_edge_band_is_not_a_board()
    test_flicker_resets_release_counter()
    test_timeout_only_warns_never_releases()
    test_bench_rule_never_runs_without_board()
    test_reset_clears_state()
    print("test_start_gate: all passed")
