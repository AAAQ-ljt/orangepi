"""台架测试脚本的决策逻辑与安全不变量（纯函数测试，不碰硬件）。

覆盖 `scripts/bench_test.py` 的 `decide()`：
- **没见过板绝不输出动力**（安全红线）
- 板在 → 停
- 无有效车道 → 停（除非 --force-run，台架逃生门）
- **起步对齐**：未对准 → 用起步脉宽（低速蠕动修正），对准 → 循迹脉宽
- `--no-lane`：只测蓝板+电调（不判车道）
- 安全优先级：板在 > 无车道 > 对齐 > 循迹（板在即使 force_run 也停）

跑法：
    cd dev && PYTHONPATH=. python tests/test_bench_logic.py
"""
from __future__ import annotations

from scripts.bench_test import NEUTRAL_US, START_US_DEFAULT, decide


def _d(**kw):
    base = dict(blocked=False, seen_board=True, lane_ok=True, aligned=True,
                force_run=False, speed_us=1550.0, start_us=START_US_DEFAULT)
    base.update(kw)
    return decide(**base)


# ------------------------------------------------------- 安全红线
def test_never_runs_without_board():
    d = _d(seen_board=False)
    assert d.out_us == NEUTRAL_US and d.phase == "idle"


def test_board_stops():
    d = _d(blocked=True)
    assert d.out_us == NEUTRAL_US and d.phase == "stopped"


def test_board_wins_over_force_run():
    """板是最高优先级停车信号：即使开了 force_run 也必须停。"""
    d = _d(blocked=True, force_run=True)
    assert d.out_us == NEUTRAL_US


# ------------------------------------------------------- 车道保护
def test_lane_loss_stops_unless_forced():
    d = _d(lane_ok=False)
    assert d.out_us == NEUTRAL_US and "无有效车道" in d.reason
    d2 = _d(lane_ok=False, force_run=True)
    assert d2.out_us > NEUTRAL_US


# ------------------------------------------------------- 起步对齐
def test_align_uses_start_speed():
    """未对准 → 低速蠕动（起步脉宽），不是循迹速度。"""
    d = _d(aligned=False)
    assert d.out_us == START_US_DEFAULT and d.phase == "align"
    assert START_US_DEFAULT < 1550.0, "起步脉宽应低于循迹脉宽"


def test_aligned_uses_track_speed():
    d = _d(aligned=True)
    assert d.out_us == 1550.0 and d.phase == "track"


def test_no_lane_mode_skips_alignment():
    """--no-lane（只测蓝板+电调）：不判车道、不对齐，直接按速度跑。"""
    d = _d(no_lane=True, lane_ok=False, aligned=False)
    assert d.out_us == 1550.0 and d.phase == "track"


# ------------------------------------------------------- 公共安全层
def test_bench_common_exposes_safety_api():
    from scripts import bench_common
    for name in ("MotorSession", "install_signal_guard", "restore_remote_stage", "svc_active"):
        assert hasattr(bench_common, name), f"bench_common 缺少 {name}"


def test_restore_is_idempotent():
    """收尾函数必须幂等（信号 + finally + atexit 可能重复调用）。"""
    import contextlib
    import io

    from scripts.bench_common import restore_remote_stage
    state = {"restored": False, "driver": None, "used_motor": False}
    with contextlib.redirect_stdout(io.StringIO()):      # 这台机器没有 systemctl，别刷屏
        restore_remote_stage(state, "第一次")
        assert state["restored"] is True
        restore_remote_stage(state, "第二次")
    assert state["restored"] is True


if __name__ == "__main__":
    test_never_runs_without_board()
    test_board_stops()
    test_board_wins_over_force_run()
    test_lane_loss_stops_unless_forced()
    test_align_uses_start_speed()
    test_aligned_uses_track_speed()
    test_no_lane_mode_skips_alignment()
    test_bench_common_exposes_safety_api()
    test_restore_is_idempotent()
    print("test_bench_logic: all passed")
