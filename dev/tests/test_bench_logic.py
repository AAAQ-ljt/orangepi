"""台架脚本的决策逻辑与安全不变量（纯函数测试，不碰硬件）。

覆盖：
- `bench_board_test.decide_output`：板在→停 / 板开→跑 / **没见过板绝不输出动力**
- `bench_track_test.decide_track_output`：同上 + "无有效车道"保护与 --force-run 逃生门
- `bench_common` 的接口存在性（收尾函数的幂等语义通过重复调用验证）

跑法：
    cd dev && PYTHONPATH=. python tests/test_bench_logic.py
"""
from __future__ import annotations

from scripts.bench_board_test import NEUTRAL_US, decide_output
from scripts.bench_track_test import decide_track_output


# ------------------------------------------------------- 板台架（原有行为）
def test_board_rule_never_runs_without_board():
    assert decide_output(blocked=False, seen_board=False, speed_us=1550) == NEUTRAL_US
    assert decide_output(blocked=True, seen_board=True, speed_us=1550) == NEUTRAL_US
    assert decide_output(blocked=False, seen_board=True, speed_us=1550) == 1550


# ------------------------------------------------------- 循迹台架
def test_track_rule_never_runs_without_board():
    out, why = decide_track_output(blocked=False, seen_board=False, lane_stop=False,
                                   force_run=False, speed_us=1550)
    assert out == NEUTRAL_US and "未见过板" in why


def test_track_rule_board_stops():
    out, why = decide_track_output(blocked=True, seen_board=True, lane_stop=False,
                                   force_run=False, speed_us=1550)
    assert out == NEUTRAL_US and "板在" in why


def test_track_rule_runs_when_lane_ok():
    out, why = decide_track_output(blocked=False, seen_board=True, lane_stop=False,
                                   force_run=False, speed_us=1550)
    assert out == 1550 and "循迹" in why


def test_track_rule_lane_loss_stops_unless_forced():
    """丢失车道时默认停车（与主程序仲裁层一致）；--force-run 只是台架逃生门。"""
    out, why = decide_track_output(blocked=False, seen_board=True, lane_stop=True,
                                   force_run=False, speed_us=1550)
    assert out == NEUTRAL_US and "无有效车道" in why
    out2, why2 = decide_track_output(blocked=False, seen_board=True, lane_stop=True,
                                     force_run=True, speed_us=1550)
    assert out2 == 1550 and "循迹" in why2


def test_track_rule_board_wins_over_force_run():
    """安全优先级：板在 → 一定停，即使开了 force_run（板是最高优先级的停车信号）。"""
    out, _why = decide_track_output(blocked=True, seen_board=True, lane_stop=False,
                                    force_run=True, speed_us=1550)
    assert out == NEUTRAL_US


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
        restore_remote_stage(state, "第二次")            # 不应抛异常、不应重复动作
    assert state["restored"] is True


if __name__ == "__main__":
    test_board_rule_never_runs_without_board()
    test_track_rule_never_runs_without_board()
    test_track_rule_board_stops()
    test_track_rule_runs_when_lane_ok()
    test_track_rule_lane_loss_stops_unless_forced()
    test_track_rule_board_wins_over_force_run()
    test_bench_common_exposes_safety_api()
    test_restore_is_idempotent()
    print("test_bench_logic: all passed")
