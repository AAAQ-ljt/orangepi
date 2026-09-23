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

from config import settings
from scripts.bench_test import (NEUTRAL_US, SPEED_US_DEFAULT, START_US_DEFAULT,
                                align_verdict, auto_target, decide)


def _d(**kw):
    base = dict(blocked=False, seen_board=True, lane_ok=True, aligned=True,
                force_run=False, speed_us=SPEED_US_DEFAULT, start_us=START_US_DEFAULT)
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
    """未对准 → 低速蠕动（起步脉宽），不是行进速度。"""
    d = _d(aligned=False)
    assert d.out_us == START_US_DEFAULT and d.phase == "align"
    # 2026-09-20：巡线速度由 1575 降到 1560 → 起步/行进同为 1560；仍必须高于电调死区
    assert START_US_DEFAULT <= SPEED_US_DEFAULT, "起步脉宽不应高于行进脉宽"
    assert START_US_DEFAULT > settings.ESC_DEADBAND_US, "要动车的脉宽必须高于死区"


def test_aligned_uses_track_speed():
    d = _d(aligned=True)
    assert d.out_us == SPEED_US_DEFAULT and d.phase == "track"


def test_no_lane_mode_skips_alignment():
    """--no-lane（只测蓝板+电调）：不判车道、不对齐，直接按速度跑。"""
    d = _d(no_lane=True, lane_ok=False, aligned=False)
    assert d.out_us == SPEED_US_DEFAULT and d.phase == "track"


# ------------------------------------------------------- 电调死区（2026-09-16 实车教训）
def test_default_pulses_are_above_esc_deadband():
    """默认脉宽必须高于电调死区，否则台架跑起来车根本不走。

    实测：1540us 轮子不转、≈1545us 才起转。曾用 1530/1550 默认值 → 整段"起步对齐"白跑。
    """
    assert START_US_DEFAULT >= settings.ESC_DEADBAND_US, \
        f"起步脉宽 {START_US_DEFAULT} 低于死区 {settings.ESC_DEADBAND_US}"
    assert SPEED_US_DEFAULT >= settings.ESC_DEADBAND_US, \
        f"循迹脉宽 {SPEED_US_DEFAULT} 低于死区 {settings.ESC_DEADBAND_US}"
    assert settings.ESC_CREEP_US > settings.ESC_DEADBAND_US > settings.ESC_STOP_US
    assert settings.ESC_DEBUG_MAX_US > settings.ESC_CREEP_US, "调试上限必须高于蠕动脉宽，否则跑不快"


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


# ------------------------------------------------------- 边跑边标定 / 转向方向判定
def test_auto_target_accepts_stable_samples():
    """车摆正在车道中央时的稳定读数 → 采纳为新的车道中心。"""
    val = auto_target([420.0 + (i % 3) - 1 for i in range(14)])
    assert val is not None and abs(val - 420.0) <= 2.0


def test_auto_target_rejects_unstable_or_absurd():
    """样本不足/乱跳/数值离谱一律不采纳（宁可用旧值，也不能把车道中心改坏）。"""
    assert auto_target([420.0, 422.0, 418.0]) is None, "帧数不够不该采纳"
    assert auto_target([200.0, 420.0, 600.0] * 5) is None, "波动过大不该采纳"
    assert auto_target([30.0] * 14) is None, "数值超出合理范围（锁到别的东西）不该采纳"


def test_align_verdict_detects_direction():
    assert "正确" in align_verdict([30, 28, 25, 20, 15, 10, 8, 6, 5, 4])
    assert "变大" in align_verdict([5, 6, 7, 9, 12, 16, 22, 28, 33, 40])
    assert "不判断" in align_verdict([10.0, 11.0]), "样本太少时必须说不判断，别乱下结论"


# ------------------------------------------------------- 跑偏保护
def test_driving_off_course():
    from scripts.bench_test import NEUTRAL_US, driving_off_course
    # 有动力 + 大误差 + 持续超时 → 停
    assert driving_off_course(120.0, 1575.0, since=0.0, now=3.0)
    # 时间没到 / 误差回到范围内 / 没有动力 / 还没开始计时 → 不停
    assert not driving_off_course(120.0, 1575.0, since=1.0, now=2.0)
    assert not driving_off_course(30.0, 1575.0, since=0.0, now=9.0)
    assert not driving_off_course(120.0, NEUTRAL_US, since=0.0, now=9.0)
    assert not driving_off_course(120.0, 1575.0, since=None, now=9.0)


# ------------------------------------------------------- 起步探路（防"看到车道才敢动"死锁）
def test_acquire_creep_without_lane():
    """发车后允许**低速探路找线**：车停在起点时下摄可能看不到白线，若这时也要求"有车道"，
    车会永远动不了（2026-09-16 实测：电调全程 1500us 一次没动）。"""
    d = _d(lane_ok=False, aligned=False, acquire=True)
    assert d.out_us == START_US_DEFAULT and d.phase == "align", d
    # 探路窗口之外 / 没见过板 → 一律不动（探路绝不放宽发车条件）
    assert _d(lane_ok=False, aligned=False, acquire=False).out_us == NEUTRAL_US
    assert _d(lane_ok=False, aligned=False, acquire=True, seen_board=False).out_us == NEUTRAL_US
    assert _d(lane_ok=False, aligned=False, acquire=True, blocked=True).out_us == NEUTRAL_US


def test_acquire_does_not_override_force_run():
    """--force-run（台架逃生门）优先级仍高于探路逻辑。"""
    d = _d(lane_ok=False, aligned=True, force_run=True, acquire=True)
    assert d.out_us == SPEED_US_DEFAULT and d.phase == "track"


# ------------------------------------------------------- 丢线时的"降速维持"（过弯道关键）
def test_creep_stage_ignores_throttle_scale():
    """探路/起步对齐阶段**必须**用蠕动脉宽：若也被仲裁 scale 乘回中位，车永远不动、找不到线。

    2026-09-19 现场：探路窗口里打的是 `电调=1500us(降速×0.0)`，车原地不动 → 死锁。
    规则：只有 track 阶段参与降速。
    """
    from scripts.bench_test import scaled_pulse
    for phase in ("align", "idle", "stopped"):
        eff = 1.0                       # 非 track 阶段强制 1.0
        assert scaled_pulse(1560.0, eff) == 1560.0, phase
    assert scaled_pulse(1560.0, 0.0) == 1500.0, "track 阶段降级到 0 才是回中位"


def test_scaled_pulse_slows_down_instead_of_stopping():
    """仲裁层降级时**降速**（往中位靠），而不是急停——过弯道时短时丢线就靠这个撑过去。"""
    from scripts.bench_test import NEUTRAL_US, scaled_pulse
    assert scaled_pulse(1575.0, 1.0) == 1575.0
    assert scaled_pulse(1575.0, 0.0) == NEUTRAL_US      # 完全降级 = 回中位（停）
    assert scaled_pulse(1560.0, 2.0) == 1560.0          # 上限钳到 1.0，不会超速
    assert scaled_pulse(1560.0, -1.0) == NEUTRAL_US


def test_scaled_pulse_clamped_above_esc_deadband():
    """降速后的脉宽不得低于电调死区上沿，否则"降速维持"实为停车。
    2026-09-23 代码审查 A7：1560×0.5=1530us < 死区 1545us → 一降速车就停死。"""
    from scripts.bench_test import scaled_pulse
    from config import settings
    floor = settings.ESC_DEADBAND_US + 1
    assert scaled_pulse(1560.0, 0.5) == floor
    assert scaled_pulse(1575.0, 0.5) == floor
    assert scaled_pulse(settings.ESC_CREEP_US, settings.ARBITER_HOLD_THROTTLE_SCALE) == floor
    # 任意 >0 的缩放比例都不能掉进死区（0 是显式的"回中位停车"，例外）
    for scale in (0.05, 0.25, 0.5, 0.75, 0.99):
        assert scaled_pulse(1560.0, scale) >= floor, scale


if __name__ == "__main__":
    test_never_runs_without_board()
    test_board_stops()
    test_board_wins_over_force_run()
    test_lane_loss_stops_unless_forced()
    test_align_uses_start_speed()
    test_aligned_uses_track_speed()
    test_no_lane_mode_skips_alignment()
    test_default_pulses_are_above_esc_deadband()
    test_auto_target_accepts_stable_samples()
    test_auto_target_rejects_unstable_or_absurd()
    test_align_verdict_detects_direction()
    test_driving_off_course()
    test_acquire_creep_without_lane()
    test_acquire_does_not_override_force_run()
    test_creep_stage_ignores_throttle_scale()
    test_scaled_pulse_slows_down_instead_of_stopping()
    test_scaled_pulse_clamped_above_esc_deadband()
    test_bench_common_exposes_safety_api()
    test_restore_is_idempotent()
    print("test_bench_logic: all passed")
