"""误差滤波（ErrorFilter）单元测试。

跑法：
    cd dev && PYTHONPATH=. python tests/test_filters.py
"""
from __future__ import annotations

from control.filters import ErrorFilter


def test_first_value_passthrough():
    f = ErrorFilter(window=5, outlier=50)
    assert f.update(10.0) == 10.0
    assert f.last == 10.0


def test_weighted_average_favours_newest():
    f = ErrorFilter(window=5, outlier=100)
    for v in (0, 10, 20, 30, 40):
        out = f.update(v)
    # 权重 1..5 → (0*1+10*2+20*3+30*4+40*5)/15 = 400/15 ≈ 26.7
    assert abs(out - 26.667) < 0.01, f"应有偏新值的加权平均，实际 {out:.3f}"
    assert out > 20.0, "结果应偏向最新值"


def test_outlier_is_rejected_and_not_stored():
    f = ErrorFilter(window=5, outlier=50)
    for v in (0, 0, 0, 0, 0):
        f.update(v)
    before = f.last
    out = f.update(200.0)          # 明显离群
    assert out == before, f"离群点应直接沿用上次有效值，实际 {out}"
    assert all(abs(v) < 1e-9 for v in f._history), "离群点不应写入历史"
    # 之后正常值仍可进入
    assert f.update(5.0) != before


def test_consecutive_rejects_accept_the_new_level():
    """★ 连续被拒到上限 → 必须接受新值（"世界真的变了"），不能永久冻结。

    2026-09-22 落地实测：误差从 −30 阶跃到 +60~+160 后，原版滤波器一直沿用旧值 **1.6 秒**，
    期间车偏了十几厘米而控制器毫无反应、跑偏保护也不触发（它读的也是滤波值）。
    """
    f = ErrorFilter(window=5, outlier=60, max_rejects=4)
    for _ in range(5):
        f.update(0.0)
    assert f.last == 0.0
    outs = [f.update(200.0) for _ in range(3)]
    assert all(o == 0.0 for o in outs), f"前几次仍应沿用旧值（抗毛刺），实际 {outs}"
    out = f.update(200.0)                    # 第 4 次 → 认定世界变了
    assert out == 200.0, f"连续被拒到上限后应接受新值，实际 {out}"
    assert f.update(200.0) == 200.0, "接受之后应当稳定跟随新水平"


def test_rejects_counter_resets_on_good_value():
    """中间插一个正常值就要把"连续被拒"计数清零（否则等于不再抗毛刺）。"""
    f = ErrorFilter(window=5, outlier=60, max_rejects=3)
    for _ in range(5):
        f.update(0.0)
    f.update(200.0)          # 拒 1
    f.update(5.0)            # 正常 → 计数清零
    assert f.update(200.0) == f.last, "计数清零后应当重新开始抗毛刺"
    assert f._rejects == 1


def test_reset_clears_state():
    f = ErrorFilter()
    f.update(30.0)
    f.reset()
    assert f.last is None
    assert f.update(7.0) == 7.0, "复位后应重新从当前值开始，不受旧历史影响"


def test_nan_is_ignored():
    f = ErrorFilter()
    f.update(12.0)
    assert f.update(float("nan")) == 12.0


if __name__ == "__main__":
    test_first_value_passthrough()
    test_weighted_average_favours_newest()
    test_outlier_is_rejected_and_not_stored()
    test_consecutive_rejects_accept_the_new_level()
    test_rejects_counter_resets_on_good_value()
    test_reset_clears_state()
    test_nan_is_ignored()
    print("test_filters: all passed")
