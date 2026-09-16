"""PID 单元测试（含 dt 行为、限幅、复位）。

跑法：
    cd dev && PYTHONPATH=. python tests/test_pid.py
"""
from __future__ import annotations

from control.pid import PID


def test_proportional_only():
    pid = PID(kp=2.0, ki=0.0, kd=0.0)
    assert abs(pid.update(3.0, 0.1) - 6.0) < 1e-9


def test_output_is_clamped():
    pid = PID(kp=1.0, out_limit=10.0)
    assert pid.update(50.0, 0.1) == 10.0
    assert pid.update(-50.0, 0.1) == -10.0


def test_integral_accumulates_with_dt_and_is_limited():
    pid = PID(kp=0.0, ki=1.0, integral_limit=2.0)
    assert abs(pid.update(1.0, 1.0) - 1.0) < 1e-9
    assert abs(pid.update(1.0, 1.0) - 2.0) < 1e-9
    assert abs(pid.update(1.0, 1.0) - 2.0) < 1e-9, "积分应被限幅在 2.0"


def test_derivative_is_dt_normalised_and_filtered():
    pid = PID(kp=0.0, ki=0.0, kd=1.0, d_alpha=1.0)   # alpha=1 → 不滤波，便于断言
    pid.update(0.0, 0.1)
    out = pid.update(10.0, 0.1)                      # d = 10/0.1 = 100
    assert abs(out - 100.0) < 1e-6, f"微分应按 dt 归一，实际 {out}"


def test_reset_clears_kick():
    """官方实现切状态不重置 → 输出踢腿；我们要求 reset 后无残留。"""
    pid = PID(kp=0.0, ki=1.0, kd=1.0, d_alpha=1.0)
    pid.update(5.0, 0.1)
    pid.update(5.0, 0.1)
    pid.reset()
    out = pid.update(0.0, 0.1)
    assert abs(out) < 1e-9, f"reset 后首个输出应为 0，实际 {out}"


def test_zero_dt_does_not_explode():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    pid.update(1.0, 0.1)
    out = pid.update(1.0, 0.0)      # 时间戳异常
    assert abs(out) < 1e3, f"dt=0 不应产生爆炸输出，实际 {out}"


if __name__ == "__main__":
    test_proportional_only()
    test_output_is_clamped()
    test_integral_accumulates_with_dt_and_is_limited()
    test_derivative_is_dt_normalised_and_filtered()
    test_reset_clears_kick()
    test_zero_dt_does_not_explode()
    print("test_pid: all passed")
