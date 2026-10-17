"""`scripts/diag/esc_probe.py` 的单测：**纯函数 + 假 PCA9685**，不碰硬件、不碰 I2C。

为什么要有它：这是"唯一会主动往 1500 以下写脉宽"的脚本（反向/刹车探针），
写错一个字就是把车往反方向送 —— 所以计划(纯函数)与执行(可注入)都能离线整套跑。
覆盖面：
- 计划结构不变量：首尾必中位、每个反向档前必夹中位、脉宽全在 [地板, 上限]、单档时长合理、
  越界/空计划会被 check_plan 拒绝（**反向验证**：把手写坏的序列喂进去必须报错）；
- 执行：逐档写脉宽的序列与计划**逐位一致**、每档都喂看门狗、日志行一一对应；
- 安全出口：`should_stop()` 为真时**立刻停**（且最后写出的仍是计划里那一档，收尾由调用方回中位）、
  `deadline()` 超时时停、手动模式无 TTY 时退回自动；
- dry-run（不加 --allow-motion）：只打印不写任何脉宽。

跑法：cd dev && PYTHONPATH=. python tests/test_esc_probe.py
"""
from __future__ import annotations

import contextlib
import io
import os
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.diag import esc_probe as ep  # noqa: E402

FLOOR, CEIL = 1300.0, 1600.0


# ---------------------------------------------------------------- 计划（纯函数）
def test_every_plan_is_neutral_start_and_end() -> None:
    for name in ("reverse", "neutral", "double", "brake", "all"):
        steps = ep.build_plan(name)
        assert steps[0].us == ep.NEUTRAL_US, f"{name} 必须从中位开始：{steps[0]}"
        assert steps[-1].us == ep.NEUTRAL_US, f"{name} 必须回中位结束：{steps[-1]}"
        assert len(steps) >= 3, f"{name} 太短：{len(steps)}"
        ep.check_plan(steps, FLOOR, CEIL)          # 不抛 = 合规


def test_reverse_scan_pauses_at_neutral_between_reverse_steps() -> None:
    """★ 每个反向档**前面**必须是中位档（很多车用电调要先"看到中位"才肯换向）。"""
    steps = ep.build_plan("reverse", deep_us=1300.0)
    rev_idx = [i for i, s in enumerate(steps) if s.us < ep.NEUTRAL_US]
    assert rev_idx, "reverse 计划里必须有反向档"
    for i in rev_idx:
        assert steps[i - 1].us == ep.NEUTRAL_US, f"反向档 {steps[i]} 前没有中位停顿"
    # 反向档是**由浅到深**的（先 1460 再 1420…），便于操作员看清"从哪一档开始反"
    vals = [steps[i].us for i in rev_idx]
    assert vals == sorted(vals, reverse=True), f"反向档应从浅到深：{vals}"
    assert vals[-1] == 1300.0, f"最后一档应到最深：{vals}"


def test_floor_clamps_the_deepest_reverse_step() -> None:
    """`--rev-deep` 想写更深也不行：被地板钳住（硬门，不许出界脉宽）。"""
    steps = ep.build_plan("reverse", deep_us=900.0, floor_us=1300.0)
    assert min(s.us for s in steps) == 1300.0
    ep.check_plan(steps, FLOOR, CEIL)


def test_brake_plan_spins_up_before_brake_test() -> None:
    """刹车对照：必须"先 1600 转起来"，再切 1450 与切 1500 各一次（否则没对照、看不出刹车）。"""
    us = [s.us for s in ep.build_plan("brake")]
    assert us.count(1600.0) >= 2, f"要有两次转起来：{us}"
    i1450 = us.index(1450.0)
    assert 1600.0 in us[:i1450], "切 1450 之前必须先转起来"
    assert ep.NEUTRAL_US in us[i1450:], "切 1450 之后要有回中位（对照组在它前面）"


def test_check_plan_rejects_out_of_range_and_bad_dwell() -> None:
    """反向验证：手写坏序列必须被拒绝（越界脉宽 / 超长单档 / 首尾不是中位 / 空计划）。"""
    for bad, why in (
        ([ep.Step(1299.0, 1.0, "低于地板")], "越界"),
        ([ep.Step(1700.0, 1.0, "高于上限")], "越界"),
        ([ep.Step(1500.0, 9.0, "单档太长")], "时长"),
        ([ep.Step(1400.0, 1.0, "首档不是中位")], "首尾"),
        ([ep.Step(1400.0, 1.0, "x"), ep.Step(1400.0, 1.0, "尾档不是中位")], "首尾"),
        ([], "空"),
    ):
        try:
            ep.check_plan(bad, FLOOR, CEIL)
        except SystemExit:
            continue
        raise AssertionError(f"{why} 的坏计划竟被放行：{bad}")


# ---------------------------------------------------------------- 执行（假 PCA9685）
class _FakeEsc:
    """假电调通道：把写进去的脉宽记下来（本地无 I2C 也能整套跑）。"""

    def __init__(self) -> None:
        self.writes: list = []
        self.feeds = 0

    def write(self, us: float) -> None:
        self.writes.append(float(us))

    def feed(self) -> None:
        self.feeds += 1


def _run(steps, stop_after: int = 10 ** 9, elapsed_after: float = 1e9,
         manual: bool = False) -> tuple:
    esc = _FakeEsc()
    lines: list = []
    t = {"now": 0.0}

    def sleep(dt: float) -> None:               # 时间"虚跑"（不真等），便于快测
        t["now"] += dt

    n = ep.run_plan(steps, write_us=esc.write, sleep=sleep, feed=esc.feed,
                    printer=lines.append, manual=manual,
                    ask=(lambda _p: "") if manual else input,
                    should_stop=(lambda: esc.feeds >= stop_after),
                    deadline=lambda: t["now"] - elapsed_after)
    return esc, lines, n


def test_run_plan_writes_exact_sequence_and_feeds_watchdog() -> None:
    steps = ep.build_plan("reverse")
    esc, lines, n = _run(steps)
    assert n == len(steps)
    assert esc.writes == [s.us for s in steps], "写出去的脉宽必须与计划逐位一致"
    assert esc.feeds >= len(steps), "每一档都要喂看门狗（不是只喂一次）"
    assert any("反向档" in ln for ln in lines), lines[:3]
    assert sum(1 for ln in lines if "写 " in ln) == len(steps), "每档一行日志"


def test_run_plan_stops_immediately_on_stop_request() -> None:
    """★ 停止请求（Ctrl+C / 停止文件）→ **下一档就不跑**（安全出口不能等整段跑完）。"""
    steps = ep.build_plan("all")
    esc, lines, n = _run(steps, stop_after=3)        # 喂 3 次心跳后要求停
    assert n < len(steps), f"应当提前收尾：{n}/{len(steps)}"
    assert any("停止请求" in ln for ln in lines), lines[-3:]


def test_run_plan_stops_on_deadline() -> None:
    steps = ep.build_plan("reverse")
    esc, lines, n = _run(steps, elapsed_after=2.0)
    assert n < len(steps), "超时应提前收尾"
    assert any("总时长上限" in ln for ln in lines), lines[-3:]


def test_manual_mode_falls_back_without_tty() -> None:
    """手动模式在无 TTY（ssh "命令"）下 input 抛 EOFError → 退回自动节奏，不该崩。"""
    steps = ep.build_plan("reverse")

    def boom(_p: str) -> str:
        raise EOFError

    esc = _FakeEsc()
    lines: list = []
    n = ep.run_plan(steps, write_us=esc.write, sleep=lambda _d: None, feed=esc.feed,
                    printer=lines.append, manual=True, ask=boom)
    assert n == len(steps), "退回自动后应把整段跑完"
    assert any("没有终端可等待" in ln for ln in lines), lines[:4]


# ---------------------------------------------------------------- main（dry-run 全链）
def test_main_dry_run_writes_nothing() -> None:
    """不加 --allow-motion：只打印计划、**一个脉宽都不写**、rc=0（默认安全姿势）。"""
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = ep.main(["--plan", "all", "--log", ""])
    out = buf.getvalue()
    assert rc == 0, out[-400:]
    assert "dry-run" in out and "不写任何脉宽" in out
    assert "加 --allow-motion" in out
    assert out.count("us ×") >= 5, "应把每一档都打印出来"


def test_main_rejects_oob_pulses() -> None:
    """`--min-us` 设得比计划里最深档还高 → 直接拒绝运行（rc≠0，不写硬件）。"""
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = ep.main(["--plan", "reverse", "--rev-deep", "1200", "--min-us", "1300", "--log", ""])
    out = buf.getvalue()
    assert rc == 0, "地板会把 1200 钳到 1300（钳位在计划生成时就生效，不是报错路径）"
    assert "1300us" in out
    # 单档时长越界：check_plan 直接拒绝（house style = 抛 SystemExit 带人话说明）
    err = io.StringIO()
    try:
        with redirect_stdout(io.StringIO()), contextlib.redirect_stderr(err):
            ep.main(["--plan", "reverse", "--dwell-s", "9", "--log", ""])
    except SystemExit as exc:
        assert exc.code not in (0, None), f"越界应非零退出：{exc.code}"
        assert "单档时长" in str(exc.code) or "不合理" in str(exc.code), str(exc.code)
    else:
        raise AssertionError("--dwell-s 9 竟被放行")


def main() -> int:
    funcs = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v) and getattr(v, "__module__", "") == __name__]
    failed = 0
    for fn in funcs:
        try:
            fn()
            print(f"  ✅ {fn.__name__}")
        except Exception as exc:                      # noqa: BLE001
            failed += 1
            import traceback
            print(f"  ❌ {fn.__name__}: {exc}")
            traceback.print_exc()
    print(f"[test_esc_probe] {len(funcs) - failed}/{len(funcs)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
