#!/usr/bin/env python3
"""电调**反向/刹车探针**（台架专用；2026-10-17）。

为什么要它
----------
"这车没有刹车"这句话目前只是"**我们的跑车代码从来没写过 <1500us**"（`planner` 只给正油门、
`lane_final_test` 只写 1500/1560/1590+；全仓无一处负值），**并不能证明电调不支持反向**。
而"以前能用后退、后来不能用"最常见的两种原因（① 电调被误改了运行模式/油门行程标定，
② 那次其实是在官方手动驾驶界面里按的）都得靠一次台架实测来分叉。

它做什么（四轮**必须悬空**）
--------------------------
按预设"计划"逐档写脉宽并**每档之间回中位**（很多车用电调把"前进→直接给反向"当**刹车**，
必须先回中位；有些还要"回中位再给第二下"才真反转）：

    --plan reverse   ★ 反向最小集（默认）：1500 → 1460/1420/1380/1340/1300 各 1.5s（档间回中位 0.6s）
    --plan neutral   中位搜索：1500 → 1505…1560 上扫（找"轮子完全不转"的中心；官方那台是 1640！）
    --plan double    双踩序列：1500 → 1300 → 1500(0.6s) → 1300（"先刹再倒"型电调第二段才反转）
    --plan brake     刹车/空档对照：1600(2.5s 转起来) → 1450(2s) vs → 1500(2s)，看是立刻停还是滑很久
    --plan all       上面四段依次跑（约 90s）

判读（跑完对着日志回答三句话，结论就出来了）
-------------------------------------------
1. **轮子有没有反着转？在哪个脉宽开始反？**（有 → 电调支持反向，反向带 = 那个脉宽往下；
   没有 → 试 `--plan double`，还没有就是电调当前模式不含反向 → 查电调编程菜单/中位）
2. **上扫到哪个脉宽轮子开始正转？**（= 死区上沿；现场登记值 1545，若差很多说明中位漂了）
3. **1600 转起来后切 1450：立刻停（刹车）还是滑很久（空档）？**——和切 1500 的对照组比。

安全（照 AGENTS.md §1.1 + §2.8 的台架姿势）
------------------------------------------
* **默认 dry-run**：不加 `--allow-motion` 只打印计划，一个字都不写；
* 四轮悬空（或场地空旷 + 人持断电）；脉宽硬钳在 `[--min-us 1300, --max-us 1600]`；
* 走 `scripts/bench_common.MotorSession`：进去停 `opi-control` 让出 PCA9685，**任何退出路径**都
  电调归零 + 舵机回中 + 恢复远程控制阶段；
* **电调看门狗（独立进程）全程在位**：脚本万一被杀/卡死，1s 后它把电调拉回 1500（中位=停），
  而不是停在反向脉宽上（这正是 10-15 事故的教训）；
* 全程有总时长上限 `--max-seconds`（默认 120），每档之间必回中位，结束再回中位停 1s。

用法（车上，台架）
----------------
    sudo python3 /root/dev/scripts/diag/esc_probe.py                     # 只看计划（dry-run）
    sudo python3 /root/dev/scripts/diag/esc_probe.py --allow-motion                 # 反向最小集
    sudo python3 /root/dev/scripts/diag/esc_probe.py --allow-motion --plan all      # 四段全跑
    sudo python3 /root/dev/scripts/diag/esc_probe.py --allow-motion --manual        # 每档等回车（看得准）
    # 日志落在 /root/dev/logs/esc_probe_*.log（跑完拿它对照观察）
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from config import settings            # noqa: E402

NEUTRAL_US = float(settings.ESC_STOP_US)   # 中位/停（1500）—— 一切收尾都回到它
REV_DEEP0 = 1300.0                     # 反向扫到的最深脉宽（地板由 --min-us 再卡一道）
REV_TOP0 = 1480.0                      # 反向扫的起点（刚过中位；官方网页的倒车带是 1400~1500）
REV_STEP0 = 40.0                       # 反向扫的步长（40us ≈ 官方网页 100 档里的 8 档）
UP_SCAN = (1505, 1510, 1520, 1530, 1540, 1545, 1550, 1560)   # 上扫档（找起转点）
PAUSE_S = 0.6                          # 档间回中位时长（电调需要"看到中位"才肯换向）

# 官方网页（/var/www/orangepi-rc/index.html + /root/opi-control）的倒车口径，2026-10-17 回原码核实：
#   前端默认 min=1400 / center=1500 / max=1600（页面设置面板里可改，**每条 drive 消息都带着这三个值**）；
#   手柄"扳机模式"油门 = RT − LT（LT=后退）、"摇杆模式" = −(左摇杆 Y)（向下=后退）；
#   后端 percent<0 → 脉宽 = center + (p/100)×(center − min) ⇒ **倒车带 = 1400~1500us，最深 1400**。
#   ⇒ 用户"网页端低于 1500 能倒退（两次）"就发生在这条带里，而我们的 --plan reverse 把 1460/1420
#     都写过了却没动静 ⇒ 差别多半在**顺序**（先前进→刹住→回中→再压），所以加了 --plan fwd-rev。


@dataclass(frozen=True)
class Step:
    """一档：写 `us` 脉宽、保持 `dwell_s` 秒、`label` 是给人看的说明。"""
    us: float
    dwell_s: float
    label: str


# ================================================================ 计划（纯函数，单测直接钉住）
def build_plan(name: str, deep_us: float = REV_DEEP0, floor_us: float = 1300.0,
               dwell_s: float = 1.5, up_scan: Sequence[float] = UP_SCAN,
               rev_step: float = REV_STEP0) -> List[Step]:
    """按名字生成脉宽序列（**首尾必是中位**；反向档之间必夹一个中位档）。"""
    deep = max(float(floor_us), float(deep_us))
    steps: List[Step] = [Step(NEUTRAL_US, 1.0, "回中位（电调解锁/清状态）")]

    def rev_scan() -> List[Step]:
        """从 1480 一路扫到 deep（步长 rev_step）：每档前先回中位（多数电调要先「看到中位」）。"""
        vals, v = [], float(REV_TOP0)
        while v > deep + 1e-6:
            vals.append(round(v))
            v -= max(1.0, float(rev_step))
        vals.append(round(deep))
        out: List[Step] = []
        for us in vals:
            if us < floor_us:
                continue
            out.append(Step(NEUTRAL_US, PAUSE_S, "中位（换向前的停顿）"))
            out.append(Step(float(us), dwell_s, f"反向档 {us:.0f}us —— 看轮子是不是**反着转**"))
        return out

    def fwd_rev() -> List[Step]:
        """★ 先前进 → 带刹车 → 回中 → **平滑下压** → 压到底保持（照官方网页那次"能倒"的顺序）。

        为什么单列这一段：官方网页的倒车带宽就是 1400~1500us，而 --plan reverse 把 1460/1420 都写过、
        轮子不动 ⇒ 剩下的最大差别是**顺序**。车用电调常见的互锁：① 反向只在"刚前进过/刹住过"之后才放行；
        ② 反向要求油门**平滑压过去**（跳变只认成刹车）。这一段把两种都覆盖：
        1600 前进 → 1450 停一下（若在刹车带上会刹）→ 回中 0.5s → 从 1480 每档 0.35s 平滑压到 deep → 压住 2.5s。
        """
        out = [
            Step(1600.0, 2.0, "前进 1600us × 2s（先真的往前转起来 —— 很多电调的反向互锁要这一步）"),
            Step(1450.0, 1.5, "收到 1450us（若在刹车带上=刹住；不在=什么都没发生）"),
            Step(NEUTRAL_US, 0.5, "回中位（松开一下，给电调一个「换向窗口」）"),
        ]
        n = 6
        for i in range(n + 1):
            us = REV_TOP0 + (deep - REV_TOP0) * (i / n)      # 平滑下压，中途不回中位
            out.append(Step(round(us), 0.35,
                            f"下压 {us:.0f}us（第 {i + 1}/{n + 1} 档，0.35s 一档，中途不回中位）"))
        out.append(Step(deep, 2.5, f"压到底 {deep:.0f}us × 2.5s —— 看轮子是不是**反着转**（并保持住）"))
        return out

    def neutral_scan() -> List[Step]:
        out: List[Step] = []
        for us in up_scan:
            out.append(Step(NEUTRAL_US, PAUSE_S, "中位"))
            out.append(Step(float(us), dwell_s, f"上扫 {us:.0f}us —— 看轮子从哪一档开始**正转**"))
        return out

    def double_tap() -> List[Step]:
        return [
            Step(deep, dwell_s, f"第 1 踩 {deep:.0f}us（多数电调这踩只是**刹车**）"),
            Step(NEUTRAL_US, PAUSE_S, "回中位"),
            Step(deep, dwell_s + 0.5, f"第 2 踩 {deep:.0f}us —— 若这时才反转 = 「先刹再倒」型电调"),
        ]

    def brake_test() -> List[Step]:
        return [
            Step(1600.0, 2.5, "前进 1600us 转起来（观察起点）"),
            Step(1500.0, 2.0, "切中位 1500us —— **对照组**：滑很久 = 空档"),
            Step(1600.0, 2.5, "再转起来"),
            Step(1450.0, 2.0, "切 1450us —— 立刻停 = **有主动刹车**；滑很久 = 空档"),
        ]

    plan = {
        "reverse": rev_scan,
        "fwd-rev": fwd_rev,
        "neutral": neutral_scan,
        "double": double_tap,
        "brake": brake_test,
        "all": lambda: rev_scan() + fwd_rev() + double_tap() + neutral_scan() + brake_test(),
    }.get(name)
    if plan is None:
        raise SystemExit(f"[ESC] 未知计划 {name!r}"
                         f"（可选 reverse/fwd-rev/neutral/double/brake/all）")
    steps += plan()
    steps.append(Step(NEUTRAL_US, 1.0, "结束回中位"))
    return steps


def check_plan(steps: Sequence[Step], floor_us: float, ceil_us: float) -> None:
    """硬门：任何一档越界就拒绝运行（宁可跑不了，也不许写出界脉宽）。"""
    if not steps:
        raise SystemExit("[ESC] ❌ 计划是空的")
    for st in steps:
        if not (floor_us - 1e-6 <= st.us <= ceil_us + 1e-6):
            raise SystemExit(f"[ESC] ❌ 计划里有越界脉宽 {st.us:.0f}us"
                             f"（允许 {floor_us:.0f}~{ceil_us:.0f}）：{st.label}")
        if st.dwell_s <= 0 or st.dwell_s > 5.0:
            raise SystemExit(f"[ESC] ❌ 单档时长不合理 {st.dwell_s}s：{st.label}")
    if abs(steps[0].us - NEUTRAL_US) > 1e-6 or abs(steps[-1].us - NEUTRAL_US) > 1e-6:
        raise SystemExit("[ESC] ❌ 计划必须**首尾都是中位**（安全：进/出都不能停在反向脉宽上）")


# ================================================================ 执行
def run_plan(steps: Sequence[Step], write_us: Callable[[float], None],
             sleep: Callable[[float], None] = time.sleep,
             feed: Optional[Callable[[], None]] = None,
             printer: Callable[[str], None] = print,
             manual: bool = False, ask: Callable[[str], str] = input,
             deadline: Optional[Callable[[], float]] = None,
             should_stop: Optional[Callable[[], bool]] = None) -> int:
    """逐档写脉宽（每档都喂看门狗）；返回写了多少档。

    `write_us/sleep/feed/printer/ask` 都可注入 → 本地单测用"假 PCA9685"整套跑完。
    `deadline()` 给一个"距开始多少秒"的回调，用于总时长上限（超了立刻回中位收尾）。
    """
    t0 = time.monotonic()
    n = 0
    for i, st in enumerate(steps, 1):
        if deadline is not None and deadline() > 0:
            printer(f"[ESC] ⏱️ 到达总时长上限 → 立刻回中位收尾（第 {i}/{len(steps)} 档未跑）")
            break
        if should_stop is not None and should_stop():
            printer(f"[ESC] ⛔ 收到停止请求（Ctrl+C / 停止文件）→ 立刻回中位收尾"
                    f"（第 {i}/{len(steps)} 档未跑）")
            break
        if manual:
            try:
                ask(f"[ESC] （{i}/{len(steps)}）下一档 {st.us:.0f}us × {st.dwell_s:.1f}s"
                    f" —— {st.label}；回车继续…")
            except EOFError:                       # 无 TTY（ssh "命令" / 重定向）→ 退回自动
                printer("[ESC] ⚠️ 没有终端可等待（input 失败）→ 退回自动节奏")
                manual = False
        printer(f"[ESC] {time.strftime('%H:%M:%S')} t={time.monotonic() - t0:5.1f}s  "
                f"写 {st.us:.0f}us × {st.dwell_s:.1f}s  —— {st.label}")
        write_us(st.us)
        n += 1
        # 把长档切成小片，每片喂一次看门狗（心跳不能等一整档）
        left = float(st.dwell_s)
        while left > 0:
            piece = min(0.2, left)
            sleep(piece)
            left -= piece
            if feed is not None:
                feed()
            if should_stop is not None and should_stop():
                printer("[ESC] ⛔ 收到停止请求 → 回中位收尾")
                return n
    return n


def _readable_int(v: str) -> int:
    try:
        return int(float(v))
    except ValueError:
        raise argparse.ArgumentTypeError(f"要整数，实得 {v!r}")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="电调反向/刹车探针（台架：四轮必须悬空；默认 dry-run 不写任何脉宽）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--plan", choices=("reverse", "fwd-rev", "neutral", "double", "brake", "all"),
                    default="reverse",
                    help="跑哪一段（见文件头判读说明；★ fwd-rev = 先前进→刹车→平滑下压，"
                         "照官方网页「能倒」的顺序）")
    ap.add_argument("--rev-step", type=float, default=REV_STEP0,
                    help="反向扫步长（us；从 1480 往下扫到 --rev-deep）")
    ap.add_argument("--rev-deep", type=_readable_int, default=int(REV_DEEP0),
                    help="反向扫到的最深脉宽（us；硬地板见 --min-us）")
    ap.add_argument("--min-us", type=_readable_int, default=1300, help="允许写出的最小脉宽（硬地板）")
    ap.add_argument("--max-us", type=_readable_int, default=1600,
                    help="允许写出的最大脉宽（调试限速；别乱抬）")
    ap.add_argument("--dwell-s", type=float, default=1.5, help="每档保持时长（秒）")
    ap.add_argument("--manual", action="store_true", help="每档等回车再走（看得准；无 TTY 时自动退回）")
    ap.add_argument("--max-seconds", type=float, default=120.0, help="总时长上限（到时立刻回中位收尾）")
    ap.add_argument("--allow-motion", "--i-know-wheels-are-up", dest="allow_motion",
                    action="store_true",
                    help="确认车可动（**四轮悬空**）——不给就是 dry-run，只打印计划")
    ap.add_argument("--watchdog-s", type=float, default=1.0,
                    help="电调看门狗超时（秒；0=关。主控被杀/卡死时由它把电调拉回中位）")
    ap.add_argument("--stop-file", default="/dev/shm/lane_stop",
                    help="停止文件（**文件存在 = 立刻回中位收尾**；空串=关）")
    ap.add_argument("--log", default="", help="日志路径（默认 /root/dev/logs/esc_probe_<时间>.log）")
    return ap


class _Tee:
    """把输出同时写进日志文件（现场"跑完对着日志回答三句话"）。"""

    def __init__(self, path: str) -> None:
        self.fh = None
        if path:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                self.fh = open(path, "a", encoding="utf-8")
            except Exception as exc:                        # noqa: BLE001
                print(f"[ESC] ⚠️ 日志打不开（只打印到屏幕）：{exc}")

    def __call__(self, *args) -> None:
        line = " ".join(str(a) for a in args)
        print(line, flush=True)
        if self.fh is not None:
            self.fh.write(line + "\n")
            self.fh.flush()

    def close(self) -> None:
        if self.fh is not None:
            self.fh.close()
            self.fh = None


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    log_path = args.log or os.path.join(
        "/root/dev/logs" if os.path.isdir("/root/dev") else os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs"),
        time.strftime("esc_probe_%Y%m%d_%H%M%S.log"))
    say = _Tee(log_path)
    try:
        steps = build_plan(args.plan, deep_us=args.rev_deep, floor_us=args.min_us,
                           dwell_s=args.dwell_s, rev_step=args.rev_step)
        check_plan(steps, args.min_us, args.max_us)
        say("=" * 78)
        say(f"[ESC] 电调反向/刹车探针 | 计划={args.plan} | {len(steps)} 档"
            f" | 脉宽范围 {args.min_us}~{args.max_us}us | {'手动' if args.manual else '自动'}"
            f"{args.dwell_s:.1f}s/档")
        say(f"[ESC] 日志：{log_path}")
        say("[ESC] ⚠️ 前提：**四轮悬空**（或场地空旷 + 人持断电）；车不停先断电")
        say("[ESC] ⚠️ 判读三条：① 轮子有没有反着转、哪个脉宽开始反 "
            "② 上扫到哪个脉宽正转（现场登记死区 1545） ③ 1600→1450 是立刻停（刹车）还是滑很久（空档）")
        if not args.allow_motion:
            say("[ESC] —— dry-run（没给 --allow-motion）：只打印计划，不写任何脉宽 ——")
            t = 0.0
            for i, st in enumerate(steps, 1):
                say(f"[ESC]   {i:2d}. t≈{t:5.1f}s  {st.us:6.0f}us × {st.dwell_s:.1f}s  {st.label}")
                t += st.dwell_s
            say(f"[ESC] 合计约 {t:.1f}s；真正要跑：加 --allow-motion（四轮悬空！）")
            return 0

        # ---- 真正的台架运行：安全层 + 看门狗 ----
        from scripts.bench_common import MotorSession, install_signal_guard
        state = {"stopped": False, "driver": None, "restored": False}
        install_signal_guard(state)            # 信号只置标志；MotorSession 自己挂了 atexit 收尾
        # 看门狗**晚一点拉**（见下面 1 处注释）：2026-10-17 首次台架实测发现，
        # 若在建会话/倒计时之前就拉起，那 ~5.5s（MotorSession 2.5s + 倒计时 3s）没有心跳
        # ⇒ 它必然触发一次并写 1500（无害，但退出码 2 会让人误以为"跑动中卡过"）。
        wd = None
        t0 = time.monotonic()
        rc = 0
        try:
            with MotorSession(state, enabled=True,
                              speed_us_max=float(args.max_us)) as pca:
                if pca is None:
                    say("[ESC] ❌ 打开 PCA9685 失败（看 car-mode.sh status / I2C 接线）")
                    return 1
                say(f"[ESC] 3 秒后开始（想中止：`touch {args.stop_file}` 或 Ctrl+C；"
                    f"跑动中车不停就**直接断电**）…")
                for _ in range(6):                             # 3s，分片以便随时响应停止
                    if state.get("stopped") or (args.stop_file and os.path.exists(args.stop_file)):
                        say("[ESC] ⛔ 收到停止请求 → 不开始")
                        return 0
                    time.sleep(0.5)
                check_plan(steps, args.min_us, args.max_us)      # 起跑前再核一次
                # ★ 现在才拉看门狗：此后的心跳间隔 ≤0.2s（run_plan 分片喂），
                #   退出码 2/3 才真正等于"跑动中主控卡过"。（此前的窗口里电调一直是 1500 中位，
                #   MotorSession 的 center_all 已经写过，没有"带着油门没人管"的风险。）
                if args.watchdog_s > 0:
                    try:
                        from scripts.esc_watchdog import WatchdogClient
                        wd = WatchdogClient(timeout_s=args.watchdog_s)
                        wd.start(log_path=os.path.join(os.path.dirname(log_path),
                                                       "esc_watchdog.log"))
                        say(f"[ESC] ✅ 电调看门狗已拉起（{args.watchdog_s:.1f}s 不喂心跳就强制回中位）")
                    except Exception as exc:                    # noqa: BLE001
                        wd = None
                        say(f"[ESC] ⚠️ 看门狗没起来（{exc}）——请人守着、手边留断电开关！")
                run_plan(steps,
                         write_us=lambda us: pca.write_us(pca.CH_ESC, us),
                         feed=(wd.feed if wd is not None else None),
                         printer=say, manual=args.manual,
                         deadline=lambda: (time.monotonic() - t0) - args.max_seconds
                         if args.max_seconds > 0 else -1.0,
                         should_stop=lambda: bool(state.get("stopped"))
                         or bool(args.stop_file and os.path.exists(args.stop_file)))
                pca.write_us(pca.CH_ESC, NEUTRAL_US)             # 收尾再明确回一次中位
                # ★ 跑完了就**立刻解除看门狗**：恢复远程阶段（停/起服务 + 校验推流）要好几秒，
                #   那期间没人喂心跳 ⇒ 它必然"触发"一次并写 1500（无害，但退出码 2 会让人
                #   误以为"跑动中卡过"——2026-10-17 两次实测都踩到这个假警告，日志时间戳已证实
                #   触发点落在收尾窗口）。此刻电调已经写回 1500，不需要它再兜底。
                if wd is not None:
                    try:
                        code = wd.stop()
                        wd = None
                        if code in (2, 3):
                            say("[ESC] ⚠️ 看门狗在**跑动期间**触发过（主控真的卡过）——看它的日志")
                        else:
                            say("[ESC] ✅ 看门狗正常解除（跑动期间一次都没触发）")
                    except Exception as exc:                    # noqa: BLE001
                        wd = None
                        say(f"[ESC] ⚠️ 解除看门狗失败：{exc}")
        except KeyboardInterrupt:
            say("[ESC] Ctrl+C → 回中位 + 收尾（电调 1500）")
        finally:
            if wd is not None:                              # 只在异常路径下才还挂着
                try:
                    code = wd.stop()
                    if code in (2, 3):
                        say("[ESC] ⚠️ 看门狗触发过（主控卡过）——看它的日志")
                except Exception as exc:                    # noqa: BLE001
                    say(f"[ESC] ⚠️ 停看门狗失败：{exc}")
            say("[ESC] 收尾：电调回中位、舵机回中、恢复远程控制阶段（MotorSession 负责）")
        say("")
        say("[ESC] —— 跑完了，请回答这三句（对着上面的日志时间/脉宽） ——")
        say("[ESC]  ① 轮子有没有**反着转**？在哪个脉宽开始？（有 → 电调支持反向；一次都没有 → "
            "再跑 --plan double；还没有 → 电调当前模式不含反向，查编程菜单/中位漂移）")
        say("[ESC]  ② 上扫（--plan neutral）到哪个脉宽轮子开始**正转**？（现场登记 1545；差得多 = 中位漂了）")
        say("[ESC]  ③ 1600→1450 与 1600→1500 相比：**立刻停 = 有主动刹车**；一样滑 = 空档（无刹车）")
        say("[ESC]      ⚠️ 四轮悬空时轮子惯性太小、两种都「瞬间停」，判不出来：改用手**拨轮子**"
            "（给 1450 时若明显发涩/有阻力 = 电调在刹车；给 1500 应很松），或在地面空旷处跑一次 --plan brake")
        say(f"[ESC] 日志已存：{log_path}（连同观察结果一起发回来）")
        return rc
    finally:
        say.close()


if __name__ == "__main__":
    sys.exit(main())
