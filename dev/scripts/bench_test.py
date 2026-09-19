#!/usr/bin/env python3
"""台架测试（**唯一入口**）：蓝板发车 + 扫线循迹 + 起步对齐，全部带同一套安全护栏。

════════════════════════════ 它做什么 ════════════════════════════
规则（可用参数关掉其中一部分）：
    没见过板   → 中位（绝不动，安全不变量）
    见板       → 中位
    板移开     → **起步对齐**：用最低能动的脉宽慢慢走，一边把车摆正到车道中央；
                 对准后（误差进入容差并稳定 N 帧）→ **加速到循迹速度**
    循迹中再见板 → 立即停
    无有效车道   → 停车（除非 --force-run，台架观察用，比赛绝不能用）
    `--no-lane`  → 只做"蓝板 + 电调"（等同旧的 bench_board_test）

起步对齐（针对"车不一定会被摆正放在车道上"）：
    车是阿克曼，原地转不了向，所以对齐 = **低速蠕动 + 转向修正**：
    电调给 `--start-us`（默认 1530，最低能动的脉宽），转向由 PID 往车道中心修；
    |误差| ≤ `--align-tol` 连续 `--align-frames` 帧（或超过 `--align-max-s` 秒）→ 提速。
    对齐期间车道一直无效 → 停车并提示重新摆放（不盲目前冲）。

════════════════════════════ 摄像头 ════════════════════════════
**默认单摄 `--camera 2`（下摄）**：蓝板与扫线都用它（实测板在车前时下摄也看得到：面积 0.638）。
不需要切摄像头 —— 这是本项目里最省事、最少机械动作的配置。
（主程序 `vision_main.py` 是另一套：发车阶段用云台主摄 `--gate-camera 0` 看板，放行后切下摄跑巡线。）

════════════════════════════ 安全（见 scripts/bench_common.py）════════════════════════════
· 只有驱动电机时才停 opi-control；--no-motor 完全不碰它
· 摄像头用 camera_exclusive：进则停推流、出则必恢复
· 不给 --allow-motion 就拒绝驱动电机；--max-seconds 到时自动停
· **任何退出路径**（Ctrl+C / kill / 关会话 SIGHUP / 异常 / 正常）都归零并恢复远程控制阶段，
  且恢复后逐项验证打印 ✅/❌

用法（车上）：
    # ① 静止标定车道中心（把车摆正在车道中央；不动电机，可反复跑）
    sudo python3 /root/dev/scripts/bench_test.py --calibrate 30

    # ② 只看读数不动电机
    sudo python3 /root/dev/scripts/bench_test.py --no-motor

    # ③ 正式跑：板在→停 / 板移开→低速对齐→循迹 / 再见板→停
    sudo python3 /root/dev/scripts/bench_test.py --target-x 377 --allow-motion --max-seconds 30

    # ④ 只测蓝板+电调（不循迹、不摆正）
    sudo python3 /root/dev/scripts/bench_test.py --no-lane --allow-motion

    # ⑤ 观察转向（忽略'无有效车道'保护）
    sudo python3 /root/dev/scripts/bench_test.py --force-run --no-motor
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from config import settings
from control.lane_arbiter import LaneArbiter
from control.planner import Planner
from scripts.bench_common import NEUTRAL_US, MotorSession, install_signal_guard
from vision.camera_guard import camera_exclusive, open_camera
from vision.lane_scan import LaneScanner
from vision.start_gate import StartGate

# 台架默认参数（都可用命令行覆盖）
# 2026-09-16 架空实测：电调 1540us 轮子不转、≈1545us 起转 → 取值必须在死区之上
START_US_DEFAULT = float(settings.ESC_CREEP_US)   # 起步对齐时的脉宽（1560us）
SPEED_US_DEFAULT = 1575.0     # 对准后的循迹速度（≈15% 行程，慢速起步用）
ALIGN_TOL_PX = 15.0           # 对齐容差（像素）
ALIGN_FRAMES = 8              # 连续多少帧在容差内算对准
ACQUIRE_S = 3.0               # 发车后允许"低速探路找线"的窗口（秒）：看不到线也能往前拱一小段

# 「边跑边标定」：竞速比赛里裁判不会留出"先标定再跑"的时间，
# 所以标定必须发生在正式流程内部 —— 发车瞬间车正停在车道中央（人工摆位就是这个前提），
# 那段静止画面就是标定样本：采够 AUTO_TARGET_FRAMES 帧、波动够小、数值在合理范围内，才敢改车道中心。
AUTO_TARGET_FRAMES = 12
AUTO_TARGET_SPREAD_PX = 25.0
AUTO_TARGET_MIN_PX = 120.0
AUTO_TARGET_MAX_PX = 540.0
AUTO_TARGET_MIN_CONF = 0.35     # 采样门槛：低于这个置信度的帧不进标定样本（宁可标不上，不能标歪）

# 跑偏保护：车在动、误差却持续这么大 → 疑似转向方向反了 / 车没跟上，直接停车
# （80px ≈ 34cm 横向误差，已经在 1.22m 赛道里明显偏离中线；2s 是"确认真跑偏"的去抖时间）
DIVERGENCE_PX = 80.0
DIVERGENCE_S = 2.0


def driving_off_course(error: float, out_us: float, since: Optional[float], now: float,
                       limit_px: float = DIVERGENCE_PX,
                       limit_s: float = DIVERGENCE_S) -> bool:
    """持续在给动力、误差却一直超过限值 → 该停车（纯函数，便于单测）。"""
    if out_us <= NEUTRAL_US or error <= limit_px or since is None:
        return False
    return (now - since) > limit_s


def auto_target(samples) -> Optional[float]:
    """从静止画面采样里算新的车道中心；不可信时返回 None（纯函数，便于单测）。"""
    if len(samples) < AUTO_TARGET_FRAMES:
        return None
    if (max(samples) - min(samples)) > AUTO_TARGET_SPREAD_PX:
        return None
    avg = statistics.mean(samples)
    if not (AUTO_TARGET_MIN_PX <= avg <= AUTO_TARGET_MAX_PX):
        return None
    return float(avg)


def align_verdict(errors) -> str:
    """用对齐阶段误差的走势判断"转向方向对不对"（纯函数，便于单测）。

    对齐阶段车在低速蠕动、舵机在按 PID 修正：如果方向对，像素误差会变小；
    方向反了（STEER_SIGN 写反）误差会越修越大 —— 与其等它冲出去，不如在这里说清楚。
    """
    if len(errors) < 8:
        return "样本太少，这次不判断转向方向"
    head = statistics.median(errors[:5])
    tail = statistics.median(errors[-5:])
    if tail <= max(6.0, head * 0.7):
        return f"✅ 转向方向正确：对齐期误差 {head:.0f}px → {tail:.0f}px（在收敛）"
    if tail >= head * 1.3 and tail > 12.0:
        return (f"⚠️ 对齐期误差在变大（{head:.0f}px → {tail:.0f}px）：先跑 --steer-test 确认方向，"
                f"反向就往 config/site.yaml 写 steer_sign: -1；也可能是车摆得太斜/白线被挡")
    return f"对齐期误差基本不变（{head:.0f}px → {tail:.0f}px，幅度小看不出方向）"


@dataclass
class Decision:
    out_us: float
    phase: str                # idle / stopped / align / track
    reason: str


def decide(blocked: bool, seen_board: bool, lane_ok: bool, aligned: bool,
           force_run: bool, speed_us: float, start_us: float,
           neutral_us: float = NEUTRAL_US, no_lane: bool = False,
           acquire: bool = False) -> Decision:
    """台架决策（纯函数，便于单测）。安全优先级：未见过板 > 板在 > 无车道 > 对齐 > 循迹。

    `acquire=True` 是**发车后的低速探路窗口**：车停在起点时下摄可能还没看到白线
    （线在视野边缘/太近处投影不到），若这时也按"无有效车道就停"，车会**永远动不了**
    ——2026-09-16 实测就这么卡住过（电调全程 1500us）。所以发车后允许用蠕动速度
    往前拱一小段（窗口由调用方限时），边拱边找线；找回线就转正常对齐/循迹。
    """
    if not seen_board:
        return Decision(neutral_us, "idle", "未见过板")
    if blocked:
        return Decision(neutral_us, "stopped", "板在")
    if no_lane:                                   # --no-lane：只测蓝板+电调
        return Decision(speed_us, "track", "跑(不循迹)")
    if not lane_ok and not force_run:
        if acquire:
            return Decision(start_us, "align", "起步探路（低速找线）")
        return Decision(neutral_us, "stopped", "无有效车道")
    if not aligned:
        return Decision(start_us, "align", "起步对齐")
    return Decision(speed_us, "track", "循迹")


def main() -> int:
    ap = argparse.ArgumentParser(description="台架测试：蓝板发车 + 扫线循迹 + 起步对齐")
    ap.add_argument("--camera", type=int, default=2, help="摄像头：2=下摄（默认，蓝板+循迹都用它），0=云台主摄")
    ap.add_argument("--start-us", type=float, default=START_US_DEFAULT,
                    help=f"起步对齐脉宽（默认 {START_US_DEFAULT:.0f}，必须 > 死区 {settings.ESC_DEADBAND_US}）")
    ap.add_argument("--speed-us", type=float, default=SPEED_US_DEFAULT,
                    help=f"对准后的循迹脉宽（默认 {SPEED_US_DEFAULT:.0f}，必须 > 死区 {settings.ESC_DEADBAND_US}）")
    ap.add_argument("--align-tol", type=float, default=ALIGN_TOL_PX, help="对齐容差 px（默认 15）")
    ap.add_argument("--align-frames", type=int, default=ALIGN_FRAMES, help="容差内连续帧数（默认 8）")
    ap.add_argument("--align-max-s", type=float, default=6.0, help="对齐阶段最长秒数（超时即提速）")
    ap.add_argument("--acquire-s", type=float, default=ACQUIRE_S,
                    help=f"发车后「低速探路找线」的窗口（默认 {ACQUIRE_S:.0f}s；窗口内没线也会低速往前找，"
                         f"超时就停）")
    ap.add_argument("--target-x", type=float, default=None, help="期望车道中心（默认 settings.TARGET_X=<320>）")
    ap.add_argument("--max-steer", type=float, default=None, help="转向限幅（默认 settings 的 30°）")
    ap.add_argument("--max-seconds", type=float, default=180.0, help="整段最长运行时间")
    ap.add_argument("--no-lane", action="store_true", help="不循迹、不对齐：只测蓝板+电调")
    ap.add_argument("--force-run", action="store_true",
                    help="忽略'无有效车道'保护（台架观察转向用，比赛绝不能用）")
    ap.add_argument("--calibrate", type=int, default=0, metavar="N",
                    help="静止标定车道中心：车摆正在车道中央，采 N 帧求平均并输出 --target-x（不动电机）")
    ap.add_argument("--steer-test", action="store_true",
                    help="转向方向自检：把舵机依次打到 中位/右/左/中位（每档 1.5s）—— 你看着前轮，"
                         "确认\"角度大\"是不是右转；不对就往 config/site.yaml 写 steer_sign: -1")
    ap.add_argument("--no-auto-target", action="store_true",
                    help="关掉「边跑边标定」：默认会在起步对齐阶段用车前静止画面自动修正车道中心并写入 site.yaml")
    ap.add_argument("--show", action="store_true", help="显示预览窗口（需要 X11）")
    ap.add_argument("--no-motor", action="store_true", help="只跑视觉与决策，不输出动力")
    ap.add_argument("--allow-motion", "--i-know-wheels-are-up", dest="allow_motion", action="store_true",
                    help="确认车辆可以移动（四轮架空，或场地空旷、有人在旁能立即断电）")
    ap.add_argument("--width", type=int, default=settings.IMG_W)
    ap.add_argument("--height", type=int, default=settings.IMG_H)
    ap.add_argument("--print-every", type=int, default=5)
    args = ap.parse_args()

    # 标定/只读模式不驱动电机，因此不需要 --allow-motion
    use_motor = (not args.no_motor) and args.calibrate == 0
    if use_motor and not args.allow_motion:
        print("[BENCH] 拒绝运行：这次会驱动电机。确认四轮架空或场地空旷、有人在旁能立即断电，"
              "再加 --allow-motion；只想看读数就加 --no-motor")
        return 2

    # 死区守卫：低于死亡脉宽 = 白跑一次台架（2026-09-16 实测 1540us 轮子不转）
    if use_motor and min(args.start_us, args.speed_us) < settings.ESC_DEADBAND_US:
        print(f"[BENCH] 拒绝运行：脉宽低于电调死区。实测 1540us 不动、≈{settings.ESC_DEADBAND_US}us 才起转，"
              f"现在 起步={args.start_us:.0f} / 循迹={args.speed_us:.0f} → 车不会动。")
        print(f"[BENCH] 请用 ≥ {settings.ESC_CREEP_US}us（建议 起步 {settings.ESC_CREEP_US} / 循迹 1575）")
        return 2
    if use_motor and args.speed_us > settings.ESC_DEBUG_MAX_US:
        print(f"[BENCH] ⚠️ 循迹脉宽 {args.speed_us:.0f}us 超过调试上限 {settings.ESC_DEBUG_MAX_US}us"
              f"（未验证过的速度），5 秒内可 Ctrl-C 中止")
        time.sleep(5.0)

    scanner = LaneScanner(target_x=args.target_x)
    gate = StartGate()
    arbiter = LaneArbiter(target_x=args.target_x)
    planner = Planner(target_x=args.target_x, max_steer=args.max_steer)
    steer_limit = float(args.max_steer or settings.LANE_STEER_LIMIT_DEG)
    steering = float(settings.SERVO_CENTER_ANGLE)
    seen_board = False
    align_samples = []      # 对齐阶段（车基本静止）的车道中心采样 → 边跑边标定
    align_errors = []       # 对齐阶段误差走势 → 给转向方向下结论
    auto_target_done = False
    verdict_printed = False
    diverge_since = None    # 跑偏保护：大误差持续多久了

    # ---------------------------------------------------------- 静止标定
    if args.calibrate > 0:
        samples = []
        with camera_exclusive():
            cap = open_camera(args.camera, args.width, args.height)
            if cap is None:
                return 1
            tries = 0
            while len(samples) < args.calibrate and tries < args.calibrate * 8:
                tries += 1
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.03)
                    continue
                obs = scanner.scan(frame)
                if obs.confidence >= 0.4:
                    samples.append(obs.center_x)
            cap.release()
        if samples:
            avg = statistics.mean(samples)
            print(f"[BENCH] 标定完成：{len(samples)}/{tries} 帧有效，平均车道中心 = {avg:.1f} "
                  f"(波动 {max(samples) - min(samples):.0f}px)")
            # 直接写进车端本地配置（config/site.yaml），以后所有程序自动读取 —— 跑车只需一条命令
            try:
                from config import site
                path = site.save({"target_x": round(float(avg), 1)})
                print(f"[BENCH] ✅ 已写入 {path}：target_x = {avg:.1f}")
                print("[BENCH] 以后直接跑就行（不用再传 --target-x）：")
                print("[BENCH]    sudo python3 /root/dev/scripts/bench_test.py --allow-motion")
                print("[BENCH]    主程序 car-mode.sh auto --real 也同样自动读这个值")
            except Exception as exc:
                print(f"[BENCH] ⚠️ 写 site.yaml 失败（请手动用 --target-x {avg:.0f}）：{exc}")
        else:
            print("[BENCH] 没取到有效读数：确认车已摆正在车道中央、白线在视野内（可加 --show 看画面）")
        return 0

    # ---------------------------------------------------------- 主循环
    state = {"stopped": False, "driver": None, "restored": False, "used_motor": use_motor}
    install_signal_guard(state)

    print(f"[BENCH] 摄像头={args.camera}(蓝板+循迹同一路)  起步={args.start_us:.0f}us → 循迹={args.speed_us:.0f}us  "
          f"对齐容差={args.align_tol:.0f}px  电机={'开' if use_motor else '关(只跑视觉/决策)'}"
          f"{'  不循迹(--no-lane)' if args.no_lane else ''}")
    print("[BENCH] 规则：没见板→中位；见板→中位；板移开→低速对齐→循迹；再见板→立即停")
    if not args.no_lane:
        from config import site as _site
        applied = _site.applied()
        if "target_x" in applied:
            print(f"[BENCH] 车道中心用 site.yaml 的值：target_x = {applied['target_x']}"
                  f"（起步瞬间还会用静止画面自动修正一次）")
        else:
            print("[BENCH] 提示：还没有车道中心标定值（先用默认 320）。**不用单独标定**——"
                  "把车摆正在车道中央直接跑，程序会在起步瞬间自动标定并写入 site.yaml")

    rc = 0
    try:
        with MotorSession(state, enabled=use_motor, speed_us_max=max(args.speed_us, args.start_us)) as pca:
            # ---- 转向方向自检（电调保持中位，车不会动，只有前轮会转）----
            if args.steer_test:
                if pca is None:
                    print("[STEER-TEST] 需要电机通道（别加 --no-motor）")
                    return 2
                print("[STEER-TEST] 转向方向自检开始：电调保持中位 1500us，车不会走，只看前轮")
                steps = ((90, "中位（车轮应朝正前）"),
                         (90 + 20, "'角度大 20°' —— 若前轮此时【向右】，说明 STEER_SIGN=+1 是对的"),
                         (90 - 20, "'角度小 20°' —— 若前轮此时【向左】，进一步确认"),
                         (90, "回中位"))
                for ang, note in steps:
                    print(f"[STEER-TEST]   舵机 = {ang}°   {note}")
                    pca.set_steering_angle(ang)
                    time.sleep(1.5)
                print("[STEER-TEST] 结束。结论：")
                print("[STEER-TEST]   · 角度大=右转  → 不用改，默认就是对的")
                print("[STEER-TEST]   · 角度大=左转  → 在 /root/dev/config/site.yaml 里写一行 steer_sign: -1")
                return 0
            with camera_exclusive():
                cap = open_camera(args.camera, args.width, args.height)
                if cap is None:
                    rc = 1
                else:
                    t0 = last_t = time.time()
                    frames = 0
                    align_since = None      # 进入对齐阶段的时刻
                    align_hits = 0          # 容差内连续帧数
                    aligned = False
                    acquire_since = None    # 低速探路窗口起点（发车瞬间）
                    hinted_no_lane = False
                    try:
                        while not state["stopped"] and (time.time() - t0) < args.max_seconds:
                            ok, frame = cap.read()
                            if not ok or frame is None:
                                time.sleep(0.03)
                                continue
                            now = time.time()
                            dt = max(1e-3, now - last_t)
                            last_t = now

                            gs = gate.update(frame)
                            if gs.blocked:
                                seen_board = True
                            obs = scanner.scan(frame) if not args.no_lane else None
                            if obs is not None:
                                arb = arbiter.update(obs.center_x, obs.confidence, now)
                                lane_ok = not arb.should_stop and obs.confidence >= settings.ARBITER_CONF_THRESH
                                error = abs(obs.center_x - planner.target_x)
                            else:
                                arb = None
                                lane_ok = True          # --no-lane 模式不判车道
                                error = 0.0

                            # ---- 起步对齐状态 ----
                            if gs.blocked or not seen_board:
                                align_since, align_hits, aligned = None, 0, False
                                acquire_since = None
                                hinted_no_lane = False
                                align_samples.clear()
                                align_errors.clear()
                            elif not args.no_lane:
                                if align_since is None:
                                    align_since = acquire_since = now
                                # 「边跑边标定」：板刚移开、车还没动，用这段画面修正车道中心
                                if (obs is not None and not auto_target_done
                                        and obs.confidence >= AUTO_TARGET_MIN_CONF
                                        and not args.no_auto_target):
                                    align_samples.append(obs.center_x)
                                    value = auto_target(align_samples)
                                    if value is not None:
                                        auto_target_done = True
                                        delta = value - planner.target_x
                                        scanner.target_x = arbiter.target_x = planner.target_x = value
                                        print(f"[BENCH] 🎯 边跑边标定：车道中心 = {value:.1f}px"
                                              f"（{len(align_samples)} 帧静止画面，波动 "
                                              f"{max(align_samples) - min(align_samples):.0f}px，"
                                              f"原值 {value - delta:.1f} → 修正 {delta:+.1f}px）")
                                        try:
                                            from config import site
                                            site.save({"target_x": round(value, 1)})
                                            print("[BENCH]    已写入 config/site.yaml，本次与以后都用它")
                                        except Exception as exc:
                                            print(f"[BENCH]    ⚠️ 写入 site.yaml 失败（本次仍用它）：{exc}")
                                if not aligned and lane_ok:
                                    align_errors.append(error)
                                if lane_ok and error <= args.align_tol:
                                    align_hits += 1
                                    if align_hits >= args.align_frames:
                                        aligned = True
                                else:
                                    align_hits = 0
                                if not aligned and lane_ok and (now - align_since) >= args.align_max_s:
                                    aligned = True      # 对齐超时 → 先走起来（避免原地磨蹭）
                                if aligned and not verdict_printed:
                                    verdict_printed = True
                                    print(f"[BENCH] {align_verdict(align_errors)}")
                                # 探路窗口用完还没看到车道 → 明确说清楚，不再盲目往前拱
                                if lane_ok:
                                    acquire_since = None      # 找到线了，探路窗口结束
                                    hinted_no_lane = False
                                elif (not hinted_no_lane and acquire_since is not None
                                      and (now - acquire_since) > args.acquire_s):
                                    hinted_no_lane = True
                                    print(f"[BENCH] ⚠️ 发车后 {args.acquire_s:.0f}s 探路（低速往前找线）"
                                          "仍没有有效车道 → 已停车。这不是「该不该走」的问题，"
                                          "是**扫线看不到线**：")
                                    print("[BENCH]    1) 白线是否在画面里（用 --no-motor 看读数，"
                                          "或跑 scripts/diag/lane_probe.py 看掩膜图）；"
                                          "2) 光照/反光是否把白线淹了；3) LANE_ROI_TOP_RATIO 是否太低（只看了近处地面）")

                            d = decide(gs.blocked, seen_board, lane_ok, aligned,
                                       args.force_run, args.speed_us, args.start_us,
                                       no_lane=args.no_lane,
                                       acquire=(not lane_ok and acquire_since is not None
                                                and (now - acquire_since) <= args.acquire_s))

                            # ---- 跑偏保护：在有动力的状态下持续大误差 → 停车并给诊断 ----
                            if d.out_us > NEUTRAL_US and error > DIVERGENCE_PX:
                                if diverge_since is None:
                                    diverge_since = now
                            else:
                                diverge_since = None
                            if driving_off_course(error, d.out_us, diverge_since, now):
                                print(f"[BENCH] ⛔ 连续 {DIVERGENCE_S:.0f}s 误差 > {DIVERGENCE_PX:.0f}px"
                                      f"（当前 {error:.0f}px）：疑似转向方向反了或车没跟上，已停车")
                                print("[BENCH]    先跑 --steer-test 看前轮方向；若『角度大=左转』，"
                                      "就往 config/site.yaml 写 steer_sign: -1")
                                state["stopped"] = True
                                d = Decision(NEUTRAL_US, "stopped", "跑偏保护")

                            # ---- 转向：停车→回中（清 PID 防踢腿）；否则真 PID + 限速 ----
                            if d.out_us <= NEUTRAL_US:
                                planner.reset()
                                target_steer = float(settings.SERVO_CENTER_ANGLE)
                            else:
                                center = (obs.center_x if obs is not None else planner.target_x)
                                target_steer = float(settings.SERVO_CENTER_ANGLE) + planner.steering_offset(center, dt)
                                target_steer = max(settings.SERVO_CENTER_ANGLE - steer_limit,
                                                   min(settings.SERVO_CENTER_ANGLE + steer_limit, target_steer))
                            max_delta = settings.STEERING_SLEW_DEG_PER_S * dt
                            steering += max(-max_delta, min(max_delta, target_steer - steering))

                            if use_motor and pca is not None:
                                pca.set_steering_angle(steering)
                                pca.write_us(pca.CH_ESC, d.out_us)

                            frames += 1
                            if frames % max(1, args.print_every) == 0:
                                extra = ""
                                if obs is not None:
                                    extra = f"车道中心={obs.center_x:5.1f} 置信={obs.confidence:.2f} " \
                                            f"误差={error:4.1f} "
                                print(f"[BENCH] {now - t0:6.1f}s  {extra}"
                                      f"有板={int(gs.blocked)} 武装={int(seen_board)} 对准={int(aligned)}  "
                                      f"舵机={steering:5.1f}°  电调={d.out_us:.0f}us  ({d.reason})")

                            if args.show:
                                disp = frame.copy()
                                if obs is not None:
                                    roi_y0 = int(frame.shape[0] * settings.LANE_ROI_TOP_RATIO)
                                    roi_y1 = frame.shape[0] - settings.LANE_ROI_BOTTOM_MARGIN
                                    cv2.line(disp, (int(obs.center_x), roi_y0),
                                             (int(obs.center_x), roi_y1), (0, 255, 0), 2)
                                cv2.line(disp, (int(planner.target_x), 0), (int(planner.target_x), frame.shape[0]),
                                         (255, 160, 0), 1)
                                color = (0, 0, 255) if gs.blocked else (0, 255, 0)
                                cv2.putText(disp, f"{d.phase} board={int(gs.blocked)} area={gs.metrics.area_ratio:.3f} "
                                                  f"aligned={int(aligned)} steer={steering:.0f} esc={d.out_us:.0f}",
                                            (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                                cv2.imshow("bench", disp)
                                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                                    break
                    finally:
                        cap.release()
    except Exception as exc:
        print(f"[BENCH] ⚠️ 异常：{exc}")
        rc = 1
    finally:
        if args.show:
            cv2.destroyAllWindows()
    return rc


if __name__ == "__main__":
    sys.exit(main())
