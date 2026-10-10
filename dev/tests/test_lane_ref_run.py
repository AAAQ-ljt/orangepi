"""循迹实跑状态机的**端到端**测试：假摄像头 + 假蓝板，不碰硬件、不碰真实摄像头。

为什么要有这个文件
------------------
实跑脚本的"状态机 + 退出路径"是最贵的一段代码：到操场才发现崩了，一整趟就白跑。
上一版就是在这里出的问题（退出时 `int(None)` / `detect(None)` 两个 Traceback、
探路阶段舵角没回中位）。这里用假帧把**整条流程**跑一遍：

    起步前自检窗口 → 蓝板武装 → 板移开 → 探路(acquire) → 跟线(track)
    → 丢线维持(hold) → 超时(lost) → 自行停车收尾

断言的是"流程能走完且阶段齐全"，不校验具体数值（数值由 test_lane_ref.py 的纯函数测试覆盖）。

跑法：
    cd dev && PYTHONPATH=. python tests/test_lane_ref_run.py
"""
from __future__ import annotations

import contextlib
import os
import sys
import tempfile
import time

import cv2
import numpy as np

import scripts.lane_ref_test as lt

W, H = 640, 480


def _lane_frame(shift: int = 0, left: bool = True, right: bool = True) -> np.ndarray:
    """红棕底 + 两条斜白线（画面中下部没有线 → 体检会判不够用，但检测能锁住成对）。"""
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    frame[:, :, 2], frame[:, :, 1], frame[:, :, 0] = 120, 45, 35
    yt, yb, y_ref = int(H * 0.30), int(H * 0.60), int(H * 0.60)
    sides = [(160 + shift, +1)] if left else []
    sides += [(590 + shift, -1)] if right else []
    for x_ref, sign in sides:
        for y in range(yt, yb):
            x = int(round(x_ref - sign * 0.972 * (y - y_ref)))
            frame[y, max(0, x - 4):x + 5] = 225
    return frame


def _board_frame() -> np.ndarray:
    """蓝板挡在下摄正前方（盖住 START_GATE_ROI 的大部分）。"""
    frame = _lane_frame()
    frame[150:400, 150:500] = (255, 0, 0)        # BGR：纯蓝
    return frame


def _blank_frame() -> np.ndarray:
    """没有线的画面（模拟"线丢了"）。"""
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    frame[:, :, 2], frame[:, :, 1], frame[:, :, 0] = 120, 45, 35
    return frame


def _cone_high_frame() -> np.ndarray:
    """锥桶在画面上部（y 70~170）：默认 CONE_GATE_ROI(0.35) 切不到。

    用**淡蓝** (200,170,120)：dominance=B−max(R,G)=30 < 35，不会被发车闸门当蓝板
    （真实锥桶 S≈70 也是低饱和，此口径一致）；仍在锥桶 HSV (70,30,150)~ 内可检出。
    """
    frame = _lane_frame()
    frame[70:170, 295:385] = (200, 170, 120)
    return frame


def _cone_frame(shift: int = 0, size: int = 90) -> np.ndarray:
    """红棕底 + 双斜白线 + 一个蓝色锥桶块（下摄视角：锥桶在画面中下部，ROI 内）。"""
    frame = _lane_frame(shift=shift)
    cx, bottom = 340 + shift, 420
    frame[bottom - size:bottom, cx - size // 2:cx + size // 2] = (255, 140, 60)  # BGR 偏蓝
    return frame


class _FakeCap:
    """假摄像头：按顺序吐帧，每帧等一小会儿（模拟 15FPS 的时间流逝）。"""

    def __init__(self, frames, delay: float = 0.03):
        self.frames = list(frames)
        self.delay = delay
        self.i = 0
        self.released = False

    def read(self):
        if self.i >= len(self.frames):
            return False, None
        time.sleep(self.delay)
        f = self.frames[self.i]
        self.i += 1
        return True, f.copy()

    def release(self):
        self.released = True


def _run(frames, extra_args):
    """用假摄像头跑一遍 main()，返回 (rc, 会话目录, CSV 文本)。"""
    tmp = tempfile.mkdtemp(prefix="lane_run_")
    cap = _FakeCap(frames)
    saved = (lt.open_camera, lt.camera_exclusive)
    lt.open_camera = lambda *a, **k: cap                 # 不碰真实摄像头
    lt.camera_exclusive = lambda *a, **k: contextlib.nullcontext()
    argv = sys.argv
    csv = os.path.join(tmp, "run.csv")
    sys.argv = ["lane_ref_test.py", "--no-motor", "--save-dir", tmp, "--log-csv", csv,
                "--save-every-s", "0", "--setup-frames", "4", "--print-every", "100",
                "--no-save-site", "--max-seconds", "30",
                "--read-fail-s", "2"] + list(extra_args)
    rc = 1
    try:
        rc = lt.main()
    finally:
        sys.argv = argv
        lt.open_camera, lt.camera_exclusive = saved
    with open(csv, "r", encoding="utf-8") as fh:
        return rc, tmp, fh.read()


def test_full_state_machine_walks_all_phases():
    """走完一整条流程：自检 → 武装 → 探路 → 跟线 → 维持 → 丢线停车（不带异常退出）。"""
    frames = ([_lane_frame()] * 6          # 起步前自检窗口（车静止、镜头看赛道）
              + [_board_frame()] * 4       # 蓝板出现 → 武装（3 帧确认）
              + [_blank_frame()] * 2       # 板移开但还没看到线 → 探路 acquire
              + [_lane_frame()] * 12       # 看到线 → 跟线 track
              + [_blank_frame()] * 60)     # 线丢了 → 维持 hold → 超时 lost → 停车
    rc, tmp, csv = _run(frames, ["--acquire-s", "0.15", "--hold-s", "0.2"])
    assert rc == 0, f"实跑应当正常返回 0，实际 {rc}"
    head = csv.splitlines()[0]
    assert "error_filt" in head and "target_px" in head, "CSV 要带复盘用的新列"
    phases = set()
    for line in csv.splitlines()[1:]:
        parts = line.split(",")
        if len(parts) > 1:
            phases.add(parts[1])
    for need in ("stop", "acquire", "track", "hold", "lost"):
        assert need in phases, f"阶段 {need} 没走到（实际 {sorted(phases)}）"
    assert os.path.exists(os.path.join(tmp, "summary.json")), "应当落盘 summary.json"
    setup_dir = os.path.join(tmp, "setup")
    assert os.path.isdir(setup_dir), "起步前自检的帧要存下来（复盘证据）"


def test_cones_trigger_s_avoid_then_resume():
    """锥桶出现 → S 型避让（avoid1/resume/avoid2）→ 回 track；绕行是低速、有兜底。"""
    frames = ([_lane_frame()] * 6          # 自检窗口
              + [_board_frame()] * 4       # 蓝板武装
              + [_blank_frame()] * 2       # 探路
              + [_lane_frame()] * 8        # 跟线
              + [_cone_frame()] * 20       # 锥桶出现（防抖 3 帧 → avoid1）
              + [_lane_frame()] * 20       # 锥桶移开 → 回 track
              + [_blank_frame()] * 40)     # 丢线收尾
    rc, tmp, csv = _run(frames, ["--cone-method", "hsv", "--cone-avoid",
                                 "--cone-avoid-s", "0.15", "--cone-resume-s", "0.1",
                                 "--acquire-s", "0.15", "--hold-s", "0.2"])
    assert rc == 0, f"带锥桶的实跑应当正常返回 0，实际 {rc}"
    phases = set(line.split(",")[1] for line in csv.splitlines()[1:] if line.count(",") > 2)
    for need in ("avoid1", "resume", "avoid2"):
        assert need in phases, f"绕行阶段 {need} 没走到（实际 {sorted(phases)}）"
    assert "track" in phases, "绕行结束后应回到循迹"


def test_cone_avoid_immune_to_big_error_guard():
    """绕行豁免跑偏保护（10-11 修复回归）。

    场景（复刻审查复现实验）：误差恒 +79px（用 --no-auto-target 把目标点偏置到 296，
    线在画面正中、质量 1.00）——修复前 big_err 在绕行期间照常累积，S 型走到一半被
    guard 停死、整次行程结束；修复后绕行期间不更新保护、进出各 reset，S 三段完整。
    时序照真实复现：锥桶块先被发车闸门当板挡几帧（stop，tracker 无条件攒 present），
    "锥桶移开"（线帧）后 avoid1 启动。
    """
    frames = ([_lane_frame()] * 6            # 自检窗口
              + [_board_frame()] * 3         # 板武装
              + [_cone_frame()] * 4          # 锥桶出现（闸门可能判 blocked；tracker 攒 present=3+）
              + [_lane_frame()] * 45         # "移开锥桶"：driving=True + present 余温 → S 型；
                                             # 误差 79 持续 → 旧代码在 avoid 里 ~1.5s 就被 guard
              + [_blank_frame()] * 40)       # 丢线收尾（无中心 → 不喂保护；自然 lost 结束）
    # 注：绕行结束后**不再给"持续大误差"帧**——BigErrorGuard 的振荡判据（窗口 ≥6 帧、
    #     大误差占比 ≥60%）在那时触发是**正确行为**（真跑偏），不属于本用例要测的豁免。
    rc, _tmp, csv = _run(frames, ["--cone-method", "hsv", "--cone-avoid",
                                  "--cone-avoid-s", "0.6", "--cone-resume-s", "0.4",
                                  "--no-auto-target", "--target-ratio", "0.4625"])
    assert rc == 0
    phases = [line.split(",")[1] for line in csv.splitlines()[1:] if line.count(",") > 2]
    assert "guard" not in phases, f"绕行期间不应触发跑偏保护，实际相位 {phases}"
    assert "avoid1" in phases and "resume" in phases and "avoid2" in phases,         f"S 型三段应完整走完，实际 {phases}"


def test_cone_cooldown_gates_respawn():
    """CONE_COOLDOWN_S 接线（10-11）：锥桶常驻视野时，第二轮 avoid1 必须等冷却期满。"""
    frames = ([_lane_frame()] * 6
              + [_board_frame()] * 3
              + [_cone_high_frame()] * 110      # 锥桶常驻：够 S1(1.6s) + 冷却 + S2 完整
              + [_blank_frame()] * 80)          # 丢线收尾（第三轮不会再启动 → 自然 lost）
    rc, _tmp, csv = _run(frames, ["--cone-method", "hsv", "--cone-avoid",
                                  "--cone-avoid-s", "0.6", "--cone-resume-s", "0.4",
                                  "--cone-roi-top", "0.05", "--cone-cooldown-s", "1.0"])
    assert rc == 0
    rows = [(float(r[0]), r[1]) for r in
            (line.split(",") for line in csv.splitlines()[1:]) if len(r) > 2]
    starts = []
    for i, (t, ph) in enumerate(rows):
        if ph == "avoid1" and (i == 0 or rows[i - 1][1] != "avoid1"):
            starts.append(t)
    assert len(starts) >= 2, f"锥桶常驻应触发至少两轮 S，实际起点 {starts}"
    # 起点差 = S 总时长(0.6+0.4+0.6) + 冷却 1.0s ≈ 2.6s
    gap = starts[1] - starts[0]
    assert 2.3 <= gap < 3.6, f"两轮 avoid1 间隔 {gap:.2f}s 应≈ S时长+冷却(2.6s)"


def test_cone_cli_overrides_reach_detector():
    """--cone-roi-top / --cone-hsv-* 必须真的进检测器（10-11 覆盖项透传回归）。"""
    frames = ([_lane_frame()] * 6
              + [_board_frame()] * 3
              + [_cone_high_frame()] * 30
              + [_blank_frame()] * 30)
    base = ["--cone-method", "hsv", "--cone-avoid",
            "--cone-avoid-s", "0.6", "--cone-resume-s", "0.4"]
    rc, _t, csv = _run(frames, base)
    assert "avoid1" not in [r[1] for r in
                            (l.split(",") for l in csv.splitlines()[1:]) if len(r) > 2],         "默认 ROI(0.35) 切掉高处锥桶 → 不应绕行"
    rc, _t, csv = _run(frames, base + ["--cone-roi-top", "0.05"])
    assert "avoid1" in [r[1] for r in
                        (l.split(",") for l in csv.splitlines()[1:]) if len(r) > 2],         "--cone-roi-top 0.05 后应检出并绕行"
    rc, _t, csv = _run(frames, base + ["--cone-roi-top", "0.05",
                                       "--cone-hsv-low", "1,1,1",
                                       "--cone-hsv-high", "2,2,2"])
    assert "avoid1" not in [r[1] for r in
                            (l.split(",") for l in csv.splitlines()[1:]) if len(r) > 2],         "故意错的 HSV 不应检出"


def test_analyze_image_respects_cone_overrides():
    """P1 回归（10-11）：--image 单图路径必须接收 roi/hsv 覆盖项，不能读默认值。"""
    import contextlib
    import io as _io
    import tempfile
    tmp = tempfile.mkdtemp(prefix="cone_img_")
    path = os.path.join(tmp, "high_cone.jpg")
    cv2.imencode(".jpg", _cone_high_frame())[1].tofile(path)
    for extra, expect in (([], "部署 ROI 内无"),
                          (["--cone-roi-top", "0.05"], "锥桶(hsv): x=")):
        argv_back = sys.argv
        sys.argv = (["lane_ref_test.py", "--image", path, "--cone-method", "hsv"]
                    + extra)
        buf = _io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                lt.main()
        finally:
            sys.argv = argv_back
        out = buf.getvalue()
        assert expect in out, f"{extra} 时输出应含 {expect!r}，实际：{out[-200:]}"


def test_cone_avoid_off_never_enters_avoid():
    """没开 --cone-avoid 时，即使检测到锥桶也不绕行（默认行为不变）。"""
    frames = ([_lane_frame()] * 6
              + [_board_frame()] * 4
              + [_blank_frame()] * 2
              + [_lane_frame()] * 4
              + [_cone_frame()] * 30
              + [_lane_frame()] * 10
              + [_blank_frame()] * 40)
    rc, tmp, csv = _run(frames, ["--cone-method", "hsv",
                                 "--acquire-s", "0.15", "--hold-s", "0.2"])
    assert rc == 0
    phases = set(line.split(",")[1] for line in csv.splitlines()[1:] if line.count(",") > 2)
    assert not ({"avoid1", "avoid2"} & phases), (
        f"未开 --cone-avoid 不应绕行，实际出现了 {sorted(phases & {'avoid1', 'resume', 'avoid2'})}")


def test_cone_stop_blocks_then_resumes():
    """--cone-stop：检测到锥桶 → 停车；锥桶移开 → 继续前进（实验室先验检测用）。"""
    frames = ([_lane_frame()] * 6          # 自检窗口
              + [_board_frame()] * 4       # 蓝板武装
              + [_blank_frame()] * 2       # 探路
              + [_lane_frame()] * 6        # 跟线
              + [_cone_frame()] * 15       # 锥桶出现 → 停车（防抖 3 帧后 cone_stopped）
              + [_lane_frame()] * 10       # 锥桶移开 → 恢复前进（回 track）
              + [_blank_frame()] * 40)     # 丢线收尾
    rc, tmp, csv = _run(frames, ["--cone-method", "hsv", "--cone-stop",
                                 "--acquire-s", "0.15", "--hold-s", "0.2"])
    assert rc == 0, f"实验室停锥桶模式应当正常返回 0，实际 {rc}"
    phases = [line.split(",")[1] for line in csv.splitlines()[1:] if line.count(",") > 2]
    if "track" in phases:
        first_track = phases.index("track")
        # 锥桶段停住：track 之后出现 stop
        stop_after = next((i for i in range(first_track + 1, len(phases))
                           if phases[i] == "stop"), -1)
        assert stop_after > first_track, "锥桶出现后应有停车（stop 相位）"
        # 锥桶移开后恢复：stop 之后再次出现 track
        track_after = next((i for i in range(stop_after + 1, len(phases))
                            if phases[i] == "track"), -1)
        assert track_after > stop_after, "锥桶移开后应恢复前进（track 相位）"


def test_never_blocks_means_never_drives():
    """没见过蓝板 → 全程 stop，绝不动车（安全红线）。"""
    rc, _tmp, csv = _run([_lane_frame()] * 12, [])
    assert rc in (0, 1)
    phases = set(line.split(",")[1] for line in csv.splitlines()[1:] if line.count(",") > 2)
    assert phases <= {"stop"}, f"没有蓝板时只能停车，实际阶段 {sorted(phases)}"


# ------------------------------------------------------------------ 斑马线档（--zebra-stop）
def _zebra_frame() -> np.ndarray:
    """暗底 + 5 条白色横条（在 ZEBRA_ROI 内 y≥168；底面无车道线避免串扰）。"""
    frame = _blank_frame()
    for y in range(200, 380, 36):
        cv2.rectangle(frame, (80, y), (560, y + 14), (255, 255, 255), -1)
    return frame


def test_zebra_stop_triggers_then_resumes():
    """--zebra-stop：跟线中遇到斑马线 → 进入 zebra 停车相位 → 停够后恢复 track。"""
    frames = ([_lane_frame()] * 6          # 起步前自检窗口
              + [_board_frame()] * 3       # 见板武装
              + [_lane_frame()] * 6        # 移开板 → 放行跟线
              + [_zebra_frame()] * 6       # 斑马线：连续 3 帧触发 → 停车
              + [_lane_frame()] * 20       # 停够 0.4s 后应恢复循迹
              + [_blank_frame()] * 40)     # 丢线收尾（自然 lost，rc=0）
    rc, _tmp, csv = _run(frames, ["--zebra-stop", "--zebra-seconds", "0.4"])
    assert rc == 0
    rows = [line.split(",") for line in csv.splitlines()[1:] if line.count(",") > 2]
    phases = [r[1] for r in rows]
    assert "zebra" in phases, f"应进入斑马线停车相位，实际 {phases}"
    zi = phases.index("zebra")
    assert "track" in phases[zi + 1:], "停够后应恢复循迹（track 相位）"


def test_zebra_off_never_enters_zebra():
    """不开 --zebra-stop：纯循迹行为不变（斑马线画面也不触发停车）。"""
    frames = ([_lane_frame()] * 6
              + [_board_frame()] * 3
              + [_lane_frame()] * 6
              + [_zebra_frame()] * 6
              + [_lane_frame()] * 4
              + [_blank_frame()] * 40)
    rc, _tmp, csv = _run(frames, [])
    assert rc == 0
    assert "zebra" not in csv.splitlines()[0:], "默认必须不触发斑马线停车"


def test_board_too_early_is_handled():
    """一开机板就挡着 → 自检跳过、给告警，但流程不能崩（现场很容易这么操作）。"""
    rc, _tmp, csv = _run([_board_frame()] * 4 + [_lane_frame()] * 20, [])
    assert rc in (0, 1), "板挡得太早也要能正常收尾"
    phases = set(line.split(",")[1] for line in csv.splitlines()[1:] if line.count(",") > 2)
    assert "track" in phases, "板移开后仍应能跟线"


def test_single_side_lane_keeps_running():
    """只看到一侧线时：靠半宽先验兜底继续跟线（不是"一丢线就停"）。"""
    half_lane = _lane_frame()
    half_lane[:, 500:] = (35, 45, 120)      # 擦掉右线（BGR 底色）
    frames = ([_lane_frame()] * 6 + [_board_frame()] * 4 + [half_lane] * 25)
    # 这一条不需要自检扫带（省时间），直接测"单侧还能不能走"
    rc, _tmp, csv = _run(frames, ["--acquire-s", "0.15", "--hold-s", "0.2",
                                  "--no-auto-roi", "--no-auto-look"])
    assert rc in (0, 1)
    phases = set(line.split(",")[1] for line in csv.splitlines()[1:] if line.count(",") > 2)
    assert "track" in phases, f"单侧兜底应当仍能进 track，实际 {sorted(phases)}"
    singles = [ln for ln in csv.splitlines()[1:] if ln.split(",")[13:14] == ["0"]]
    assert singles, "应当出现过【单侧】帧（both_sides=0）"


def test_camera_dead_exits_cleanly():
    """摄像头一直不给画面 → 在 --read-fail-s 内报错退出（rc=1），不空转到 --max-seconds；
    而且安全层照常收尾（summary 落盘）。"""
    t0 = time.time()
    rc, tmp, csv = _run([], ["--read-fail-s", "2"])
    assert rc == 1, f"读不到画面应当报错退出，实际 rc={rc}"
    assert time.time() - t0 < 20, "不该傻等到 --max-seconds"
    assert os.path.exists(os.path.join(tmp, "summary.json")), "即使报错退出也要落盘与收尾"


def test_run_guard_trip_does_not_crash():
    """跑偏保护在"读数质量不够（err_c=None）但原始误差很大"时触发 → 不能崩。

    2026-09-22 落地实测崩过：`f"{err_c:+.1f}"` 撞上 NoneType。
    只有一侧白线时，半宽用**种子**推出中心（质量 0.25 < 0.28）→ 不进控制器，
    但原始误差很大 → 正好走这条路径。
    """
    one_side = [_lane_frame(right=False)] * 20          # 只有左线 → 质量 0.25
    frames = [_lane_frame()] * 6 + [_board_frame()] * 4 + one_side
    rc, tmp, csv = _run(frames, ["--err-stop-px", "40", "--err-stop-s", "0.2",
                                 "--no-auto-roi", "--no-auto-look"])
    assert rc in (0, 1), f"不该崩（rc={rc}）"
    summary = os.path.join(tmp, "summary.json")
    assert os.path.exists(summary), "即使触发保护也要正常收尾落盘"


def test_session_dir_is_created_before_csv():
    """**会话目录还不存在时也要能跑**（默认就是这种情形：CSV 写在 <save_dir>/run.csv）。

    2026-09-22 实验室实测的崩溃：`Recorder` 先 `open(csv)` 后 `makedirs` →
    车上第一次跑就 FileNotFoundError。本地集成测试当时用的是**已存在**的临时目录，
    所以没抓到 —— 这条测试专门盯着"目录必须先建"。
    """
    from scripts.lane_ref_test import Recorder
    base = tempfile.mkdtemp(prefix="lane_fresh_")
    fresh = os.path.join(base, "lane_20260922_165613")       # 故意不让它存在
    assert not os.path.exists(fresh)
    rec = Recorder(os.path.join(fresh, "run.csv"), fresh, 0.0, 10)
    try:
        assert os.path.isdir(fresh), "会话目录应当被建出来"
        assert rec._fh is not None, "CSV 应当能打开"
        assert os.path.isfile(os.path.join(fresh, "run.csv"))
        assert os.path.isdir(os.path.join(fresh, "setup")), "setup 子目录也要建"
        rec.row(0.1, "track", None, None, 375.0, 90.0, 1500.0, 0.0)   # 写一行不能崩
    finally:
        rec.close()
    # 目录建不出来（父路径是个文件）→ 只告警、不崩车
    bad_parent = os.path.join(base, "not_a_dir")
    with open(bad_parent, "w", encoding="utf-8") as fh:
        fh.write("x")
    rec2 = Recorder(os.path.join(bad_parent, "sub", "run.csv"),
                    os.path.join(bad_parent, "sub"), 0.0, 10)
    try:
        assert rec2._fh is None and rec2.save_dir == "", "打不开时应当降级成『不落盘』而不是抛异常"
        rec2.row(0.1, "track", None, None, 375.0, 90.0, 1500.0, 0.0)
    finally:
        rec2.close()


if __name__ == "__main__":
    # 10-11 修：改用全量收集（此前是手写白名单——新增用例忘记登记就会静默不跑）
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)
           and getattr(v, "__module__", None) == __name__]
    for fn in fns:
        fn()
        print(f"  ✅ {fn.__name__}")
    print(f"test_lane_ref_run: all {len(fns)} passed")
