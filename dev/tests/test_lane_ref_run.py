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


def test_never_blocks_means_never_drives():
    """没见过蓝板 → 全程 stop，绝不动车（安全红线）。"""
    rc, _tmp, csv = _run([_lane_frame()] * 12, [])
    assert rc in (0, 1)
    phases = set(line.split(",")[1] for line in csv.splitlines()[1:] if line.count(",") > 2)
    assert phases <= {"stop"}, f"没有蓝板时只能停车，实际阶段 {sorted(phases)}"


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
    for name in ("test_full_state_machine_walks_all_phases",
                 "test_never_blocks_means_never_drives",
                 "test_board_too_early_is_handled",
                 "test_single_side_lane_keeps_running",
                 "test_camera_dead_exits_cleanly",
                 "test_run_guard_trip_does_not_crash",
                 "test_session_dir_is_created_before_csv"):
        globals()[name]()
        print(f"  ✅ {name}")
    print("test_lane_ref_run: all 7 passed")
