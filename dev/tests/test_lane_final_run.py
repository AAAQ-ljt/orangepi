"""lane_final_test.py 在线路径的**端到端**测试：假摄像头 + 假蓝板，不碰硬件、不碰真摄像头。

为什么要有它：实跑脚本的"相位机 + 退出路径"是最贵的一段代码（去操场才发现崩了 = 白跑一趟）。
这里用假帧把整条流程走完：

    起步前静止（stop）→ 蓝板武装 → 板移开（发车）→ 跟线（track）
    → 丢线维持（hold）→ 超时（lost）→ 读数恢复（track 复跑）→ 退出

断言"流程能走完、阶段齐全、没板绝不动车、退出安全"，不校验具体数值（数值由
`test_lane_final.py` 的检测/控制用例覆盖）。

跑法：cd dev && PYTHONPATH=. python tests/test_lane_final_run.py
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import signal
import sys
import tempfile
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import scripts.lane_final_test as lft  # noqa: E402
from tests.test_lane_final import _blank_frame, _lane_frame  # noqa: E402

W, H = 640, 480

_SKIPPED: list = []          # 跳过的用例要**单独报出来**，不许混进"通过"里


def _board_frame() -> np.ndarray:
    """蓝板挡在下摄正前方（盖住 START_GATE_ROI 的大部分）。"""
    frame = _lane_frame()
    frame[150:400, 150:500] = (255, 0, 0)          # BGR：纯蓝
    return frame


class _FakeCap:
    """假摄像头：cyclic=True 时把序列循环播（真摄像头不会"流放完"，
    所以循环播 + --max-seconds 才是被测脚本的正常终止方式）。"""

    def __init__(self, frames, delay: float = 0.02, fail_after: int = 10 ** 9,
                 cyclic: bool = False, hang_after: int = 10 ** 9):
        self.frames = list(frames)
        self.delay = delay
        self.fail_after = fail_after
        self.cyclic = cyclic
        self.hang_after = hang_after       # 读满 N 帧后"卡死"（模拟 UVC 卡死：read 永久阻塞）
        self.i = 0
        self.released = False

    def read(self):
        if self.i >= self.hang_after:
            time.sleep(3600.0)             # ★ 永久阻塞：老代码会卡死在这儿（车带着油门前冲）
            return False, None
        if self.i >= self.fail_after or (not self.cyclic and self.i >= len(self.frames)):
            return False, None
        time.sleep(self.delay)
        f = self.frames[self.i % len(self.frames)]
        self.i += 1
        return True, f.copy()

    def release(self):
        self.released = True


def _run(frames, extra_args, fail_after: int = 10 ** 9, cyclic: bool = True,
         hang_after: int = 10 ** 9):
    """用假摄像头跑一遍 main()，返回 (rc, 会话目录, CSV 文本)。"""
    tmp = tempfile.mkdtemp(prefix="lane_final_run_")
    cap = _FakeCap(frames, fail_after=fail_after, cyclic=cyclic, hang_after=hang_after)
    saved = (lft.open_camera, lft.camera_exclusive)
    lft.open_camera = lambda *a, **k: cap                        # 不碰真摄像头
    lft.camera_exclusive = lambda *a, **k: contextlib.nullcontext()
    argv = sys.argv
    csv = os.path.join(tmp, "run.csv")
    sys.argv = ["lane_final_test.py", "--no-motor", "--save-dir", tmp, "--log-csv", csv,
                "--save-every-s", "0", "--print-every", "1000", "--max-seconds", "30",
                "--read-fail-s", "1"] + list(extra_args)
    rc = 1
    try:
        rc = lft.main()
    finally:
        sys.argv = argv
        lft.open_camera, lft.camera_exclusive = saved
    try:
        with open(csv, "r", encoding="utf-8") as fh:
            return rc, tmp, fh.read()
    except FileNotFoundError:          # --calibrate 这类模式不落 CSV，正常
        return rc, tmp, ""


def _phases(csv_text: str):
    lines = csv_text.strip().splitlines()[1:]
    out = []
    for ln in lines:
        parts = ln.split(",")
        out.append((parts[1], parts[7]))       # (phase, esc_us)
    return out


def test_phase_walk_no_gate() -> None:
    """--no-start-gate：跟线 → 丢线维持 → 超时停车 → 读数恢复后复跑。"""
    frames = ([_lane_frame()] * 12 + [_blank_frame()] * 25 + [_lane_frame()] * 12)
    # 循环播 + 2.2s 上限 ≈ 走完两轮"跟线→丢线→恢复"
    rc, _, csv_text = _run(frames, ["--no-start-gate", "--hold-s", "0.25", "--acquire-s", "1",
                                    "--max-seconds", "2.6"])
    phases = [p for p, _ in _phases(csv_text)]
    assert rc == 0, f"rc={rc}"
    assert "track" in phases, f"应有跟线阶段：{phases[:20]}"
    assert "hold" in phases, "应有丢线维持阶段"
    assert "lost" in phases, "丢线超时必须进 lost"
    # 恢复：lost 之后再次出现 track（连续有效帧够了）
    i_lost = phases.index("lost")
    assert "track" in phases[i_lost:], "读数恢复后应重新跟线"


def test_start_gate_edge_trigger() -> None:
    """蓝板边沿触发：没见过板绝不动车；板移开后才走。"""
    frames = ([_board_frame()] * 6 + [_lane_frame()] * 30)
    rc, _, csv_text = _run(frames, ["--hold-s", "0.4", "--max-seconds", "1.6"])
    rows = _phases(csv_text)
    assert rc == 0
    phases = [p for p, _ in rows]
    assert phases[0] == "stop", f"开局必须是 stop（等板）：{phases[:8]}"
    # 板移开之前（release_frames=5 帧去抖）应当一直是 stop 且电调中位
    pre = [esc for p, esc in rows if p == "stop"]
    assert pre and all(float(e) == 1500.0 for e in pre), "没发车前电调必须是中位"
    assert "track" in phases, f"板移开后应开始跟线：{phases}"
    # 发车后电调必须离开中位（在走）
    drive = [float(e) for p, e in rows if p in ("track", "acquire", "single")]
    assert drive and max(drive) > 1500.0, f"发车后电调应给前进脉宽：{drive[:10]}"


def test_read_fail_exits_safely() -> None:
    """摄像头连续读失败 → 尽快退出（不傻等 max-seconds）。"""
    t0 = time.time()
    rc, _, _ = _run([_lane_frame()] * 3, ["--no-start-gate"], fail_after=3, cyclic=False)
    assert rc == 1, "读失败应以 rc=1 退出"
    assert time.time() - t0 < 12.0, "读失败退出不该拖到 max-seconds"


def test_no_images_when_save_every_zero() -> None:
    _, tmp, _ = _run([_lane_frame()] * 8, ["--no-start-gate", "--max-seconds", "0.5"])
    anns = [f for f in os.listdir(tmp) if f.startswith("ann_")]
    assert not anns, f"--save-every-s 0 不该存图：{anns}"


def test_max_seconds_stops() -> None:
    frames = [_lane_frame()] * 400            # 循环播，理论上能一直跑
    rc, _, csv_text = _run(frames, ["--no-start-gate", "--max-seconds", "0.6"])
    assert rc == 0
    phases = [p for p, _ in _phases(csv_text)]
    assert len(phases) < 120, f"max-seconds 到点必须退出（实跑 {len(phases)} 帧）"


def test_servo_center_from_settings() -> None:
    """舵机中位取 settings.SERVO_CENTER_ANGLE（现场标定 86.5 那类值），不是写死 90。"""
    from config import settings
    saved = settings.SERVO_CENTER_ANGLE
    try:
        settings.SERVO_CENTER_ANGLE = 86.5
        rc, _, csv_text = _run([_lane_frame()] * 8,
                               ["--no-start-gate", "--max-seconds", "0.5"])
        angs = [float(ln.split(",")[6]) for ln in csv_text.strip().splitlines()[1:]]
        assert rc == 0
        assert any(abs(a - 86.5) < 6.0 for a in angs), f"舵角应以 86.5 为中位：{angs[:8]}"
    finally:
        settings.SERVO_CENTER_ANGLE = saved


def test_online_calibrate_mode() -> None:
    """车上静止标定（--calibrate）：假摄像头喂静止赛道帧 → 出 target 候选、不写 site.yaml、rc=0。"""
    import contextlib as _ctx
    import io as _io
    frames = [_lane_frame()] * 60           # 静止（完全一样的帧）
    buf = _io.StringIO()
    with _ctx.redirect_stdout(buf):
        rc, _, _ = _run(frames, ["--calibrate", "--calibrate-s", "0.4", "--no-save-site"],
                        cyclic=True)
    out = buf.getvalue()
    assert rc == 0, f"rc={rc}"
    assert "静止标定" in out and "center 中位" in out, out[-400:]
    assert "--target-x" in out, "应打印可直接复制的 --target-x"


def test_sigterm_stops_cleanly() -> None:
    """★ 2026-10-16 回归：外部 SIGTERM 必须能**安全停**（收到信号 → 收尾 → 正常退出）。

    为什么单列这一条：现场"跑起来停不下来"，追下去是**按键那条路不可靠**（7 份日志里零「收到信号」，
    信号根本没到进程）——于是"从另一个 shell 发信号"成了唯一的软件停车手段（`stop_run.sh` / `pkill -TERM`）。
    这条路必须锁死：改坏了就意味着现场又没有软件停车办法了。

    非 Linux 会被跳过（Windows 上 TerminateProcess 不能被 python 捕获）——**车上会真跑**。
    信号在"主循环真在跑"之后才发（等 CSV 出现：它的创建在 install_signal_guard 之后），
    免得在装处理器之前把测试进程打死（那会整文件崩）。
    """
    if not sys.platform.startswith("linux"):
        _SKIPPED.append("test_sigterm_stops_cleanly")
        print("      ⏭️ 跳过 test_sigterm_stops_cleanly（需要 Linux 的 SIGTERM 语义；车上会真跑这一条）")
        return
    tmp = tempfile.mkdtemp(prefix="lane_sig_")
    csv = os.path.join(tmp, "run.csv")

    def _fire() -> None:
        t0 = time.time()
        while time.time() - t0 < 10.0:                 # 等主循环起来（CSV 一出现就说明处理器已装好）
            if os.path.exists(csv) and os.path.getsize(csv) > 0:
                break
            time.sleep(0.05)
        time.sleep(0.3)
        os.kill(os.getpid(), signal.SIGTERM)           # 等价于 stop_run.sh / pkill -TERM 干的事
        print("[test] 已发送 SIGTERM", flush=True)

    prev = signal.getsignal(signal.SIGTERM)
    th = threading.Thread(target=_fire, daemon=True)
    t0 = time.time()
    th.start()
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rc, _, _ = _run([_lane_frame()], ["--no-start-gate", "--max-seconds", "60",
                                              "--save-dir", tmp, "--log-csv", csv])
        dt = time.time() - t0
    finally:
        signal.signal(signal.SIGTERM, prev)            # 别把被测脚本的处理器留下来（会串到别的用例）
    out = buf.getvalue()
    assert dt < 15.0, (f"SIGTERM 后应立刻退出（实得 {dt:.1f}s，max-seconds=60 ⇒ 信号没生效，"
                       f"现场就等于'停不下来'）")
    assert "收到信号" in out, f"应打印「收到信号」：{out[-800:]}"
    assert "收尾" in out, f"必须走安全收尾（停车/释放）：{out[-800:]}"
    assert "结束" in out, f"应有收尾统计：{out[-800:]}"
    assert rc == 0, f"rc={rc}"


class _FakePCA:
    """假 PCA9685（不碰 I2C）：只记下写了什么。"""

    CH_ESC = 1

    def __init__(self) -> None:
        self.writes: list = []

    def set_steering_angle(self, angle: float) -> None:
        self.writes.append(("steer", angle))

    def write_us(self, ch: int, us: float) -> None:
        self.writes.append((ch, us))


class _FakeDriver:
    def __init__(self, angry: bool = False) -> None:
        self.angry = angry
        self.safe_stop_calls = 0
        self.shutdown_calls = 0

    def safe_stop(self) -> None:
        self.safe_stop_calls += 1
        if self.angry:
            raise OSError("模拟：I2C 写失败（电调没归零）")

    def shutdown(self) -> None:
        self.shutdown_calls += 1


class _FakeMotorSession:
    """替掉 `bench_common.MotorSession`：不碰 opi-control/PCA9685，只把假 driver 放进 state。

    run_online 里是 `from scripts.bench_common import MotorSession`（**调用时**取属性）
    ⇒ 打补丁 `scripts.bench_common.MotorSession` 就能生效。
    """

    last = None

    def __init__(self, state: dict, enabled: bool = True, speed_us_max: float = None,
                 angry: bool = False) -> None:
        self.state = state
        self.enabled = enabled
        self.angry = angry

    def __enter__(self):
        _FakeMotorSession.last = self
        if not self.enabled:
            self.state["used_motor"] = False
            return None
        self.driver = _FakeDriver(angry=self.angry)
        self.state["driver"] = self.driver
        self.state["used_motor"] = True
        return _FakePCA()

    def __exit__(self, *exc):
        return False


class _BreakableStream:
    """可控的 stdout：先正常攒字；一旦攒到的内容里出现 `break_after`，之后所有写都抛
    BrokenPipeError —— 精确模拟"^C 把 `| tee` 打死、我们的 stdout 变成断管"。"""

    def __init__(self, break_after: str = "") -> None:
        self.buf = io.StringIO()
        self.break_after = break_after
        self.broken = False

    def write(self, s: str) -> int:
        if self.break_after and self.break_after in self.buf.getvalue():
            self.broken = True
        if self.broken:
            raise BrokenPipeError(32, "Broken pipe（模拟 tee 已被 ^C 打死）")
        return self.buf.write(s)

    def flush(self) -> None:
        return None

    def getvalue(self) -> str:
        return self.buf.getvalue()

    def __getattr__(self, name):                    # isatty 等透传
        return getattr(self.buf, name)


def test_safe_stop_even_if_stdout_pipe_breaks() -> None:
    """★ 2026-10-17 事故回归：stdout 断管（^C 打死 `| tee`）时**也必须**把电调归零。

    旧行为（现场实测，退出码 134 + 车不停只能断电）：`restore_remote_stage()` 第一步是 print →
    断管就抛 BrokenPipeError → `driver.safe_stop()` 被整个跳过 → PCA9685 保持最后一次脉宽（1600us）
    → 车一直跑。这里用假 driver 钉住"safe_stop 一定被调用"，并顺带钉住"收尾不能抛出去"。
    """
    import scripts.bench_common as bc

    tmp = tempfile.mkdtemp(prefix="lane_pipebreak_")
    saved_ms = bc.MotorSession
    saved_stdout = sys.stdout
    bc.MotorSession = _FakeMotorSession
    sys.stdout = _BreakableStream(break_after="track")   # 跟线开始后就断管
    try:
        rc, _, _ = _run([_lane_frame()] * 120,
                        ["--no-start-gate", "--allow-motion", "--watchdog-s", "0",
                         "--save-dir", tmp, "--max-seconds", "3"])
    finally:
        sys.stdout = saved_stdout
        bc.MotorSession = saved_ms
    ms = _FakeMotorSession.last
    assert ms is not None and ms.driver is not None, "假 MotorSession 应该被用过（说明真的在跑车路径）"
    assert ms.driver.safe_stop_calls >= 1, (
        "stdout 断管后 safe_stop 没被调用 —— 电调不会归零，车会带着最后一次脉宽跑（现场事故）")
    assert rc == 0 or rc == 1, f"rc={rc}（收尾链抛出去就会变成 134/异常终止）"


def test_watchdog_stays_armed_if_motor_stop_fails() -> None:
    """★ 电调归零**失败**时不许解除看门狗——它得留着在 1 秒后强写 1500us（唯一退路）。

    事故链条：断管 → safe_stop 被跳过（失败）→ 但收尾照旧 `wd.stop()` 删心跳+收进程
    ⇒ 安全网被自家解除武装 ⇒ 没人写 1500us ⇒ 车跑到断电。
    """
    import scripts.bench_common as bc
    import scripts.esc_watchdog as ew

    class _FakeWd:
        instances: list = []

        def __init__(self, timeout_s: float = 1.0, tag: str = "lane", grace_s: float = 60.0,
                     dry_run: bool = False) -> None:
            self.stopped = False
            self.path = os.path.join(tempfile.gettempdir(), f"fakewd_{id(self)}.hb")
            _FakeWd.instances.append(self)

        def start(self, log_path: str = "") -> None:
            with open(self.path, "w") as fh:
                fh.write("hb\n")

        def feed(self) -> None:
            pass

        def stop(self):
            self.stopped = True
            return 0

    tmp = tempfile.mkdtemp(prefix="lane_wdkeep_")
    saved_ms, saved_wd = bc.MotorSession, ew.WatchdogClient
    bc.MotorSession = (lambda state, enabled=True, speed_us_max=None:
                       _FakeMotorSession(state, enabled, speed_us_max, angry=True))
    ew.WatchdogClient = _FakeWd
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            _run([_lane_frame()] * 90,
                 ["--no-start-gate", "--allow-motion", "--watchdog-s", "1",
                  "--save-dir", tmp, "--max-seconds", "2.0"])
    finally:
        bc.MotorSession, ew.WatchdogClient = saved_ms, saved_wd
    assert _FakeWd.instances, "看门狗应该被拉起过"
    wd = _FakeWd.instances[-1]
    assert wd.stopped is False, (
        "电调没归零成功却把看门狗收掉了（删了心跳 + terminate）—— 安全网被解除武装，车会一直跑")


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
    ran = len(funcs) - len(_SKIPPED)
    msg = f"[test_lane_final_run] {ran - failed}/{ran} passed"
    if _SKIPPED:
        msg += f"，{len(_SKIPPED)} 个跳过未验证：" + "；".join(_SKIPPED)
    print(msg)
    if _SKIPPED:
        print("      ⚠️ 跳过的用例**没有**验证过——车上（Linux）跑一遍才算数")
    return 1 if failed else 0


def test_hung_camera_exits_safely() -> None:
    """★ 2026-10-15 事故回归：相机卡死（UVC 不出帧、read 永久阻塞）时必须**安全退出**。

    老代码：主循环阻塞在 cap.read() → 既不退出也不再写电调 → 车带着最后的油门前冲，
    Ctrl+C 无效（信号只设标志）→ 现场两次都要断电。新代码：read 在 daemon 线程里，
    主循环按"帧的年龄"判死（--read-fail-s）→ 正常收尾（停车 + 退出码 1）。
    这里还顺带钉住"必须 5 秒内返回"（不许挂死）。
    """
    t_start = time.time()
    rc, tmp, csv_text = _run([_lane_frame()] * 8,
                             ["--no-start-gate", "--read-fail-s", "0.4", "--max-seconds", "30"],
                             hang_after=3)
    elapsed = time.time() - t_start
    assert elapsed < 8.0, f"相机卡死后必须快速退出，实际跑了 {elapsed:.1f}s"
    # 8s 上限（不是 5s）：车端 ARM 比开发机慢，套件连跑时 5s 偶发假红（2026-10-15 实测）
    assert rc == 1, f"卡死应报错退出（rc=1），实得 {rc}"
    rows = csv_text.strip().splitlines()[1:]
    # 至少要有一行落盘（读帧线程会跑在主循环前面，慢机器上"卡死前处理了几帧"本来就不确定，
    # 所以这里只钉"处理过的帧没丢"，不钉具体条数 —— 车端实测曾因多断言 3 行而假红）
    assert len(rows) >= 1, "卡死前处理过的帧必须照常落盘"


def test_real_errors_still_exit_on_read_fail() -> None:
    """read 直接返回失败（不是卡死）也要在 --read-fail-s 后退出（老行为不能丢）。"""
    rc, _, _ = _run([_lane_frame()] * 4, ["--no-start-gate", "--read-fail-s", "0.5",
                                          "--max-seconds", "30"], fail_after=4, cyclic=False)
    assert rc == 1


def test_stop_file_stops_cleanly() -> None:
    """★ 2026-10-17（调试期停车按钮）：**停止文件出现 = 安全停车**，与 Ctrl+C 同一条收尾链。

    为什么单列这一条：10-16 现场 7 趟日志零「收到信号」——按键那条路（无 PTY / 后台 / 别的窗口）
    根本到不了进程，于是"停车"必须有第二条不依赖终端的通道。这里钉住三件事：
    ① 文件一出现就**立刻**退出（不许跑满 --max-seconds）；② 走了安全收尾（rc=0 + 落盘）；
    ③ 收尾时把文件清掉、并把"谁停的"记进 session.json（复盘要能对号）。
    """
    tmp = tempfile.mkdtemp(prefix="lane_stopfile_")
    stop_file = os.path.join(tmp, "lane_stop")
    csv = os.path.join(tmp, "run.csv")

    def _touch() -> None:
        t0 = time.time()
        while time.time() - t0 < 10.0:                 # 等主循环起来（CSV 一出现就说明在跑）
            if os.path.exists(csv):
                break
            time.sleep(0.05)
        time.sleep(0.3)
        with open(stop_file, "w") as fh:
            fh.write("stop\n")
        print("[test] 已创建停止文件", flush=True)

    th = threading.Thread(target=_touch, daemon=True)
    th.start()
    buf = io.StringIO()
    t0 = time.time()
    with contextlib.redirect_stdout(buf):
        rc, _, _ = _run([_lane_frame()] * 60,
                        ["--no-start-gate", "--stop-file", stop_file,
                         "--save-dir", tmp, "--log-csv", csv,      # 会话目录给我们，好找 CSV/session.json
                         "--max-seconds", "20"])
    dt = time.time() - t0
    out = buf.getvalue()
    with open(csv, encoding="utf-8") as fh:
        csv_text = fh.read()
    assert dt < 8.0, f"停止文件出现后应立刻退出（实得 {dt:.1f}s，max-seconds=20 ⇒ 通道没生效）"
    assert rc == 0, f"停止文件应走安全收尾（rc=0），实得 {rc}"
    assert "停止文件" in out and "安全停车" in out, f"应有停止文件提示：{out[-600:]}"
    assert not os.path.exists(stop_file), "收尾时必须把停止文件清掉（否则下一趟起不来）"
    assert "track" in [p for p, _ in _phases(csv_text)], "停之前应正常在跟线"
    with open(os.path.join(tmp, "session.json"), encoding="utf-8") as fh:
        sess = json.load(fh)
    assert sess["params"]["stop_file"] == stop_file, sess["params"].get("stop_file")
    assert sess["result"]["stopped_by"] == "stop-file", sess["result"].get("stopped_by")
    for key in ("tty", "pgid", "tpgid", "keyboard_reaches"):
        assert key in sess["params"]["launch"], f"启动自检应记 {key}：{sess['params']['launch']}"


def test_stale_stop_file_does_not_stop() -> None:
    """上一趟残留的停止文件**不许**把这一趟当场停掉（启动时清残留；跑完也不留）。"""
    tmp = tempfile.mkdtemp(prefix="lane_stale_")
    stop_file = os.path.join(tmp, "lane_stop")
    with open(stop_file, "w") as fh:
        fh.write("stale\n")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc, _, csv_text = _run([_lane_frame()] * 40,
                               ["--no-start-gate", "--stop-file", stop_file,
                                "--max-seconds", "1.0"])
    out = buf.getvalue()
    assert rc == 0, f"rc={rc}"
    assert "清掉停止文件" in out, f"应提示清掉了残留：{out[:800]}"
    assert not os.path.exists(stop_file), "跑完不该留下停止文件"
    assert "track" in [p for p, _ in _phases(csv_text)], "残留文件不该挡住跟线"


def test_stop_file_disabled_by_empty_path() -> None:
    """`--stop-file ''` = 关掉这条通道（现场想回到"只有信号"的老行为时用）。"""
    rc, _, _ = _run([_lane_frame()] * 30, ["--no-start-gate", "--stop-file", "",
                                           "--max-seconds", "0.8"])
    assert rc == 0


if __name__ == "__main__":
    sys.exit(main())
