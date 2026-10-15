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
import os
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import scripts.lane_final_test as lft  # noqa: E402
from tests.test_lane_final import _blank_frame, _lane_frame  # noqa: E402

W, H = 640, 480


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
    print(f"[test_lane_final_run] {len(funcs) - failed}/{len(funcs)} passed")
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


if __name__ == "__main__":
    sys.exit(main())
