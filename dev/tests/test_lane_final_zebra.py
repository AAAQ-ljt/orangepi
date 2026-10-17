"""lane_final_test.py 的**斑马线相位（--zebra）**测试：合成图 + 假摄像头，不碰硬件。

为什么单独一个文件：`doc/开发文档.md` §2.8 给每个元素定死了一套流程（阈值进 settings、
带开关且默认关、单测 + 离线误检率、现场先架空）。这个文件是"斑马线相位"那一套的验收：
检测器在裁剪后画面上该检的检得出（见 `test_zebra_detect.py`），相位机按设计**刹在哪、停多久**。

覆盖：
- 相位机：武装期（发车后 N 秒才认）→ slow（慢行档之前照常巡航、到了才压蠕速）→
  停车行断油 1500 → **停够 hold_s 才回循迹**（期间检测丢了也照停）→ 冷却 + 重挂载闩锁；
  接近段超时强制断油；检测早早消失判误检中止（中止不设冷却）；`--extra-creep-s` 的"停在横道之后"变体；
  已经停过车的横道不会二进宫、换一道横道照样触发；护栏停车不被斑马线相位加油门；
  蠕速被死区下限兜住；`--zebra` 关 = 给 ZebraState 也一律不理；
- 端到端（假摄像头）：一趟接近 → CSV 里 zebra-slow/zebra-stop 齐全、ESC 1560→1500、
  结束后回 track、session.json 的 zebra 段计数正确（含"没播报"的告警口径）；
- 离线回放（--frames --zebra）：横道帧目录 → 汇总行"相位触发 N 次"，落进 summary.txt。

跑法：cd dev && PYTHONPATH=. python tests/test_lane_final_zebra.py
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import tempfile

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import scripts.lane_final_test as lft  # noqa: E402
from config import settings  # noqa: E402
from tests.test_lane_final import _both_reading, _lane_frame, _runner  # noqa: E402
from tests.test_lane_final_run import _phases, _run  # noqa: E402
from tests.test_zebra_detect import _ground, _zebra_frame  # noqa: E402
from vision.camera_guard import crop_frame  # noqa: E402
from vision.zebra_detect import ZebraHit, ZebraTracker  # noqa: E402

SLOW = 240.0
STOP = 330.0


def _zs(row: float = 110.0, present: bool = True) -> lft.ZebraState:
    return lft.ZebraState(present=present, near_row=row, far_row=(row - 60.0),
                          n_rows=60, n_pixels=20000, conf=1.0)


def _zrun(**kw) -> lft.LaneRunner:
    params = dict(zebra_enabled=True, zebra_slow_row=SLOW, zebra_stop_row=STOP,
                  zebra_hold_s=10.0, zebra_arm_after_s=0.0, zebra_cooldown_s=6.0)
    params.update(kw)
    return _runner(**params)


def _zebra_det_and_tracker():
    args = lft.build_parser().parse_args(["--zebra"])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        return lft.zebra_setup(args)


# ================================================================ 检测侧（合成图）
def test_detector_finds_synthetic_crosswalk_ignores_lanes() -> None:
    det, _trk = _zebra_det_and_tracker()
    assert det is not None
    lane = lft.LaneFinalDetector(target_x=330.0, fps=30.0)

    frame = crop_frame(_zebra_frame(near_row=300, far_row=200, base=_lane_frame()))
    hit = det.detect(frame)
    assert hit is not None, "合成横道应当检出"
    assert abs(hit.near_row - 300) <= 2, hit
    r = lane.detect(frame)
    assert r.valid, f"横道不该毁掉车道读数：{r}"          # 同帧两者都要成立

    assert det.detect(crop_frame(_lane_frame())) is None, "空赛道（只有车道线）不该检出"


def test_zebra_state_of_debounce() -> None:
    """入口胶水：连续 3 帧才 present；退出 2 帧才消失；没开 = None（不干预）。"""
    trk = ZebraTracker()
    hit = ZebraHit(near_row=200.0, far_row=150.0, x0=0.0, x1=640.0,
                       n_rows=50, n_pixels=20000, coverage=0.5, conf=1.0)
    assert lft.zebra_state_of(trk, hit).present is False
    assert lft.zebra_state_of(trk, hit).present is False
    st = lft.zebra_state_of(trk, hit)
    assert st.present and st.near_row == 200.0 and st.n_rows == 50
    assert lft.zebra_state_of(trk, None).present is True
    assert lft.zebra_state_of(trk, None).present is False
    assert lft.zebra_state_of(None, hit) is None


# ================================================================ 相位机（直接喂 Reading/ZebraState）
def test_disabled_by_default_ignores_zebra() -> None:
    """--zebra 不开：给 ZebraState 也一律不理（默认行为 = 与上一版逐位一致）。"""
    run = _runner()
    for i in range(20):
        co = run.step(i * 0.05, _both_reading(), allowed=True, zebra=_zs(row=400.0))
        assert co.phase == "track", co.phase
    assert run.zebra_triggers == 0 and run.zebra_phase is None


def test_arm_after_launch_blocks_early_trigger() -> None:
    """发车后 arm_after_s 内不认横道（防起点白线/画线打头误触发）。"""
    run = _zrun(zebra_arm_after_s=3.0)
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    co = run.step(1.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    assert run.zebra_triggers == 0 and co.phase == "track", f"1s < 3s 不该触发：{co.phase}"
    co = run.step(3.1, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    assert run.zebra_triggers == 1 and co.phase == "zebra-slow", co.phase


def test_slow_row_gates_creep_then_stop_row_cuts() -> None:
    """慢行档之前照常巡航；到了压蠕速；到停车行断油 1500 并开始保温。"""
    run = _zrun()
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    co = run.step(0.2, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    assert co.phase == "zebra-slow" and co.out_us > 1500.0, \
        f"慢行档之前不该降速（照常巡航）：phase={co.phase} us={co.out_us}"

    co = run.step(0.4, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 1))
    assert co.phase == "zebra-slow" and co.out_us == run.zebra_creep_us, \
        f"过了慢行档应压到蠕速：{co.out_us}"

    co = run.step(0.6, _both_reading(), allowed=True, zebra=_zs(row=STOP - 5))
    assert co.phase == "zebra-slow" and co.out_us == run.zebra_creep_us, "还没到停车行：继续蠕"

    co = run.step(0.8, _both_reading(), allowed=True, zebra=_zs(row=STOP + 1))
    assert co.phase == "zebra-stop" and co.out_us == 1500.0, f"到停车行必须断油：{co}"
    assert run.zebra_stops == 1 and run.zebra_triggers == 1
    assert abs((run.zebra_stop_row_final or 0) - (STOP + 1)) < 1e-6


def test_stop_lead_px_cuts_earlier() -> None:
    """提前量：`--zebra-stop-lead-px 20` 时，近边行到 (停车行−20) 就断油。"""
    run = _zrun(zebra_stop_lead_px=20.0)
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    run.step(0.2, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 1))
    co = run.step(0.4, _both_reading(), allowed=True, zebra=_zs(row=STOP - 20))
    assert co.phase == "zebra-stop" and co.out_us == 1500.0, f"{co}"


def test_hold_lasts_full_duration_and_survives_detection_loss() -> None:
    """★ 断油后**必须停够 hold_s**：期间检测丢了、读数丢了都照停（保温是时间口径）。"""
    run = _zrun(zebra_hold_s=10.0)
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    run.step(0.2, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 1))
    co = run.step(0.4, _both_reading(), allowed=True, zebra=_zs(row=STOP + 1))
    assert co.phase == "zebra-stop"
    t0 = 0.4
    for t in (1.0, 3.0, 6.0, 9.5, 10.3):
        co = run.step(t, _both_reading(), allowed=True)          # 检测已消失
        assert co.phase == "zebra-stop" and co.out_us == 1500.0, \
            f"t={t}（停了 {t - t0:.1f}s）必须还在停车相：{co.phase}"
    co = run.step(t0 + 10.05, _both_reading(), allowed=True)
    assert co.phase == "track", f"停够 10s 应回循迹：{co.phase}"
    assert run.zebra_stops == 1 and run.zebra_aborts == 0


def test_detection_vanishing_before_stop_row_aborts() -> None:
    """检测早早消失（误检）→ 中止回循迹；**中止不设冷却**（任务还没做完，允许再触发）。"""
    run = _zrun()
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    co = run.step(0.2, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 1))
    assert co.phase == "zebra-slow"
    co = run.step(0.6, _both_reading(), allowed=True)            # 0.4s 没检测（宽限 1.0s 内）
    assert co.phase == "zebra-slow", "宽限期内不该中止"
    co = run.step(1.5, _both_reading(), allowed=True)            # >1.0s 没检测
    assert co.phase == "track" and run.zebra_aborts == 1, f"{co.phase}"
    assert run.zebra_stops == 0
    co = run.step(1.7, _both_reading(), allowed=True, zebra=_zs(row=120.0))
    assert run.zebra_triggers == 2, f"中止后没有冷却，检测还在就该能再触发：{co.phase}"


def test_approach_timeout_forces_stop() -> None:
    """接近段超时（近边行一直到不了停车行）→ 强制断油 + 保温（不许无限蠕行）。"""
    run = _zrun(zebra_approach_max_s=2.0, zebra_hold_s=1.0)
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    co = run.step(1.0, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 1))
    assert co.phase == "zebra-slow"
    co = run.step(2.1, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 5))
    assert co.phase == "zebra-stop" and co.out_us == 1500.0, f"超时应强制断油：{co}"
    assert run.zebra_stops == 1
    co = run.step(3.2, _both_reading(), allowed=True)
    assert co.phase == "track", "停够 hold_s 回循迹"


def test_extra_creep_stops_past_the_crosswalk() -> None:
    """`--zebra-extra-creep-s`：到停车行后**先继续蠕行** N 秒才断油（"停在横道之后"变体）。"""
    run = _zrun(zebra_extra_creep_s=1.5, zebra_hold_s=1.0)
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    run.step(0.2, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 1))
    co = run.step(1.0, _both_reading(), allowed=True, zebra=_zs(row=STOP + 5))
    assert co.phase == "zebra-slow" and co.out_us == run.zebra_creep_us, \
        f"extra_creep 期间应继续蠕行：{co}"
    co = run.step(1.2, _both_reading(), allowed=True, zebra=_zs(row=STOP + 20))
    assert co.phase == "zebra-slow", "1.5s 还没到：继续蠕"
    co = run.step(2.6, _both_reading(), allowed=True, zebra=_zs(row=STOP + 40))
    assert co.phase == "zebra-stop" and co.out_us == 1500.0, f"过了 extra 时间应断油：{co}"


def test_rearm_latch_blocks_same_crosswalk_twice() -> None:
    """★ 停过车的那道横道不会二进宫（要先"检测消失一次"+ 冷却）；换一道横道照样触发。"""
    run = _zrun(zebra_hold_s=0.5, zebra_cooldown_s=6.0)
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    run.step(0.2, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 1))
    co = run.step(0.4, _both_reading(), allowed=True, zebra=_zs(row=STOP + 1))
    assert co.phase == "zebra-stop"
    co = run.step(1.0, _both_reading(), allowed=True)                 # 停够 0.5s → 回循迹
    assert co.phase == "track" and run.zebra_stops == 1
    # 同一道横道还在眼前（检测一直在）→ 不许再触发
    for t in (1.2, 2.0, 3.0):
        co = run.step(t, _both_reading(), allowed=True, zebra=_zs(row=STOP + 30))
        assert co.phase == "track", f"重挂载闩锁没生效：t={t} {co.phase}"
    assert run.zebra_triggers == 1
    # 横道出画（检测消失一次）→ 闩锁释放；冷却过后新横道可触发
    run.step(3.2, _both_reading(), allowed=True)                      # 检测消失
    co = run.step(3.4, _both_reading(), allowed=True, zebra=_zs(row=120.0))
    assert run.zebra_triggers == 1, f"冷却（6s）内不该触发：{co.phase}"
    co = run.step(7.2, _both_reading(), allowed=True, zebra=_zs(row=120.0))
    assert run.zebra_triggers == 2 and co.phase == "zebra-slow", f"{co.phase}"


def test_guard_stop_is_not_throttled_by_zebra() -> None:
    """★ 护栏（画龙/跑偏）停车后，斑马线相位**绝不能给它加油门**（否则"停不下来"）。"""
    run = _zrun(err_stop_px=60.0, err_stop_s=0.3)
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    run.step(0.1, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    # 误差 100px 持续 > 0.3s → 跑偏护栏触发（永久停）
    co = None
    for i in range(12):
        co = run.step(0.15 + i * 0.05, _both_reading(center=430.0), allowed=True,
                      zebra=_zs(row=110.0))
    assert run.stopped and run.stop_reason == "big-err", f"{run.stop_reason}"
    assert co is not None and co.phase == "guard" and co.out_us == 1500.0, \
        f"护栏停车必须保持中位脉宽：{co.phase} {co.out_us}"
    assert run.zebra_aborts >= 1, "护栏停车时斑马线相位应中止"
    # 护栏停着时即使横道压在眼前，也不许给油
    co = run.step(5.0, _both_reading(), allowed=True, zebra=_zs(row=STOP + 5))
    assert co.out_us == 1500.0, f"停车中不许给油：{co.out_us}"


def test_lost_reading_stop_wins_over_zebra() -> None:
    """丢线阶梯优先于斑马线相位（安全优先）：读数断超时 → 停车，相位中止。"""
    run = _zrun(zebra_hold_s=10.0, zebra_approach_max_s=10.0)
    run.step(0.0, _both_reading(), allowed=True, zebra=_zs(row=110.0))
    run.step(0.2, _both_reading(), allowed=True, zebra=_zs(row=SLOW + 1))
    co = None
    for i in range(10):                                  # 丢线（hold 0.3s → lost）
        co = run.step(0.25 + i * 0.1, lft.Reading(), allowed=True, zebra=_zs(row=SLOW + 2))
    assert run.stopped and co is not None and co.out_us == 1500.0, f"{co}"
    assert run.zebra_phase is None and run.zebra_aborts >= 1


def test_creep_speed_floored_above_deadband() -> None:
    """蠕速低于死区 = "接近"变"停死半路"→ 必须被下限兜住；默认值不被下限顶上去。"""
    floor = float(settings.ESC_DEADBAND_US)
    assert _zrun(zebra_creep_us=1500.0).zebra_creep_us == floor + 1.0
    assert _zrun().zebra_creep_us == float(settings.ZEBRA_CREEP_US)
    assert float(settings.ZEBRA_CREEP_US) > floor, "默认蠕速必须在死区之上"


def test_stop_row_must_exceed_slow_row() -> None:
    """停车行必须 > 慢行档（等于就是把车停死在慢行档上）—— 由 main() 拒绝。"""
    rc = lft.main(["--zebra", "--zebra-slow-row", "300", "--zebra-stop-row", "300",
                   "--image", "", "--frames", ""])
    assert rc == 2, f"应当拒绝运行（rc=2），实得 {rc}"


# ================================================================ 端到端（假摄像头）
def _approach_frames(n_far: int = 8, step_px: float = 6.0,
                     near_start: float = 110.0, near_end: float = 345.0) -> list:
    """合成"接近横道"：白带近边逐帧下移（车在往前开），最后横道出画。"""
    frames = [_lane_frame() for _ in range(n_far)]
    row = near_start
    while row <= near_end:
        frames.append(_zebra_frame(near_row=int(row), far_row=int(row) - 70,
                                   base=_lane_frame()))
        row += step_px
    frames += [_lane_frame() for _ in range(40)]
    return frames


def test_fake_camera_zebra_sequence() -> None:
    """带横道的一趟：CSV 里 zebra-slow/zebra-stop 齐全、ESC 1560→1500、停完回 track。

    `--max-seconds 3.5`：假摄像头是**循环播**的，跑久了会再来一道合成横道（那就成两次停车）。
    本用例只钉"一次完整流程"；多次/冷却由 `test_rearm_latch_blocks_same_crosswalk_twice` 覆盖。
    """
    frames = _approach_frames()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc, tmp, csv_text = _run(frames, ["--zebra", "--no-start-gate", "--no-zebra-voice",
                                          "--zebra-hold-s", "0.4", "--zebra-arm-after-s", "0.2",
                                          "--max-seconds", "3.5", "--print-every", "1000"])
    out = buf.getvalue()
    assert rc == 0, out[-800:]
    rows = _phases(csv_text)
    phases = [p for p, _ in rows]
    assert "zebra-slow" in phases, f"应有斑马线接近相：{sorted(set(phases))}"
    assert "zebra-stop" in phases, f"应有斑马线停车相：{sorted(set(phases))}"
    assert phases.index("zebra-slow") < phases.index("zebra-stop")
    assert phases[phases.index("zebra-stop"):].count("zebra-stop") >= 5, "停车相要连续多帧"
    assert "track" in phases[phases.index("zebra-stop"):], "停够时间必须回循迹"
    esc_slow = [float(e) for p, e in rows if p == "zebra-slow"]
    esc_stop = [float(e) for p, e in rows if p == "zebra-stop"]
    assert all(abs(v - 1560.0) < 1e-6 for v in esc_slow), f"接近相蠕速应=1560：{esc_slow[:6]}"
    assert all(abs(v - 1500.0) < 1e-6 for v in esc_stop), f"停车相必须是断油 1500：{esc_stop[:6]}"
    line = [ln for ln in out.splitlines() if "斑马线（--zebra）" in ln]
    assert line and "断油 1 次" in line[-1], out[-600:]
    with open(os.path.join(tmp, "session.json"), encoding="utf-8") as fh:
        sess = json.load(fh)
    assert sess["result"]["zebra"]["stops"] == 1, sess["result"]["zebra"]
    assert sess["result"]["zebra"]["triggers"] == 1
    assert sess["params"]["zebra"]["stop_row"] == 330.0
    assert sess["params"]["zebra"]["voice_on"] is False, "--no-zebra-voice 时不该挂播放器"


def test_fake_camera_empty_track_zero_triggers() -> None:
    """空赛道（只有车道线）→ 零触发 + 汇总行明确写"零触发"（§2.8 的验收门槛）。"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc, _tmp, csv_text = _run([_lane_frame()] * 90,
                                  ["--zebra", "--no-start-gate", "--no-zebra-voice",
                                   "--max-seconds", "2.5", "--print-every", "1000"])
    out = buf.getvalue()
    assert rc == 0
    phases = set(p for p, _ in _phases(csv_text))
    assert not ({"zebra-slow", "zebra-stop"} & phases), f"空赛道不该触发：{phases}"
    line = [ln for ln in out.splitlines() if "斑马线（--zebra）" in ln]
    assert line and "相位触发 0 次" in line[-1] and "零触发" in line[-1], out[-600:]


def test_offline_replay_zebra_summary_and_trigger() -> None:
    """离线回放（--frames --zebra）：横道帧目录 → 触发 1 次、落进 summary.txt / run.csv。

    这条就是现场量"空赛道误检率/横道次数"的那条（把 --frames 指到当天的实拍帧目录）。
    """
    tmp = tempfile.mkdtemp(prefix="lane_final_zebra_off_")
    fdir = os.path.join(tmp, "frames")
    os.makedirs(fdir)
    for i, fr in enumerate(_approach_frames(n_far=5, step_px=8.0,
                                            near_start=110.0, near_end=330.0)):
        cv2.imwrite(os.path.join(fdir, f"frame_{i:06d}.jpg"), fr)
    out = os.path.join(tmp, "out")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = lft.main(["--frames", fdir, "--zebra", "--out", out, "--fps", "30",
                       "--print-every", "0", "--save-every-s", "0",
                       "--zebra-arm-after-s", "0.1", "--zebra-slow-row", "200",
                       "--zebra-stop-row", "320"])
    assert rc == 0, buf.getvalue()[-600:]
    with open(os.path.join(out, "summary.txt"), encoding="utf-8") as fh:
        summary = fh.read()
    assert "斑马线（--zebra）" in summary and "相位触发 1 次" in summary, summary
    assert "断油 1 次" in summary, summary
    csv_text = open(os.path.join(out, "run.csv"), encoding="utf-8").read()
    phases = [p for p, _ in _phases(csv_text)]
    assert "zebra-slow" in phases and "zebra-stop" in phases, sorted(set(phases))
    assert phases[phases.index("zebra-slow"):].count("zebra-stop") >= 1


def test_offline_replay_empty_track_zero_triggers() -> None:
    """离线回放空赛道（同一入口、同一检测档）→ 零触发（"空赛道零误检"验收）。"""
    tmp = tempfile.mkdtemp(prefix="lane_final_zebra_off2_")
    fdir = os.path.join(tmp, "frames")
    os.makedirs(fdir)
    frames = [_lane_frame() for _ in range(40)] + [_ground() for _ in range(20)]
    for i, fr in enumerate(frames):
        cv2.imwrite(os.path.join(fdir, f"frame_{i:06d}.jpg"), fr)
    out = os.path.join(tmp, "out")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = lft.main(["--frames", fdir, "--zebra", "--out", out, "--fps", "30",
                       "--print-every", "0", "--save-every-s", "0"])
    assert rc == 0
    with open(os.path.join(out, "summary.txt"), encoding="utf-8") as fh:
        summary = fh.read()
    assert "相位触发 0 次" in summary and "零触发" in summary, summary


def test_online_calibrate_with_zebra() -> None:
    """车上静止标定 + 横道（`--calibrate --zebra`）：出近边行读数与可抄的 stop_row、不崩。"""
    frames = [_zebra_frame(near_row=320, far_row=250, base=_lane_frame())] * 40
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc, _tmp, _ = _run(frames, ["--calibrate", "--zebra", "--calibrate-s", "0.4",
                                    "--no-save-site"])
    out = buf.getvalue()
    assert rc == 0, out[-500:]
    assert "斑马线读数" in out, out[-800:]
    assert "--zebra-stop-row" in out, out[-800:]


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
    print(f"[test_lane_final_zebra] {len(funcs) - failed}/{len(funcs)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
