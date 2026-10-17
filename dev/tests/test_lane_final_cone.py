"""lane_final_test.py 的**锥桶相位（--cone）**测试：合成图 + 假摄像头，不碰硬件。

为什么单独一个文件：`doc/开发文档.md` §2.8 给每个元素定死了一套流程（阈值进 settings、
带开关且默认关、单测+离线误检率、现场先架空）。这个文件就是"锥桶相位"那一套的验收：
检测器在裁剪后画面上**该检的检得出、不该检的不误检**，相位机"谁此刻拥有方向盘"按设计仲裁。

覆盖：
- 检测侧：合成蓝锥（ROI 内）检出；车道线本身、ROI 之上的蓝色塑料布（10-15 误检源）不检出；
- 相位机：触发 → avoid1 → resume（目标点归零）→ avoid2 → 回循迹；绕行侧符号（锥偏左→往右绕）；
  优先级 cone > single；绕行期豁免跑偏护栏（但结束后照样会触发）；丢线立刻中止；冷却不重复触发；
  `--cone` 不开时给 ConeState 也不理；绕行油门被死区下限兜住；
- 端到端（假摄像头）：带锥的一趟 → CSV 里 avoid1/resume/avoid2 齐全且绕行油门 = --cone-speed-us；
  空赛道一趟 → 零触发 + 汇总行明确写"零触发"（§2.8 的验收门槛）。

跑法：cd dev && PYTHONPATH=. python tests/test_lane_final_cone.py
"""
from __future__ import annotations

import contextlib
import io
import os
import sys
import tempfile

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import scripts.lane_final_test as lft  # noqa: E402
from config import settings  # noqa: E402
from tests.test_lane_final import _both_reading, _lane_frame, _runner, _single_reading  # noqa: E402
from tests.test_lane_final_run import _phases, _run  # noqa: E402
from vision.camera_guard import crop_frame  # noqa: E402
from vision.cone_detect import ConeBox, ConeTracker  # noqa: E402

CONE_BGR = (200, 120, 40)      # HSV≈(105, 204, 200)：在 CONE_BLUE_HSV_LOW/HIGH 之内


def _cone_frame(cx: float = 200.0, y_top: int = 110, y_bot: int = 230,
                w_top: int = 28, w_bot: int = 64) -> np.ndarray:
    """车道帧 + 一只合成蓝锥（梯形）。锥桶落在 ROI（裁剪后 y76~384）内，且不影响车道读数。"""
    frame = _lane_frame()
    pts = np.array([[cx - w_top / 2, y_top], [cx + w_top / 2, y_top],
                    [cx + w_bot / 2, y_bot], [cx - w_bot / 2, y_bot]], np.int32)
    cv2.fillPoly(frame, [pts], CONE_BGR)
    return frame


def _cone_state(cx: float = 200.0, present: bool = True) -> lft.ConeState:
    return lft.ConeState(present=present, center_x=cx, n_boxes=1, conf=0.6)


def _cone_det_and_tracker():
    args = lft.build_parser().parse_args(["--cone"])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        return lft.cone_setup(args)


# ================================================================ 检测侧（合成图）
def test_detector_finds_synthetic_cone_ignores_above_roi() -> None:
    """该检的检得出；ROI 之上的蓝色塑料布（10-15 的误检源）不检。"""
    det, _trk = _cone_det_and_tracker()
    assert det is not None
    lane = lft.LaneFinalDetector(target_x=330.0, fps=30.0)

    frame = crop_frame(_cone_frame(cx=200.0))
    cones = det.detect(frame)
    assert cones, "合成蓝锥（ROI 内）应当检出"
    assert abs(cones[0].center_x - 200.0) < 12.0, cones[0]
    r = lane.detect(frame)
    assert r.valid and r.both_sides, f"锥桶不该毁掉车道读数：{r}"   # 同帧两者都要成立

    assert det.detect(crop_frame(_lane_frame())) == [], "空赛道的车道线不是锥桶"

    tarp = _lane_frame()
    tarp[10:60, 70:200] = CONE_BGR          # 上沿 y10~60：在 ROI（y76 起）之上
    assert det.detect(crop_frame(tarp)) == [], "ROI 之上的蓝色块必须被裁掉（零误检档）"


def test_cone_state_of_debounce() -> None:
    """cone_detect 的防抖器 + 入口胶水：连续 3 帧才 present，退出 2 帧才消失。"""
    trk = ConeTracker()
    box = ConeBox(xyxy=(180.0, 110.0, 240.0, 230.0), area_ratio=0.02, conf=0.6)
    assert lft.cone_state_of(trk, [box]).present is False      # 第 1 帧：还没成立
    assert lft.cone_state_of(trk, [box]).present is False
    st = lft.cone_state_of(trk, [box])
    assert st.present and st.center_x == box.center_x and st.n_boxes == 1
    assert lft.cone_state_of(trk, []).present is True          # 丢 1 帧还不算消失
    assert lft.cone_state_of(trk, []).present is False
    assert lft.cone_state_of(None, [box]) is None              # 没开 = None（不干预）


# ================================================================ 相位机（直接喂 Reading/ConeState）
def test_phase_sequence_and_offset_zeroed_on_resume() -> None:
    """触发 → avoid1 → resume（目标点归零）→ avoid2（反向）→ 回循迹。"""
    run = _runner(cone_enabled=True, cone_avoid_s=0.5, cone_segue_s=0.3,
                  cone_cooldown_s=1.0, cone_offset_px=100.0)
    co = run.step(0.0, _both_reading(), allowed=True)
    assert co.phase == "track" and run.cone_triggers == 0

    co = run.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
    assert co.phase == "avoid1", co.phase
    assert run.cone_target_offset() == -100.0, "锥偏左 → 往右绕 → 目标点偏移 −100px"
    assert abs(co.err_filt - 100.0) < 1e-6, f"绕行期用的误差 = err − offset：{co.err_filt}"
    assert co.out_us == run.cone_speed_us, "绕行期走低速档"
    assert "偏移" in co.note

    co = run.step(0.30, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
    assert co.phase == "avoid1", "0.25s < avoid_s：还在第一段"

    co = run.step(0.60, _both_reading(), allowed=True)       # 0.55s ≥ 0.5s → 回正段
    assert co.phase == "resume" and run.cone_target_offset() == 0.0
    assert abs(co.err_filt or 0.0) < 1e-6, "回正段目标点必须归零（误差 = 原误差）"

    co = run.step(0.95, _both_reading(), allowed=True)       # 0.35s ≥ 0.3s → 反向绕
    assert co.phase == "avoid2" and run.cone_target_offset() == 100.0
    assert abs(co.err_filt + 100.0) < 1e-6

    co = run.step(1.50, _both_reading(), allowed=True)       # 0.55s ≥ 0.5s → 结束
    assert co.phase == "track", co.phase
    assert run.cone_triggers == 1 and run.cone_aborts == 0
    assert run.cone_phase is None


def test_offset_sign_follows_cone_side() -> None:
    """锥偏左→往右绕（负偏移）、偏右→往左绕（正偏移）、居中→ --cone-first-dir。"""
    left_right = _runner(cone_enabled=True, cone_first_dir="left")
    left_right.step(0.0, _both_reading(), allowed=True)
    left_right.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
    assert left_right.cone_target_offset() == -lft.CONE_OFFSET_PX0      # 锥偏左 → 往右

    right_left = _runner(cone_enabled=True, cone_first_dir="left")
    right_left.step(0.0, _both_reading(), allowed=True)
    right_left.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=480.0))
    assert right_left.cone_target_offset() == lft.CONE_OFFSET_PX0       # 锥偏右 → 往左

    centered = _runner(cone_enabled=True, cone_first_dir="right")
    centered.step(0.0, _both_reading(), allowed=True)
    centered.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=335.0))
    assert centered.cone_target_offset() == -lft.CONE_OFFSET_PX0        # 居中 → first_dir=right


def test_offset_steers_to_pass_side_with_field_sign() -> None:
    """现场符号（steer_sign=−1，角度小=右转）：锥偏左→往右绕 = 舵角比中位**小**。

    这条钉死的是"目标点偏移到底把车往哪边推"——符号搞反就是**朝锥桶撞**。
    """
    run = _runner(cone_enabled=True, sign=-1.0, cone_offset_px=100.0)
    run.step(0.0, _both_reading(), allowed=True)
    co = run.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
    assert co.phase == "avoid1"
    assert co.err_filt > 50.0, "误差为正 = 应往右打"
    assert co.angle < run.center_deg, f"锥偏左必须往右绕（角度小）：{co.angle}"
    # 反例：锥偏右 → 往左绕 = 舵角比中位大
    run2 = _runner(cone_enabled=True, sign=-1.0, cone_offset_px=100.0)
    run2.step(0.0, _both_reading(), allowed=True)
    co2 = run2.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=480.0))
    assert co2.err_filt < -50.0 and co2.angle > run2.center_deg, f"{co2}"


def test_in_lane_gate_blocks_out_of_lane_cone() -> None:
    """★ "锥桶只在两条车道线之间"先验：线外的蓝物不触发；线内的正常触发。

    合成车道：左右线在 y_ref=100 处 x=60 / 600、k=∓0.5（= 合成帧的几何）。
    - 车道外（x=30，靠左墙）拿不到"在车道内" → 不触发（先验把远景误检挡在门外）；
    - 车道内（x=330）→ 正常触发；
    - 读数不可信（单侧帧）→ 这一帧**不算证据**（绕行是相对车道中心做的动作；单侧帧放行会
      让"5 帧连续误检"点着一次避让——2026-10-16 实测）；
    - `--no-cone-in-lane` 关掉先验 → 回到老行为（单侧帧也能触发）。
    """
    run = _runner(cone_enabled=True, cone_in_lane=True)
    run.step(0.0, _both_reading(), allowed=True)
    out_cone = lft.ConeState(present=True, center_x=30.0, top_y=60.0, bottom_y=110.0)
    co = run.step(0.05, _both_reading(), allowed=True, cone=out_cone)
    assert co.phase == "track" and run.cone_triggers == 0 and run.cone_out_lane == 1, co.phase

    run2 = _runner(cone_enabled=True, cone_in_lane=True)
    run2.step(0.0, _both_reading(), allowed=True)
    in_cone = lft.ConeState(present=True, center_x=330.0, top_y=60.0, bottom_y=110.0)
    co2 = run2.step(0.05, _both_reading(), allowed=True, cone=in_cone)
    assert co2.phase == "avoid1" and run2.cone_triggers == 1

    run3 = _runner(cone_enabled=True, cone_in_lane=True)      # 单侧帧 → 不算证据（不触发）
    run3.step(0.0, _both_reading(), allowed=True)
    co3 = run3.step(0.05, _single_reading(), allowed=True, cone=out_cone)
    assert co3.phase == "single" and run3.cone_triggers == 0, \
        "单侧帧不该提供锥桶证据（读数不可信时不做相对车道的动作）"

    run4 = _runner(cone_enabled=True, cone_in_lane=False)     # --no-cone-in-lane：先验关
    run4.step(0.0, _both_reading(), allowed=True)
    co4 = run4.step(0.05, _both_reading(), allowed=True, cone=out_cone)
    assert co4.phase == "avoid1" and run4.cone_out_lane == 0


def test_too_near_cone_does_not_trigger() -> None:
    """★ 只对"前方"的锥桶起绕：框底太低（贴着车头/已经绕过）不触发，框底在前方则正常触发。

    2026-10-16 实验室实测：画面底部角落的蓝色物（框底 384 = 贴着裁剪线）被判"在车道线内"，
    点着了一次 3.2s 的 S 型 —— 那种情况绕也没用（车已经在它旁边）。
    """
    near = lft.ConeState(present=True, center_x=330.0, top_y=300.0, bottom_y=384.0)
    run = _runner(cone_enabled=True, cone_max_bottom_y=300.0)
    run.step(0.0, _both_reading(), allowed=True)
    co = run.step(0.05, _both_reading(), allowed=True, cone=near)
    assert co.phase == "track" and run.cone_triggers == 0 and run.cone_too_near == 1, co.phase

    ahead = lft.ConeState(present=True, center_x=330.0, top_y=60.0, bottom_y=110.0)
    run2 = _runner(cone_enabled=True, cone_max_bottom_y=300.0)
    run2.step(0.0, _both_reading(), allowed=True)
    co2 = run2.step(0.05, _both_reading(), allowed=True, cone=ahead)
    assert co2.phase == "avoid1" and run2.cone_triggers == 1

    run3 = _runner(cone_enabled=True, cone_max_bottom_y=0.0)     # 0 = 关（老行为）
    run3.step(0.0, _both_reading(), allowed=True)
    co3 = run3.step(0.05, _both_reading(), allowed=True, cone=near)
    assert co3.phase == "avoid1" and run3.cone_too_near == 0, "0 应关闭该闸门"


def test_priority_cone_over_single() -> None:
    """绕行期间单侧帧不让位（相位优先级 cone > single）：单侧也按偏移线走。"""
    run = _runner(cone_enabled=True)
    run.step(0.0, _both_reading(), allowed=True)
    run.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
    co = run.step(0.10, _single_reading(), allowed=True)
    assert co.phase == "avoid1", f"绕行期不该被 single-hold 抢走方向盘：{co.phase}"
    assert co.out_us == run.cone_speed_us


def test_lost_reading_aborts_avoid() -> None:
    """读数一断（丢线）→ 立刻中止绕行，把方向盘交回丢线阶梯（宁可停，不要瞎绕）。"""
    run = _runner(cone_enabled=True, hold_s=0.5)
    run.step(0.0, _both_reading(), allowed=True)
    run.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
    co = run.step(0.10, lft.Reading(), allowed=True)          # 没读数
    assert run.cone_phase is None and run.cone_aborts == 1
    assert co.phase in ("hold", "lost", "acquire"), co.phase


def test_guards_exempt_during_avoid_but_armed_after() -> None:
    """绕行期豁免跑偏护栏（刻意的大幅转向）；一轮走完护栏照样会开火。"""
    run = _runner(cone_enabled=True, cone_avoid_s=0.3, cone_segue_s=0.2,
                  err_stop_px=60.0, err_stop_s=0.3, cone_cooldown_s=9.0)
    run.step(0.0, _both_reading(), allowed=True)
    t = 0.05
    co = run.step(t, _both_reading(center=430.0), allowed=True, cone=_cone_state(cx=200.0))
    assert co.phase == "avoid1"
    phases = []
    for i in range(1, 21):                                    # 覆盖 0.3+0.2+0.3 = 0.8s 全程
        co = run.step(0.05 + i * 0.05, _both_reading(center=430.0), allowed=True)
        phases.append(co.phase)
    assert not run.stopped and run.guard_trips == 0, f"绕行期不该被护栏停：{phases}"
    assert {"avoid1", "resume", "avoid2"} <= set(phases), phases
    assert phases[-1] == "track", phases
    for i in range(12):                                       # 之后误差 100px 持续 > err_stop_s
        co = run.step(1.05 + i * 0.05, _both_reading(center=430.0), allowed=True)
    assert run.stopped and co.phase == "guard", "回到循迹后跑偏保护必须恢复"


def test_cooldown_blocks_retrigger() -> None:
    """一轮结束后的冷却期内，同一个锥不再二进宫；冷却过后可以再触发。"""
    run = _runner(cone_enabled=True, cone_avoid_s=0.1, cone_segue_s=0.1, cone_cooldown_s=1.0)
    run.step(0.0, _both_reading(), allowed=True)
    run.step(0.05, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
    t, phases = 0.05, []
    while t < 1.0:                                            # 冷却期内一路喂"有锥桶"
        t += 0.05
        co = run.step(t, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
        phases.append(co.phase)
    assert run.cone_triggers == 1, f"冷却期内不该再触发：{phases}"
    assert "track" in phases
    co = run.step(1.45, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
    assert run.cone_triggers == 2, f"冷却过后应能再次触发：{co.phase}"


def test_disabled_by_default_ignores_cone() -> None:
    """--cone 不开：给 ConeState 也一律不理（默认行为 = 10-15 现场验证过的版本）。"""
    run = _runner()
    for i in range(12):
        co = run.step(i * 0.05, _both_reading(), allowed=True, cone=_cone_state(cx=200.0))
        assert co.phase == "track", co.phase
    assert run.cone_triggers == 0 and run.cone_phase is None


def test_cone_speed_floored_above_deadband() -> None:
    """绕行油门低于死区 = "绕行"变"停车"（会停死在锥桶前面）→ 必须被下限兜住。

    同时钉住"默认不动现场档位"：settings 的 1575（现场 A/B 菜单 1585/1580/1575/1570）不能被下限顶上去。
    """
    from config import settings
    floor = float(settings.ESC_DEADBAND_US)
    assert _runner(cone_enabled=True, cone_speed_us=1500.0).cone_speed_us == floor + 1.0
    assert _runner(cone_enabled=True).cone_speed_us == float(settings.CONE_AVOID_US)
    assert float(settings.CONE_AVOID_US) > floor, "默认绕行油门必须在死区之上"


# ================================================================ 端到端（假摄像头）
def test_fake_camera_cone_sequence() -> None:
    """带锥的一趟：CSV 里 avoid1/resume/avoid2 齐全、绕行油门 = --cone-speed-us、结束回 track。"""
    frames = ([_lane_frame()] * 25 + [_cone_frame(cx=200.0)] * 40 + [_lane_frame()] * 60)
    rc, _tmp, csv_text = _run(frames, ["--cone", "--no-start-gate", "--max-seconds", "6",
                                       "--cone-avoid-s", "0.4", "--cone-segue-s", "0.25",
                                       "--cone-offset-px", "90", "--print-every", "1000"])
    rows = _phases(csv_text)
    phases = [p for p, _ in rows]
    assert rc == 0
    for name in ("avoid1", "resume", "avoid2"):
        assert name in phases, f"缺 {name} 相位：{sorted(set(phases))}"
    assert phases.index("avoid1") < phases.index("resume") < phases.index("avoid2")
    esc = [float(e) for p, e in rows if p in ("avoid1", "resume", "avoid2")]
    assert esc and all(abs(v - float(settings.CONE_AVOID_US)) < 1e-6 for v in esc), \
        f"绕行油门应=settings.CONE_AVOID_US：{esc[:6]}"
    assert "track" in phases[phases.index("avoid2"):], "S 型结束应回到循迹"


def test_fake_camera_empty_track_zero_triggers() -> None:
    """空赛道（只有车道线）→ 零触发 + 汇总行写"零触发"（§2.8 的验收门槛）。"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc, _tmp, csv_text = _run([_lane_frame()] * 60,
                                  ["--cone", "--no-start-gate", "--max-seconds", "2.0",
                                   "--print-every", "1000"])
    out = buf.getvalue()
    phases = set(p for p, _ in _phases(csv_text))
    assert rc == 0
    assert not ({"avoid1", "resume", "avoid2"} & phases), f"空赛道不该触发：{phases}"
    line = [ln for ln in out.splitlines() if "锥桶（--cone）" in ln]
    assert line and "相位触发 0 次" in line[-1] and "零触发" in line[-1], out[-600:]


def test_offline_replay_cone_summary_and_trigger() -> None:
    """离线回放（--frames --cone）：带锥的一段 → 触发 S 型；统计行落进 summary.txt。

    这条路径就是现场量"空赛道误检率"的那条（把 --frames 指到当天的实拍帧目录）。
    """
    tmp = tempfile.mkdtemp(prefix="lane_final_cone_off_")
    fdir = os.path.join(tmp, "frames")
    os.makedirs(fdir)
    frames = ([_lane_frame()] * 10 + [_cone_frame(cx=200.0)] * 20 + [_lane_frame()] * 30)
    for i, fr in enumerate(frames):
        cv2.imwrite(os.path.join(fdir, f"frame_{i:06d}.jpg"), fr)
    out = os.path.join(tmp, "out")
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = lft.main(["--frames", fdir, "--cone", "--out", out, "--fps", "30",
                       "--print-every", "0", "--save-every-s", "0", "--cone-avoid-s", "0.3",
                       "--cone-segue-s", "0.2"])
    assert rc == 0
    with open(os.path.join(out, "summary.txt"), encoding="utf-8") as fh:
        summary = fh.read()
    assert "锥桶（--cone）" in summary and "相位触发 1 次" in summary, summary
    csv_text = open(os.path.join(out, "run.csv"), encoding="utf-8").read()
    phases = [p for p, _ in _phases(csv_text)]
    for name in ("avoid1", "resume", "avoid2"):
        assert name in phases, f"离线回放应走完 S 型：{sorted(set(phases))}"


def test_online_calibrate_with_cone() -> None:
    """车上静止标定 + 锥桶（`--calibrate --cone`）：出锥桶落点读数、不崩（rc=0）。

    这条是现场"确认锥桶落在画面哪几行"的那条命令（回标 ROI 用），必须在无硬件下先跑通。
    """
    frames = [_cone_frame(cx=200.0)] * 40
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc, _tmp, _ = _run(frames, ["--calibrate", "--cone", "--calibrate-s", "0.4",
                                    "--no-save-site"])
    out = buf.getvalue()
    assert rc == 0, out[-500:]
    assert "锥桶读数" in out, out[-800:]
    assert "ROI 上沿建议" in out, out[-800:]      # 给了可抄的 ROI 建议


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
    print(f"[test_lane_final_cone] {len(funcs) - failed}/{len(funcs)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
