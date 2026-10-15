"""lane_final_test.py 的纯函数/合成图测试（不碰硬件、不碰摄像头、不碰网络）。

覆盖：
- 检测器：双侧中心/误差符号、平移响应、单侧兜底（半宽先验 + 种子）、丢线、
  斜率闸门（近水平亮带不当线）、宽亮区不当线、配对宽越界（pair-reject）、
  时域一致性（jump-reject 且 **keep-prev**：持续幻影从第二帧起照样被拒）、
  可见长度不足、annotate 冒烟、检测耗时预算；
- 控制/相位：min-quality 闸门、单侧回中位直行、弱对限幅、丢线阶梯、
  跑偏护栏、离线回放的 CSV 契约、--calibrate 能跑；
- 真实素材回放契约（`train/opencv-level1/video/frames/2` 在场时才跑，不在就跳过）。

跑法：cd dev && PYTHONPATH=. python tests/test_lane_final.py
"""
from __future__ import annotations

import os
import sys
import tempfile
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import scripts.lane_final_test as lft  # noqa: E402
from scripts.lane_final_test import (LaneFinalDetector, LaneRunner, Reading,  # noqa: E402
                                     annotate, CSV_HEADER)

W, H = 640, 480


# ---------------------------------------------------------------- 合成图（几何与检测档对齐）
def _lane_frame(xl: float = 60.0, xr: float = 600.0, kl: float = -0.5, kr: float = 0.5,
                draw_l: bool = True, draw_r: bool = True, speckle: bool = True,
                seed: int = 7) -> np.ndarray:
    """红棕底 + 颗粒噪声 + 两条**细亮斜线**（y 5~200，|k|=0.5，与 1014 素材同几何口径）。"""
    rng = np.random.default_rng(seed)
    frame = np.full((H, W, 3), 95, np.uint8)
    frame[:, :, 2], frame[:, :, 1] = 105, 80          # 偏红棕的底
    if speckle:                                        # 路面颗粒（细密亮点，防"纯色底"假阳性）
        ys = rng.integers(5, 400, 2600)
        xs = rng.integers(0, W, 2600)
        frame[ys, xs] = rng.integers(130, 175, (2600, 3))
    for x_ref, k, on in ((xl, kl, draw_l), (xr, kr, draw_r)):
        if not on:
            continue
        for y in range(5, 200):
            x = int(round(x_ref + (y - 100) / k))
            if 0 <= x < W:
                frame[y, max(0, x - 2):x + 3] = 238    # 5px 宽亮线
    return frame


def _blank_frame(seed: int = 3) -> np.ndarray:
    frame = np.full((H, W, 3), 95, np.uint8)
    frame[:, :, 2], frame[:, :, 1] = 105, 80
    return frame


def _mk_det(**kw) -> LaneFinalDetector:
    return LaneFinalDetector(target_x=330.0, fps=30.0, **kw)


# ================================================================ 检测器
def test_both_sides_center_symmetric() -> None:
    det = _mk_det()
    r = det.detect(_lane_frame())
    assert r.both_sides, f"应判双侧：{r}"
    assert abs(r.center_x - 330.0) < 12.0, f"对称车道中心应 ≈330，实得 {r.center_x}"
    assert r.error is not None and abs(r.error) < 12.0
    assert r.quality >= 0.28
    assert r.n_l > 40 and r.n_r > 40, f"可见采样点应够：{r.n_l}/{r.n_r}"
    assert abs(r.left_k - (-0.5)) < 0.12 and abs(r.right_k - 0.5) < 0.12, "斜率应≈±0.5"


def test_shift_moves_center_and_error_sign() -> None:
    det = _mk_det()
    r0 = det.detect(_lane_frame())
    det.reset()
    r1 = det.detect(_lane_frame(xl=110.0, xr=650.0))     # 整体右移 50px
    assert r1.center_x > r0.center_x + 40, f"右移应让中心增大：{r0.center_x} → {r1.center_x}"
    assert r1.error > r0.error + 40


def test_single_side_uses_measured_half_width() -> None:
    det = _mk_det()
    for _ in range(4):                                   # 先喂几帧双侧，让半宽先验"实测"
        det.detect(_lane_frame())
    r = det.detect(_lane_frame(draw_l=False))            # 只剩右线
    assert not r.both_sides and r.valid
    assert "1side-width" in r.tag, r.tag
    assert r.quality == lft.Q_SINGLE_MEASURED
    assert abs(r.center_x - (r.right_x - r.half_w)) < 1e-6
    assert abs(r.half_w - 270.0) < 25.0, f"半宽应≈(600−60)/2=270，实得 {r.half_w}"


def test_single_side_seed_before_any_pair() -> None:
    det = _mk_det()
    r = det.detect(_lane_frame(draw_l=False))            # 一上来就单侧：半宽只有种子
    assert "1side-seed" in r.tag, r.tag
    assert r.quality == lft.Q_SINGLE_SEED
    assert r.quality < lft.MIN_QUALITY0                  # 控制层应把它当"维持"处理
    assert abs(r.half_w - lft.HALF_W_SEED) < 1e-6


def test_single_side_jump_is_rejected() -> None:
    """单侧帧也要过时域门限（2026-10-15 事故回归）。

    实测场景：车出线后唯一那条线"锁到别的结构上"，x@y_ref 一跳 220px（+163 → −58），
    q=0.55 照样算有效 → single_hold 保速直冲。这里的合成版：
      · 先双侧（左线 x@100=60）→ 记住该侧；
      · 左线跳到 260（+200px）→ 必须**丢本帧读数**（invalid + tag）；
      · 小幅移动（60 → 80）→ 必须照收（别把正常读数误杀）。
    """
    det = _mk_det()
    for _ in range(4):                                             # 喂够双侧帧让半宽先验"实测"
        r0 = det.detect(_lane_frame(xl=60, xr=600))
    assert r0.both_sides
    r1 = det.detect(_lane_frame(xl=260, xr=600, draw_r=False))     # 只剩左线且跳 +200px
    assert not r1.valid, f"单侧跳 200px 应被丢：c={r1.center_x} tag={r1.tag}"
    assert "1side-jump-L" in r1.tag, r1.tag
    assert r1.center_x is None and r1.left_x is None
    r2 = det.detect(_lane_frame(xl=80, xr=600, draw_r=False))      # 小幅移动（+20px）
    assert r2.valid and "1side-width" in r2.tag, f"小幅移动必须照收：{r2.tag}"
    assert abs(r2.left_x - 80.0) < 8.0, r2.left_x


def test_single_side_jump_reject_resets_with_prior() -> None:
    """被拒的单侧帧不得污染参照；重新出现双侧后门限从新值起算。"""
    det = _mk_det()
    det.detect(_lane_frame(xl=60, xr=600))
    det.detect(_lane_frame(xl=260, xr=600, draw_r=False))          # 被拒
    det.detect(_lane_frame(xl=60, xr=600))                         # 双侧回到 60/600
    r = det.detect(_lane_frame(xl=70, xr=600, draw_r=False))       # 相对 60 只跳 10px
    assert r.valid, f"参照应仍是 60（被拒帧不入记忆），实得 tag={r.tag}"


def test_no_lines_is_lost() -> None:
    det = _mk_det()
    r = det.detect(_blank_frame())
    assert not r.valid and r.center_x is None, f"空画面不该给中心：{r}"


def test_horizontal_bright_band_is_not_a_line() -> None:
    """近水平亮带（绿漆线/路面刷痕/遮挡边）——斜率闸门 + 脊筛选都必须挡住。"""
    frame = _lane_frame()
    frame[60:92, 0:640] = 235                            # 一大条近水平亮带（|k|≈0）
    det = _mk_det()
    r = det.detect(frame)
    assert r.both_sides, "真线仍在，应照样给双侧"
    assert abs(r.center_x - 330.0) < 20.0, f"近水平亮带不该把中心带走：{r.center_x}"


def test_wide_bright_region_is_not_a_line() -> None:
    """宽亮区（冲刷/过曝区）不是细亮脊 → 顶帽后应不构成候选。"""
    frame = _lane_frame(xl=60, xr=600)
    frame[30:190, 300:430] = 220                         # 130px 宽的亮区（比顶帽核大得多）
    det = _mk_det()
    r = det.detect(frame)
    assert r.both_sides, f"{r}"
    assert abs(r.center_x - 330.0) < 25.0, f"宽亮区不该当线：{r.center_x}"


def test_pair_width_reject_falls_to_single() -> None:
    det = _mk_det()
    r = det.detect(_lane_frame(xl=280.0, xr=340.0))       # 宽 60px < pair_w_min(240)
    assert not r.both_sides, "配对宽越界必须降级"
    assert "pair-reject" in r.tag, r.tag


def test_visible_length_gate() -> None:
    """线只在很少几行可见（断裂/遮挡）→ 采样点不足 → 不认。"""
    frame = _blank_frame()
    for y in range(96, 108):                             # 只有 12 行
        frame[y, 296:301] = 238
    det = _mk_det()
    r = det.detect(frame)
    assert r.n_l == 0, f"可见长度不足不该成线：n_l={r.n_l}"


def test_jump_reject_keep_prev() -> None:
    """时域一致性：只跳一侧 → 丢该侧；且**持续幻影从第二帧起也要被拒**（keep-prev）。"""
    det = _mk_det()
    for _ in range(4):
        det.detect(_lane_frame())
    # 右线 600→500（内跳 100px，仍在画面内），左线不动 = 典型"只跳一侧"的假特征指纹
    phantom = det.detect(_lane_frame(xl=60.0, xr=500.0))
    assert "jump-reject-R" in phantom.tag, phantom.tag
    assert not phantom.both_sides
    phantom2 = det.detect(_lane_frame(xl=60.0, xr=500.0))   # 幻影持续：prev 没被污染 → 照样拒
    assert "jump-reject-R" in phantom2.tag, \
        f"持续幻影第二次也必须被拒（keep-prev）：{phantom2.tag}"


def test_real_jump_after_stable_recovery() -> None:
    """真实的"两线同移"（车在动）不该被 jump-reject 误伤。"""
    det = _mk_det()
    for _ in range(4):
        det.detect(_lane_frame())
    r = det.detect(_lane_frame(xl=140.0, xr=680.0))          # 两条都右移 80px（相干运动）
    assert r.both_sides and "jump-reject" not in r.tag, f"{r.tag}"


def test_annotate_smoke() -> None:
    det = _mk_det()
    frame = _lane_frame()
    r = det.detect(frame)
    vis = annotate(frame, r, det, "track")
    assert vis.shape == frame.shape and vis.dtype == frame.dtype


def test_detect_speed_budget() -> None:
    det = _mk_det()
    frame = _lane_frame()
    det.detect(frame)                                     # 预热
    t0 = time.perf_counter()
    for _ in range(5):
        det.detect(frame)
    ms = (time.perf_counter() - t0) / 5 * 1000
    print(f"    [计时] 单帧检测 {ms:.1f}ms（30fps 预算 33ms）")
    assert ms < 60.0, f"单帧检测 {ms:.1f}ms 超预算（30fps 要 <33ms）"


# ================================================================ 控制 / 相位（直接喂 Reading）
def _runner(**kw) -> LaneRunner:
    det = _mk_det()
    params = dict(center_deg=90.0, sign=1.0, kp=0.15, ki=0.02, kd=0.12, limit_deg=15.0,
                  smooth=0.7, speed_us=1580.0, start_us=1560.0, single_speed_us=None,
                  single_hold=True, single_limit=5.0, weak_limit=10.0, min_quality=0.28,
                  hold_s=0.3, acquire_s=3.0, err_stop_px=60.0, err_stop_s=1.5, fps_ref=15.0)
    params.update(kw)
    return LaneRunner(det, lft.FrameRate(), **params)


def _both_reading(center: float = 330.0, n_l: int = 60, n_r: int = 60) -> Reading:
    r = Reading(center_x=center, error=center - 330.0, left_x=60.0, right_x=600.0,
                left_k=-0.5, right_k=0.5, score_l=20.0, score_r=20.0, n_l=n_l, n_r=n_r,
                both_sides=True, half_w=270.0, width_measured=True, quality=lft.Q_BOTH)
    return r


def test_runner_track_and_steer_cap() -> None:
    run = _runner()
    co = run.step(0.0, _both_reading(center=330.0), allowed=True)
    assert co.phase == "track" and abs(co.angle - 90.0) < 8.0
    co2 = run.step(0.1, _both_reading(center=430.0), allowed=True)   # 误差 100px → 限幅内
    assert abs(co2.angle - 90.0) <= 15.0 + 1e-6, "舵角必须限幅 ±15°"


def test_runner_single_hold_straight_and_fast() -> None:
    run = _runner()
    run.step(0.0, _both_reading(), allowed=True)
    single = Reading(center_x=600.0, error=270.0, left_x=330.0, right_x=None, right_k=None,
                     left_k=-0.5, score_l=20.0, n_l=60, n_r=0, both_sides=False,
                     half_w=270.0, width_measured=True, quality=lft.Q_SINGLE_MEASURED,
                     tag="1side-width")
    co = run.step(0.1, single, allowed=True)
    assert co.phase == "single"
    assert abs(co.angle - 90.0) < 1e-6, "单侧帧必须回中位直行（single-hold 默认开）"
    assert co.out_us == run.single_speed_us, "单侧帧保速"
    assert "已" in co.note, f"单侧帧 note 应带持续时长（复盘用）：{co.note}"


def _single_reading(center: float = 600.0) -> Reading:
    return Reading(center_x=center, error=center - 330.0, left_x=330.0, right_x=None,
                   right_k=None, left_k=-0.5, score_l=20.0, n_l=60, n_r=0, both_sides=False,
                   half_w=270.0, width_measured=True, quality=lft.Q_SINGLE_MEASURED,
                   tag="1side-width")


def test_runner_single_timeout_stops() -> None:
    """单侧连续超时 → 按丢线停车（2026-10-15 事故回归："出线后回中位保速直冲"）。"""
    run = _runner(single_max_s=0.5, hold_s=0.3)
    run.step(0.0, _both_reading(), allowed=True)
    co1 = run.step(0.1, _single_reading(), allowed=True)
    assert co1.phase == "single" and co1.out_us == run.single_speed_us
    co2 = run.step(0.4, _single_reading(), allowed=True)          # 0.3s < 0.5s：还在时限内
    assert co2.phase == "single" and co2.out_us == run.single_speed_us
    co3 = run.step(0.65, _single_reading(), allowed=True)         # 0.55s ≥ 0.5s：停
    assert co3.phase == "lost", f"单侧超时应按丢线停车，实得 {co3.phase}"
    assert co3.out_us == lft.NEUTRAL_US and run.stopped
    # 恢复：连续 5 帧有效（双侧）→ 重新跟线
    for i in range(5):
        co4 = run.step(0.7 + i * 0.05, _both_reading(), allowed=True)
    assert not run.stopped and co4.phase in ("track", "single", "hold")


def test_runner_single_timer_resets_on_both_sides() -> None:
    """单侧计时只统计**连续**单侧：中途回到双侧就清零（别把零星单侧攒成超时）。"""
    run = _runner(single_max_s=0.5, hold_s=0.3)
    run.step(0.0, _both_reading(), allowed=True)
    run.step(0.1, _single_reading(), allowed=True)
    run.step(0.3, _single_reading(), allowed=True)                # 0.2s
    run.step(0.4, _both_reading(), allowed=True)                  # 双侧 → 清零
    co = run.step(0.5, _single_reading(), allowed=True)
    assert co.phase == "single" and not run.stopped, "双侧帧应把单侧计时清零"


def test_runner_single_timeout_can_be_disabled() -> None:
    """--single-max-s 0 = 关（老行为：一直回中位保速）。"""
    run = _runner(single_max_s=0.0, hold_s=0.3)
    run.step(0.0, _both_reading(), allowed=True)
    co = None
    for i in range(10):
        co = run.step(0.1 + i * 0.2, _single_reading(), allowed=True)
    assert co.phase == "single" and not run.stopped, "0 应关闭时限"


def test_runner_weak_pair_cap() -> None:
    run = _runner()
    weak = _both_reading(center=430.0, n_l=20, n_r=60)     # 一侧支持不足 = 弱对
    co = run.step(0.0, weak, allowed=True)
    assert abs(co.angle - 90.0) <= 10.0 + 1e-6, f"弱对帧应限 ±10°，实得 {co.angle}"


def test_runner_seed_quality_below_gate_goes_hold() -> None:
    run = _runner()
    seed = Reading(center_x=600.0, error=270.0, right_x=600.0, right_k=0.5,
                   score_r=20.0, n_r=60, both_sides=False, half_w=300.0,
                   width_measured=False, quality=lft.Q_SINGLE_SEED, tag="1side-seed")
    co = run.step(0.0, seed, allowed=True)
    assert co.phase in ("acquire", "hold"), f"种子质量(0.25) < min-quality(0.28) 应维持：{co.phase}"


def test_runner_loss_ladder() -> None:
    run = _runner(hold_s=0.4)
    co = run.step(0.0, _both_reading(), allowed=True)
    assert co.phase == "track"
    lost = Reading()                                       # 没读数
    co_h = run.step(0.1, lost, allowed=True)
    assert co_h.phase == "hold", co_h.phase
    co_l = run.step(0.6, lost, allowed=True)               # 超过 hold_s
    assert co_l.phase == "lost" and run.stopped, "丢线超时必须停车"
    assert co_l.out_us == lft.NEUTRAL_US


def test_runner_big_error_guard_trips() -> None:
    run = _runner(err_stop_px=60.0, err_stop_s=0.3)
    t = 0.0
    for _ in range(40):
        co = run.step(t, _both_reading(center=430.0), allowed=True)   # 误差恒 100px
        t += 0.05
        if run.stopped:
            break
    assert run.stopped and co.phase == "guard", "持续大误差必须触发跑偏保护"
    assert run.guard_trips >= 1


def test_runner_not_allowed_is_stop() -> None:
    run = _runner()
    co = run.step(0.0, _both_reading(), allowed=False)
    assert co.phase == "stop" and co.out_us == lft.NEUTRAL_US
    assert abs(co.angle - 90.0) < 1e-6


def test_offline_replay_csv_contract() -> None:
    """离线回放：CSV 头/行数/摘要文件都对，且 rcf=0。"""
    tmp = tempfile.mkdtemp(prefix="lane_final_off_")
    fdir = os.path.join(tmp, "frames")
    os.makedirs(fdir)
    for i in range(8):
        cv2.imwrite(os.path.join(fdir, f"frame_{i:06d}.jpg"), _lane_frame())
    rc = lft.main(["--frames", fdir, "--out", os.path.join(tmp, "out"),
                   "--print-every", "0", "--fps", "30"])
    assert rc == 0
    out = os.path.join(tmp, "out")
    csv_path = os.path.join(out, "run.csv")
    assert os.path.isfile(csv_path) and os.path.isfile(os.path.join(out, "summary.txt"))
    with open(csv_path, encoding="utf-8") as fh:
        lines = fh.read().strip().splitlines()
    assert lines[0] + "\n" == CSV_HEADER, "CSV 表头契约"
    assert len(lines) == 9, f"8 帧应写 8 行（+表头）：{len(lines)}"


def test_calibrate_runs() -> None:
    tmp = tempfile.mkdtemp(prefix="lane_final_cal_")
    fdir = os.path.join(tmp, "frames")
    os.makedirs(fdir)
    for i in range(5):
        cv2.imwrite(os.path.join(fdir, f"frame_{i:06d}.jpg"), _lane_frame())
    rc = lft.main(["--frames", fdir, "--calibrate", "--fps", "30"])
    assert rc == 0


def test_real_frames_replay_if_available() -> None:
    """真实素材回放契约（素材不在仓库里 → 跳过；在车上跑测试时也会跳过）。"""
    frames_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                              "train", "opencv-level1", "video", "frames", "2")
    paths = sorted(__import__("glob").glob(os.path.join(frames_dir, "frame_*.jpg")))
    if len(paths) < 30:
        print("    [跳过] 1014 素材不在本机")
        return
    det = LaneDetector_fixed()
    step = max(1, len(paths) // 40)
    rows = [det.detect(cv2.imread(p)) for p in paths[::step][:40]]
    both = sum(1 for r in rows if r.both_sides)
    print(f"    [真实素材] {len(rows)} 帧抽样：双侧 {both}（{both / len(rows) * 100:.0f}%）")
    assert both >= len(rows) * 0.5, f"真实素材双侧率过低：{both}/{len(rows)}"


def LaneDetector_fixed() -> LaneFinalDetector:     # noqa: N802 （测试内部小工厂）
    return LaneFinalDetector(target_x=330.0, fps=28.9)


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
    print(f"[test_lane_final] {len(funcs) - failed}/{len(funcs)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
