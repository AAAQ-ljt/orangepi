"""帧率口径测试（2026-10-12）：把"按 15 帧写的"控制律钉在任意帧率上。

为什么要有这个文件
------------------
下摄的真实出帧率是变的（亮 ≈30fps、暗 9~16fps），而循迹的增益/窗口是 **15fps 现场
整定**的。控制律必须做到：**同一段物理运动，不管被 9/15/30fps 采样，输出的舵角一致**
（"阻尼、每秒积分动作、平滑时间常数"不变），否则换帧率就等于换一辆车。

这里不测"数值好不好看"，测的是三条**结构性质**：
  1. `step(e, dt=1/REF_FPS)` 与老逐帧路径**数学等价**（兼容性：不改变 15fps 下的手感）；
  2. 各分项（积分/微分/平滑）在任意帧率下**物理量不变**（把 kp/ki/kd 分别置 0 隔离出来测）；
  3. 帧数类参数（滤波窗口、护栏样本门槛）按**秒**语义折算，不再随帧率漂。

跑法：
    cd dev && PYTHONPATH=. python tests/test_lane_rate.py
"""
from __future__ import annotations

import numpy as np

from control.filters import ErrorFilter
from control.lane_control import (BigErrorGuard, DERIV_CLAMP_PX, INTEGRAL_LIMIT, OscillationGuard,
                                  PidRef)
from control.rate import REF_FPS, FrameRate, frames_at
from vision.lane_ref import LaneRefDetector


# ---------------------------------------------------------------- 估计器
def test_frame_rate_tracks_constant_and_jitter():
    """常帧率要收敛到真值；抖动帧率取 EMA（近似真值）；异常 dt 不入队。"""
    rate = FrameRate(alpha=0.3)
    assert rate.n == 0 and rate.dt is None, "还没有样本时 dt 必须是 None（调用方退回逐帧）"
    for _ in range(30):
        rate.update_dt(1.0 / 30.0)
    assert abs(rate.fps - 30.0) < 1e-6, f"常 30fps 应收敛到 30，实际 {rate.fps}"
    assert rate.n == 30

    rate2 = FrameRate(alpha=0.2)
    rng = np.random.default_rng(0)
    for _ in range(200):
        rate2.update_dt(1.0 / 15.0 + rng.normal(0, 0.002))
    assert abs(rate2.fps - 15.0) < 0.6, f"抖动 15fps 估计偏差太大：{rate2.fps}"

    # 异常 dt：0 / 负 / 停顿 30s —— 都不许参与估计
    before = rate2.fps
    rate2.update_dt(0.0)
    rate2.update_dt(-1.0)
    rate2.update_dt(30.0)
    assert rate2.fps == before, "异常 dt 改了帧率估计（会瞬间拉偏增益）"

    # update_now 的绝对时间戳路径与 update_dt 一致
    rate3 = FrameRate()
    t = 100.0
    for _ in range(20):
        t += 1.0 / 20.0
        rate3.update_now(t)
    assert abs(rate3.fps - 20.0) < 1e-6, f"update_now 口径不对：{rate3.fps}"


def test_frame_rate_pin_overrides_measurement():
    """--fps 钉死口径：pin 之后 fps/dt 都按钉住的值（离线对照/复现历史会话）。"""
    rate = FrameRate()
    for _ in range(20):
        rate.update_dt(1.0 / 30.0)
    assert abs(rate.fps - 30.0) < 1e-6
    rate.pin(15.0)
    assert abs(rate.fps - 15.0) < 1e-6 and abs(rate.dt - 1.0 / 15.0) < 1e-9
    assert abs(rate.scale - 1.0) < 1e-9, "钉住 15fps 时折算倍率必须是 1（登记口径）"


def test_frames_at_keeps_seconds_semantics():
    """帧数折算：15fps 口径的帧数 → 实际帧率下的帧数（保持秒级语义）。"""
    assert frames_at(5, 1.0) == 5
    assert frames_at(5, 2.0) == 10          # 30fps：窗口翻倍
    assert frames_at(3, 0.6) == 2           # 9fps：3 帧 @15 = 0.2s → 9fps 下 2 帧
    assert frames_at(5, 0.001) == 1 and frames_at(5, 1000.0) == 600, "必须钳位"


# ---------------------------------------------------------------- PID：兼容性
def test_pid_dt_path_equals_frame_path_at_ref_fps():
    """dt=1/REF_FPS 时两条路径**数学等价**（唯一例外：reset 后第一帧的假踢腿）。"""
    rng = np.random.default_rng(7)
    errs = np.cumsum(rng.normal(0, 12.0, size=120))

    # ---- ① 关掉输出平滑（smooth=1）：两条路径应当**逐位一致**（首帧除外）----
    kw = dict(kp=0.15, ki=0.02, kd=0.12, limit_deg=15.0, smooth=1.0,
              sign=-1.0, center=86.5)
    a, b = PidRef(**kw), PidRef(**kw)
    legacy = np.asarray([a.step(float(e)) for e in errs])
    dtp = np.asarray([b.step(float(e), 1.0 / REF_FPS) for e in errs])
    # 首帧：dt 路径**不**产生微分（老路径把"上一帧误差=0"当真 → kd×error 的假踢腿）
    kick = abs(legacy[0] - dtp[0])
    assert abs(kick - 0.12 * abs(float(errs[0]))) < 1e-9, \
        f"首帧差异应当正好是 kd×error（老路径的踢腿），实测 {kick}"
    assert abs(dtp[0] - (86.5 - (0.15 + 0.02) * errs[0])) < 1e-9, \
        "dt 路径首帧只该有 kp + 一步积分（没有微分）"
    diff = float(np.max(np.abs(legacy[1:] - dtp[1:])))
    assert diff < 1e-9, f"15fps 下两条路径必须逐位一致，实测最大差 {diff}"

    # ---- ② 打开输出平滑（smooth=0.7）：唯一差别就是首帧那记踢腿，按 0.3 每步衰减 ----
    kw["smooth"] = 0.7
    a2, b2 = PidRef(**kw), PidRef(**kw)
    legacy2 = np.asarray([a2.step(float(e)) for e in errs])
    dtp2 = np.asarray([b2.step(float(e), 1.0 / REF_FPS) for e in errs])
    d = np.abs(legacy2 - dtp2)
    assert d[0] >= d[1] * 2.9 and float(np.max(d[30:])) < 1e-6, \
        f"平滑打开后差异应当只是首帧踢腿的余波（每步 ×0.3），实测 d0={d[0]:.2e} d30max={np.max(d[30:]):.2e}"


def test_pid_prime_suppresses_kick_in_both_paths():
    """prime(e) 之后下一帧的微分从 0 起跳（10-11 修的那记假踢腿，两条路径都要管）。

    隔离成"只有 kp + 微分"：e=60px 时 kp 项 = 9°（→ 77.5°）；不带 prime 时
    Δe=60px 会被微分钳位压到 ±20px（= 同一个物理上限 300px/s）→ 只踢 2.4°（→ 75.1°）。
    所以"有没有踢"这件事在 ±20px 钳位之下仍然测得到，不会因为钳位而失效。
    """
    for dt in (None, 1.0 / 30.0):
        pid = PidRef(kp=0.15, ki=0.0, kd=0.12, limit_deg=15.0, smooth=1.0,
                     sign=-1.0, center=86.5)
        pid.reset()
        pid.prime(60.0)
        ang = pid.step(60.0, dt) if dt else pid.step(60.0)
        assert abs(ang - 77.5) < 1e-9, f"dt={dt} 时仍有微分踢腿：{ang}（应当是 77.5）"
        # 反向验证：两条路径在"不 prime"时行为不同，各有各的道理 ——
        #   老逐帧路径：Δe 从 0 起跳（被 ±20px 钳位兜住）→ 踢 kd×20 = 2.4°
        #   dt 路径：内部记"没有上一帧误差"→ 首帧天然不产生微分（= 连 prime 都不需要）
        fresh = PidRef(kp=0.15, ki=0.0, kd=0.12, limit_deg=15.0, smooth=1.0,
                       sign=-1.0, center=86.5)
        fresh.reset()
        fresh_ang = fresh.step(60.0, dt) if dt else fresh.step(60.0)
        expect = 77.5 if dt else 75.1
        assert abs(fresh_ang - expect) < 1e-9, \
            f"dt={dt} 不 prime 时应当 {expect}（dt 路径不踢 / 老路径踢 2.4°），实际 {fresh_ang}"


# ---------------------------------------------------------------- PID：不变性
def test_pid_integral_action_per_second_is_rate_invariant():
    """积分：**每秒**的动作与帧率无关（隔离 kp/kd，只留 ki）。

    误差取 1px（**别让积分钳位饱和**，否则测的是钳位不是折算）：1 秒后
      dt 路径：∫e dt = 1px·s → ki_per_s×1 = 0.02×15 = 0.3°（任意帧率都一样）
      老路径：ki×Σe → 15fps 0.02×15 = 0.3°、30fps 0.02×30 = 0.6°（**翻倍**，就是被修的病）
    """
    def integral_term(fps, secs=1.0, e=1.0):
        pid = PidRef(kp=0.0, ki=0.02, kd=0.0, limit_deg=90.0, smooth=1.0,
                     sign=-1.0, center=86.5)
        n = int(round(fps * secs))
        ang = 86.5
        for _ in range(n):
            ang = pid.step(e, 1.0 / fps)
        return abs(ang - 86.5)

    t15, t30, t9 = integral_term(15), integral_term(30), integral_term(9)
    assert abs(t15 - 0.3) < 1e-9, f"15fps 下 1px·s 的积分动作应当是 0.3°，实际 {t15}"
    assert abs(t15 - t30) < 1e-9 and abs(t15 - t9) < 1e-9, \
        f"积分动作随帧率漂了：15fps={t15:.4f} 30fps={t30:.4f} 9fps={t9:.4f}"
    # 参考实现的逐帧口径在 30fps 下积分动作翻倍（这就是被修掉的病）
    legacy = PidRef(kp=0.0, ki=0.02, kd=0.0, limit_deg=90.0, smooth=1.0, sign=-1.0,
                    center=86.5)
    ang = 86.5
    for _ in range(30):
        ang = legacy.step(1.0)
    assert abs(abs(ang - 86.5) - 2 * t15) < 1e-9, "老路径在 30fps 下积分动作确实是 2 倍"


def test_pid_derivative_term_is_rate_invariant():
    """微分：同样的**物理角速度**下，微分项与帧率无关（隔离 kp/ki，只留 kd）。

    e(t) = 200·sin(2π·0.5t) → de/dt 峰值 = 200·π ≈ 628px/s（大于钳位 300px/s 的部分被钳住，
    所以这里只看 |de/dt| < 300px/s 的段）。
    """
    def deriv_term(fps, dt_mode, t_end=1.0, amp=80.0, f=0.4):
        pid = PidRef(kp=0.0, ki=0.0, kd=0.12, limit_deg=90.0, smooth=1.0,
                     sign=-1.0, center=86.5)
        n = int(round(fps * t_end))
        out = []
        for i in range(1, n + 1):
            t = i / fps
            e = amp * np.sin(2 * np.pi * f * t)
            out.append(abs(pid.step(e, (1.0 / fps) if dt_mode else None) - 86.5))
        return float(np.max(out))

    # 同样的物理轨迹：dt 路径在 15/30/9fps 下微分项一致
    # （残留 ~0.1° 是**后向差分离散化**本身的误差：9fps 采样间隔粗、sin 的差商偏小；
    #   要排除的是"按 1/FPS 缩放"那种 2 倍级漂移 —— 见下面的老路径对照）
    d15 = deriv_term(15, True)
    d30 = deriv_term(30, True)
    d9 = deriv_term(9, True)
    assert max(d15, d30, d9) - min(d15, d30, d9) < 0.15, \
        f"微分项随帧率漂了：15fps={d15:.3f}° 30fps={d30:.3f}° 9fps={d9:.3f}°"
    # 老路径：30fps 的阻尼只有 15fps 的一半（等效连续阻尼 = kd/FPS）
    l15 = deriv_term(15, False)
    l30 = deriv_term(30, False)
    assert abs(l30 - l15 / 2.0) < 0.05, f"老路径 30fps 阻尼应当减半：{l15:.3f} → {l30:.3f}"
    # dt 路径复现的是**登记口径 15fps 的阻尼**（换帧率不换手感）
    assert abs(d15 - l15) < 0.15 and abs(d30 - l15) < 0.15, \
        f"dt 路径应当保持 15fps 手感：d15={d15:.3f} d30={d30:.3f} 老15={l15:.3f}"


def test_pid_deriv_clamp_is_px_per_second():
    """Δe 钳位折算成"px/s"：一帧内误差跳 200px，微分项被钳在 kd×DERIV_CLAMP_PX 之内。"""
    for dt in (1.0 / 15.0, 1.0 / 30.0, 1.0 / 9.0):
        pid = PidRef(kp=0.0, ki=0.0, kd=0.12, limit_deg=90.0, smooth=1.0,
                     sign=-1.0, center=86.5)
        pid.step(0.0, dt)                       # 建立上一帧
        ang = pid.step(200.0, dt)               # 一帧跳 200px（远超钳位）
        term = abs(ang - 86.5)
        assert abs(term - 0.12 * DERIV_CLAMP_PX) < 1e-6, \
            f"dt={dt:.4f} 钳位不等于登记口径的 kd×{DERIV_CLAMP_PX}：{term}"


def test_pid_smooth_alpha_keeps_time_constant():
    """输出平滑的时间常数与帧率无关：15fps 一步 ≈ 30fps 两步的等效衰减。"""
    a = PidRef(kp=0.0, ki=0.0, kd=0.0, limit_deg=90.0, smooth=0.7, sign=1.0, center=86.5)
    assert abs(a._smooth_alpha(1.0 / 15.0) - 0.7) < 1e-12, "15fps 下必须就是登记的 0.7"
    # 30fps 下用 0.452，两步的残留 = (1-0.452)² = 0.3 = 15fps 一步的残留
    assert abs(a._smooth_alpha(1.0 / 30.0) - (1.0 - np.sqrt(0.3))) < 1e-9
    assert abs(a._smooth_alpha(1.0 / 9.0) - (1.0 - 0.3 ** (15.0 / 9.0))) < 1e-9


def test_pid_frame_equivalent_matches_registry_math():
    """frame_equivalent 是给日志/现场看的"逐帧等效值"：15fps = 登记值，30fps 是 ki 半/kd 双。"""
    pid = PidRef(kp=0.15, ki=0.02, kd=0.12)
    ki15, kd15 = pid.frame_equivalent(15.0)
    assert abs(ki15 - 0.02) < 1e-12 and abs(kd15 - 0.12) < 1e-12, \
        f"15fps 的等效值必须就是登记值，实际 {ki15} {kd15}"
    ki30, kd30 = pid.frame_equivalent(30.0)
    assert abs(ki30 - 0.01) < 1e-12 and abs(kd30 - 0.24) < 1e-12
    assert pid.ki_per_s == 0.02 * REF_FPS and abs(pid.kd_per_s - 0.12 / REF_FPS) < 1e-12
    assert abs(pid.integral_limit_s - INTEGRAL_LIMIT / REF_FPS) < 1e-12


def test_pid_ref_fps_is_configurable():
    """登记口径可改（--fps-ref）：登记成 30fps 时，dt=1/30 直接就是"逐帧口径"。"""
    pid = PidRef(kp=0.15, ki=0.01, kd=0.12, limit_deg=15.0, smooth=1.0, sign=-1.0,
                 center=86.5, ref_fps=30.0)
    pid.step(0.0, 1.0 / 30.0)
    ang = pid.step(60.0, 1.0 / 30.0)
    legacy = PidRef(kp=0.15, ki=0.01, kd=0.12, limit_deg=15.0, smooth=1.0, sign=-1.0,
                    center=86.5)
    legacy.step(0.0)
    legacy_ang = legacy.step(60.0)
    assert abs(ang - legacy_ang) < 1e-9, f"登记 30fps 时 dt=1/30 应与逐帧一致：{ang} vs {legacy_ang}"


# ---------------------------------------------------------------- 帧数类参数
def test_error_filter_window_scales_with_fps():
    """滤波窗口/拒收次数按秒语义折算；折算结果不变时 set_fps 是幂等的。"""
    f = ErrorFilter()
    assert (f.window, f.max_rejects) == (5, 4), "默认就是 15fps 登记口径"
    f.set_fps(30.0)
    assert (f.window, f.max_rejects) == (10, 8), f"30fps 应折算成 10/8，实际 {f.window}/{f.max_rejects}"
    f.set_fps(30.0)                       # 幂等：值不变就不动 deque
    assert (f.window, f.max_rejects) == (10, 8)
    f.set_fps(9.0)
    assert (f.window, f.max_rejects) == (3, 2), f"9fps 应折算成 3/2，实际 {f.window}/{f.max_rejects}"
    # 重建 deque 不能丢已入队的样本
    g = ErrorFilter()
    for v in (0.0, 10.0, 20.0):
        g.update(v)
    g.set_fps(30.0)
    assert len(g._history) == 3, "折算窗口后历史样本不该被清掉"
    assert g.last is not None


def test_error_filter_time_constant_is_fps_invariant():
    """同一段物理误差序列：30fps 的滤波输出应当≈15fps 的（窗口翻倍 = 时间不变）。"""
    base = np.concatenate([np.full(20, 0.0), np.full(40, 60.0), np.full(20, 0.0)])
    def run(fps):
        f = ErrorFilter(fps=fps)
        # 用同样的**物理**序列：15fps 的点在 30fps 里重复一次（零阶保持采样）
        out = []
        for v in base:
            out.append(f.update(float(v)))
            if fps > 20:
                out.append(f.update(float(v)))
        return np.asarray(out)
    o15, o30 = run(15), run(30)
    assert np.max(np.abs(o15 - o30[1::2])) < 12.0, "滤波时间常数随帧率漂了（30fps 应当≈15fps）"


def test_detector_width_prior_window_scales_with_fps():
    """半宽先验窗口：15fps → 15 帧（1.0s），30fps → 30 帧（还是 1.0s）。"""
    assert LaneRefDetector().width_prior.window == 15
    assert LaneRefDetector(fps=30.0).width_prior.window == 30
    assert LaneRefDetector(fps=9.0).width_prior.window == 9


def test_guards_need_time_span_not_sample_count():
    """护栏的样本门槛改成时间跨度：高帧率下"攒了 6 帧"不再是判据。"""
    # 30fps：20 个大误差样本只有 0.63s 跨度 → 占比判据不许触发（窗口 1.5s 的一半 = 0.75s）
    # hold_s 给个大值 → 隔离出"占比判据"这一条（连续计时那条本来就不会触发）
    g = BigErrorGuard(err_px=60.0, hold_s=100.0, frac_window_s=1.5, frac=0.6)
    fired = False
    for i in range(20):
        fired = g.update((i + 1) / 30.0, 100.0, 100.0)
    assert not fired, "0.63s 的观测不足以用占比判据判跑偏"
    # 攒够 0.75s（23 帧 @30fps）→ 必须触发
    for i in range(20, 30):
        fired = g.update((i + 1) / 30.0, 100.0, 100.0) or fired
    assert fired, "攒够时间跨度后占比判据必须能触发（保护不能被帧率修正关掉）"

    # OscillationGuard：同样两条（时间跨度不够 → 不判；够了 → 判）
    o = OscillationGuard(window_s=2.0, flips=3, amp_deg=12.0)
    for i, a in enumerate([75.0, 105.0, 75.0, 105.0, 75.0]):
        assert not o.update((i + 1) / 30.0, a)
    fired = False
    for i in range(5, 30):
        fired = o.update((i + 1) / 30.0, 75.0 if i % 2 else 105.0) or fired
    assert fired


def test_runner_csv_and_replay_contracts():
    """跑车脚本的契约：CSV 必须带 fps/相位/舵角/丢线时长等列（复盘与离线统计都吃这张表）。

    2026-10-14：契约来源从已归档的 `lane_ref_test.py` 换成最终脚本
    `scripts/lane_final_test.py`（逐行内容契约在 tests/test_lane_final.py 另有用例钉住）。
    """
    from scripts.lane_final_test import CSV_HEADER
    cols = CSV_HEADER.splitlines()[0].split(",")
    for need in ("t", "phase", "center_x", "error_px", "error_filt", "esc_at_us", "esc_us",
                 "angle_deg", "left_x", "right_x", "quality", "both_sides",
                 "since_valid_s", "tag", "fps"):
        if need == "esc_at_us":
            continue          # 旧列名兼容占位（老表叫 esc_us，新表一致）
        assert need in cols, f"CSV 缺列 {need}：{cols}"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"  ✅ {fn.__name__}")
    print(f"test_lane_rate: all {len(fns)} passed")
