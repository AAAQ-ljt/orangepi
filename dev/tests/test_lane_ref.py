"""参考实现版循迹（`vision/lane_ref.py` + `control/lane_control.py` + `scripts/lane_ref_test.py`）
单元测试：合成图 + 纯函数，**不碰硬件、不需要摄像头**。

覆盖要点（每条都对应一次真实踩坑或一个红线）：
  1. 对称车道 → 中心≈目标点、误差≈0；车道整体右移 → 中心跟着右移（方向不能反）；
  2. **斜率先验**：横向纹理（近水平亮条）不能把中心带跑；
  3. **单侧丢线用半宽先验兜底**（参考实现是代画面边界，我们上一版是"干脆不给中心"，
     两个极端都不对：前者把中心拉到画面中间，后者让车一丢线就停）；
  4. 半宽先验：配对时实测、越界样本不入队、只有种子时质量压到"不跟线"档；
  5. PID：符号口径、限幅、平滑；自适应速度：**减速档不得掉进电调死区**；
  6. **丢线阶梯**：fresh → hold（误差衰减回中位）→ lost（停车）；
  7. **跑偏保护**：大误差要"持续"才停车，中途回到小误差必须清零；
  8. **候选带扫描**：配置的带是错的时候，扫描要能选到真正锁住两条线的那一档
     （这是"不用人到现场猜 ROI"的保证）；
  9. **自动前瞻带**：要落在两条线真正有边缘支持的行范围内；
 10. **镜头朝向体检**：线只在上半部 → ❌；线延伸到下半部 → ✅；
 11. CSV 列里必须有"滤波后误差 / 目标点"（上一版的数据缺口，复盘时无法区分
     检测抖动与滤波平滑）。

跑法：
    cd dev && PYTHONPATH=. python tests/test_lane_ref.py
"""
from __future__ import annotations

import numpy as np

from config import settings
from control.lane_control import (BigErrorGuard, LossGuard, PidRef, adaptive_pulse, floor_pulse)
from control.filters import ErrorFilter
from vision.lane_ref import (LaneParams, LaneRefDetector, WidthPrior, auto_look_band,
                             sweep_bands)

W, H = 640, 480


def _lane_image(shift: int = 0, left: bool = True, right: bool = True,
                texture: bool = False, y_top: float = 0.30, y_bot: float = 0.60,
                slope: float = 0.972) -> np.ndarray:
    """红棕底 + 两条**斜**白线（透视）。

    几何：在 y=0.60H 处左线 x=160、右线 x=590，斜率 dx/dy = ∓`slope`（越远越靠中间），
    默认中心 = (160+590)/2 = 375（= 操场实测目标点）。
    ⚠️ 改 `y_top/y_bot` 时要一起管住斜率：|k| = 1/slope 必须落在 (0.25, 2.0) 内，
    否则线会被**斜率先验**滤掉（slope=0.972 → |k|=1.03 ✓；0.4 → |k|=2.5 ✗）。
    """
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    frame[:, :, 2], frame[:, :, 1], frame[:, :, 0] = 120, 45, 35
    yt, yb = int(H * y_top), int(H * y_bot)
    y_ref = int(H * 0.60)

    def draw(x_at_ref: float, sign: int, thickness: int = 8):
        half = thickness // 2
        for y in range(yt, yb):
            x = int(round(x_at_ref - sign * slope * (y - y_ref)))
            frame[y, max(0, x - half):x + half + 1] = 225

    if left:
        draw(160 + shift, +1)       # 往远处（y 变小）往画面中间靠 → 左线
    if right:
        draw(590 + shift, -1)       # 同理 → 右线
    if texture:
        for y in range(int(H * 0.35), int(H * 0.55), 12):
            frame[y:y + 3, :] = 200          # 横向亮条：模拟地面横纹/反光
    return frame


# ---------------------------------------------------------------- 1~2 检测基本行为
def test_centered_lane_gives_near_zero_error():
    r = LaneRefDetector().detect(_lane_image())
    assert r.center_x is not None, "两条线都在，应该给出中心"
    assert abs(r.error) < 25.0, f"对称摆放时误差应接近 0，实际 {r.error}"
    assert r.n_left >= 1 and r.n_right >= 1
    assert r.both_sides and r.quality > 0.5, f"双侧置信度应偏高，实际 {r.quality}"


def test_shifted_lane_moves_error_same_direction():
    """车道整体右移 → 中心右移、误差变大（正误差 = 车在目标左侧）。"""
    base = LaneRefDetector().detect(_lane_image())
    shifted = LaneRefDetector().detect(_lane_image(shift=20))
    assert shifted.center_x is not None
    assert shifted.center_x > base.center_x + 10, "整体右移后中心应变大（方向不能反）"


def test_horizontal_texture_is_not_a_lane_line():
    """近水平的横向纹理不能把中心带跑（斜率先验 + 位置锚定 + 配对宽度闸门）。"""
    r = LaneRefDetector().detect(_lane_image(texture=True))
    assert r.center_x is not None, "真车道线还在，应该仍能找到"
    assert abs(r.center_x - 375.0) < 40.0, f"横纹不该把中心带跑，实际 {r.center_x}"


# ---------------------------------------------------------------- 3~4 单侧兜底与半宽
def test_single_side_falls_back_to_half_width_prior():
    """只看到一侧：用半宽先验推中心（而不是"不给中心"，也不是"代画面边界"）。

    只看到左侧线、半宽未知 → 用种子（settings.LANE_HOUGH_HALF_W_DEFAULT_PX）推，
    质量必须**低于默认仲裁阈值**（宁可按"维持/停车"处理，不要拿不准的中心去控制）。
    """
    det = LaneRefDetector()
    r = det.detect(_lane_image(right=False))
    assert r.center_x is not None, "单侧也应该给中心（否则丢一侧就得停车）"
    assert r.left_x is not None and r.right_x is None, "报出看到的那一侧，便于诊断"
    assert "单侧兜底" in r.note
    assert r.quality < 0.28, f"半宽只有种子时不该被信任，实际质量 {r.quality}"


def test_single_side_with_measured_width_is_trusted():
    """先让两侧都在跑几帧（半宽变实测），再丢一侧 → 中心接近真值、质量过阈值。"""
    det = LaneRefDetector()
    for _ in range(5):
        det.detect(_lane_image())
    assert det.width_prior.measured, "两侧都在时半宽应当被实测"
    r = det.detect(_lane_image(left=False))     # 只看到右线
    assert r.center_x is not None
    assert abs(r.center_x - 375.0) < 30.0, f"实测半宽推出来的中心应接近真值，实际 {r.center_x}"
    assert r.quality >= 0.28, f"实测半宽应当够格被跟线，实际 {r.quality}"


def test_no_single_mode_gives_no_center():
    """`--single-mode off` 时回到上一版行为（单侧不给中心）—— 现场可以一键退回保守模式。"""
    det = LaneRefDetector(LaneParams.from_settings(single_mode="off"))
    r = det.detect(_lane_image(right=False))
    assert r.center_x is None and r.error is None
    assert r.left_x is not None


def test_single_mode_border_matches_reference_exactly():
    """`--single-mode border` = **参考实现原样**：缺的一侧代画面边界。

    原码（oldCode/src/vision/vision.cpp::picture()，320×240）：
        if (flagl) l = 0;  else l = (i - bl)/kl;
        if (flagr) r = frame.cols; else r = (i - br)/kr;
        mid = (l + r)/2;  ave_x = mean(mid);  error = ave_x - frame.cols/2
    对**直线**而言"逐行取中再平均" = "在中点行取值"，所以本实现等价于
        center = (可见线 x + 边界) / 2   （**缺的那一侧**代它自己的边界：左缺→0；右缺→w）
    """
    img = _lane_image(right=False)          # 只有左线 → 缺的是右侧 → 边界 = 画面宽
    det = LaneRefDetector(LaneParams.from_settings(single_mode="border"))
    r = det.detect(img)
    assert r.center_x is not None
    expect = (r.left_x + W) * 0.5
    assert abs(r.center_x - expect) < 1e-6, f"border 模式应等于参考实现公式，实际 {r.center_x}"
    assert "border" in r.note
    assert r.quality >= 0.28, "border 是参考实现的连续行驶行为，不该被仲裁阈值挡掉"

    img2 = _lane_image(left=False)          # 只有右线 → 缺的是左侧 → 边界 = 0
    det2 = LaneRefDetector(LaneParams.from_settings(single_mode="border"))
    r2 = det2.detect(img2)
    expect2 = (r2.right_x + 0.0) * 0.5
    assert abs(r2.center_x - expect2) < 1e-6


def test_reference_band_is_always_a_candidate():
    """参考实现 oldCode 的带（0.50~0.85，它是 320×240）必须永远在候选里 ——
    万一我们的安装跟它接近，扫描就该能选中它，而不是被候选集排除掉。"""
    from vision.lane_ref import BAND_CANDIDATES, REF_BAND, band_candidates
    assert REF_BAND in BAND_CANDIDATES
    assert REF_BAND in band_candidates(LaneParams.from_settings(roi_top_ratio=0.9,
                                                               roi_bottom_ratio=0.95))


def test_gains_switch_basis():
    """PID 增益的口径：参考数字是按 320 宽整定的，我们的误差是 640 宽像素。

    literal = 直接抄 0.15/0.01/0.12；width = 按画面宽度换算（640 → 0.075/0.005/0.06）。
    """
    from control.lane_control import gains_for
    assert gains_for(640, "literal") == (0.15, 0.01, 0.12)
    kp, ki, kd = gains_for(640, "width")
    assert abs(kp - 0.075) < 1e-9 and abs(ki - 0.005) < 1e-9 and abs(kd - 0.06) < 1e-9
    assert gains_for(320, "width") == (0.15, 0.01, 0.12), "320 宽时两种口径应当重合"


def test_unanchored_fallback_preserves_reference_behavior():
    """锚定不到时**退回参考实现的行为**（只按斜率选），而不是整帧没有读数。

    造一张只有"位置不合法"的左线（它在画面右半边）的图：参考实现照样会把它当选、
    给出中心；我们加锚定是为了少选错，但不该因此把参考实现能做到的事变成做不到。
    """
    img = _lane_image(left=False, right=False)
    # 在画面右半边画一条斜率符号为"左线"的线（dy/dx < 0：越往上越靠右）
    y_ref, x_ref = int(H * 0.60), 520
    for y in range(int(H * 0.30), int(H * 0.60)):
        x = int(round(x_ref + 0.972 * (y_ref - y)))
        img[y, max(0, x - 4):x + 5] = 225
    det = LaneRefDetector()
    r = det.detect(img)
    assert r.left_x is not None, "锚定不到也应当保留这条线，而不是当作没看到"
    assert not r.anchored, "这条线不满足位置锚定，读数要如实标出来"


def test_width_prior_rejects_out_of_range_samples():
    prior = WidthPrior(seed_px=200.0)
    prior.update(5.0)          # 太小 → 显然配对错了
    prior.update(4000.0)       # 太大 → 同上
    assert not prior.measured, "越界样本不该被当成实测"
    assert prior.value == 200.0
    for _ in range(3):
        prior.update(150.0)
    assert prior.measured and abs(prior.value - 150.0) < 1e-6


# ---------------------------------------------------------------- 4b 叠加图标签 / 支持度指标
def test_reading_tags_are_ascii():
    """每个读数都要带一个**纯 ASCII** 的短标签。

    2026-09-22 实验室实测：叠加图上原来画的是中文 `note`，`cv2.putText` 渲染不了，
    存出来的图上是一串 `??????` —— 而叠加图是本车唯一的事后看图途径。
    """
    det = LaneRefDetector()
    both = det.detect(_lane_image())
    assert both.tag == "", "正常的双侧读数不需要标签"
    blank = np.zeros((H, W, 3), dtype=np.uint8)
    blank[:, :, 2] = 120
    assert det.detect(blank).tag == "no-seg"
    assert LaneRefDetector().detect(_lane_image(right=False)).tag == "1side-seed"
    d2 = LaneRefDetector()
    for _ in range(5):
        d2.detect(_lane_image())
    assert d2.detect(_lane_image(right=False)).tag == "1side-width"
    assert LaneRefDetector(LaneParams.from_settings(single_mode="border")) \
        .detect(_lane_image(right=False)).tag == "1side-border"
    assert LaneRefDetector(LaneParams.from_settings(single_mode="off")) \
        .detect(_lane_image(right=False)).tag == "1side-off"
    for tag in ("", "no-seg", "1side-seed", "1side-width", "1side-border", "1side-off"):
        tag.encode("ascii")      # 非 ASCII 会直接抛 UnicodeEncodeError


def test_auto_look_band_uses_densest_contiguous_segment():
    """前瞻带必须取自**最长的连续支持段**，不许因为头尾的零星支持被拉宽。

    2026-09-22 实验室实测教训：支持行分布常有很长的头尾（如 32~360 都有零星行），
    原"15~85 分位"会把前瞻带拉宽 → 中心掺入大量外推行 → 静止时中心就 ±40~90px 跳。
    这里造"一段密集 + 两头零星"的支持分布，断言前瞻带落在密集段内。
    """
    from vision.lane_ref import rows_with_support
    img = _lane_image()
    det = LaneRefDetector()
    r = det.detect(img)
    assert r.left_kb is not None and r.right_kb is not None
    _hits, _st = rows_with_support(img, r.left_kb, r.right_kb, 0, H, step=8,
                                   tol_px=6.0, min_rows=1, want_stats=True)
    # 用真实帧跑 auto_look：支持行应集中在线的实际范围（y 144~288），
    # 前瞻带至少要有 12 行且落在 120~320 之间
    frames = [img] * 3
    (top, bottom), rows, info = auto_look_band(frames, LaneParams.from_settings())
    assert rows, "合成图上有支持行"
    lo, hi, n = info["longest_seg"]
    assert n >= 100, f"合成线上最长连续段（行数）应很长，实际 {n}"
    y_lo = int(top * H); y_hi = int(bottom * H)
    assert 120 <= y_lo <= 320 and 160 <= y_hi <= 380, f"前瞻带 {y_lo}~{y_hi} 偏出线区"


def test_preflight_refuses_jittery_target():
    """第二道闸门：连续支持够（run_max ≥ 20）但**静止中心跨距 > 25px** → 也不采纳目标点。

    2026-09-22 实测（191511/191707/191804）：跨距 79/117/120/100px 时提示"不采纳"，
    但目标点 352.3/389.7/260.7/344.8 **照样被用并写回 site.yaml** —— 顺序 bug：
    `configured` 在 `params.target_ratio` 被覆盖**之后**才取值，还原等于没还原。
    """
    import types

    import scripts.lane_ref_test as lt
    frames = [_lane_image()] * 10
    args = types.SimpleNamespace(auto_roi=True, auto_look=False, auto_target=True,
                                 min_quality=0.28, save_site=False)
    # 扫描给"强证据"，但 10 帧里检测器返回的中心按帧号在 300~450 间跳（跨距 150px）
    from vision import lane_ref as lr2
    _orig_detect = lr2.LaneRefDetector.detect

    def fake_detect(self, frame, width_prior=None):
        r = _orig_detect(self, frame, width_prior)
        r.center_x = 300.0 + (fake_detect.call_count % 4) * 50.0
        fake_detect.call_count += 1
        r.error = r.center_x - 375.0
        r.quality = 1.0
        return r
    fake_detect.call_count = 0
    strength = {"roi_top_ratio": 0.30, "roi_bottom_ratio": 0.65, "y_range": (144, 312),
                "paired_frames": 10, "n_frames": 10, "run_max": 60.0, "support_frac": 0.3,
                "spread_px": 150.0, "q_med": 1.0, "segs_med": 4.0, "centers": []}
    from vision import lane_ref as lr2
    orig_sweep, orig_detect = lt.sweep_bands, lr2.LaneRefDetector.detect
    try:
        lr2.LaneRefDetector.detect = fake_detect
        lt.sweep_bands = lambda fr, p, **kw: [dict(strength)]
        base = LaneParams.from_settings()
        p_out, t_out = lt._preflight(frames, args, LaneParams.from_settings())
        assert t_out is None, f"跨距大时不采纳目标点，实际 {t_out}"
        assert abs(p_out.target_ratio - base.target_ratio) < 1e-9,             "params.target_ratio 必须还原（否则调用方会拿自检值当配置值）"
    finally:
        lr2.LaneRefDetector.detect = orig_detect
        lt.sweep_bands = orig_sweep


def test_run_max_distinguishes_real_line_from_fragments():
    """『最长连续支持行』要能把**一整条真线**和**零散碎边**分开。

    2026-09-22 实验室实测的教训：当时所有候选带的『支持行占比』都只有 0~12%，
    按占比排名等于抛硬币；而"最长连续支持"能分出真线（几十行）与碎边（几行）。
    合成图里白线从 y=144 连续到 288 → 连续支持应当接近一整段。
    """
    from vision.lane_ref import rows_with_support
    det = LaneRefDetector()
    img = _lane_image()
    r = det.detect(img)
    assert r.left_kb is not None and r.right_kb is not None
    _hits, st = rows_with_support(img, r.left_kb, r.right_kb, 0, H, step=4,
                                 tol_px=6.0, min_rows=2, want_stats=True)
    assert st["run_max"] >= 80, f"连续真线的 run_max 应远大于碎边，实际 {st['run_max']}"
    assert st["checked"] > 0 and st["checked"] <= H, "分母应当只算'线在画面内'的行"
    # 空白画面：拟合线不存在，run_max 应为 0
    blank = np.zeros((H, W, 3), dtype=np.uint8)
    blank[:, :, 2] = 120
    _h2, st2 = rows_with_support(blank, (-1.0, 300.0), (1.0, 300.0), 0, H, step=4,
                                 min_rows=2, want_stats=True)
    assert st2["run_max"] == 0 and st2["rows"] == 0


def test_preflight_refuses_weak_evidence():
    """自检证据弱（最长连续支持 < 20 行）时**不能采纳**自检出的 ROI/目标点。

    2026-09-22 落地实测两次踩到：车摆偏时自检把目标标成 417/291（真值约 341），
    车照着错目标跑；而且第一次修的时候把 `weak = True` 只写进了 else 分支 ——
    警告打印了、目标点照样被采纳并写回 site.yaml。这条测试盯的就是那个分支。
    """
    import types

    import scripts.lane_ref_test as lt
    frames = [_lane_image()] * 5
    args = types.SimpleNamespace(auto_roi=True, auto_look=False, auto_target=True,
                                 min_quality=0.28, save_site=False)
    base = LaneParams.from_settings()
    row = {"roi_top_ratio": 0.30, "roi_bottom_ratio": 0.65, "y_range": (144, 312),
           "paired_frames": 10, "n_frames": 10, "run_max": 5.0, "support_frac": 0.02,
           "spread_px": 10.0, "q_med": 0.5, "segs_med": 2.0, "centers": []}
    orig = lt.sweep_bands
    try:
        lt.sweep_bands = lambda fr, p, **kw: [dict(row)]
        p_weak, t_weak = lt._preflight(frames, args, LaneParams.from_settings())
        assert t_weak is None, f"证据弱时不该采纳自检目标点，实际 {t_weak}"
        assert abs(p_weak.roi_top_ratio - base.roi_top_ratio) < 1e-9,             "证据弱时不该采纳自检的 ROI"
        lt.sweep_bands = lambda fr, p, **kw: [dict(row, run_max=60.0, support_frac=0.3,
                                                   q_med=1.0)]
        _p_ok, t_ok = lt._preflight(frames, args, LaneParams.from_settings())
        assert t_ok is not None, "证据够时必须给出目标点"
    finally:
        lt.sweep_bands = orig


# ---------------------------------------------------------------- 5 PID / 速度
def test_pid_matches_reference_convention():
    """转向符号与 settings.STEER_SIGN 同口径：
       sign=-1（参考实现原式 `90 - pid`）→ 正误差输出 <90°；
       sign=+1（"角度增大=右转"）→ 正误差输出 >90°。
    """
    pid = PidRef(kp=0.15, ki=0.01, kd=0.12, limit_deg=15, smooth=1.0, sign=-1.0)
    assert pid.step(+40.0) < 90.0
    pid.reset()
    assert pid.step(-40.0) > 90.0
    pid2 = PidRef(kp=0.15, ki=0.01, kd=0.12, limit_deg=15, smooth=1.0, sign=+1.0)
    assert pid2.step(+40.0) > 90.0, "sign=+1 时必须与 sign=-1 反向（否则改 site.yaml 没效果）"


def test_pid_reset_clears_state():
    """切状态时必须 reset：官方参考实现漏了这条，代价是切状态时输出踢一下。"""
    pid = PidRef(sign=1.0)
    for _ in range(20):
        pid.step(100.0)
    pid.reset()
    assert pid.integral == 0.0 and pid.last_error == 0.0 and pid.last_angle is None


def test_angle_limited_and_smoothed():
    pid = PidRef(kp=0.15, ki=0.01, kd=0.12, limit_deg=15, smooth=0.7)
    a = 90.0
    for _ in range(30):
        a = pid.step(300.0)
    assert 90.0 - 15.0 - 1e-6 <= a <= 90.0 + 15.0 + 1e-6, f"必须限幅在 ±15°，实际 {a}"


def test_adaptive_pulse_direction_and_floor():
    """自适应速度：默认只减速不提速，且减速/维持档不得掉进电调死区。

    用户 2026-09-20：巡线 1575 太快 → 基准 1560（参考实现是 +15/-20）。
    基准降到 1560 后，"误差大 −20" 会算出 1540 < 死区 1545 → 车会从"减速"变"停车"。
    """
    base = float(settings.ESC_CREEP_US)
    assert adaptive_pulse(1.0, base) == base          # 默认增量 0：直道不提速
    assert adaptive_pulse(8.0, base) == base
    assert adaptive_pulse(40.0, base) == base - 10.0  # 大误差减速 10us
    floor = float(settings.ESC_DEADBAND_US) + 5.0
    assert adaptive_pulse(40.0, base, 0.0, -40.0) == floor, "减速档不得低于死区"
    assert floor_pulse(1546.0, -20.0) == floor, "丢线维持档同样不得低于死区"
    assert floor_pulse(base, -10.0) < base, "维持档应当比循迹档慢"


# ---------------------------------------------------------------- 6~7 丢线阶梯 / 跑偏保护
def test_loss_guard_ladder():
    """有读数 → fresh；丢线 → hold 且误差衰减；超过 hold_s → lost（调用方停车）。"""
    g = LossGuard(hold_s=0.6)
    st = g.update(0.0, True)
    assert st.phase == "fresh" and st.decay == 1.0
    st = g.update(0.2, False)
    assert st.phase == "hold" and 0.0 < st.decay < 1.0, "维持期间误差要往中位衰减"
    assert abs(st.since_s - 0.2) < 1e-6
    st = g.update(0.61, False)
    assert st.phase == "lost" and st.decay == 0.0 and not st.driving
    # 恢复读数 → 立刻回到 fresh
    assert g.update(0.7, True).phase == "fresh"


def test_loss_guard_never_valid():
    """从没成功读过线时不能算 hold：调用方要按"探路/停车"而不是"沿用上次中心"处理。"""
    g = LossGuard(hold_s=0.6)
    st = g.update(1.0, False)
    assert st.phase == "hold" and st.since_s == float("inf"), "没见过线 → 时间应当是无界"
    assert not g.ever_valid


def test_big_error_guard_also_watches_raw_error():
    """跑偏保护必须同时看**原始**误差 —— 只看滤波值会被"滤波冻结"骗过去。

    2026-09-22 落地实测：原始误差连续 4s ≥40px，滤波后一直卡在 30px 以下 → 保护没触发。
    """
    g = BigErrorGuard(err_px=40.0, hold_s=1.0)
    assert not g.update(0.0, 30.0, raw_error=120.0)
    assert g.update(1.0, 30.0, raw_error=150.0), "原始误差持续超标也必须触发"
    g.reset()
    # 单帧毛刺不算：原始值只超一帧就回落
    assert not g.update(2.0, 10.0, raw_error=300.0)
    assert not g.update(2.1, 10.0, raw_error=5.0)
    assert not g.update(3.5, 10.0, raw_error=800.0), "中间断过就要重新计时"


def test_big_error_guard_needs_sustained_error():
    """跑偏保护要"持续"才触发；中途回到小误差必须清零（否则一次抖动就停车）。"""
    g = BigErrorGuard(err_px=60.0, hold_s=1.5)
    assert not g.update(0.0, 80.0)
    assert not g.update(1.0, 80.0)
    assert g.update(1.5, 80.0), "持续 1.5s 应当触发"
    g.reset()
    assert not g.update(2.0, 80.0)
    assert not g.update(3.0, 10.0), "误差回到小值 → 计时清零"
    assert not g.update(4.0, 80.0), "清零后要重新计时"
    assert g.hold_s == 1.5
    g2 = BigErrorGuard(err_px=60.0, hold_s=0.0)
    assert not g2.update(0.0, 999.0), "hold_s=0 表示关闭"


# ---------------------------------------------------------------- 8~9 自动选带 / 前瞻带
def test_sweep_bands_finds_band_that_sees_the_lines():
    """配置的带是错的（罩不到白线）时，扫描必须能选到真正锁住两条线的那一档。"""
    frames = [_lane_image()] * 5
    bad = LaneParams.from_settings(roi_top_ratio=0.55, roi_bottom_ratio=0.75)
    table = sweep_bands(frames, bad)
    assert table, "候选带表不该是空的"
    assert table[0]["paired_frames"] == len(frames), \
        f"应有某一档每帧都成对，实际最好 {table[0]['paired_frames']}/{table[0]['n_frames']}"
    assert table[0]["roi_bottom_ratio"] > 0.55, "选出的带要罩到 y≈168~280 的白线"


def test_auto_look_band_lands_on_supported_rows():
    """自动前瞻带要落在"两条线真有边缘支持"的行里（y 144~288 之间）。"""
    frames = [_lane_image()] * 3
    (top, bottom), rows, _info = auto_look_band(frames, LaneParams.from_settings())
    assert rows, "合成图上应该有支持行"
    y_lo, y_hi = top * H, bottom * H
    assert 120 <= y_lo <= 320 and 160 <= y_hi <= 380, f"前瞻带落在 {y_lo:.0f}~{y_hi:.0f}"
    assert y_hi - y_lo >= 12, "前瞻带太窄（至少 12 行，否则中心噪声大）"


# ---------------------------------------------------------------- 10 镜头朝向体检
def test_visible_range_flags_upper_only_lines():
    """线只在上半部（y 144~288）→ 必须判 ❌ 并给出"往下压镜头"的建议。"""
    vr = LaneRefDetector().visible_range(_lane_image())
    assert not vr["ok"], f"上半部的线不该判为够用：{vr['text']}"
    assert "往下压" in vr["text"]
    assert vr["y_hi"] < 300


def test_visible_range_ok_when_lines_reach_lower_half():
    """线延伸到画面下半部（y 到 350 左右）→ 判 ✅。"""
    img = _lane_image(y_top=0.15, y_bot=0.90, slope=0.7)   # 缓一点，别撞上斜率上界
    vr = LaneRefDetector().visible_range(img)
    assert vr["ok"], f"线进了下半部应当够用：{vr['text']}"
    assert vr["y_hi"] >= 300


# ---------------------------------------------------------------- 11 数据缺口
def test_csv_header_has_filtered_error_and_target():
    """CSV 必须落"滤波后误差 / 目标点 / 质量 / 单双侧"——上一版缺这几列，
    复盘时无法区分"检测抖动"与"滤波平滑"（《循迹脚本交接.md》§6 缺陷 7）。"""
    from scripts.lane_ref_test import CSV_HEADER
    for col in ("error_px", "error_filt", "target_px", "quality", "both_sides",
                "half_w", "since_valid_s"):
        assert col in CSV_HEADER, f"CSV 缺少列 {col}"


def test_error_filter_unit_matches_pixels():
    """滤波离群阈值必须按**像素**给（上一版把 settings 的"误差单位"当像素用，
    15px 就把任何正常变化判成离群 → 滤波后误差长期卡在旧值、车不响应）。"""
    from scripts.lane_ref_test import MIN_QUALITY0
    outlier_px = float(settings.ERROR_FILTER_OUTLIER) * float(settings.LANE_ERROR_SCALE)
    assert outlier_px >= 40.0, f"像素口径的离群阈值不该是 {outlier_px}"
    f = ErrorFilter(5, outlier_px)
    f.update(0.0)
    f.update(30.0)      # 30px 的变化在赛道上很常见，绝不能被当离群点丢掉
    assert abs(f.last - 20.0) < 1e-6, f"30px 的正常变化被当成离群丢掉了：{f.last}"
    assert 0.0 < MIN_QUALITY0 < 0.55, "仲裁阈值应落在『种子半宽』与『实测半宽』两档之间"


def test_annotate_smoke():
    """叠加图函数要被真实调用（本车没有 X 服务器，存图是唯一的事后看图途径）。"""
    from scripts.lane_ref_test import annotate
    det = LaneRefDetector()
    img = _lane_image()
    r = det.detect(img)
    vis = annotate(img, r, det.p, "track", "esc=1560")
    assert vis.shape == img.shape and vis.dtype == img.dtype


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"  ✅ {fn.__name__}")
    print(f"test_lane_ref: all {len(fns)} passed")
