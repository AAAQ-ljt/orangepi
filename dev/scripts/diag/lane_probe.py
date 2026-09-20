#!/usr/bin/env python3
"""扫线诊断探针：把"扫线到底看到了什么"摊开给人看，用于现场调参。

它做四件事（都不驱动电机）：

1. 打印**逐行白线位置**：整幅图从上到下每 8 行，白点在哪些 x（左/右候选），
   —— 直接回答"白线在画面的哪个高度范围"，ROI（`LANE_ROI_TOP_RATIO`）该设多少一眼就看得出来；
2. 打印**连通域表**：面积最大的若干个连通域的 长/宽/面积/厚度/细长比/是否被判为"线"，
   —— 用来核对 `LANE_LINE_MIN_ELONG`、`LANE_LINE_MIN_LEN_PX` 这两个形状阈值；
3. 打印**不同 ROI 候选下的扫线结果**（center / confidence / 有效行数），
   —— 一眼看出"哪个 ROI 才是真的锁在两条线上"；
4. 存图：原图 + 叠加图（绿=通过形状过滤的掩膜，红=被过滤掉的候选，黄=扫线中心，品红=target_x）。

两种用法：

    # 车上：抓 N 帧并分析（会临时停掉两路推流，退出自动恢复）
    sudo python3 /root/dev/scripts/diag/lane_probe.py --camera 2 --frames 6

    # 本地：分析已经抓下来的图（不需要摄像头，可在 Windows 上跑）
    python scripts/diag/lane_probe.py --analyze /root/dev/logs/lane_probe

抓下来的图用 `python scripts/ssh_get.py -r <远端目录> .` 拉回本地分析。
"""
from __future__ import annotations

import argparse
import os
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import cv2
import numpy as np

from config import settings

ROI_CANDIDATES = (0.0, 0.15, 0.25, 0.35, 0.45, 0.55)


# ------------------------------------------------------------------ 自带白线掩膜
# 2026-09-19：旧的 lane_scan 模块已归档到 dev/attic/，本探针自带实现（只做诊断用）。
def white_mask(frame_bgr, roi_top_ratio=None, roi_bottom_margin=None,
               apply_shape_filter=True):
    """低饱和 + 自适应亮度的白线掩膜；返回 (掩膜, (roi_y0, roi_y1))。"""
    top = settings.LANE_ROI_TOP_RATIO if roi_top_ratio is None else roi_top_ratio
    bot = settings.LANE_ROI_BOTTOM_MARGIN if roi_bottom_margin is None else roi_bottom_margin
    h, w = frame_bgr.shape[:2]
    y0 = max(0, int(h * top))
    y1 = min(h, h - int(bot))
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    s_chan, v_chan = hsv[:, :, 1], hsv[:, :, 2]
    roi_v = v_chan[y0:y1, :]
    if roi_v.size:
        v_med = float(np.percentile(roi_v, 50))
        v_hi = float(np.percentile(roi_v, 95))
        thr = max(settings.LANE_WHITE_V_MIN,
                  v_med + settings.LANE_WHITE_V_SPLIT * (v_hi - v_med))
    else:
        thr = float(settings.LANE_WHITE_V_MIN)
    mask = ((s_chan <= settings.LANE_WHITE_S_MAX) & (v_chan >= thr)).astype(np.uint8) * 255
    mask[:y0, :] = 0
    mask[y1:, :] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    open_px = int(getattr(settings, "LANE_MASK_OPEN_PX", 3))
    if open_px >= 3:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_px, open_px)))
    if apply_shape_filter:
        n, labels, stats, _c = cv2.connectedComponentsWithStats(mask, 8)
        keep = np.zeros_like(mask)
        for i in range(1, n):
            _x, _yy, bw, bh, area = stats[i]
            length = max(int(bw), int(bh))
            if length < settings.LANE_LINE_MIN_LEN_PX:
                continue
            thick = area / float(max(1, length))
            if thick > settings.LANE_LINE_MAX_THICK_PX:
                continue
            if length / max(1e-6, thick) >= settings.LANE_LINE_MIN_ELONG:
                keep[labels == i] = 255
        mask = keep
    return mask, (y0, y1)


def line_like(bw: int, bh: int, area: int) -> bool:
    """连通域"像不像线"（探针表里显示用）。"""
    length = max(int(bw), int(bh))
    if length < settings.LANE_LINE_MIN_LEN_PX:
        return False
    thick = area / float(max(1, length))
    return (thick <= settings.LANE_LINE_MAX_THICK_PX
            and length / max(1e-6, thick) >= settings.LANE_LINE_MIN_ELONG)


def _components(mask: np.ndarray):
    n, _labels, stats, _cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = (int(v) for v in stats[i])
        length = max(w, h)
        thickness = area / float(max(1, length))
        out.append({"xywh": (x, y, w, h), "area": area, "length": length,
                    "thickness": round(thickness, 1),
                    "elong": round(length / max(1e-6, thickness), 2),
                    "kept": line_like(w, h, area)})
    out.sort(key=lambda c: -c["area"])
    return out


def _row_map(frame: np.ndarray, step: int = 8, cols: int = 64) -> None:
    """逐行打印白点位置（左/右候选），回答'白线在画面哪个高度'。"""
    raw, (roi_y0, roi_y1) = white_mask(frame, apply_shape_filter=False)
    kept, _ = white_mask(frame, apply_shape_filter=True)
    h, w = raw.shape
    print(f"    逐行白点（每 {step} 行；ROI 当前 = y {roi_y0}~{roi_y1}，"
          f"列宽 {w // cols}px/字符；x=过滤后仍保留 . =被形状过滤丢弃）")
    for y in range(0, h, step):
        row_raw, row_kept = raw[y] > 0, kept[y] > 0
        if not row_raw.any():
            continue
        strip = []
        for c in range(cols):
            seg = slice(c * (w // cols), (c + 1) * (w // cols))
            strip.append("x" if bool(row_kept[seg].any()) else
                         ("." if bool(row_raw[seg].any()) else " "))
        lt = [i for i in range(cols) if strip[i] != " "]
        print(f"      y={y:3d} 白点数={int(row_raw.sum()):3d} 保留={int(row_kept.sum()):3d} "
              f"跨度=[{lt[0] * (w // cols)},{lt[-1] * (w // cols)}]  |{''.join(strip)}|")


def _print_components(frame: np.ndarray, top: int = 8) -> None:
    raw, _ = white_mask(frame, apply_shape_filter=False)
    comps = _components(raw)
    print(f"    连通域（过滤前共 {len(comps)} 个，按面积取前 {top}；判据：长度≥"
          f"{settings.LANE_LINE_MIN_LEN_PX} 且 细长比≥{settings.LANE_LINE_MIN_ELONG} "
          f"且 厚度≤{settings.LANE_LINE_MAX_THICK_PX}）")
    print("      #   x    y    w    h   面积   长度  厚度  细长比  判定")
    for i, c in enumerate(comps[:top]):
        x, y, w, h = c["xywh"]
        print(f"      {i:<3} {x:<4} {y:<4} {w:<4} {h:<4} {c['area']:<6} {c['length']:<5} "
              f"{c['thickness']:<5} {c['elong']:<6} {'✅线' if c['kept'] else '❌丢弃'}")
    if not comps:
        print("      （一个连通域都没有：白线判据（亮度/饱和度阈值）根本没抓到东西）")


def _print_roi_sweep(frame: np.ndarray) -> None:
    """不同 ROI 下的扫线读数（需要扫线模块；已归档就跳过，本探针仍给掩膜诊断）。"""
    if _scan(frame) is None:
        print("    不同 ROI 下的扫线结果：跳过（旧扫线模块已归档到 dev/attic/，"
              "循迹请用 scripts/lane_ref_test.py --image）")
        return
    print("    不同 ROI 下的扫线结果（看哪个才是真锁在线上）")
    print("      ROI_TOP  y范围       中心   置信度  有效行   左   右   宽度")
    for ratio in ROI_CANDIDATES:
        obs = _scan(frame)
        lw = (obs.right_x - obs.left_x) if (obs.left_x is not None and obs.right_x is not None) else None
        mark = " ←当前" if abs(ratio - settings.LANE_ROI_TOP_RATIO) < 1e-6 else ""
        print(f"      {ratio:<8} {int(frame.shape[0] * ratio):>4}~{frame.shape[0] - settings.LANE_ROI_BOTTOM_MARGIN:<6}"
              f"{obs.center_x:7.1f} {obs.confidence:7.2f} {obs.valid_rows:6d} "
              f"{str(obs.left_x):>5} {str(obs.right_x):>5} {str(lw):>6}{mark}")


def _annotate(frame: np.ndarray, name: str | None = None, out_dir: str | None = None) -> np.ndarray:
    raw, (roi_y0, roi_y1) = white_mask(frame, apply_shape_filter=False)
    kept, _ = white_mask(frame, apply_shape_filter=True)
    vis = frame.copy()
    vis[(kept > 0)] = (0, 255, 0)                      # 保留（判为线）
    vis[(raw > 0) & (kept == 0)] = (0, 0, 255)         # 被形状过滤丢弃
    vis = cv2.addWeighted(vis, 0.55, frame, 0.45, 0)

    # 扫线结果（蓝=拟合出的左线，红=右线，黄=前瞻带里逐行的车道中心）
    obs = _scan(frame)
    if obs is not None and obs.left_x is not None and obs.right_x is not None:
        y_mid = (roi_y0 + roi_y1) // 2
        cv2.line(vis, (int(obs.left_x), y_mid), (int(obs.left_x), roi_y1), (255, 80, 0), 2)
        cv2.line(vis, (int(obs.right_x), y_mid), (int(obs.right_x), roi_y1), (0, 0, 255), 2)
        cv2.circle(vis, (int(obs.center_x), y_mid), 4, (0, 255, 255), -1)
        cv2.putText(vis, f"center={obs.center_x:.0f}", (int(obs.center_x) - 40, y_mid - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    cv2.line(vis, (0, roi_y0), (frame.shape[1], roi_y0), (255, 128, 0), 1)
    cv2.line(vis, (0, roi_y1), (frame.shape[1], roi_y1), (255, 128, 0), 1)
    obs = _scan(frame)
    cv2.line(vis, (int(settings.TARGET_X), 0), (int(settings.TARGET_X), frame.shape[0]),
             (255, 0, 255), 1)
    cv2.line(vis, (int(obs.center_x), 0), (int(obs.center_x), frame.shape[0]), (0, 255, 255), 1)
    cv2.putText(vis, f"center={obs.center_x:.0f} conf={obs.confidence:.2f} "
                     f"L={obs.left_x} R={obs.right_x} segs={obs.valid_rows}",
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    if out_dir:
        cv2.imwrite(os.path.join(out_dir, f"annotated_{name}.jpg"), vis)
        cv2.imwrite(os.path.join(out_dir, f"frame_{name}.jpg"), frame)
    return vis


def angle_advice(frame: np.ndarray) -> str:
    """根据白点分布给出**该怎么调镜头**的具体建议（这是最常需要的结论）。

    判据来自实车复盘：
      · 白点集中在画面上半部（远处）+ 下半部几乎没有 → 镜头太"朝前"，要往下压；
      · 白点贴着画面左右边缘、成片被裁 → 镜头太朝前/视场太窄，近处车道比画面还宽；
      · 白点只在画面最下方一条窄带 → 视场只看到车头前一点点，要么压角度要么退后安装。
    """
    raw, (roi_y0, roi_y1) = white_mask(frame, apply_shape_filter=False)
    h, w = raw.shape
    ys, xs = np.nonzero(raw)
    if ys.size < 50:
        return "画面里几乎没有白点：检查曝光/光照，或白线是否在视野内"
    y_med = float(np.median(ys))
    edge = float(np.mean((xs <= 3) | (xs >= w - 4)))
    lower = float(np.mean(ys > 0.6 * h))
    if y_med < 0.4 * h and lower < 0.05:
        return ("白点集中在**画面上半部**（远处），近处地面看不到 → 镜头太朝前，"
                "**把下摄往下压**，直到两条白线出现在画面下半部")
    if edge > 0.35:
        return ("白点大量贴着**画面左右边缘**（被裁掉）→ 视场里近处车道比画面还宽，"
                "**把下摄往下压**（或换更广角镜头），让两条白线完整进画面")

    # 2026-09-19 实车最典型的一种：左右线各自成段、y 范围完全不重叠
    return ("白点在画面里但不成对/不连续 → 先按上面的逐行白点图确认白线的确切位置，"
            "再决定是压角度还是调 LANE_ROI_TOP_RATIO")


def _scan(frame):
    """有扫线模块就用它，没有就返回 None（探针仍能给出掩膜诊断）。"""
    try:
        from vision.lane_scan import LaneScanner
    except ImportError:
        return None
    return _scan(frame)


def verdict_of(frame: np.ndarray) -> tuple:
    """(是否可用, 结论文本, 左右跟踪行数) —— 终端与叠加图共用同一套判据。"""
    obs = _scan(frame)
    if obs is None:
        return (False, "（无扫线模块）本轮只做掩膜诊断：看上面的逐行白点图与连通域表", (0, 0))
    ok = obs.confidence >= 0.4
    if ok:
        text = (f"✅ 可用：两侧各 {obs.valid_rows} 条支持线段，"
                f"L={obs.left_x} R={obs.right_x} conf={obs.confidence:.2f}")
    else:
        text = (f"❌ 没配到车道线（conf={obs.confidence:.2f}，支持线段 {obs.valid_rows}）"
                f"→ 看上面的『逐行白点图』确认白线在哪、斜率对不对；"
                f"必要时调 LANE_SLOPE_MIN/MAX 或 ROI")
    return ok, text, (obs.valid_rows, obs.valid_rows)


def analyze_frame(frame: np.ndarray, name: str, out_dir: str | None = None, verbose: bool = True) -> tuple:
    if verbose:
        print(f"  ---- {name}  ({frame.shape[1]}x{frame.shape[0]}) ----")
        _row_map(frame)
        _print_components(frame)
        _print_roi_sweep(frame)
    ok, text, _rows = verdict_of(frame)
    raw, _roi = white_mask(frame, apply_shape_filter=False)
    kept, _ = white_mask(frame, apply_shape_filter=True)
    pct_kept = float((kept > 0).mean()) * 100.0
    _o = _scan(frame)
    conf = float(_o.confidence) if _o is not None else 0.0
    print(f"    白像素占比：过滤前 {float((raw > 0).mean()) * 100:.1f}% → 过滤后 {pct_kept:.1f}%"
          f"（占比高但判定不可用 = 那些白都在反光/地面上）")
    print(f"    判定：{text}")
    print(f"    镜头建议：{angle_advice(frame)}")
    if out_dir:
        _annotate(frame, name, out_dir)
        print(f"    存图：{os.path.join(out_dir, 'annotated_' + name + '.jpg')}")
    return ok, text, _rows, pct_kept, conf


def sweep_tilt(camera: int, tilts=None, frames: int = 3, step_s: float = 1.2) -> int:
    """**自动扫云台仰角，逐档给扫线打分**（解决"人猜角度猜不中"）。

    只动云台（CH3），电调始终保持中位 1500us —— 车不会动。
    对每个角度：等云台到位 → 抓 frames 帧 → 报告 conf / 左右跟踪行数 / 配对行数，
    最后按分数排序给出**建议写进 config/site.yaml 的巡线仰角**。
    """
    from scripts.bench_common import MotorSession, install_signal_guard, restore_remote_stage
    from vision.camera_guard import camera_exclusive, open_camera

    if tilts is None:
        # 从"最朝前"到"最朝下"都试一遍；两边的机械行程都覆盖，避免搞不清正负方向
        tilts = [90, 105, 75, 120, 60, 135, 45, 140, 40]
    tilts = [t for t in tilts if settings.GIMBAL_TILT_MIN <= t <= settings.GIMBAL_TILT_MAX]

    state = {"stopped": False, "driver": None, "restored": False, "used_motor": True}
    install_signal_guard(state)
    results = []
    print("[SWEEP] 只动云台（电调保持中位，车不会动）；每档抓 %d 帧" % frames)
    try:
        with MotorSession(state, enabled=True, speed_us_max=settings.ESC_CREEP_US) as pca:
            with camera_exclusive():
                cap = open_camera(camera, settings.IMG_W, settings.IMG_H)
                if cap is None:
                    return 1
                try:
                    for tilt in tilts:
                        pca.set_tilt_angle(tilt)
                        time.sleep(step_s)                     # 等云台停稳
                        for _ in range(4):                     # 丢掉到位瞬间的旧帧
                            cap.read()
                        best_conf, rows = 0.0, (0, 0)
                        for _ in range(frames):
                            ok, frame = cap.read()
                            if not ok or frame is None:
                                continue
                            obs = _scan(frame)
                            if obs.confidence >= best_conf:
                                best_conf, rows = obs.confidence, (obs.valid_rows, obs.valid_rows)
                        results.append((best_conf, tilt, rows))
                        mark = "✅" if best_conf >= 0.4 else ("~" if best_conf >= 0.2 else " ")
                        print(f"[SWEEP]   tilt={tilt:3d}°  conf={best_conf:.2f}  "
                              f"左{rows[0]:3d}行 右{rows[1]:3d}行  {mark}")
                finally:
                    # 扫完把云台回中位：推流画面立刻恢复正常朝向，别把车留在"对着地面"的状态
                    try:
                        pca.set_tilt_angle(settings.GIMBAL_TILT_CENTER)
                        pca.set_pan_angle(settings.GIMBAL_PAN_CENTER)
                        time.sleep(step_s)
                        print(f"[SWEEP] 云台已回到中位（tilt={settings.GIMBAL_TILT_CENTER}°，"
                              f"推流画面恢复正常朝向）")
                    except Exception as exc:
                        print(f"[SWEEP] ⚠️ 云台回中位失败：{exc}")
                    cap.release()
    finally:
        restore_remote_stage(state, "扫角度结束")

    if not results:
        print("[SWEEP] 没有结果")
        return 1
    results.sort(reverse=True)
    best_conf, best_tilt, rows = results[0]
    print("")
    print("[SWEEP] ===== 结果（按 conf 排序）=====")
    for conf, tilt, rows in results:
        print(f"[SWEEP]   tilt={tilt:3d}°  conf={conf:.2f}  左{rows[0]}行 右{rows[1]}行")
    if best_conf >= 0.4:
        print(f"[SWEEP] ✅ 建议巡线仰角 = {best_tilt}°（conf={best_conf:.2f}）")
        print("[SWEEP]    写入方式（之后所有程序自动读）：")
        print(f"[SWEEP]    echo 'gimbal_tilt_lane: {best_tilt}' >> /root/dev/config/site.yaml")
        print("[SWEEP]    或由 AI/人 直接把 config/site.yaml 的 gimbal_tilt_lane 写进去")
    else:
        print(f"[SWEEP] ⚠️ 最好的也只有 conf={best_conf:.2f}（<0.4）——这一路摄像头/这个位置还是不行，"
              f"换 --camera 0/2 再扫一次，或把车摆到正常赛道上再扫")
    return 0


def live_view(camera: int, quiet: bool = False) -> int:
    """实时预览（需要 X11；本车目前没装 X 服务器，用浏览器看图传 + 下面的数字模式即可）。

    目标画面（决定了扫线能不能工作）：
      · 两条白线从画面**下半部**向中间上方收拢（成 V），左右大致对称；
      · 绿点（被判为线的掩膜）压在两条白线上，蓝/红点（连续跟踪）跟着它们走；
      · 顶部状态行显示 `OK 双侧跟踪`、conf ≥ 0.4；
      · 画面中间那一片"反光"即使还是白的，也不该被跟到（跟踪会跳过它）。
    按 q / ESC 退出。
    """
    from vision.camera_guard import camera_exclusive, open_camera

    if not os.environ.get("DISPLAY"):
        print("[PROBE] 没有 DISPLAY（本车没装 X 服务器）：改用不需要画面的两招——")
        print("[PROBE]   · 调角度：用浏览器看你平时那路图传（副摄 cam_car0027_sub），"
              "把两条白线调进画面下半部、左右对称")
        print("[PROBE]   · 看数字：不带 --show 跑本脚本，终端会直接给出『可用/线不足』的结论 + 叠加图")
        return 2

    print("[PROBE] 实时预览：把两条白线调到画面下半部、左右对称（V 字），按 q 退出")
    with camera_exclusive():
        cap = open_camera(camera, settings.IMG_W, settings.IMG_H)
        if cap is None:
            return 1
        n = 0
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue
            vis = _annotate(frame)
            cv2.imshow("lane_probe (q=quit)", vis)
            n += 1
            if n % 15 == 0 and not quiet:
                print(f"[PROBE] 第 {n} 帧：见上图状态行")
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        cap.release()
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="扫线诊断探针（不驱动电机）")
    ap.add_argument("--camera", type=int, default=2, help="摄像头（默认 2=下摄，巡线用）")
    ap.add_argument("--frames", type=int, default=6, help="抓几帧（默认 6）")
    ap.add_argument("--out", default="/root/dev/logs/lane_probe", help="存图目录")
    ap.add_argument("--analyze", default="", help="只分析已有图片（本地跑，不需要摄像头）")
    ap.add_argument("--show", action="store_true",
                    help="实时预览窗口（需要 X11；本车没装 X 服务器，用浏览器看图传 + 本脚本的数字/叠加图即可）")
    ap.add_argument("--sweep-tilt", action="store_true",
                    help="自动扫云台仰角并逐档给扫线打分（只动云台，车不会动；用来找巡线视角）")
    ap.add_argument("--tilt", type=float, default=None,
                    help="先把云台设到这个仰角再分析（需要配合 --frames；只动云台）")
    ap.add_argument("--tilts", default="",
                    help="--sweep-tilt 的角度列表，逗号分隔（默认 90,105,75,120,60,135,45,140,40）")
    ap.add_argument("--save-annotated", action="store_true",
                    help="离线分析时也输出叠加图（绿=判为线，红=被形状过滤丢弃，黄=中心，品红=target_x）")
    ap.add_argument("--quiet", action="store_true", help="只存图不打印明细")
    ap.add_argument("--allow-motion", action="store_true",
                    help="允许动云台舵机（--sweep-tilt/--tilt 需要；电调始终中位，车不会走）")
    args = ap.parse_args()

    if args.show:
        return live_view(args.camera, quiet=args.quiet)

    if args.sweep_tilt or args.tilt is not None:
        if not args.allow_motion:
            print("[PROBE] 这个模式会动云台舵机（电调保持中位、车不会走），请加 --allow-motion 确认")
            return 2
        tilts = ([float(args.tilt)] if args.tilt is not None else
                 ([float(x) for x in args.tilts.split(",") if x.strip()] if args.tilts else None))
        return sweep_tilt(args.camera, tilts=tilts,
                          frames=max(1, args.frames if args.tilt is not None else 3))

    if args.analyze:
        files = sorted(f for f in os.listdir(args.analyze)
                       if f.lower().endswith((".jpg", ".png")) and not f.startswith("annotated_"))
        if not files:
            print(f"[PROBE] {args.analyze} 里没有图片")
            return 1
        print(f"[PROBE] 离线分析 {len(files)} 张")
        for f in files:
            img = cv2.imread(os.path.join(args.analyze, f))
            if img is None:
                continue
            name = os.path.splitext(f)[0]
            analyze_frame(img, name, out_dir=(args.out if args.save_annotated else None),
                          verbose=not args.quiet)
        return 0

    os.makedirs(args.out, exist_ok=True)
    from vision.camera_guard import camera_exclusive, open_camera

    print(f"[PROBE] 打开摄像头 {args.camera}（会临时停掉两路推流，退出自动恢复）")
    good = 0
    last = ("", (0, 0))
    kept_pcts: list = []
    confs: list = []
    with camera_exclusive():
        cap = open_camera(args.camera, settings.IMG_W, settings.IMG_H)
        if cap is None:
            return 1
        for i in range(args.frames):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            ok_v, text, rows, pct_kept, conf_v = analyze_frame(frame, f"{i:02d}", out_dir=args.out,
                                                               verbose=not args.quiet)
            good += 1 if ok_v else 0
            last = (text, rows)
            kept_pcts.append(pct_kept)
            confs.append(conf_v)
        cap.release()

    avg_kept = (sum(kept_pcts) / len(kept_pcts)) if kept_pcts else 0.0
    best_conf = max(confs) if confs else 0.0
    print("")
    print(f"[PROBE] ===== 结论：{args.frames} 帧里 {good} 帧可用 =====")
    print(f"[PROBE] 对比用读数：白像素占比（过滤后）均值 {avg_kept:.1f}%，最好置信度 {best_conf:.2f}"
          f"   ← 开灯/关灯/换角度前后各跑一次，比这两个数最直接")
    print(f"[PROBE] 最后一帧：{last[0]}")
    if good == 0:
        print("[PROBE] 下一步（按顺序试，别一上来就调阈值）：")
        print("[PROBE]   1) 看『白像素占比（过滤后）』：若 >3% 却判定不可用，说明白都在**反光/地面**上 → "
              "先解决**光照/覆膜反光**：关掉正对跑道的直射灯、把跑道挪开灯的正下方、或在灯下挂一层柔光纸；"
              "若表面是后覆的亮膜，换成哑光膜")
        print("[PROBE]   2) 再跑一次本命令，对比两个数：**白像素占比应明显下降、最好置信度应上升**")
        print("[PROBE]   3) 若白线在画面里但仍没被跟到 → 看『逐行白点图』量它的亮度/饱和度"
              "（白线只比赛道亮约 25 级，反光能亮 40 级），据此调 LANE_WHITE_* 阈值")
        print("[PROBE]   4) 只有确认白线**根本不在画面里**（近处被裁）时，才考虑压镜头/换广角")
    print(f"[PROBE] 图在 {args.out}（拉回本地看：python scripts/ssh_get.py -r {args.out} .）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
