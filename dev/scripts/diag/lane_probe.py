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
from vision.lane_scan import LaneScanner, line_like, trace_boundary, white_mask

ROI_CANDIDATES = (0.0, 0.15, 0.25, 0.35, 0.45, 0.55)


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
    print("    不同 ROI 下的扫线结果（看哪个才是真锁在线上）")
    print("      ROI_TOP  y范围       中心   置信度  有效行   左   右   宽度")
    for ratio in ROI_CANDIDATES:
        sc = LaneScanner(roi_top_ratio=ratio)
        obs = sc.scan(frame)
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

    # 连续跟踪到的左右边界（蓝=左，红=右）—— 调相机角度时就看这两条能不能压在真白线上
    left = trace_boundary(kept, roi_y1, roi_y0, "left")
    right = trace_boundary(kept, roi_y1, roi_y0, "right")
    for y, x, _w in left:
        cv2.circle(vis, (x, y), 1, (255, 80, 0), -1)
    for y, x, _w in right:
        cv2.circle(vis, (x, y), 1, (0, 0, 255), -1)

    cv2.line(vis, (0, roi_y0), (frame.shape[1], roi_y0), (255, 128, 0), 1)
    cv2.line(vis, (0, roi_y1), (frame.shape[1], roi_y1), (255, 128, 0), 1)
    obs = LaneScanner().scan(frame)
    cv2.line(vis, (int(settings.TARGET_X), 0), (int(settings.TARGET_X), frame.shape[0]),
             (255, 0, 255), 1)
    cv2.line(vis, (int(obs.center_x), 0), (int(obs.center_x), frame.shape[0]), (0, 255, 255), 1)
    cv2.putText(vis, f"center={obs.center_x:.0f} conf={obs.confidence:.2f} "
                     f"left={len(left)}row right={len(right)}row",
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    if out_dir:
        cv2.imwrite(os.path.join(out_dir, f"annotated_{name}.jpg"), vis)
        cv2.imwrite(os.path.join(out_dir, f"frame_{name}.jpg"), frame)
    return vis


def verdict_of(frame: np.ndarray) -> tuple:
    """(是否可用, 结论文本, 左右跟踪行数) —— 终端与叠加图共用同一套判据。"""
    mask, (roi_y0, roi_y1) = white_mask(frame)
    left = trace_boundary(mask, roi_y1, roi_y0, "left")
    right = trace_boundary(mask, roi_y1, roi_y0, "right")
    obs = LaneScanner().scan(frame)
    ok = obs.confidence >= 0.4
    if ok:
        text = f"✅ 可用：双侧跟踪 左{len(left)}行 右{len(right)}行 conf={obs.confidence:.2f}"
    elif len(left) < 10 and len(right) < 10:
        text = (f"❌ 线不在画面里（左{len(left)}行/右{len(right)}行 conf={obs.confidence:.2f}）"
                f"→ **先调下摄俯仰角**：让两条白线进画面下半部，别调阈值")
    else:
        text = (f"⚠️ 只跟到零碎边（左{len(left)}行/右{len(right)}行 conf={obs.confidence:.2f}）"
                f"→ 多半是锁在地面反光/边缘上了，看上面逐行白点图确认白线位置")
    return ok, text, (len(left), len(right))


def analyze_frame(frame: np.ndarray, name: str, out_dir: str | None = None, verbose: bool = True) -> tuple:
    if verbose:
        print(f"  ---- {name}  ({frame.shape[1]}x{frame.shape[0]}) ----")
        _row_map(frame)
        _print_components(frame)
        _print_roi_sweep(frame)
    _ok, text, _rows = verdict_of(frame)
    print(f"    判定：{text}")
    if out_dir:
        _annotate(frame, name, out_dir)
        print(f"    存图：{os.path.join(out_dir, 'annotated_' + name + '.jpg')}")
    return verdict_of(frame)


def live_view(camera: int, quiet: bool = False) -> int:
    """实时预览（需要 X11；本车目前没装 X 服务器，用浏览器看图传 + 下面的数字模式即可）。

    目标画面（决定了扫线能不能工作）：
      · 两条白线从画面**下半部**向中间上方收拢（成 V），左右大致对称；
      · 绿点（被判为线的掩膜）压在两条白线上，蓝/红点（连续跟踪）跟着它们走；
      · 顶部状态行显示 `OK 双侧跟踪`、conf ≥ 0.4；
      · 画面中间那一片"反光"即使还是白的，也不该被跟到（跟踪会跳过它）。
    按 q / ESC 退出。
    """
    from vision.camera_guard import camera_exclusive

    if not os.environ.get("DISPLAY"):
        print("[PROBE] 没有 DISPLAY（本车没装 X 服务器）：改用不需要画面的两招——")
        print("[PROBE]   · 调角度：用浏览器看你平时那路图传（副摄 cam_car0027_sub），"
              "把两条白线调进画面下半部、左右对称")
        print("[PROBE]   · 看数字：不带 --show 跑本脚本，终端会直接给出『可用/线不足』的结论 + 叠加图")
        return 2

    print("[PROBE] 实时预览：把两条白线调到画面下半部、左右对称（V 字），按 q 退出")
    with camera_exclusive():
        cap = cv2.VideoCapture(camera, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings.IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.IMG_H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print("[PROBE] 摄像头打不开")
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
    ap.add_argument("--save-annotated", action="store_true",
                    help="离线分析时也输出叠加图（绿=判为线，红=被形状过滤丢弃，黄=中心，品红=target_x）")
    ap.add_argument("--quiet", action="store_true", help="只存图不打印明细")
    args = ap.parse_args()

    if args.show:
        return live_view(args.camera, quiet=args.quiet)

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
    from vision.camera_guard import camera_exclusive

    print(f"[PROBE] 打开摄像头 {args.camera}（会临时停掉两路推流，退出自动恢复）")
    good = 0
    last = ("", (0, 0))
    with camera_exclusive():
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings.IMG_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.IMG_H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print("[PROBE] 摄像头打不开（推流是否已恢复？）")
            return 1
        for i in range(args.frames):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            ok_v, text, rows = analyze_frame(frame, f"{i:02d}", out_dir=args.out,
                                             verbose=not args.quiet)
            good += 1 if ok_v else 0
            last = (text, rows)
        cap.release()

    print(f"\n[PROBE] ===== 结论：{args.frames} 帧里 {good} 帧可用 =====")
    print(f"[PROBE] 最后一帧：{last[0]}")
    if good == 0:
        print("[PROBE] 下一步（按顺序试）：")
        print("[PROBE]   1) **调下摄俯仰角**：让两条白线进入画面下半部、左右大致对称（用浏览器看图传，"
              "或看存下来的 annotated_*.jpg）")
        print("[PROBE]   2) 若白线已在画面里但仍判不可用 → 看上面的『逐行白点图』确认白线的高度范围，"
              "再按『不同 ROI』那栏把 settings.LANE_ROI_TOP_RATIO 调到合适的值")
        print("[PROBE]   3) 若白线根本找不到 → 看『连通域表』：白线是不是被亮度阈值漏掉了"
              "（LANE_WHITE_V_MIN / LANE_WHITE_S_MAX）")
    print(f"[PROBE] 图在 {args.out}（拉回本地看：python scripts/ssh_get.py -r {args.out} .）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
