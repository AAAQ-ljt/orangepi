#!/usr/bin/env python3
"""对比"官方 2026 室外赛 demo 的 ROI"与"我们现场标定的 ROI"到底谁看得到白线。

起因（2026-10-08）：官方新资料里放出了本届室外赛的完整 demo
（`比赛资料/5G远程驾驶无人车(2026)学习资料/竞赛进阶案例/14.…/demo(中文注释).cpp`），
它的 ROI 是**画面底部 180 行**：

    ROI_Y1=300, ROI_Y2=480     → 比例 0.625 ~ 1.0
    SCAN_Y_START=0, SCAN_Y_END=100（相对 ROI，即绝对 y 300~400）→ 前瞻带 0.625 ~ 0.833

而我们 `config/settings.py` 现在标定的是 `LANE_ROI_TOP_RATIO=0.35` /
`LANE_ROI_BOTTOM_MARGIN=200`（→ y 168~280），前瞻带 0.35~0.48（y 168~230）。
**两者几乎不重叠**：我们看画面中段，官方看画面底部。这不是参数微调量级的差异。

本脚本不猜，直接在真实图片上把两套 ROI 跑同一台检测器（`vision/lane_ref.py`，同一份
Canny+Hough 链路），用团队自己的 `score_frames()` 客观指标打分，并额外打印
"白线到底在画面的哪几行"的行剖面 —— 这样"该把 ROI 放哪"是有数据支撑的结论，
而不是又一次拍脑袋。

用法（本地，不需要摄像头/硬件）：

    PY="E:/venvs/smartcar-ultra/Scripts/python.exe"
    cd dev && PYTHONPATH=. "$PY" scripts/diag/ref_roi_probe.py \
        --images "../logs_pull/*.jpg" \
        --images "../比赛资料/5G远程驾驶无人车(2026)学习资料/竞赛进阶案例/12.图像处理-数据集获取/图像处理-数据集获取/captured_photos/*.jpg"

    # 只看行剖面（解释"为什么"）：
    ... --profile-only --profile-every 20
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import List, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import cv2
import numpy as np

from config import settings
from vision.lane_ref import LaneParams, LaneRefDetector, score_frames, sweep_bands

# ---------------------------------------------------------------- 官方 demo 的 ROI
# demo(中文注释).cpp: ROI_Y1=300, ROI_Y2=480, SCAN_Y_START=0, SCAN_Y_END=100（相对 ROI）
OFFICIAL_ROI_TOP = 300 / 480.0
OFFICIAL_ROI_BOTTOM = 1.0
OFFICIAL_LOOK_TOP = 300 / 480.0
OFFICIAL_LOOK_BOTTOM = 400 / 480.0


def official_params() -> LaneParams:
    """完全照官方 demo 的 ROI 与前瞻带（斜率/阈值/Hough 参数与我们同源）。"""
    p = LaneParams.from_settings()
    p.roi_top_ratio = OFFICIAL_ROI_TOP
    p.roi_bottom_ratio = OFFICIAL_ROI_BOTTOM
    p.look_top_ratio = OFFICIAL_LOOK_TOP
    p.look_bottom_ratio = OFFICIAL_LOOK_BOTTOM
    return p


def ours_params() -> LaneParams:
    return LaneParams.from_settings()


def official_roi_our_look() -> LaneParams:
    """隔离变量：只把 ROI 换成官方的，前瞻带仍用我们的。"""
    p = official_params()
    p.look_top_ratio = float(settings.LANE_ROI_TOP_RATIO)
    p.look_bottom_ratio = 1.0 - float(settings.LANE_ROI_BOTTOM_MARGIN) / float(settings.IMG_H)
    return p


def our_roi_official_look() -> LaneParams:
    """反向隔离：ROI 用我们的，前瞻带用官方的比例 —— ⚠️ 前瞻带在 ROI 之外，
    中心是拟合线**外推**到 y≈350 处的值，不是常规用法，只当参考读数看。"""
    p = ours_params()
    p.look_top_ratio = OFFICIAL_LOOK_TOP
    p.look_bottom_ratio = OFFICIAL_LOOK_BOTTOM
    return p


CONFIGS = [
    ("ours (现场标定)", ours_params),
    ("official (底部180行)", official_params),
    ("official ROI+our look", official_roi_our_look),
    ("our ROI+外推到官方前瞻带", our_roi_official_look),
]


# ---------------------------------------------------------------- 白线行剖面
def imread_unicode(path: str):
    """Windows 版 cv2.imread 遇到非 ASCII（中文）路径会直接失败（本机实测：
    PIL 可读、cv2 返回 None），用 np.fromfile + imdecode 绕开，两种路径通吃。"""
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def white_line_profile(frame: np.ndarray, s_max: int = 70, v_min: int = 120):
    """逐行找"白线候选"并归类成左右簇 —— 直接回答"白线在画面的哪几行"。

    判据沿用 settings 的白线定义：低饱和 + 高亮（比同图中位数亮）。
    返回 [(y, n_clusters, xs_left, xs_right, pair_w)]，pair_w 为左右簇中心距。
    """
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    s_chan, v_chan = hsv[:, :, 1], hsv[:, :, 2]
    v_med = float(np.median(v_chan))
    v_p95 = float(np.percentile(v_chan, 95))
    # 与 settings.LANE_WHITE_* 同一套思路：在"地面底色"与"更亮的线"之间自动分界
    thr = max(float(v_min), v_med + float(settings.LANE_WHITE_V_SPLIT) * (v_p95 - v_med))
    mask = (s_chan <= s_max) & (v_chan >= thr)
    rows = []
    for y in range(0, h):
        xs = np.flatnonzero(mask[y])
        if xs.size == 0:
            rows.append((y, 0, None, None, None))
            continue
        # 把该行白点聚成簇（间隔 > 24px 算不同簇）
        gaps = np.flatnonzero(np.diff(xs) > 24)
        groups = np.split(xs, gaps + 1)
        centers = [float(g.mean()) for g in groups if g.size >= 2]
        if len(centers) >= 2:
            lo, hi = min(centers), max(centers)
            rows.append((y, len(centers), lo, hi, hi - lo))
        else:
            rows.append((y, len(centers), centers[0] if centers else None, None, None))
    return rows


def report_profile(images: Sequence[str], every: int = 20, limit: int = 3) -> None:
    print("\n" + "=" * 100)
    print("白线行剖面（每 every 行采样：n=该行白线簇数，L/R=左右簇中心x，W=L-R）")
    print("  判据：在**同一张图内**自适应分界（与 settings.LANE_WHITE_* 同思路），低饱和+高亮")
    print("=" * 100)
    for path in list(images)[:limit]:
        img = imread_unicode(path)
        if img is None:
            continue
        h, w = img.shape[:2]
        rows = white_line_profile(img)
        pairs = [r for r in rows if r[1] >= 2]
        print(f"\n--- {os.path.basename(path)}  {w}x{h}  "
              f"双侧可见行数={len(pairs)}/{h}  "
              f"y范围={('-', '-') if not pairs else (min(r[0] for r in pairs), max(r[0] for r in pairs))}")
        for y, n, lx, rx, pw in rows:
            if y % every:
                continue
            band = ""
            if OFFICIAL_ROI_TOP * h <= y < OFFICIAL_ROI_BOTTOM * h:
                band += " [官方ROI]"
            if float(settings.LANE_ROI_TOP_RATIO) * h <= y < (h - float(settings.LANE_ROI_BOTTOM_MARGIN)):
                band += " [我ROI]"
            print(f"  y={y:4d} n={n}{band:16s} "
                  f"L={'--' if lx is None else f'{lx:6.1f}'} "
                  f"R={'--' if rx is None else f'{rx:6.1f}'} "
                  f"W={'--' if pw is None else f'{pw:6.1f}'}")


# ---------------------------------------------------------------- 主流程
def main() -> int:
    ap = argparse.ArgumentParser(description="官方 ROI vs 我们 ROI 的白线可见性对比")
    ap.add_argument("--images", action="append", default=None,
                    help="图片 glob，可重复给多次（相对路径按当前工作目录解析）")
    ap.add_argument("--every", type=int, default=20, help="行剖面采样间隔")
    ap.add_argument("--max-frames", type=int, default=40, help="每套图片最多用多少帧打分")
    ap.add_argument("--profile-only", action="store_true", help="只打行剖面，不跑检测器")
    ap.add_argument("--profile-every", type=int, default=20)
    ap.add_argument("--profile-limit", type=int, default=3)
    ap.add_argument("--sweep", action="store_true", help="额外跑一次全 ROI 档位扫描（慢）")
    args = ap.parse_args()

    globs = args.images or [
        "../logs_pull/*.jpg",
        "../logs_pull_probe/frame_*.jpg",
    ]
    images: List[str] = []
    for g in globs:
        images.extend(sorted(glob.glob(g)))
    if not images:
        print("没有匹配到任何图片，检查 --images 的路径。", file=sys.stderr)
        return 2

    print(f"共载入 {len(images)} 张图")

    if args.profile_only:
        report_profile(images, args.profile_every, args.profile_limit)
        return 0

# 按来源分组：官方给的图 与 我们自己拍的图，结论必须分开看
    # ⚠️ 注意："中心跨距"是**全部帧**的 center 极差，而采集图往往混了不同阶段
    # （acquire/track/hold/stop），跨距大 ≠ 帧间抖动大，只说明"这些帧中心分布广"。
    groups: List[Tuple[str, List[str]]] = []
    off = [p for p in images if "captured_photos" in p]
    our = [p for p in images if "captured_photos" not in p]
    if our:
        groups.append(("我们自己的图", our))
    if off:
        groups.append(("官方资料里的图", off))

    for gname, paths in groups:
        frames = []
        for p in paths[: args.max_frames]:
            img = imread_unicode(p)
            if img is None:
                continue
            if (img.shape[1], img.shape[0]) != (settings.IMG_W, settings.IMG_H):
                img = cv2.resize(img, (settings.IMG_W, settings.IMG_H))
            frames.append(img)
        if not frames:
            continue
        print("\n" + "=" * 100)
        print(f"【{gname}】{len(frames)} 帧   "
              f"我们ROI=y{int(settings.LANE_ROI_TOP_RATIO*480)}~{480-int(settings.LANE_ROI_BOTTOM_MARGIN)}  "
              f"官方ROI=y{int(OFFICIAL_ROI_TOP*480)}~480")
        print("=" * 100)
        print(f"{'配置':<24} {'成对帧':>7} {'最长连续行':>10} {'支持率':>8} "
              f"{'线段中位':>9} {'中心跨距':>9} {'q中位':>7}")
        rows = []
        for label, mk in CONFIGS:
            p = mk()
            det = LaneRefDetector(LaneParams(**p.__dict__))
            st = score_frames(det, frames)
            rows.append((label, st))
            print(f"{label:<24} {st['paired_frames']:>4}/{st['n_frames']:<3} "
                  f"{st['run_max']:>10.1f} {st['support_frac']*100:>7.1f}% "
                  f"{st['segs_med']:>9.1f} {st['spread_px']:>9.1f} {st['q_med']:>7.2f}")

        best = max(rows, key=lambda r: (r[1]["paired_frames"], r[1]["run_max"]))
        print(f"→ 成对帧最多的是：{best[0]}")

        if args.sweep:
            print("\n--- 全 ROI 档位扫描（含官方档）---")
            base = ours_params()
            cands = [(float(base.roi_top_ratio), float(base.roi_bottom_ratio)),
                     (OFFICIAL_ROI_TOP, OFFICIAL_ROI_BOTTOM)]
            res = sweep_bands(frames, base, candidates=cands, max_frames=10)
            print(f"{'ROI 上沿':>9} {'ROI 下沿':>9} {'y范围':>14} {'成对帧':>7} "
                  f"{'最长连续行':>10} {'支持率':>8} {'中心跨距':>9}")
            for s in res:
                mark = " ★官方" if abs(s["roi_top_ratio"] - OFFICIAL_ROI_TOP) < 1e-6 else ""
                mark += " ←当前" if abs(s["roi_top_ratio"] - float(base.roi_top_ratio)) < 1e-6 else ""
                print(f"{s['roi_top_ratio']:>9.3f} {s['roi_bottom_ratio']:>9.3f} "
                      f"{str(s['y_range']):>14} {s['paired_frames']:>4}/{s['n_frames']:<3}"
                      f"{s['run_max']:>10.1f} {s['support_frac']*100:>7.1f}% "
                      f"{s['spread_px']:>9.1f}{mark}")

    report_profile(images, args.every, args.profile_limit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
