#!/usr/bin/env python3
"""斑马线检测档**现场标定器**（离线；2026-10-16，与 `cone_calibrate.py` 同一套做法）。

为什么要有它
------------
`ZEBRA_GATE_ROI`（尤其**上沿**）是**机位相关**档：现场视频实测画面顶部有一条"**永久亮带**"
（赛道尽头 + 白色围栏/楼，行 0~75~95、单行白占比最高 0.9，**每一帧都在**）——上沿开在 ≤80 时
整片 855 帧**每帧都触发**。所以换场地/动过镜头之后，必须拿**当天的斑马线视频 + 一段空赛道**
把"横道落在画面哪几行、顶部亮带压到第几行、空赛道会不会误触发"量出来再定 ROI。

用法（本地，无车；两条命令即可决策）
------------------------------------
    PY="E:/venvs/smartcar-ultra/Scripts/python.exe"; cd dev && export PYTHONPATH=.

    "$PY" scripts/diag/zebra_calibrate.py \
        --pos ../train/opencv-level1/video/true_track_sub_video/斑马线1016.mp4 \
        --neg ../train/opencv-level1/video/true_track_sub_video/2.mp4

输出（三张表 + 两句结论）：
  ① **顶部亮带有多深**：把"从第 0 行起、连续达标的行段"逐帧量出来 → ROI 上沿必须比它更深；
  ② **闸门扫档**：每个（上沿 × 行白占比 × 最小行数）的"横道命中/触发/首帧"与"空赛道命中/触发"
     —— 要的是**空赛道触发 0 次**且**横道触发 ≥1 次、首帧尽量早**的那一档；
  ③ **正样本行段明细 + 停车段参考行**：挑 `--zebra-slow-row` / `--zebra-stop-row` 的依据
     （停车段 = 帧差≈0 的冻结区间，取那一段的近边行中位数 = 视频里那次停车的落点）。

读表三句诀（写进 doc/开发文档.md §2.3）：
  · 上沿必须**深于顶部亮带的末行**（亮带每帧都在 ⇒ 撞上就是每帧误触发，不是"偶尔看一眼"）；
  · 空赛道误检**连续 ≥enter 帧** = 入口会真的停车 10s，必须压到 0；
  · 行号 → 距离**极度非线性**（实测停车那一段 0.3m 横道占 182 行 ≈ 1.6mm/行）⇒ 标定用卷尺，
    别按米数折算；本工具只给"横道落在哪几行"，`--zebra-stop-row` 由**车摆到目标位置**时读出来
    （`lane_final_test.py --calibrate --zebra`，可 `--zebra-calib-write` 写 site.yaml）。
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from config import settings                                        # noqa: E402
from vision.camera_guard import crop_frame                         # noqa: E402
from vision.zebra_detect import detect_from_stats, merge_runs, row_stats  # noqa: E402


def imread_any(path: str) -> Optional[np.ndarray]:
    img = cv2.imread(path)
    if img is None:                       # 中文路径兜底
        try:
            img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception:                                   # noqa: BLE001
            img = None
    return img


def load_source(path: str, limit: int, stride: int) -> Tuple[str, List[np.ndarray]]:
    """视频或帧目录 → [(帧号, 裁剪后画面)]（与入口同源：都走 crop_frame）。"""
    if os.path.isdir(path):
        paths: List[str] = []
        for pat in ("frame_*.jpg", "frame_*.png", "*.jpg", "*.png"):
            paths = sorted(glob.glob(os.path.join(path, pat)))
            if paths:
                break
        out = []
        for p in paths[::max(1, stride)]:
            if limit and len(out) >= limit:
                break
            img = imread_any(p)
            if img is not None:
                out.append(crop_frame(img))
        return os.path.basename(os.path.normpath(path)), out
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f"打不开：{path}")
    out, i = [], 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % max(1, stride) == 0:
            out.append(crop_frame(frame))
            if limit and len(out) >= limit:
                break
        i += 1
    cap.release()
    return os.path.basename(path), out


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def hits_and_triggers(stats: Sequence[Dict[str, np.ndarray]], roi_w: float, row_lo: int,
                      row_frac: float, min_rows: int, enter: int,
                      min_w_frac: float, min_pix: int) -> Tuple[int, int, int, int]:
    """返回 (命中帧数, 最长连续帧数, 连续≥enter 的触发段数, 命中帧里的首帧序号)。"""
    flags = []
    for st in stats:
        hit = detect_from_stats(st, roi_w, row_lo, row_frac=row_frac, min_rows=min_rows,
                                min_width_frac=min_w_frac, min_pixels=min_pix)
        flags.append(hit is not None)
    runs, cur = [], 0
    for v in flags:
        if v:
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    first = next((i for i, v in enumerate(flags) if v), -1)
    return (sum(1 for v in flags if v), (max(runs) if runs else 0),
            sum(1 for x in runs if x >= enter), first)


def top_band_end(stats: Dict[str, np.ndarray], row_frac: float, gap: int = 2) -> int:
    """"从第 0 行起"那条行的末行（-1 = 第 0 行就不达标）。顶部亮带就是它。"""
    runs = merge_runs([bool(v >= row_frac) for v in stats["frac"]], gap)
    return runs[0][1] if runs and runs[0][0] == 0 else -1


def freeze_segments(frames: Sequence[np.ndarray], thresh: float = 1.0) -> List[Tuple[int, int]]:
    """帧差≈0 的冻结区间 [(起, 止)]（= 视频里车停着的那几段；手推视频就是"那次停车"）。"""
    out, start = [], None
    prev = None
    for i, f in enumerate(frames):
        if prev is not None:
            d = float(np.mean(cv2.absdiff(f, prev))) if f.shape == prev.shape else 999.0
            if d < thresh:
                if start is None:
                    start = i - 1
            elif start is not None:
                out.append((start, i - 1))
                start = None
        prev = f
    if start is not None:
        out.append((start, len(frames) - 1))
    return [seg for seg in out if seg[1] - seg[0] >= 5]


def main() -> int:
    ap = argparse.ArgumentParser(description="斑马线检测档现场标定（横道视频 + 空赛道 → ROI/闸门/行号）")
    ap.add_argument("--pos", required=True, help="横道片段：视频或帧目录（车经过/停在横道前的片段）")
    ap.add_argument("--neg", default="", help="空赛道片段：视频或帧目录（同机位、无横道）")
    ap.add_argument("--pos-limit", type=int, default=1200, help="横道片段最多取多少帧（0=不限）")
    ap.add_argument("--neg-limit", type=int, default=900, help="空赛道片段最多取多少帧（0=不限）")
    ap.add_argument("--stride", type=int, default=1, help="隔几帧取一帧（长视频用 2~4 提速）")
    ap.add_argument("--roi-tops", default="0.20,0.26,0.30,0.35",
                    help="要扫的 ROI 上沿（裁剪后画面比例）")
    ap.add_argument("--row-fracs", default="0.05,0.10,0.15", help="要扫的单行白占比下限")
    ap.add_argument("--min-rows", default="3,4", help="要扫的连续行段长度下限")
    ap.add_argument("--enter", type=int, default=3, help="防抖帧数（= ZEBRA_HYSTERESIS_FRAMES）")
    ap.add_argument("--detail-every", type=int, default=30, help="行段明细每几帧打一行（0=不打）")
    args = ap.parse_args()

    pos_tag, pos = load_source(args.pos, args.pos_limit, args.stride)
    if not pos:
        raise SystemExit(f"横道片段一帧都没读到：{args.pos}（路径/格式？）")
    neg_tag, neg = (load_source(args.neg, args.neg_limit, args.stride) if args.neg else ("", []))
    crop_h, crop_w = (pos[0].shape[0], pos[0].shape[1])
    print(f"[CAL] 横道片段 {pos_tag}：{len(pos)} 帧（裁剪后 {crop_w}x{crop_h}）")
    if neg:
        print(f"[CAL] 空赛道片段 {neg_tag}：{len(neg)} 帧")
    else:
        print("[CAL] ⚠️ 没给 --neg：只能看横道在哪，**无法判断会不会误检**（务必补一段空赛道）")
    print(f"[CAL] 当前 settings：ROI={settings.ZEBRA_GATE_ROI} thresh={settings.ZEBRA_GRAY_THRESH} "
          f"S<={settings.ZEBRA_S_MAX} "
          f"row_frac={settings.ZEBRA_ROW_FRAC} min_rows={settings.ZEBRA_MIN_ROWS} "
          f"min_w={settings.ZEBRA_MIN_WIDTH_FRAC} min_pix={settings.ZEBRA_MIN_PIXELS}")
    base_roi = settings.ZEBRA_GATE_ROI
    roi_w = float(int(crop_w * base_roi[1]) - int(crop_w * base_roi[0]))

    print("\n[CAL] 逐帧逐行统计（每帧一次）…")
    pos_stats = [row_stats(f, roi=(base_roi[0], base_roi[1], 0.0, 1.0)) for f in pos]
    neg_stats = [row_stats(f, roi=(base_roi[0], base_roi[1], 0.0, 1.0)) for f in neg]

    print("\n[CAL] —— ① 顶部'永久亮带'有多深（ROI 上沿必须比它更深）——")
    fracs = [float(v) for v in args.row_fracs.split(",")]
    for tag, stats in (("横道片段", pos_stats), ("空赛道", neg_stats)):
        if not stats:
            continue
        for fr in (fracs[0], fracs[-1]):
            ends = np.array([top_band_end(st, fr) for st in stats])
            hit = ends >= 0
            print(f"  [{tag}] 行白占比≥{fr:.2f}：{int(hit.sum())}/{len(stats)} 帧有「顶带」 | "
                  f"末行 p50/p95/max = {pct(ends[hit], 50):.0f}/{pct(ends[hit], 95):.0f}/"
                  f"{float(ends[hit].max()) if hit.any() else -1:.0f}")
    ends_all = np.array([top_band_end(st, fracs[0]) for st in (pos_stats + neg_stats)])
    ends_all = ends_all[ends_all >= 0]
    if len(ends_all):
        print(f"  ⇒ 建议 ROI 上沿 ≥ ({float(ends_all.max()):.0f}+4)/{crop_h} = "
              f"{min(0.95, (float(ends_all.max()) + 4) / crop_h):.2f}"
              f"（{float(ends_all.max()):.0f} = 两段素材里顶带末行的最大值）")
        print("     顶带 = 从第 0 行起、连续达标的行段；它是**远景亮物**（赛道尽头/围栏/楼），"
              "每帧都在、与横道无关")

    print("\n[CAL] —— ② 闸门扫档（连续≥%d 帧才算触发；空赛道误检必须 0）——" % args.enter)
    print(f"{'上沿':>6}{'y起':>6}{'行白':>6}{'行数':>5} | "
          f"{'横道命中':>9}{'触发':>5}{'首帧':>6}{'最长连':>7} | {'空赛道命中':>10}{'触发':>5}")
    for top in [float(v) for v in args.roi_tops.split(",")]:
        row_lo = int(crop_h * top)
        for fr in fracs:
            for mr in [int(v) for v in args.min_rows.split(",")]:
                ph, p_run, p_trig, p_first = hits_and_triggers(
                    pos_stats, roi_w, row_lo, fr, mr, args.enter,
                    settings.ZEBRA_MIN_WIDTH_FRAC, settings.ZEBRA_MIN_PIXELS)
                nh, n_run, n_trig, _ = hits_and_triggers(
                    neg_stats, roi_w, row_lo, fr, mr, args.enter,
                    settings.ZEBRA_MIN_WIDTH_FRAC, settings.ZEBRA_MIN_PIXELS)
                ok = (not neg or n_trig == 0) and p_trig > 0
                flag = " ✅" if ok else (" ❌ 空赛道误触发" if n_trig else " ⚠️ 横道也没触发")
                print(f"{top:>6.2f}{row_lo:>6}{fr:>6.2f}{mr:>5} | "
                      f"{ph:>6}/{len(pos):<3}{p_trig:>5}{p_first:>6}{p_run:>7} | "
                      f"{nh:>7}/{len(neg) if neg else 0:<3}{n_trig:>5}{flag}")

    print("\n[CAL] —— ③ 正样本行段明细（挑 slow_row/stop_row 用；按 settings 当前 ROI 上沿 "
          f"{base_roi[2]:.2f}）——")
    row_lo = int(crop_h * base_roi[2])
    fr0 = fracs[1] if len(fracs) > 1 else fracs[0]
    mr0 = int(args.min_rows.split(",")[0])
    last_near = None
    for i, st in enumerate(pos_stats):
        hit = detect_from_stats(st, roi_w, row_lo, row_frac=fr0, min_rows=mr0,
                                min_width_frac=settings.ZEBRA_MIN_WIDTH_FRAC,
                                min_pixels=settings.ZEBRA_MIN_PIXELS)
        if hit is None:
            continue
        step = max(1, args.detail_every)
        if args.detail_every and (last_near is None or abs(hit.near_row - last_near) >= 2.0) \
                and (i % step == 0 or last_near is None or abs(hit.near_row - last_near) >= 24.0):
            print(f"  f{i:04d} 行段 {hit.far_row:.0f}-{hit.near_row:.0f}"
                  f"（{hit.n_rows} 行）跨度 {hit.width / max(1.0, roi_w):.2f} "
                  f"白像素 {hit.n_pixels}")
        last_near = hit.near_row
    if last_near is not None:
        all_hits = [detect_from_stats(st, roi_w, row_lo, row_frac=fr0, min_rows=mr0,
                                      min_width_frac=settings.ZEBRA_MIN_WIDTH_FRAC,
                                      min_pixels=settings.ZEBRA_MIN_PIXELS) for st in pos_stats]
        idxs = [i for i, h in enumerate(all_hits) if h is not None]
        print(f"  首次可检出 f{idxs[0]}（近边 {all_hits[idxs[0]].near_row:.0f}）"
              f" | 末次 f{idxs[-1]}（近边 {all_hits[idxs[-1]].near_row:.0f}）")

    print("\n[CAL] 停车段（帧差≈0 = 视频里那次停车）的近边行参考：")
    segs = freeze_segments(pos)
    if not segs:
        print("  没找到冻结段（整段都在动？）——那就用 `--calibrate --zebra` 现场量目标位置的行号")
    for a, b in segs:
        ns = [detect_from_stats(pos_stats[i], roi_w, row_lo, row_frac=fr0, min_rows=mr0,
                                min_width_frac=settings.ZEBRA_MIN_WIDTH_FRAC,
                                min_pixels=settings.ZEBRA_MIN_PIXELS) for i in range(a, b + 1)]
        vals = np.array([h.near_row for h in ns if h is not None])
        if len(vals):
            print(f"  f{a}-f{b}（{b - a + 1} 帧）：近边行 中位 {pct(vals, 50):.0f} "
                  f"（p05~p95 {pct(vals, 5):.0f}~{pct(vals, 95):.0f}）"
                  f" ⇒ 这趟视频的停车落点参考 `ZEBRA_STOP_ROW≈{pct(vals, 50):.0f}`"
                  f"（**手推视频不是卷尺标定，只当默认值用**）")
        else:
            print(f"  f{a}-f{b}：冻结段里没有横道读数")

    print("\n[CAL] 定稿后写回：settings.ZEBRA_GATE_ROI（上沿）/ ZEBRA_ROW_FRAC / ZEBRA_MIN_ROWS，"
          "行号用 `lane_final_test.py --calibrate --zebra [--zebra-calib-write]` 卷尺标定；\n"
          "      然后用入口复跑验收：\n"
          "      python3 scripts/lane_final_test.py --video <横道视频> --zebra"
          " → 汇总行「相位触发 N 次」（N = 视频里的横道次数）\n"
          "      python3 scripts/lane_final_test.py --video <空赛道视频> --zebra"
          " → 汇总行「相位触发 0 次」")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
