#!/usr/bin/env python3
"""蓝锥桶 HSV 阈值扫描标定（2026-10-09 T3，离线部分）。

在真实正/负样本目录上，网格扫 H/S/V 三通道下界，找"检得出锥桶、又最不误检"的
阈值组合，并附官方三套配方（我们的 / 官方14号demo蓝帽 / 穿越火线）作对照。

用法（本地，无车）：
    PYTHONPATH=dev "E:/venvs/smartcar-ultra/Scripts/python.exe" scripts/diag/cone_hsv_sweep.py \
        --pos-dir "train/project/蓝色锥桶1" --pos-dir "train/project/蓝色锥桶2" \
        --neg-dir "../logs_pull" --limit 45

输出：按 (检出率, 误检率) 排序的前 20 组 + 三套官方配方读数。
现场抓帧回来后，把 --pos-dir/--neg-dir 指到当天的实拍目录即可复跑同一套扫描。
"""
from __future__ import annotations

import argparse
import itertools
import os
import sys
from typing import List, Sequence, Tuple

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from vision.cone_detect import detect_cones_hsv        # noqa: E402


def imread_unicode(path: str):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def load_images(dirs: Sequence[str], limit: int) -> List[np.ndarray]:
    imgs = []
    for d in dirs:
        names = sorted(os.listdir(d)) if os.path.isdir(d) else []
        for n in names[:limit]:
            img = imread_unicode(os.path.join(d, n))
            if img is not None:
                imgs.append(img)
    return imgs


def eval_combo(imgs: Sequence[np.ndarray], hsv_low, hsv_high,
               roi: Tuple[float, float, float, float] = None) -> Tuple[int, int]:
    hit = 0
    for img in imgs:
        if detect_cones_hsv(img, roi=roi, hsv_low=hsv_low, hsv_high=hsv_high):
            hit += 1
    return hit, len(imgs)


# 官方三套配方（出处见 doc/开发方案-2026-10-08.md §三）
OFFICIAL_RECIPES = [
    ("我们的当前 (70,30,150)-(145,255,255)", (70, 30, 150), (145, 255, 255)),
    ("官方14号demo 蓝帽 (90,50,50)-(130,255,255)", (90, 50, 50), (130, 255, 255)),
    ("穿越火线 (95,75,55)-(132,255,255)", (95, 75, 55), (132, 255, 255)),
]


def main() -> int:
    ap = argparse.ArgumentParser(description="锥桶 HSV 扫描标定")
    ap.add_argument("--pos-dir", action="append", default=[], help="有锥桶目录（可多次）")
    ap.add_argument("--neg-dir", action="append", default=[], help="无锥桶目录（可多次）")
    ap.add_argument("--limit", type=int, default=45, help="每个目录最多看几张")
    ap.add_argument("--h-low", default="65,75,85,95", help="H 下界候选")
    ap.add_argument("--h-high", default="125,135,145,155", help="H 上界候选")
    ap.add_argument("--s-low", default="30,55,80", help="S 下界候选")
    ap.add_argument("--v-low", default="55,100,150", help="V 下界候选")
    ap.add_argument("--roi-top", default="0.05,0.35",
                    help="ROI 上沿候选比例（0.05=几乎全幅；0.35=当前配置。2026-10-09 实测 0.35 会把数据集锥桶切掉）")
    args = ap.parse_args()

    if not args.pos_dir:
        print("至少给 --pos-dir", file=sys.stderr)
        return 2
    pos = load_images(args.pos_dir, args.limit)
    neg = load_images(args.neg_dir, args.limit) if args.neg_dir else []
    print(f"正样本 {len(pos)} 张 / 负样本 {len(neg)} 张")

    def report(low, high, roi_top):
        roi = (0.10, 0.90, roi_top, 1.00)
        ph, pt = eval_combo(pos, low, high, roi)
        nh, nt = eval_combo(neg, low, high, roi) if neg else (0, 0)
        return (ph, pt, nh, nt)

    print("\n--- 官方三套配方对照（ROI=当前 0.35）---")
    for name, low, high in OFFICIAL_RECIPES:
        ph, pt, nh, nt = report(low, high, 0.35)
        print(f"{name}: 检出 {ph}/{pt} = {ph / max(1, pt) * 100:5.1f}%"
              f"  误检 {nh}/{nt} = {nh / max(1, nt) * 100:5.1f}%")

    print("\n--- 网格扫描（HSV × ROI 上沿，按 检出率↓、误检率↑ 排序，前 20）---")
    h_lows = [int(v) for v in args.h_low.split(",")]
    h_highs = [int(v) for v in args.h_high.split(",")]
    s_lows = [int(v) for v in args.s_low.split(",")]
    v_lows = [int(v) for v in args.v_low.split(",")]
    roi_tops = [float(v) for v in args.roi_top.split(",")]
    rows = []
    for h_lo, h_hi, s_lo, v_lo, r_top in itertools.product(h_lows, h_highs, s_lows, v_lows, roi_tops):
        if h_hi <= h_lo:
            continue
        low = (h_lo, s_lo, v_lo)
        high = (h_hi, 255, 255)
        ph, pt, nh, nt = report(low, high, r_top)
        rows.append((ph / max(1, pt), nh / max(1, nt), low, high, r_top, ph, pt, nh, nt))
    rows.sort(key=lambda r: (-r[0], r[1]))
    # 过滤掉"全检出但全误检"的垃圾组（正负一样都能触发=阈值太松）
    rows = [r for r in rows if not (r[0] >= 0.9 and r[5] >= max(1, len(neg) * 0.5))]
    for r in rows[:20]:
        hit_rate, neg_rate, low, high, r_top, ph, pt, nh, nt = r
        print(f"HSV {low}~{high} ROI_top={r_top}: 检出 {ph}/{pt} = {hit_rate * 100:5.1f}%"
              f"  误检 {nh}/{nt} = {neg_rate * 100:5.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())