"""本地批量验证：锥桶检测（hsv 通道）在真实采集图上的检出率/误检率。

用法（本地，无车无模型）：
    PYTHONPATH=dev "E:/venvs/smartcar-ultra/Scripts/python.exe" scripts/cone_check.py \
        --pos-dir "train/project/蓝色锥桶1" --pos-dir "train/project/蓝色锥桶2" \
        --neg-dir logs_pull --limit 60 --save-samples logs_pull/cone_check

输出：
    - 正样本（有锥桶）检出率、每图检测框个数分布；
    - 负样本（无锥桶）误检率；
    - 筛选出的样例叠加图画到 --save-samples。
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from vision.cone_detect import detect_cones_hsv        # noqa: E402


def imread_unicode(path: str):
    """Windows 下 cv2.imread 不支持中文路径，用 fromfile 绕开。"""
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def scan(dir_path: str, limit: int, save_dir: str, tag: str):
    names = sorted(os.listdir(dir_path))
    if not names:
        print(f"[{tag}] {dir_path}: 无文件")
        return 0, 0
    n_detected = 0
    n_boxes = []
    saved = 0
    for n in names[:limit]:
        p = os.path.join(dir_path, n)
        img = imread_unicode(p)
        if img is None:
            continue
        cones = detect_cones_hsv(img)
        if cones:
            n_detected += 1
        n_boxes.append(len(cones))
        if save_dir and cones and saved < 8:
            vis = img.copy()
            for c in cones:
                x0, y0, x1, y1 = (int(v) for v in c.xyxy)
                cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 0, 0), 2)
                cv2.putText(vis, f"c{c.conf:.2f}", (x0, max(0, y0 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
            os.makedirs(save_dir, exist_ok=True)
            cv2.imencode(".jpg", vis)[1].tofile(
                os.path.join(save_dir, f"{tag}_{saved:02d}.jpg"))
            saved += 1
    tot = len(names)
    rate = n_detected / tot if tot else 0.0
    import statistics
    mean_b = statistics.mean(n_boxes) if n_boxes else 0.0
    print(f"[{tag}] {dir_path}")
    print(f"  图 {tot} 张（前 {limit}）：检出 {n_detected} 张，检出率 {rate * 100:.1f}%"
          f"，每图框数 均值 {mean_b:.2f} / 分布 {sorted(set(n_boxes))[:10]}")
    return n_detected, tot


def main() -> int:
    ap = argparse.ArgumentParser(description="锥桶检测批量验证（本地离线）")
    ap.add_argument("--pos-dir", action="append", default=[], help="有锥桶的目录（可多次）")
    ap.add_argument("--neg-dir", action="append", default=[], help="无锥桶的目录（可多次）")
    ap.add_argument("--limit", type=int, default=60, help="每个目录最多看几张")
    ap.add_argument("--save-samples", default="", help="样例叠加图输出目录")
    args = ap.parse_args()

    if not args.pos_dir and not args.neg_dir:
        print("至少给 --pos-dir 或 --neg-dir")
        return 2
    pos_tot = pos_hit = 0
    neg_tot = neg_hit = 0
    for d in args.pos_dir:
        hit, tot = scan(d, args.limit, args.save_samples, "pos")
        pos_hit += hit
        pos_tot += tot
    for d in args.neg_dir:
        hit, tot = scan(d, args.limit, args.save_samples, "neg")
        neg_hit += hit
        neg_tot += tot
    print("=" * 50)
    if pos_tot:
        print(f"正样本检出率：{pos_hit}/{pos_tot} = {pos_hit / pos_tot * 100:.1f}%")
    if neg_tot:
        print(f"负样本误检率：{neg_hit}/{neg_tot} = {neg_hit / neg_tot * 100:.1f}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())