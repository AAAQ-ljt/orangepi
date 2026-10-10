#!/usr/bin/env python3
"""4 套官方循迹方案的离线对照探针（明天操场 A/B/C/D 的预排）。

官方资料里能数出 4 套循迹程序（见 doc/开发方案-2026-10-08.md §三）：
  1. 黑线质心   line_follow.cpp     —— 二值化(INV)+全局质心，KP7/KD3，死区 3
  2. 单线质心   line_follow2.cpp    —— 二值化+全局质心，KP0.6/KD0.1，阈值 185
  3. 双线质心   9 号案例            —— 二值化+全局质心，KP7/KD3，阈值 190
  4. 边缘+霍夫  双线循迹代码.txt    —— Canny+Hough+斜率分左右+逐行中线（= 我们 lane_ref 同源）
1/2/3 本质上同一算法（ROI 内前景质心），区别只是 ROI/阈值/增益；4 是边缘法。

本探针在**同一批真实图片**上跑各方案的检测部分，出读数表：
  有中心帧率 / 中心跨距 / （质心法）前景占比，供选型前先淘汰纸面上就不可用的方案。

⚠️ 只借方法不借数字：每套方案的 ROI 都按"本车实测线带"适配（命令行可调），
   官方写死的 y300~480、y205~405 之类数字不在我们的摄像头安装下直接成立。

用法（本地，无硬件）：
    PYTHONPATH=. python scripts/diag/lane4schemes_probe.py \
        --images "../logs_pull/*.jpg" \
        --images "../比赛资料/5G远程驾驶无人车(2026)学习资料/竞赛进阶案例/12.图像处理-数据集获取/图像处理-数据集获取/captured_photos/*.jpg"
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

# ------------------------------------------------------------------ I/O 帮助
def imread_unicode(path: str):
    """Windows 版 cv2.imread 遇中文路径返回 None（本地实测），用 imdecode 绕开。"""
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


# ------------------------------------------------------------------ 方案 1/2/3：质心法
class CentroidScheme:
    """官方"二值化 + ROI 内前景质心"（line_follow.cpp / line_follow2.cpp / 9 号案例）。

    参数差异（官方文件）：黑线 THRESHOLD=195/INV/KP7 vs 白线阈值 185/BINARY/KP0.6。
    我们只做**检测**（质心给不给得出），不跑 PID——选型时不比较控制增益。
    """

    def __init__(self, name: str, roi: Tuple[int, int, int, int],
                 threshold: int = 185, invert: bool = False,
                 dead_zone_use: bool = False):
        self.name = name
        self.roi = roi                      # (x0, y0, x1, y1) 原图坐标
        self.threshold = int(threshold)
        self.invert = invert
        self.dead_zone_use = dead_zone_use

    def scan(self, frame: np.ndarray):
        x0, y0, x1, y1 = self.roi
        h, w = frame.shape[:2]
        if x1 <= x0 + 1 or y1 <= y0 + 1:
            return None                      # ROI 非法
        roi = frame[y0:y1, x0:x1]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mode = cv2.THRESH_BINARY_INV if self.invert else cv2.THRESH_BINARY
        _, binary = cv2.threshold(gray, self.threshold, 255, mode)
        pts = cv2.findNonZero(binary)
        n = 0 if pts is None else len(pts)
        if n < 30:                           # 官方 WHITE_MAXNUM_STOP=30
            return {"center_x": None, "n_white": n, "roi": self.roi}
        xy = pts.reshape(-1, 2)              # (N,1,2) 或 (N,2) 都收成 (N,2)
        cx = int(xy[:, 0].mean()) + x0
        cy = int(xy[:, 1].mean()) + y0
        center_x = float(cx)
        # 前景占比 = 判定"ROI 里是不是干净"（反光/杂物会把占比顶高、质心拉偏）
        frac = n / float(roi.shape[0] * roi.shape[1])
        return {"center_x": center_x, "center_y": cy, "n_white": n,
                "frac": frac, "roi": self.roi}


# ------------------------------------------------------------------ 方案 4：边缘+霍夫（= lane_ref）
from vision.lane_ref import LaneParams, LaneRefDetector  # noqa: E402


class HoughScheme:
    """官方室外赛双线（Canny+Hough，与我们 lane_ref 同源），复用其检测器。"""

    def __init__(self, name: str, roi_top: float, roi_bottom: float,
                 look_top: float, look_bottom: float):
        self.name = name
        self.det = LaneRefDetector(LaneParams.from_settings(
            roi_top_ratio=roi_top, roi_bottom_ratio=roi_bottom,
            look_top_ratio=look_top, look_bottom_ratio=look_bottom))

    def scan(self, frame: np.ndarray):
        r = self.det.detect(frame)
        return {"center_x": r.center_x, "quality": r.quality,
                "tag": r.tag, "n_left": r.n_left, "n_right": r.n_right,
                "note": r.note}


# ------------------------------------------------------------------ 主流程
def _y_band(images: Sequence[str], sample: int = 12) -> str:
    """粗略给出一批图的"白线可见带"，帮人肉定 ROI（读数的辅助信息）。"""
    rows_ok = []
    for p in list(images)[:sample]:
        img = imread_unicode(p)
        if img is None:
            continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        s, v = hsv[:, :, 1], hsv[:, :, 2]
        v_med = float(np.median(v))
        v_p95 = float(np.percentile(v, 95))
        thr = max(120.0, v_med + 0.5 * (v_p95 - v_med))
        mask = (s <= 70) & (v >= thr)
        for y in range(0, 480, 20):
            row = mask[y]
            if row.sum() >= 8:
                rows_ok.append(y)
    if not rows_ok:
        return "（没找出明显的白线带，可能需要调白线判据）"
    return f"白点带 y≈{min(rows_ok)}~{max(rows_ok)}"


def main() -> int:
    ap = argparse.ArgumentParser(description="4 套官方循迹方案离线对照读数")
    ap.add_argument("--images", action="append", required=True, help="图片 glob，可重复")
    ap.add_argument("--max-frames", type=int, default=40, help="每批最多用多少张")
    ap.add_argument("--roi-single", default="0,120,640,456",
                    help="质心法 ROI（本车实测带，默认 y120~456 贴近车上 site.yaml 0.25/24）")
    ap.add_argument("--roi-hough", default="0.25,0.95,0.25,0.60",
                    help="边缘法 ROI/前瞻带（顶部比例,底部比例,前瞻上,前瞻下）")
    ap.add_argument("--threshold", type=int, default=185, help="质心法二值化阈值")
    args = ap.parse_args()

    images: List[str] = []
    for g in args.images:
        images.extend(sorted(glob.glob(g)))
    if not images:
        print("没有匹配图片", file=sys.stderr)
        return 2
    print(f"共 {len(images)} 张；白线带提示：{_y_band(images)}")

    x0, y0, x1, y1 = (int(v) for v in args.roi_single.split(","))
    rt, rb, lt, lb = (float(v) for v in args.roi_hough.split(","))
    schemes = [
        CentroidScheme("①黑线质心(INV)", (x0, y0, x1, y1),
                       threshold=args.threshold, invert=True),
        CentroidScheme("②单线质心(BIN)", (x0, y0, x1, y1),
                       threshold=args.threshold, invert=False),
        CentroidScheme("③双线质心(BIN)", (x0, y0, x1, y1),
                       threshold=args.threshold, invert=False),
        HoughScheme("④边缘+霍夫(ref)", rt, rb, lt, lb),
    ]

    # 按来源分组：官方给的图 与 我们自己拍的图 是两台不同的车，结论必须分开看
    groups = []
    off = [p for p in images if "captured_photos" in p]
    own = [p for p in images if "captured_photos" not in p]
    if own:
        groups.append(("我们自己的图", own))
    if off:
        groups.append(("官方资料里的图", off))

    for gname, gimgs in groups:
        frames = gimgs[: args.max_frames]
        print(f"\n【{gname}】{len(frames)} 张")
        print(f"{'方案':<20} {'有中心':>7} {'中心跨距':>8} {'中位frac':>9} {'说明'}")
        for sc in schemes:
            centers, notes, fracs = [], [], []
            for p in frames:
                img = imread_unicode(p)
                if img is None:
                    continue
                res = sc.scan(img)
                if res.get("center_x") is not None:
                    centers.append(float(res["center_x"]))
                if res.get("frac") is not None:
                    fracs.append(res["frac"])
                if res.get("note"):
                    notes.append(res["note"])
            n_ok = len(centers)
            spread = (max(centers) - min(centers)) if n_ok >= 2 else 0.0
            frac_med = float(np.median(fracs)) if fracs else 0.0
            note = ""
            if isinstance(sc, CentroidScheme):
                note = (f"前景占比中位 {frac_med * 100:.1f}%"
                        + ("（≈满幅=反相错用，方案判死）" if frac_med > 0.5 else "（<15% 才算干净）"))
            elif isinstance(sc, HoughScheme):
                note = f"tag 示例：{notes[0]}" if notes else "无读数"
            print(f"{sc.name:<20} {n_ok:>4}/{len(frames):<3} "
                  f"{spread:>8.0f}px {frac_med * 100:>8.1f}%  {note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())