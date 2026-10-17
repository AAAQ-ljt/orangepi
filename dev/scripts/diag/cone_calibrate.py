#!/usr/bin/env python3
"""锥桶检测档**现场标定器**（离线；2026-10-16 起，为"当前机位重标"准备）。

为什么要有它
------------
`CONE_GATE_ROI` / `CONE_MIN_H` / HSV 都是**机位相关**档，而现有正样本是 2026-09-07 拍的
（那时镜头角度高、画面中部才有锥桶），**对现在的压低机位不作数**。所以规矩改成了：
**拿当天手动推车的锥桶视频 + 一段空赛道视频，把"锥桶落在画面哪几行""会不会和误检源撞车"
量出来，再定 ROI**。这个脚本就是那条命令。

用法（本地，无车；输完两条即可决策）
------------------------------------
    PY="E:/venvs/smartcar-ultra/Scripts/python.exe"; cd dev && export PYTHONPATH=.

    # ① 锥桶视频（手推车经过锥桶那一段，下摄）+ ② 空赛道视频/帧目录（同一机位、无锥桶）
    "$PY" scripts/diag/cone_calibrate.py \
        --pos ../train/opencv-level1/video/true_track_sub_video/cone_1016.mp4 \
        --neg ../train/opencv-level1/video/frames/2 --neg-limit 900

输出（一张表 + 两句结论）：
  1. **几何分布**：锥桶框的 y 顶/底、宽高、x 的中位与 p05/p95（ROI 上沿就从"锥桶框顶最低那一档"倒推）；
  2. **ROI 上沿扫档**：每个候选上沿的"锥桶检出帧率 / 空赛道误检帧率 / 连续≥3 帧触发次数"
     —— **要的是"误检触发 0 次"且"锥桶检出高"的那一档**（连续 ≥3 帧才算触发，因为入口用 3 帧防抖）；
  3. **误检源画像**：空赛道上那些"过闸门"的框长什么样（位置/尺寸），一眼看出能不能用几何分开。

读表三句诀（写进 doc/开发文档.md §2.5）：
  · 锥桶框顶 y 若**高于** ROI 上沿 → 会被整块切掉（所以上沿要"锥桶最早出现的那一行之上"）；
  · 空赛道误检**连续 ≥3 帧** = 入口会真的触发避让（不是"偶尔看一眼"），必须压到 0；
  · **"带先验"那一列才是入口的真实行为**：锥桶只在两条车道线之间（用户给的场地事实）→ 上沿可以开得
    很早，靠这条先验挡远景误检。**实测（2026-10-16 现场锥桶视频 `锥桶1016.mp4`，手推 2564 帧）**：
    7 段锥桶接近全部检出（首帧框顶 y≈50~72、框 28~34 × 40~46px）、**7 次触发全是真锥**；
    空赛道（1014 帧目录 3573 帧 / 2.mp4 / 1.mp4）在上沿 0.00~0.20 **全 0 触发**（先验拦 104~113 框/片段）
    ⇒ 当天定稿上沿 **0.10**（0.20 会把锥桶首帧切成 h≈21 被 min_h 拒掉、迟 0.2~0.3s）。
    ⚠️ 手推时人按车尾 → 摄像头上浮 → 顶部冒"赛道尽头之外"的东西（墙/长椅/塑料膜，y0~48、多为单帧）；
    **真跑车头更低 → 顶部噪声更少、同一行对应更近的距离（锥桶更晚入画）** ⇒ 首次真跑存图复核。
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from config import settings                                        # noqa: E402
from scripts.lane_final_test import (TARGET_X0, Y_REF_PX, LaneFinalDetector,  # noqa: E402
                                     cone_in_lane)
from vision.camera_guard import crop_frame                         # noqa: E402
from vision.cone_detect import ConeBox, detect_cones_hsv           # noqa: E402


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
        for i, p in enumerate(paths[::max(1, stride)]):
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


def roi_at(top: float, x: Optional[Tuple[float, float]] = None) -> Tuple[float, float, float, float]:
    x0, x1 = x if x else (settings.CONE_GATE_ROI[0], settings.CONE_GATE_ROI[1])
    return (float(x0), float(x1), float(top), 1.00)


def hits_and_triggers(frames: Sequence[np.ndarray], roi, enter: int = 3,
                      lane_readings: Optional[Sequence] = None,
                      tol: float = 20.0) -> Tuple[int, int, int, int]:
    """返回 (命中帧数, 最长连续帧数, 连续≥enter 帧的段数, 被"在车道线内"先验滤掉的框数)。

    `lane_readings` 给了就同时算"带先验"的那一列（先验 = 锥桶框顶/底两行都要落在两条车道线之间，
    见 `lane_final_test.cone_in_lane`）——**入口就是按这个先验在喂防抖器之前过滤的**。
    lane 读数在外面算一次传进来（同一批帧扫 5~6 档 ROI，别每档重跑一遍车道检测）。
    """
    flags, flags_gated, dropped = [], [], 0
    for i, f in enumerate(frames):
        boxes = detect_cones_hsv(f, roi=roi)
        flags.append(bool(boxes))
        if lane_readings is None:
            continue
        r = lane_readings[i]
        kept = [c for c in boxes if cone_in_lane(r, c, Y_REF_PX, tol)]
        dropped += len(boxes) - len(kept)
        flags_gated.append(bool(kept))

    def runs(fl):
        out, cur = [], 0
        for v in fl:
            if v:
                cur += 1
            else:
                if cur:
                    out.append(cur)
                cur = 0
        if cur:
            out.append(cur)
        return out

    ra = runs(flags)
    if lane_readings is None:
        return sum(1 for f in flags if f), (max(ra) if ra else 0), \
            sum(1 for x in ra if x >= enter), 0
    rg = runs(flags_gated)
    return (sum(1 for f in flags_gated if f), (max(rg) if rg else 0),
            sum(1 for x in rg if x >= enter), dropped)


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def geom_report(tag: str, frames: Sequence[np.ndarray], top: float, show: int = 8,
                is_pos: bool = True) -> List[ConeBox]:
    """在**给定的几何口径**下把候选框列出来（默认上沿放到 0 = 不纵向裁剪，专看"锥桶原本在哪"）。"""
    roi = roi_at(top)
    boxes: List[ConeBox] = []
    for f in frames:
        boxes += detect_cones_hsv(f, roi=roi)
    if not boxes:
        print(f"  [{tag}] 无候选框（HSV/几何闸门全不过）")
        return boxes
    yt = np.array([b.xyxy[1] for b in boxes])
    yb = np.array([b.bottom_y for b in boxes])
    ws = np.array([b.width for b in boxes])
    hs = np.array([b.height for b in boxes])
    xs = np.array([b.center_x for b in boxes])
    crop_h = max(1, frames[0].shape[0])
    print(f"  [{tag}] 候选框 {len(boxes)} 个 | y顶 p05/中位/p95 = {pct(yt,5):.0f}/"
          f"{pct(yt,50):.0f}/{pct(yt,95):.0f} | y底 {pct(yb,5):.0f}/{pct(yb,50):.0f}/"
          f"{pct(yb,95):.0f} | 宽 {pct(ws,50):.0f} 高 {pct(hs,50):.0f}（p05 {pct(hs,5):.0f}）"
          f" | x {pct(xs,50):.0f}（p05~p95 {pct(xs,5):.0f}~{pct(xs,95):.0f}）")
    if is_pos:
        print(f"        y顶最低 {yt.min():.0f}（= 最高处那只）⇒ 建议 ROI 上沿 ≤ "
              f"{max(0.0, (yt.min() - 8) / crop_h):.2f}（(最低框顶−8)/{crop_h}）")
    else:
        print(f"        ⚠️ 这一段**本该没有锥桶**：上面的框就是误检源（y顶最低 {yt.min():.0f}）"
              f"——ROI/闸门要把它们排除在外，不是包进来")
    if show:
        order = np.argsort(yt)
        rows = [boxes[int(i)] for i in order[:show]]
        print("        最高的几个框：" + " | ".join(
            f"y{b.xyxy[1]:.0f}-{b.bottom_y:.0f} {b.width:.0f}x{b.height:.0f} "
            f"x{b.center_x:.0f} c{b.conf:.2f}" for b in rows))
    return boxes


def main() -> int:
    ap = argparse.ArgumentParser(description="锥桶检测档现场标定（锥桶视频 + 空赛道视频 → ROI/闸门）")
    ap.add_argument("--pos", required=True, help="锥桶片段：视频或帧目录（手推车经过锥桶）")
    ap.add_argument("--neg", default="", help="空赛道片段：视频或帧目录（同机位、无锥桶）")
    ap.add_argument("--pos-limit", type=int, default=600, help="锥桶片段最多取多少帧（0=不限）")
    ap.add_argument("--neg-limit", type=int, default=900, help="空赛道片段最多取多少帧（0=不限）")
    ap.add_argument("--stride", type=int, default=1, help="隔几帧取一帧（长视频用 2~4 提速）")
    ap.add_argument("--roi-tops", default="0.00,0.05,0.10,0.13,0.15,0.20,0.35",
                    help="要扫的 ROI 上沿（裁剪后画面比例；0=不纵向裁剪）")
    ap.add_argument("--geom-top", type=float, default=0.0,
                    help="几何分布用哪个上沿（默认 0.0=不裁，先看锥桶原本落在哪）")
    ap.add_argument("--enter", type=int, default=3, help="防抖帧数（= CONE_HYSTERESIS_FRAMES）")
    ap.add_argument("--tol", type=float, default=20.0, help="'在车道线内'先验的容差（px）")
    args = ap.parse_args()

    pos_tag, pos = load_source(args.pos, args.pos_limit, args.stride)
    if not pos:
        raise SystemExit(f"锥桶片段一帧都没读到：{args.pos}（路径/格式？）")
    neg_tag, neg = (load_source(args.neg, args.neg_limit, args.stride)
                    if args.neg else ("", []))
    crop_h = max(1, pos[0].shape[0]) if pos else 0
    print(f"[CAL] 锥桶片段 {pos_tag}：{len(pos)} 帧（裁剪后 {640}x{crop_h}）")
    if neg:
        print(f"[CAL] 空赛道片段 {neg_tag}：{len(neg)} 帧")
    else:
        print("[CAL] ⚠️ 没给 --neg：只能看锥桶在哪，**无法判断会不会误检**（务必补一段空赛道）")
    print(f"[CAL] 当前 settings：ROI={settings.CONE_GATE_ROI} HSV={settings.CONE_BLUE_HSV_LOW}~"
          f"{settings.CONE_BLUE_HSV_HIGH} MIN_H={settings.CONE_MIN_H} MIN_W={settings.CONE_MIN_W}")

    print("\n[CAL] —— ① 几何分布（上沿放到 %.2f = 先看锥桶原本落在哪几行）——" % args.geom_top)
    pos_boxes = geom_report("锥桶", pos, args.geom_top)
    neg_boxes = geom_report("空赛道", neg, args.geom_top, is_pos=False) if neg else []

    print("\n[CAL] —— ② ROI 上沿扫档（连续≥%d 帧才算触发；误检要压到 0）——" % args.enter)
    print("     '带先验' = 只算落在**两条车道线之间**的框（= 入口 --cone 的默认行为，见 §2.5）")
    print(f"{'ROI上沿':>8}{'y起':>6} | {'锥桶命中':>10}{'锥桶触发':>9} | "
          f"{'命中(带先验)':>13}{'触发(带先验)':>13}{'先验滤掉':>9} | "
          f"{'空赛道命中':>11}{'空赛道触发':>11} | {'空赛道(带先验)触发':>18}")
    lane = LaneFinalDetector(target_x=TARGET_X0, fps=30.0)
    print(f"[CAL] 车道读数（算先验用）：锥桶片段 {len(pos)} 帧…")
    pos_lane = [lane.detect(f) for f in pos]
    lane.reset()
    neg_lane = [lane.detect(f) for f in neg] if neg else []
    for top in [float(v) for v in args.roi_tops.split(",")]:
        roi = roi_at(top)
        ph, p_run, p_trig, _d_p = hits_and_triggers(pos, roi, args.enter)
        pg, _pg_run, pg_trig, pg_drop = hits_and_triggers(pos, roi, args.enter,
                                                          lane_readings=pos_lane,
                                                          tol=args.tol)
        if neg:
            nh, _n_run, n_trig, _d = hits_and_triggers(neg, roi, args.enter)
            ng, _ng_run, ng_trig, _ng_drop = hits_and_triggers(neg, roi, args.enter,
                                                               lane_readings=neg_lane,
                                                               tol=args.tol)
        else:
            nh = n_trig = ng_trig = 0
        ok = (neg and ng_trig == 0 and pg_trig > 0)
        flag = " ✅" if ok else (" ❌ 误检" if (neg and ng_trig) else
                                 (" ⚠️ 锥桶也被滤了" if (pg_trig == 0 and p_trig > 0) else ""))
        print(f"{top:>8.2f}{int(top * crop_h):>6} | {ph:>7}/{len(pos):<3}{p_trig:>9} | "
              f"{pg:>10}/{len(pos):<3}{pg_trig:>13}{pg_drop:>9} | "
              f"{nh:>8}/{len(neg) if neg else 0:<3}{n_trig:>11} | {ng_trig:>18}{flag}")

    if pos_boxes and neg_boxes:
        py = np.array([b.xyxy[1] for b in pos_boxes])
        ny = np.array([b.xyxy[1] for b in neg_boxes])
        ph = np.array([b.height for b in pos_boxes])
        nh = np.array([b.height for b in neg_boxes])
        print("\n[CAL] —— ③ 分离性（锥桶 vs 空赛道误检源）——")
        print(f"  y顶：锥桶 p05 {pct(py,5):.0f}~p95 {pct(py,95):.0f} | 误检源 p05 {pct(ny,5):.0f}"
              f"~p95 {pct(ny,95):.0f} | 重叠 {'有' if pct(ny,5) < pct(py,95) else '无'}")
        print(f"  框高：锥桶 p05 {pct(ph,5):.0f}/中位 {pct(ph,50):.0f} | 误检源 p05 {pct(nh,5):.0f}"
              f"/中位 {pct(nh,50):.0f}")
        print("  ⇒ ROI 切不开就回到 `vision/cone_detect.py` 的几何闸门（h≥0.5w / 面积 / 横向位置）"
              "上想办法；改阈值前先看上面两行的重叠程度。")
    print("\n[CAL] 定稿后写回：settings.CONE_GATE_ROI / CONE_MIN_H（+ 必要时 CONE_BLUE_HSV_*），"
          "然后用入口复跑验收：\n"
          "      python3 scripts/lane_final_test.py --frames <空赛道帧目录> --cone"
          " → 汇总行必须「相位触发 0 次」")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
