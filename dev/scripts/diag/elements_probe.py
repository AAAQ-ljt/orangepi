#!/usr/bin/env python3
"""现场元素探针（2026-10-09）：对每个元素摆位、逐帧给读数 + 叠加图。

复用 vision/color_elements（zebra / 红绿灯互比 / 停车黄区 / choose_gap / classify_ab）
与 vision/cone_detect（蓝锥）。红锥为现场 HSV 红双区间小助手（配方：穿越火线
H0-10∪170-180, S≥90, V≥65；探针专用，产品化见 TODO）。

用法：
    单帧（本地验证）:  PYTHONPATH=dev python scripts/diag/elements_probe.py --image 某图.jpg
    车上连续:          sudo python3 scripts/diag/elements_probe.py --camera 2 --frames 8 \
                       --save-dir elem_1010
    发车/元摆位前请把车放稳；本脚本**不驱动电机**（只读摄像头）。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from config import settings                                       # noqa: E402
from vision.color_elements import (choose_gap, classify_ab, detect_parking_yellow,
                                   detect_traffic_light, detect_zebra,
                                   blue_board_contour)           # noqa: E402
from vision.cone_detect import (ConeBox, detect_cones_hsv)        # noqa: E402


# ------------------------------------------------------------------ 红锥小助手（探针专用）
def detect_red_cones(frame_bgr: np.ndarray, roi=None) -> list:
    """红色锥桶候选（穿越火线配方）：双区间 HSV → 最大轮廓 bbox，简单闸门。"""
    h, w = frame_bgr.shape[:2]
    roi = roi or settings.CONE_GATE_ROI
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    patch = frame_bgr[y0:y1, x0:x1]
    if patch.size == 0:
        return []
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array((0, 90, 65), np.uint8), np.array((10, 255, 255), np.uint8))
    m2 = cv2.inRange(hsv, np.array((170, 90, 65), np.uint8), np.array((180, 255, 255), np.uint8))
    mask = cv2.bitwise_or(m1, m2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 30:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        if bw < 7 or bh < 10 or bw / max(1.0, bh) > 1.8:
            continue
        out.append(ConeBox(xyxy=(x0 + bx, y0 + by, x0 + bx + bw, y0 + by + bh),
                           area_ratio=area / max(1.0, patch.shape[0] * patch.shape[1]),
                           conf=1.0))
    return out


def probe_frame(frame_bgr: np.ndarray) -> dict:
    """一帧的所有元素读数。"""
    r = {
        "zebra": detect_zebra(frame_bgr),
        "light_ratio": detect_traffic_light(frame_bgr, use_zone=False),
        "light_zone": detect_traffic_light(frame_bgr, use_zone=True),
        "parking_yellow": detect_parking_yellow(frame_bgr),
        "blue_cones": len(detect_cones_hsv(frame_bgr)),
        "red_cones": len(detect_red_cones(frame_bgr)),
        "gap": None,
    }
    blue = detect_cones_hsv(frame_bgr)
    red = detect_red_cones(frame_bgr)
    if blue:
        gap = choose_gap(blue, red, img_center=settings.IMG_W / 2)
        if gap is not None:
            r["gap"] = {"center": round(gap.center_x, 1), "width": round(gap.width, 1),
                        "spans_center": gap.spans_center}
    board = blue_board_contour(frame_bgr,
                               roi=(0.10, 0.90, 0.20, 0.95),
                               hsv_low=settings.START_BLUE_HSV_LOW,
                               hsv_high=settings.START_BLUE_HSV_HIGH)
    r["ab"] = classify_ab(board) if board is not None else None
    return r


def _draw(frame_bgr: np.ndarray, r: dict) -> np.ndarray:
    vis = frame_bgr.copy()
    y = 24
    for k, v in r.items():
        label = f"{k}={v}"
        cv2.putText(vis, label, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        y += 24
    for c in detect_cones_hsv(frame_bgr):
        x0, y0, x1, y1 = (int(v) for v in c.xyxy)
        cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 0, 0), 2)
    for c in detect_red_cones(frame_bgr):
        x0, y0, x1, y1 = (int(v) for v in c.xyxy)
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 0, 255), 2)
    return vis


def main() -> int:
    ap = argparse.ArgumentParser(description="元素摆位探针（不驱动电机）")
    ap.add_argument("--camera", type=int, default=2)
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--save-dir", default="", help="叠加图目录")
    ap.add_argument("--image", default=None, help="本地单帧分析")
    args = ap.parse_args()

    if args.image:
        img = cv2.imdecode(np.fromfile(args.image, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print(f"cannot read {args.image}")
            return 1
        r = probe_frame(img)
        print(json_dumps(r))
        return 0

    from scripts.bench_common import install_signal_guard
    state = {"stopped": False, "driver": None, "restored": False, "used_motor": False}
    install_signal_guard(state)          # Ctrl+C/TERM/HUP(SSH 断线) → 置位 → 循环收尾
    from vision.camera_guard import camera_exclusive, open_camera
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
    with camera_exclusive():
        cap = open_camera(args.camera, settings.IMG_W, settings.IMG_H)
        if cap is None:
            print("[PROBE] 摄像头打不开")
            return 2
        try:
            for i in range(args.frames):
                if state["stopped"]:
                    print("[PROBE] 收到停止信号，提前结束")
                    break
                ok, frame = cap.read()
                if not ok or frame is None:
                    time.sleep(0.05)
                    continue
                r = probe_frame(frame)
                print(f"[PROBE #{i}] {json_dumps(r)}")
                if args.save_dir:
                    vis = _draw(frame, r)
                    cv2.imencode(".jpg", vis)[1].tofile(
                        os.path.join(args.save_dir, f"probe_{i:03d}.jpg"))
        finally:
            cap.release()
    print("[PROBE] 结束")
    return 0


def json_dumps(r: dict) -> str:
    import json
    return json.dumps(r, ensure_ascii=False)


if __name__ == "__main__":
    sys.exit(main())