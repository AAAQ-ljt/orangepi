#!/usr/bin/env python3
"""小车视觉调试视图：画面 + 扫线 + 元素检测 + 发车判据读数 + FPS。

用法（小车上手动运行，需要 X11）：
    cd /root/dev
    PYTHONPATH=/root/dev python3 vision/debug_view.py            # 只看扫线+发车判据
    PYTHONPATH=/root/dev python3 vision/debug_view.py --model /root/dev/models/best11nseg.rknn --profile legacy

按键：
    q / ESC 退出     m 切换白线掩膜      g 切换发车判据读数
"""
from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

from config import settings
from vision.camera_guard import camera_exclusive
from vision.elements import load_profile
from vision.lane_scan import LaneScanner
from vision.postprocess import postprocess
from vision.start_gate import StartGate

# 元素配色（BGR）：一类一色，肉眼区分
_ELEMENT_COLORS = {
    "zebra": (0, 255, 255),
    "cone": (0, 165, 255),
    "parking_sign": (255, 0, 255),
    "parking_area": (0, 200, 0),
    "blue_board": (255, 128, 0),
    "light": (255, 255, 255),
}


def draw_lane(display: np.ndarray, mask: np.ndarray, obs, blend: bool = True) -> np.ndarray:
    h, w = display.shape[:2]
    roi_y0 = int(h * settings.LANE_ROI_TOP_RATIO)
    roi_y1 = h - settings.LANE_ROI_BOTTOM_MARGIN
    if blend and mask is not None:
        overlay = np.zeros_like(display)
        overlay[:, :, 1] = mask
        display = cv2.addWeighted(display, 1.0, overlay, 0.35, 0.0)
    cv2.line(display, (0, roi_y0), (w, roi_y0), (80, 80, 80), 1)
    cv2.line(display, (0, roi_y1), (w, roi_y1), (80, 80, 80), 1)
    cv2.line(display, (int(settings.TARGET_X), roi_y0), (int(settings.TARGET_X), roi_y1),
             (255, 160, 0), 1)
    if obs is not None and obs.confidence > 0:
        cx = int(obs.center_x)
        cv2.line(display, (cx, roi_y0), (cx, roi_y1), (0, 255, 0), 2)
        for x, color in ((obs.left_x, (255, 0, 255)), (obs.right_x, (255, 255, 0))):
            if x is not None:
                cv2.line(display, (int(x), roi_y0), (int(x), roi_y1), color, 1)
    return display


def draw_elements(display: np.ndarray, elements) -> np.ndarray:
    for e in elements:
        x1, y1, x2, y2 = [int(v) for v in e.xyxy]
        color = _ELEMENT_COLORS.get(e.name, (0, 255, 0))
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        label = f"{e.name}{'/' + e.color if e.color else ''} {e.conf:.2f}"
        cv2.putText(display, label, (x1, max(12, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        # 接地线（估距离用它，不是几何中心）
        cv2.circle(display, (int(e.center_x), int(e.bottom_y)), 3, color, -1)
    return display


def main() -> int:
    parser = argparse.ArgumentParser(description="Car vision debug view")
    parser.add_argument("--model", default=None, help="可选：RKNN 模型（给了才画元素框）")
    parser.add_argument("--profile", default="legacy", help="模型 profile：legacy / smartcar2026")
    parser.add_argument("--camera", type=int, default=2, help="2=下摄（默认），0=云台主摄")
    parser.add_argument("--width", type=int, default=settings.IMG_W)
    parser.add_argument("--height", type=int, default=settings.IMG_H)
    parser.add_argument("--conf", type=float, default=None)
    args = parser.parse_args()

    profile = load_profile(args.profile)
    model = None
    if args.model:
        from vision.rknn_detector import RKNNYoloSeg
        model = RKNNYoloSeg(args.model)
        if not model.load():
            print("[DEBUG] model load failed")
            return 1

    scanner = LaneScanner()
    gate = StartGate(verbose=False)
    show_mask = True
    show_gate = True

    print("[DEBUG] stopping ffmpeg to free camera")
    try:
        with camera_exclusive():
            cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                print("[DEBUG] camera open failed")
                return 1

            print("[DEBUG] q/ESC 退出，m 切掩膜，g 切发车读数")
            frames = 0
            start_time = time.time()
            try:
                while True:
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        time.sleep(0.05)
                        continue

                    t0 = time.time()
                    obs = scanner.scan(frame)
                    mask = (scanner.debug_mask(frame) if show_mask
                            else np.zeros(frame.shape[:2], dtype=np.uint8))
                    gs = gate.update(frame)
                    infer_ms = 0.0
                    elements = []
                    if model is not None:
                        ti = time.time()
                        elements = postprocess(model.infer(model.preprocess(frame)), profile,
                                                   box_transform=model.restore,
                                               conf_threshold=args.conf)
                        infer_ms = (time.time() - ti) * 1000.0
                    loop_ms = (time.time() - t0) * 1000.0

                    frames += 1
                    fps = frames / max(1e-6, time.time() - start_time)

                    display = draw_lane(frame.copy(), mask, obs, blend=show_mask)
                    display = draw_elements(display, elements)

                    cv2.putText(display, f"FPS {fps:.1f} loop {loop_ms:.0f}ms infer {infer_ms:.0f}ms "
                                         f"cam{args.camera} profile={profile.name}",
                                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(display,
                                f"center={obs.center_x:.0f} conf={obs.confidence:.2f} "
                                f"L={obs.left_x} R={obs.right_x} rows={obs.valid_rows}/{obs.total_rows}",
                                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
                    if show_gate:
                        m = gs.metrics
                        color = (0, 0, 255) if gs.blocked else (0, 255, 0)
                        cv2.putText(display,
                                    f"board={gs.blocked} armed={gs.armed} released={gs.released} | "
                                    f"area={m.area_ratio:.3f} dom={m.dominance:.1f} detail={m.detail:.0f}",
                                    (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
                        cv2.putText(display,
                                    f"thr: area>={settings.START_BLUE_AREA_THRESH:.3f} "
                                    f"dom>={settings.START_BLUE_DOMINANCE_THRESH:.1f}",
                                    (10, 94), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                    cv2.imshow("car vision debug", display)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        print("[DEBUG] user quit")
                        break
                    if key == ord("m"):
                        show_mask = not show_mask
                    if key == ord("g"):
                        show_gate = not show_gate
            except KeyboardInterrupt:
                print("\n[DEBUG] interrupted")
            finally:
                cap.release()
    finally:
        cv2.destroyAllWindows()
        if model is not None:
            model.release()
        print("[DEBUG] ffmpeg 推流已恢复")
    return 0


if __name__ == "__main__":
    sys.exit(main())
