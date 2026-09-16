#!/usr/bin/env python3
"""小车视觉调试视图：画面 + 扫线 + 检测框 + 发车遮挡判据 + FPS。

用法（小车上手动运行，需要 X11）：
    cd /root/dev
    PYTHONPATH=/root/dev python3 vision/debug_view.py            # 只看扫线
    PYTHONPATH=/root/dev python3 vision/debug_view.py --model /root/dev/models/xxx.rknn

按键：
    q / ESC 退出        m 切换白线掩膜叠加
"""
from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np

from config import settings
from vision.camera_guard import camera_exclusive
from vision.lane_scan import LaneScanner
from vision.start_gate import StartGate
from vision.postprocess import CLASS_NAMES


def draw_lane(display: np.ndarray, mask: np.ndarray, obs, blend: bool = True) -> np.ndarray:
    h, w = display.shape[:2]
    roi_y0 = int(h * settings.LANE_ROI_TOP_RATIO)
    roi_y1 = h - settings.LANE_ROI_BOTTOM_MARGIN
    if blend:
        overlay = np.zeros_like(display)
        overlay[:, :, 1] = mask          # 掩膜画成绿色
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Car vision debug view")
    parser.add_argument("--model", default=None, help="可选：RKNN 模型（给了才画检测框）")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=settings.IMG_W)
    parser.add_argument("--height", type=int, default=settings.IMG_H)
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()

    model = None
    postprocess = None
    if args.model:
        from vision.rknn_detector import RKNNYoloSeg
        from vision.postprocess import postprocess as _postprocess
        model = RKNNYoloSeg(args.model)
        if not model.load():
            print("[DEBUG] model load failed")
            return 1
        postprocess = _postprocess

    scanner = LaneScanner()
    gate = StartGate()
    show_mask = True

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

            print("[DEBUG] press q/ESC to exit, m to toggle mask")
            frame_count = 0
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
                    loop_ms = (time.time() - t0) * 1000.0

                    frame_count += 1
                    fps = frame_count / max(1e-6, time.time() - start_time)

                    display = draw_lane(frame.copy(), mask, obs, blend=show_mask)

                    if postprocess is not None:
                        img = model.preprocess(frame)
                        det = postprocess(model.infer(img), conf_threshold=args.conf)
                        for d in det.detections:
                            x1, y1, x2, y2 = [int(v) for v in d.xyxy]
                            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 255), 2)
                            cv2.putText(display, f"{CLASS_NAMES[d.class_id]} {d.conf:.2f}",
                                        (x1, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                        (0, 255, 255), 1)

                    cv2.putText(display, f"FPS {fps:.1f}  scan {loop_ms:.0f}ms", (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(display,
                                f"center={obs.center_x:.0f} conf={obs.confidence:.2f} "
                                f"L={obs.left_x} R={obs.right_x} rows={obs.valid_rows}/{obs.total_rows}",
                                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
                    gate_color = (0, 0, 255) if gs.blocked else (0, 255, 0)
                    cv2.putText(display,
                                f"board={gs.blocked} armed={gs.armed} released={gs.released} "
                                f"blue={gs.blue_ratio:.2f} edge={gs.edge_density:.3f}",
                                (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.55, gate_color, 1)

                    cv2.imshow("car vision debug", display)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        print("[DEBUG] user quit")
                        break
                    if key == ord("m"):
                        show_mask = not show_mask
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
