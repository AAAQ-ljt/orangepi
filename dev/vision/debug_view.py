#!/usr/bin/env python3
"""小车 RKNN 调试视图：显示摄像头画面、FPS、检测框和识别结果。

用法（在小车上手动运行，需要 X11 显示）：
    cd /root/dev
    PYTHONPATH=/root/dev python3 vision/debug_view.py --model /root/dev/models/best11nseg.rknn

按键：
    q / ESC 退出
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time

import cv2
import numpy as np

from vision.rknn_detector import RKNNYoloSeg
from vision.postprocess import CLASS_NAMES, postprocess

FFMPEG_SERVICES = [
    "ffmpeg-stream.service",
    "ffmpeg-stream-sub.service",
]


def stop_ffmpeg() -> None:
    for svc in FFMPEG_SERVICES:
        subprocess.run(["systemctl", "stop", svc], check=False)
    time.sleep(1.0)


def start_ffmpeg() -> None:
    for svc in FFMPEG_SERVICES:
        subprocess.run(["systemctl", "start", svc], check=False)


def draw_detections(frame: np.ndarray, result) -> np.ndarray:
    for det in result.detections:
        x1, y1, x2, y2 = det.xyxy
        name = CLASS_NAMES[det.class_id]
        color = (0, 255, 0)
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        label = f"{name} {det.conf:.2f}"
        cv2.putText(frame, label, (int(x1), max(0, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return frame


def main() -> int:
    parser = argparse.ArgumentParser(description="Car RKNN debug view")
    parser.add_argument("--model", default="/root/dev/models/best11nseg.rknn")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()

    print("[DEBUG] loading RKNN model")
    model = RKNNYoloSeg(args.model)
    if not model.load():
        print("[DEBUG] model load failed")
        return 1

    print("[DEBUG] stopping ffmpeg to free camera")
    stop_ffmpeg()

    cap = None
    try:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print("[DEBUG] camera open failed")
            return 1

        print("[DEBUG] press q or ESC to exit")
        frame_count = 0
        start_time = time.time()
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            img = model.preprocess(frame)
            t0 = time.time()
            outputs = model.infer(img)
            infer_ms = (time.time() - t0) * 1000
            result = postprocess(outputs, conf_threshold=args.conf)

            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0

            display = frame.copy()
            display = draw_detections(display, result)
            cv2.putText(display, f"FPS: {fps:.1f}  infer: {infer_ms:.0f}ms",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            info = (f"center={result.center_x:.0f} "
                    f"zebra={int(result.has_zebra_crossing)} "
                    f"cone={result.blue_cone_count} "
                    f"A={int(result.has_sign_a)} B={int(result.has_sign_b)} "
                    f"L={int(result.has_left_sign)} R={int(result.has_right_sign)}")
            cv2.putText(display, info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

            cv2.imshow("car RKNN debug", display)
            key = cv2.waitKey(1)
            if key in (ord("q"), 27):
                print("[DEBUG] user quit")
                break
    except KeyboardInterrupt:
        print("\n[DEBUG] interrupted")
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        model.release()
        print("[DEBUG] restoring ffmpeg")
        start_ffmpeg()
    return 0


if __name__ == "__main__":
    sys.exit(main())
