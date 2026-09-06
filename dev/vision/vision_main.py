#!/usr/bin/env python3
"""小车本地 RKNN 视觉主进程：摄像头 -> RKNN -> UDP -> 控制端。"""
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time

import cv2
import numpy as np

from vision.rknn_detector import RKNNYoloSeg
from vision.postprocess import postprocess, result_to_message

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


def main() -> int:
    parser = argparse.ArgumentParser(description="Car local RKNN vision")
    parser.add_argument("--model", default="/root/dev/models/best11nseg.rknn")
    parser.add_argument("--udp-ip", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5000)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--test-image", default=None, help="只测试单张图片后退出")
    args = parser.parse_args()

    print("[VISION] loading RKNN model")
    model = RKNNYoloSeg(args.model)
    if not model.load():
        print("[VISION] model load failed")
        return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    if args.test_image:
        frame = cv2.imread(args.test_image)
        if frame is None:
            print(f"[VISION] cannot read {args.test_image}")
            return 1
        outputs = model.infer(frame)
        print(f"[VISION] outputs shapes: {[np.asarray(o).shape for o in outputs]}")
        result = postprocess(outputs, conf_threshold=args.conf)
        print(f"[VISION] result: {result}")
        model.release()
        return 0

    print("[VISION] stopping ffmpeg to free camera")
    stop_ffmpeg()
    cap = None
    try:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print("[VISION] camera open failed")
            return 1

        print(f"[VISION] sending to {args.udp_ip}:{args.udp_port}")
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            try:
                outputs = model.infer(frame)
                result = postprocess(outputs, conf_threshold=args.conf)
                msg = result_to_message(result, time.time())
                data = json.dumps(msg, ensure_ascii=False).encode("utf-8")
                sock.sendto(data, (args.udp_ip, args.udp_port))
                print(f"[VISION] center={result.center_x:.1f} "
                      f"zebra={result.has_zebra_crossing} cone={result.blue_cone_count} "
                      f"A={result.has_sign_a} B={result.has_sign_b} "
                      f"L={result.has_left_sign} R={result.has_right_sign}")
            except Exception as exc:
                print(f"[VISION] infer error: {exc}")

    except KeyboardInterrupt:
        print("\n[VISION] interrupted")
    finally:
        if cap is not None:
            cap.release()
        sock.close()
        model.release()
        print("[VISION] restoring ffmpeg")
        start_ffmpeg()
    return 0


if __name__ == "__main__":
    sys.exit(main())
