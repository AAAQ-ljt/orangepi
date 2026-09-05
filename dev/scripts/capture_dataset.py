#!/usr/bin/env python3
"""小车端数据集采集脚本（全新独立脚本，不修改官方代码）。

功能：
- 运行前通过 --output 指定保存目录；
- 每 0.5 秒拍摄一张 JPEG 照片；
- 自动停止 FFmpeg 推流释放摄像头，退出后自动恢复；
- Ctrl+C 安全退出。

用法：
    python3 scripts/capture_dataset.py --output /root/dataset/lane
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from datetime import datetime

import cv2

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
    parser = argparse.ArgumentParser(description="Car dataset capture")
    parser.add_argument("--output", required=True, help="保存目录")
    parser.add_argument("--interval", type=float, default=0.5, help="拍摄间隔秒数")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    out_dir = os.path.abspath(args.output)
    os.makedirs(out_dir, exist_ok=True)
    print(f"[CAPTURE] save to {out_dir}")
    print(f"[CAPTURE] interval {args.interval}s, camera {args.device}")

    print("[CAPTURE] stopping ffmpeg to free camera")
    stop_ffmpeg()

    cap = None
    try:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print("[CAPTURE] ERROR: cannot open camera")
            return 1

        print("[CAPTURE] capturing... Ctrl+C to stop")
        counter = 0
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            counter += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(out_dir, f"frame_{ts}_{counter:04d}.jpg")
            cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            print(f"[CAPTURE] saved {filename}")

            time.sleep(max(0.05, args.interval))
    except KeyboardInterrupt:
        print("\n[CAPTURE] interrupted by user")
    finally:
        if cap is not None:
            cap.release()
        print("[CAPTURE] restoring ffmpeg")
        start_ffmpeg()
        print("[CAPTURE] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
