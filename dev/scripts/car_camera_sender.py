#!/usr/bin/env python3
"""小车端摄像头直传脚本：把摄像头 JPEG 帧通过 TCP 发送到电脑。

协议：4 字节大端长度 + JPEG 数据。
运行前会自动停止 FFmpeg 推流，退出后恢复。
用法：
    python3 scripts/car_camera_sender.py --host <电脑IP> --port 9002
"""
from __future__ import annotations

import argparse
import socket
import struct
import subprocess
import sys
import time

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
    parser = argparse.ArgumentParser(description="Car camera TCP sender")
    parser.add_argument("--host", required=True, help="电脑 IP")
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=20.0)
    args = parser.parse_args()

    print("[SENDER] stopping ffmpeg to free camera")
    stop_ffmpeg()

    cap = None
    sock = None
    try:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print("[SENDER] cannot open camera")
            return 1

        interval = 1.0 / max(1.0, args.fps)
        print(f"[SENDER] connecting to {args.host}:{args.port}")
        while True:
            try:
                sock = socket.create_connection((args.host, args.port), timeout=5)
                print("[SENDER] connected")
                break
            except OSError as exc:
                print(f"[SENDER] connect failed: {exc}, retry...")
                time.sleep(2)

        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                continue
            data = encoded.tobytes()
            try:
                sock.sendall(struct.pack(">I", len(data)) + data)
            except OSError:
                print("[SENDER] connection lost, reconnect...")
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None
                while sock is None:
                    try:
                        sock = socket.create_connection((args.host, args.port), timeout=5)
                        print("[SENDER] reconnected")
                    except OSError:
                        time.sleep(2)

            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n[SENDER] interrupted")
    finally:
        if cap is not None:
            cap.release()
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        print("[SENDER] restoring ffmpeg")
        start_ffmpeg()
    return 0


if __name__ == "__main__":
    sys.exit(main())
