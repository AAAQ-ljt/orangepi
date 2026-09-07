#!/usr/bin/env python3
"""小车端数据集采集脚本（全新独立脚本，不修改官方代码）。

功能：
- 用 --folder 指定物体文件夹名，图片存到「当前目录/<文件夹名>」；
- 也可用 --output 指定完整保存路径；
- 每 0.5 秒拍摄一张 JPEG 照片；
- 自动停止 FFmpeg 推流释放摄像头，退出后自动恢复；
- Ctrl+C 安全退出。

用法：
    python3 /root/dev/scripts/capture_dataset.py --folder lane
    python3 /root/dev/scripts/capture_dataset.py --folder cone --interval 1.0
    python3 /root/dev/scripts/capture_dataset.py --output /root/dataset/lane
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
DEFAULT_BASE_DIR = "."


def folder_name_ok(name: str) -> bool:
    """只允许单层文件夹名，禁止路径分隔和 ..。"""
    if not name or name in (".", ".."):
        return False
    if "/" in name or "\\" in name:
        return False
    return True


def resolve_output_dir(args: argparse.Namespace) -> str:
    if args.output:
        return os.path.abspath(args.output)
    if args.folder:
        name = args.folder.strip()
        if not folder_name_ok(name):
            raise ValueError(
                f"非法文件夹名: {args.folder!r}（不要带 / 或 ..，只写物体名）"
            )
        return os.path.abspath(os.path.join(args.base, name))
    raise ValueError("请用 --folder 指定物体文件夹名，或用 --output 指定完整路径")


def stop_ffmpeg() -> None:
    for svc in FFMPEG_SERVICES:
        subprocess.run(["systemctl", "stop", svc], check=False)
    time.sleep(1.0)


def start_ffmpeg() -> None:
    for svc in FFMPEG_SERVICES:
        subprocess.run(["systemctl", "start", svc], check=False)


def main() -> int:
    parser = argparse.ArgumentParser(description="Car dataset capture")
    parser.add_argument(
        "--folder",
        "-f",
        help="物体文件夹名，图片保存到当前目录/<名字>，例如 lane / cone",
    )
    parser.add_argument(
        "--output",
        help="完整保存目录（优先于 --folder）。不填则用当前目录/--folder",
    )
    parser.add_argument(
        "--base",
        default=DEFAULT_BASE_DIR,
        help="配合 --folder 使用的根目录，默认是命令执行时的当前目录",
    )
    parser.add_argument("--interval", type=float, default=0.5, help="拍摄间隔秒数")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    try:
        out_dir = resolve_output_dir(args)
    except ValueError as exc:
        print(f"[CAPTURE] ERROR: {exc}", file=sys.stderr)
        return 2

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
