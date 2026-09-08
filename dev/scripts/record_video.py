#!/usr/bin/env python3
"""小车端录像脚本（全新独立脚本，不修改官方代码）。

功能：
- 视频保存到 /root/dev/video/，可用 --folder 再分子文件夹；
- 开始前停止 FFmpeg 推流释放摄像头，结束后自动恢复；
- Ctrl+C 停止录像。

用法：
    python3 /root/dev/scripts/record_video.py
    python3 /root/dev/scripts/record_video.py --folder lane
    python3 /root/dev/scripts/record_video.py --folder cone --device 2
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

FFMPEG_SERVICES = [
    "ffmpeg-stream.service",
    "ffmpeg-stream-sub.service",
]
DEFAULT_VIDEO_DIR = "/root/dev/video"
FFMPEG_BIN = "/usr/bin/ffmpeg"


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
        return os.path.abspath(os.path.join(DEFAULT_VIDEO_DIR, name))
    return os.path.abspath(DEFAULT_VIDEO_DIR)


def stop_ffmpeg() -> None:
    for svc in FFMPEG_SERVICES:
        subprocess.run(["systemctl", "stop", svc], check=False)
    time.sleep(1.0)


def start_ffmpeg() -> None:
    for svc in FFMPEG_SERVICES:
        subprocess.run(["systemctl", "start", svc], check=False)


def build_ffmpeg_cmd(
    device: int,
    width: int,
    height: int,
    fps: int,
    filename: str,
) -> list[str]:
    return [
        FFMPEG_BIN,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-y",
        "-thread_queue_size",
        "512",
        "-f",
        "v4l2",
        "-input_format",
        "mjpeg",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(fps),
        "-i",
        f"/dev/video{device}",
        "-c:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+frag_keyframe+empty_moov+default_base_moof",
        filename,
    ]


def stop_recorder(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=8)
        return
    except subprocess.TimeoutExpired:
        pass
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def main() -> int:
    parser = argparse.ArgumentParser(description="Car camera video recorder")
    parser.add_argument(
        "--folder",
        "-f",
        help="物体文件夹名，视频保存到 /root/dev/video/<名字>；不填则直接存 /root/dev/video",
    )
    parser.add_argument(
        "--output",
        help="完整保存目录（优先于 --folder）",
    )
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    try:
        out_dir = resolve_output_dir(args)
    except ValueError as exc:
        print(f"[RECORD] ERROR: {exc}", file=sys.stderr)
        return 2

    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(out_dir, f"vid_{ts}.mp4")
    dev_node = f"/dev/video{args.device}"
    if not os.path.exists(dev_node):
        print(f"[RECORD] ERROR: {dev_node} 不存在", file=sys.stderr)
        return 2

    print(f"[RECORD] save to {filename}")
    print(f"[RECORD] camera {args.device} {args.width}x{args.height}@{args.fps}")
    print("[RECORD] stopping ffmpeg to free camera")
    stop_ffmpeg()

    proc = None
    try:
        cmd = build_ffmpeg_cmd(
            args.device, args.width, args.height, args.fps, filename
        )
        print("[RECORD] recording... Ctrl+C to stop")
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        returncode = proc.wait()
        if returncode not in (0, 255):
            # ffmpeg often exits 255 on SIGINT after a clean stop
            print(f"[RECORD] ffmpeg exit {returncode}", file=sys.stderr)
            return 1
    except KeyboardInterrupt:
        print("\n[RECORD] interrupted by user")
        if proc is not None:
            stop_recorder(proc)
    finally:
        if proc is not None and proc.poll() is None:
            stop_recorder(proc)
        print("[RECORD] restoring ffmpeg")
        start_ffmpeg()
        if os.path.isfile(filename):
            size = os.path.getsize(filename)
            print(f"[RECORD] saved {filename} ({size} bytes)")
        else:
            print("[RECORD] no video file written")
        print("[RECORD] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
