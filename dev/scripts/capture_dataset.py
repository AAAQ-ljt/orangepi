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
import sys
import time
from datetime import datetime

# 本脚本可能以裸 `python3 /root/dev/scripts/capture_dataset.py` 方式运行（run_capture.sh /
# camera0.sh / camera2.sh 都这么调），sys.path[0] 是脚本目录而非 dev/ —— 必须自己引导
# （2026-09-23 代码审查 A5：缺这一行时 `from vision.camera_guard import ...` 直接 ModuleNotFoundError）。
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from vision.camera_guard import open_camera, start_ffmpeg, stop_ffmpeg

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
    parser.add_argument("--count", type=int, default=0,
                        help="最多拍摄多少张后自动停止（0=不限，靠 Ctrl+C 或 --max-seconds 停）。"
                             "非交互/被脚本调用时**务必设上限**，否则进程可能被 orphan 后一直写盘")
    parser.add_argument("--max-seconds", type=float, default=1800.0,
                        help="最长拍摄时长（秒）兜底：SSH 断开 / 忘按 Ctrl+C 也不会无限写盘"
                             "（AGENTS.md §1.4.1 不写无退出条件的循环）")
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--show", action="store_true", help="显示 OpenCV 预览窗口")
    args = parser.parse_args()

    if args.show and not os.environ.get("DISPLAY"):
        print("[CAPTURE] WARNING: DISPLAY not set, preview window may not show. Saving will continue.")

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
        cap = open_camera(args.device, args.width, args.height)   # 等设备释放 + 重试
        if cap is None:
            print("[CAPTURE] ERROR: cannot open camera")
            return 1

        print("[CAPTURE] capturing... Ctrl+C to stop"
              + (f"（本次最多 {args.count} 张）" if args.count > 0 else "")
              + f"（最长 {args.max_seconds:.0f}s，到点自动停）")
        counter = 0
        started = time.time()

        if args.show:
            cv2.namedWindow("capture", cv2.WINDOW_NORMAL)
            cv2.startWindowThread()
            print("[CAPTURE] preview window enabled")
        while True:
            if time.time() - started >= args.max_seconds:
                print(f"[CAPTURE] 达到最长拍摄时长 {args.max_seconds:.0f}s，停止")
                break
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            counter += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(out_dir, f"frame_{ts}_{counter:04d}.jpg")
            cv2.imwrite(filename, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            print(f"[CAPTURE] saved {filename}")

            if args.count > 0 and counter >= args.count:
                print(f"[CAPTURE] 已达到 --count {args.count}，停止")
                break

            if args.show:
                cv2.imshow("capture", frame)
                key = cv2.waitKey(30)
                if key in (ord("q"), 27):
                    print("[CAPTURE] user quit")
                    break

            time.sleep(max(0.05, args.interval))
    except KeyboardInterrupt:
        print("\n[CAPTURE] interrupted by user")
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        print("[CAPTURE] restoring ffmpeg")
        start_ffmpeg()
        print("[CAPTURE] done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
