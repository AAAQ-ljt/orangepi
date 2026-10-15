#!/usr/bin/env python3
"""cv2 读取路径的**帧率 / 重复帧**探针（与循迹脚本同源：vision.camera_guard.open_camera）。

为什么单独有这个（2026-10-15）
------------------------------
"下摄到底给了多少帧"有两种口径，别混：
  · 相机/驱动的能力：`scripts/diag/cam_fps_probe.py`（ffmpeg showinfo 逐帧计时）；
  · **循迹主循环实际消费到的**：cv2 `cap.read()` —— 这才是控制律 dt 的来源。
后者可能因为 `CAP_PROP_BUFFERSIZE` 被砍半（2026-10-15 实测 14.1 vs 28.0fps，
`camera_guard.open_camera` 已改掉）、也可能因为驱动/管线在灌**重复帧**
（消费速率 ≠ 信息速率 —— 那样 PID 的 dt 折算会被骗，等于"帧率虚高"）。
所以本探针除了测速率，还逐帧比"相邻帧内容差"，把重复帧占比直接打出来。

用法（车上，需 root：要停/恢复推流）：
    python3 /root/dev/scripts/diag/cam_cv2_fps.py            # 默认 10s，/dev/video2（下摄）
    python3 /root/dev/scripts/diag/cam_cv2_fps.py 30         # 测 30s
    python3 /root/dev/scripts/diag/cam_cv2_fps.py --camera 0 # 主摄
停/恢复推流走 camera_guard（等设备真释放 + reset-failed，别手写 systemctl）。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from config import settings                      # noqa: E402
from vision.camera_guard import camera_exclusive, open_camera   # noqa: E402

SVC = {0: "ffmpeg-stream.service", 2: "ffmpeg-stream-sub.service"}
DUP_DIFF = 0.5          # 灰度平均绝对差阈值：低于它算"同一帧"


def main() -> int:
    ap = argparse.ArgumentParser(description="cv2 读取路径帧率/重复帧探针")
    ap.add_argument("seconds", nargs="?", type=float, default=10.0, help="采样时长（秒）")
    ap.add_argument("--camera", type=int, default=2, help="摄像头 index（下摄=2）")
    ap.add_argument("--fourcc", default="MJPG")
    args = ap.parse_args()

    svc = SVC.get(args.camera)
    crop_y = int(getattr(settings, "FRAME_CROP_LINE_Y", 0) or 0)
    print(f"[FPS] 探针：/dev/video{args.camera}（{args.fourcc}）采 {args.seconds:.0f}s；"
          f"循迹同源路径；画面裁剪线 y<{crop_y}")
    with camera_exclusive(services=(svc,) if svc else ()):
        cap = open_camera(args.camera, settings.IMG_W, settings.IMG_H, fourcc=args.fourcc)
        if cap is None:
            print("[FPS] 摄像头打不开")
            return 1
        try:
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"[FPS] 协商到 {w}x{h}，CAP_PROP_FPS={cap.get(cv2.CAP_PROP_FPS):.1f}")
            t0 = time.monotonic()
            ts, diffs = [], []
            prev = None
            n_read = 0
            while time.monotonic() - t0 < args.seconds:
                ok, frame = cap.read()
                if not ok or frame is None:
                    continue
                now = time.monotonic()
                n_read += 1
                cur = frame[::4, ::4].astype(np.int16)
                if prev is not None:
                    diffs.append(float(np.abs(cur - prev).mean()))
                    ts.append(now)
                prev = cur
                print(f"[FPS] t={now - t0:5.1f}s 已读 {n_read} 帧", end="\r", flush=True)
        finally:
            cap.release()
    if len(ts) < 3:
        print("\n[FPS] 帧太少，测不到")
        return 1
    dt = np.diff(np.asarray(ts))
    dur = ts[-1] - ts[0]
    print(f"\n[FPS] ==== 结果（{len(ts)} 帧 / {dur:.1f}s）====")
    print(f"[FPS] 平均帧率 {len(ts) / dur:.1f} fps")
    print(f"[FPS] 帧间隔 ms：min {dt.min() * 1000:.1f}  中位 {np.median(dt) * 1000:.1f}  "
          f"p90 {np.percentile(dt, 90) * 1000:.1f}  max {dt.max() * 1000:.1f}"
          f"（30fps≈33.3ms / 16.7fps≈60ms）")
    if diffs:
        dups = sum(1 for d in diffs if d < DUP_DIFF)
        print(f"[FPS] 相邻帧内容差：中位 {np.median(diffs):.1f}  p10 {np.percentile(diffs, 10):.1f}"
              f"  → 重复帧 {dups}/{len(diffs)}（{dups / len(diffs) * 100:.0f}%）"
              f"，判定阈值 <{DUP_DIFF}")
    print("[FPS] 期望：亮环境 ~30fps（间隔 ~33ms、重复 0%）；暗环境 AE 长曝光 → 16.7/8.3fps。"
          "重复帧 >0 说明驱动/管线在灌旧帧，控制律的 dt 会虚高。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
