#!/usr/bin/env python3
"""云台归属探针（只读）：用户在网页操控云台时，对比两路推流的画面变化量。

哪一路的画面"跟着云台动"就是云台摄像头，另一路 = 固定下摄。
原理：从自建 mediamtx 分别拉 cam_car0027(video0) / cam_car0027_sub(video2) 两路，
每轮并行抓一帧 64x48 灰度缩略图，与上一轮做均值差：
   - 云台动 → 对应路 diff 冲高（静态噪声一般 <8，动作一般 >20）
   - 固定路 → diff 始终贴近自身噪声
不停止推流服务、不碰 PCA9685 —— 云台动作由用户在网页手动完成（121.40.149.155:8080）。

用法（车上）：
    mkdir -p /root/dev/img/diag
    nohup python3 /root/dev/scripts/diag/cam_role_probe.py > /root/dev/logs/cam_role_probe.log 2>&1 &
首次检测到单路大变化时，会把两路全尺寸帧存到 /root/dev/img/diag/role_spike_* 作证据。
跑满 --max-rounds（缺省 150，约 5~10 分钟）自动退出。
"""
from __future__ import annotations

import subprocess
import sys
import time

SPIKE_DIFF = 20.0
FULL_OUT = "/root/dev/img/diag"
ENV_FILE = "/etc/default/smartcar-media"
MAX_ROUNDS = max(int(sys.argv[1]), 20) if len(sys.argv) > 1 else 150


def media_rtsp() -> str:
    """推流地址从车上唯一配置读，脚本里不写任何凭据。"""
    try:
        with open(ENV_FILE, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.startswith("MEDIA_RTSP="):
                    return line.split("=", 1)[1].strip().strip("\"'")
    except OSError as e:
        print(f"读 {ENV_FILE} 失败: {e}", file=sys.stderr)
    return ""


def grab_gray(url: str, timeout: int = 25) -> bytes | None:
    p = subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
         "-fflags", "nobuffer", "-flags", "low_delay",
         "-rtsp_transport", "tcp", "-i", url, "-an",
         "-frames:v", "1", "-vf", "scale=64:48", "-f", "rawvideo",
         "-pix_fmt", "gray", "-"],
        capture_output=True, timeout=timeout)
    return p.stdout if p.returncode == 0 else None


def mean_absdiff(a: bytes | None, b: bytes | None) -> float:
    if a is None or b is None or len(a) != len(b):
        return -1.0
    return sum(abs(x - y) for x, y in zip(a, b)) / len(a)


def grab_jpg(url: str, path: str) -> bool:
    p = subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
         "-rtsp_transport", "tcp", "-i", url, "-an",
         "-frames:v", "1", "-q:v", "4", "-y", path],
        capture_output=True, timeout=25)
    return p.returncode == 0


def main() -> None:
    base = media_rtsp()
    if not base:
        print("拿不到 MEDIA_RTSP，退出", file=sys.stderr)
        return
    streams = {
        "video0_main": f"{base}/cam_car0027",
        "video2_sub": f"{base}/cam_car0027_sub",
    }
    prev: dict[str, bytes | None] = {}
    saved = False
    for rnd in range(MAX_ROUNDS):
        procs = []
        for name, url in streams.items():
            procs.append((name, subprocess.Popen(
                ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
                 "-fflags", "nobuffer", "-flags", "low_delay",
                 "-rtsp_transport", "tcp", "-i", url, "-an",
                 "-frames:v", "1", "-vf", "scale=64:48", "-f", "rawvideo",
                 "-pix_fmt", "gray", "-"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)))
        cur: dict[str, bytes | None] = {}
        for name, pr in procs:
            try:
                out, _ = pr.communicate(timeout=25)
                cur[name] = out if pr.returncode == 0 else None
            except subprocess.TimeoutExpired:
                pr.kill()
                cur[name] = None
        row = [f"{n}:{mean_absdiff(cur[n], prev.get(n)):6.1f}" for n, _ in streams.items()]
        print(f"r{rnd:03d} {time.strftime('%H:%M:%S')} " + "  ".join(row), flush=True)
        if rnd > 0:
            spike = [n for n in streams if mean_absdiff(cur[n], prev.get(n)) > SPIKE_DIFF]
            if len(spike) == 1:
                print(f"  ⇒ {spike[0]} 画面大变化 → 云台摄像头候选（另一路静止）", flush=True)
            if spike and not saved:
                ts = time.strftime("%H%M%S")
                ok = [grab_jpg(streams[n], f"{FULL_OUT}/role_spike_{ts}_{n}.jpg") for n in spike]
                saved = True
                print(f"  → 证据图已存 {FULL_OUT}/role_spike_{ts}_*（成功={all(ok)}）", flush=True)
        prev = cur
        time.sleep(0.8)


if __name__ == "__main__":
    main()