#!/usr/bin/env python3
"""副摄（下摄 /dev/video2）实测帧率探针：自动停推流 → ffmpeg 逐帧计时 → 自动恢复推流。

用法（车上，需 root——内部要调 systemctl）：
    python3 /root/dev/scripts/diag/cam_fps_probe.py             # 默认 30s，/dev/video2（副摄/下摄）
    python3 /root/dev/scripts/diag/cam_fps_probe.py 60          # 测 60s（中途 Ctrl+C 也照样给部分统计）
    python3 /root/dev/scripts/diag/cam_fps_probe.py --camera 0  # 测主摄（停 ffmpeg-stream.service）

停/恢复推流一律走 `vision.camera_guard`（= 循迹/台架/采集脚本同一条路径），不手写 systemctl：
它会在停服务后**等 /dev/videoN 真被释放**、并顺手 `reset-failed`——否则 ffmpeg 收 SIGTERM
后以非零码退出、单元落在 `failed`，stream-watch 看门狗"只救 failed"就会把刚停掉的推流
重新拉起，把摄像头抢回去（2026-10-12 实测踩到；坑与修法见 camera_guard.py docstring）。

帧率按 ffmpeg `-vf showinfo` 每帧日志行的**到达墙钟**计（请求 MJPG 640x480@30，与推流服务同参数）：
每 3s 一条心跳，帧间隔 >150ms 立刻告警，结尾给平均帧率 + 瞬时帧率 min/中位/p90/max + 曝光台阶分布。
帧率由曝光时间决定（33/60/120ms 台阶 → 30/16.7/8.3fps）：暗环境 16.7、操场白天应回 30。
"""
from __future__ import annotations

import os
import re
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from vision.camera_guard import start_ffmpeg, stop_ffmpeg      # noqa: E402

SVC = {0: "ffmpeg-stream.service", 2: "ffmpeg-stream-sub.service"}
FRAME_RE = re.compile(r"\bn:\s*\d+\s+pts:")                    # showinfo 每帧一行
FFMPEG_ERR_RE = re.compile(r"busy|Cannot open|No such|Input/output error|Invalid argument|VIDIOC", re.I)

# 曝光台阶对应的帧间隔分档（ms）—— 台阶分布一眼看出 AE 在哪个档
DT_BUCKETS = (
    (0, 25, "<25ms(高于30fps/突发)"),
    (25, 45, "30fps档(25~45ms)"),
    (45, 75, "16.7档(45~75ms)"),
    (75, 150, "中间/8.3档(75~150ms)"),
    (150, 1e9, "卡顿/停帧(>150ms)"),
)
HEARTBEAT_S = 3.0
STALL_MS = 150.0


def read_frames(dev: int, seconds: int):
    """跑 ffmpeg 逐帧读 showinfo，按日志行到达时刻计时。

    返回 (每帧到达墙钟列表, ffmpeg 关键错误行, 是否被 Ctrl+C 打断)。
    """
    cmd = ["timeout", str(seconds + 8), "ffmpeg", "-hide_banner", "-nostdin",
           "-f", "v4l2", "-input_format", "mjpeg", "-video_size", "640x480",
           "-framerate", "30", "-i", f"/dev/video{dev}",
           "-vf", "showinfo", "-f", "null", "-"]
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                         text=True, bufsize=1, start_new_session=True)
    stamps, errs, interrupted = [], [], False
    t0 = time.time()
    beat_t, beat_n = t0, 0
    try:
        for line in p.stderr:
            t = time.time()
            if FRAME_RE.search(line):
                stamps.append(t)
                if len(stamps) == 1:
                    print(f"  首帧到达（{t - t0:.1f}s，相机握手完成）")
                elif (t - stamps[-2]) * 1000.0 > STALL_MS:
                    print(f"  ⚠ 停帧 {(t - stamps[-2]) * 1000.0:.0f}ms（第 {len(stamps)} 帧前）")
            elif FFMPEG_ERR_RE.search(line) and len(errs) < 5:
                errs.append(line.strip())
                print(f"  [ffmpeg] {line.strip()[:150]}")
            if t - beat_t >= HEARTBEAT_S:                      # 现场心跳：别让人对着黑屏猜
                if stamps:
                    print(f"  [{t - t0:4.0f}s] 已到 {len(stamps)} 帧"
                          f"（近 {t - beat_t:.0f}s {(len(stamps) - beat_n) / (t - beat_t):.1f}fps）")
                else:
                    print(f"  [{t - t0:4.0f}s] 还没有帧——设备被占用或视频端点卡死（继续等…）")
                beat_t, beat_n = t, len(stamps)
            if t - t0 > seconds + 2:                           # 到点自己收，不等 timeout 兜底
                break
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if p.poll() is None:                                   # 提前退出也要整组杀净，别留孤儿占着摄像头
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except OSError:
                p.kill()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
        p.stderr.close()
    return stamps, errs, interrupted


def summarize(stamps, interrupted: bool) -> None:
    n = len(stamps)
    if n < 2:
        print(f"  ⇒ 只拿到 {n} 帧，不足以算节奏")
        return
    window = stamps[-1] - stamps[0]
    steady = (n - 1) / window if window > 0 else 0.0
    note = "（Ctrl+C 提前结束，样本比目标短）" if interrupted else ""
    print(f"  ⇒ 共 {n} 帧；测到 {window:.1f}s → 稳态平均 {steady:.1f}fps{note}")
    dts = [(b - a) * 1000.0 for a, b in zip(stamps, stamps[1:])]
    inst = sorted(1000.0 / d for d in dts if d > 1e-6)
    med_dt = sorted(dts)[len(dts) // 2]
    print(f"  ⇒ 帧间隔中位 {med_dt:.1f}ms；瞬时帧率 min {inst[0]:.1f}  中位 "
          f"{inst[len(inst) // 2]:.1f}  p90 {inst[int(len(inst) * 0.9)]:.1f}  max {inst[-1]:.1f} fps")
    parts = []
    for lo, hi, name in DT_BUCKETS:
        cnt = sum(1 for d in dts if lo <= d < hi)
        if cnt:
            parts.append(f"{name}={cnt}")
    if parts:
        print("  ⇒ 台阶分布: " + " · ".join(parts))
    if steady < 20:
        print("  暗/受限环境：帧率台阶由曝光决定（16.7/8.3），加光或锁曝光才能提帧；"
              "白天操场应回 30")


def main() -> int:
    dev = 2
    seconds = 30
    argv = [a for a in sys.argv[1:] if not a.startswith("--camera")]
    if "--camera" in sys.argv:
        dev = int(sys.argv[sys.argv.index("--camera") + 1])
    if argv:
        seconds = int(argv[0])

    svc = SVC.get(dev)
    if svc is None:
        print(f"不支持的摄像头 index {dev}（只有 0/2）")
        return 2
    subprocess.run(["v4l2-ctl", "-d", f"/dev/video{dev}", "-C", "gain", "-C", "exposure_auto"],
                   check=False)
    print(f"测量 {seconds}s × /dev/video{dev}（请求 30fps MJPG 640x480）；先停推流 {svc}…")
    stop_ffmpeg([svc])                                         # 停 + 等设备真释放 + reset-failed
    try:
        t0 = time.time()
        stamps, errs, interrupted = read_frames(dev, seconds)
        elapsed = time.time() - t0
        if not stamps:
            print(f"  [err] {elapsed:.1f}s 一帧都没有（设备被占用 / 视频端点卡死）：")
            for line in errs:
                print(f"  [ffmpeg] {line}")
            return 1
        summarize(stamps, interrupted)
    finally:
        start_ffmpeg([svc])                                    # 正常/异常/Ctrl+C 都会恢复推流
        print(f"恢复推流 {svc}；4s 后状态：", flush=True)       # flush：别让 is-active 的输出插到前面
        time.sleep(4)
        subprocess.run(["systemctl", "is-active", svc], check=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
