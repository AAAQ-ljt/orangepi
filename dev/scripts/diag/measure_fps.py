#!/usr/bin/env python3
"""帧率与处理耗时测量（2026-10-11 排查"为什么只有 15fps"）：
1) YUYV vs MJPG 空读帧率实测
2) lane_ref 检测单帧耗时
用法（车上，需先停推流释放摄像头）：sudo python3 measure_fps.py
"""
import os
import time
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import cv2


def bench(idx: int, fourcc: str = None, n: int = 60, w: int = 640, h: int = 480) -> float:
    c = cv2.VideoCapture(idx, cv2.CAP_V4L)
    if fourcc:
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    got = 0
    for _ in range(3):                      # 预热
        ok, _ = c.read()
        if not ok:
            break
    t = time.time()
    for _ in range(n):
        ok, _ = c.read()
        if not ok:
            break
        got += 1
    dt = time.time() - t
    c.release()
    fps = got / dt if dt > 0 else 0.0
    print(f"cam={idx} {w}x{h} fourcc={fourcc or 'YUYV'} -> {fps:.1f} FPS ({got}/{n} 帧)")
    return fps


def bench_detect(n: int = 30) -> None:
    from vision.lane_ref import LaneParams, LaneRefDetector
    c = cv2.VideoCapture(2, cv2.CAP_V4L)
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, frame = c.read()
    c.release()
    if not ok or frame is None:
        print("detect: 拿不到帧")
        return
    det = LaneRefDetector(LaneParams.from_settings())
    t = time.time()
    for _ in range(n):
        det.detect(frame)
    dt = (time.time() - t) / n
    print(f"lane_ref detect 单帧耗时: {dt*1000:.1f} ms（对应上限 {1/dt:.0f} FPS）")


if __name__ == "__main__":
    bench(2, None)                 # 640x480 YUYV
    bench(2, "MJPG")               # 640x480 MJPG
    bench(2, None, w=320, h=240)   # 320x240 YUYV（官方 2025 口径，看帧率换不换得到）
    bench(2, "MJPG", w=320, h=240)
    bench_detect()