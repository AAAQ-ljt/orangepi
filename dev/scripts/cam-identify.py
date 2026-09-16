#!/usr/bin/env python3
"""一次性诊断：判断哪一路摄像头装在云台上（一次性实验，用完可删）

原理：让云台在两个俯仰角之间切换，分别抓两路画面。
      **画面跟着动的那一路 = 云台摄像头**，画面基本不变的那一路 = 固定摄像头（下摄）。

用法（车上，需 root）：
    python3 /root/dev/scripts/cam-identify.py
输出：打印两路画面的变化量，并把对比图写到 /root/dev/img/diag/
"""
from __future__ import annotations

import subprocess
import time

import cv2
import numpy as np

STREAMS = ["ffmpeg-stream.service", "ffmpeg-stream-sub.service"]
OUT_DIR = "/root/dev/img/diag"

TILT_A, TILT_B = 60, 120      # 两个俯仰角（度），别用极限值以免机械顶死
PAN = 90


def pwm_us(deg: float) -> float:
    return 500.0 + (max(0.0, min(180.0, deg)) / 180.0) * 2000.0


def main() -> None:
    subprocess.run(["systemctl", "stop", "opi-control.service"], check=False)
    for svc in STREAMS:
        subprocess.run(["systemctl", "stop", svc], check=False)
    time.sleep(2.5)

    from adafruit_extended_bus import ExtendedI2C as I2C
    from adafruit_pca9685 import PCA9685

    pca = PCA9685(I2C(5), address=0x40)
    pca.frequency = 50

    def gimbal(tilt_deg: float) -> None:
        pca.channels[2].duty_cycle = int(pwm_us(PAN) / 20000.0 * 65535)
        pca.channels[3].duty_cycle = int(pwm_us(tilt_deg) / 20000.0 * 65535)
        time.sleep(1.5)

    def grab(idx: int):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            print(f"  video{idx} 打不开")
            return None
        frame = None
        for _ in range(8):                 # 丢掉前几帧，等曝光稳定
            ok, frame = cap.read()
            if not ok:
                frame = None
        cap.release()
        return frame

    def change(a, b) -> float:
        if a is None or b is None:
            return -1.0
        return float(np.mean(cv2.absdiff(a, b)))

    try:
        gimbal(TILT_A)
        a0, a2 = grab(0), grab(2)
        gimbal(TILT_B)
        b0, b2 = grab(0), grab(2)
        gimbal(90)                         # 回中

        d0, d2 = change(a0, b0), change(a2, b2)
        print(f"云台 {TILT_A}° → {TILT_B}° 后画面变化量：")
        print(f"  /dev/video0 (index 0): {d0:8.1f}")
        print(f"  /dev/video2 (index 2): {d2:8.1f}")
        if d0 < 0 or d2 < 0:
            print("  有一路抓图失败，结论不可靠")
        elif d0 > d2 * 2 and d0 > 5:
            print("  ⇒ **video0 是云台摄像头**（画面随云台大幅变化）")
        elif d2 > d0 * 2 and d2 > 5:
            print("  ⇒ **video2 是云台摄像头**（画面随云台大幅变化）")
        else:
            print("  ⇒ 两者变化相近，无法判定（可能云台没动/卡住）")

        for name, img in (("tiltA_video0", a0), ("tiltB_video0", b0),
                          ("tiltA_video2", a2), ("tiltB_video2", b2)):
            if img is not None:
                cv2.imwrite(f"{OUT_DIR}/{name}.jpg", img)
        print(f"  对比图已写入 {OUT_DIR}/tilt*_video*.jpg")
    finally:
        for ch in (2, 3):
            pca.channels[ch].duty_cycle = int(pwm_us(90) / 20000.0 * 65535)
        pca.deinit()
        for svc in STREAMS:
            subprocess.run(["systemctl", "start", svc], check=False)
        subprocess.run(["systemctl", "start", "opi-control.service"], check=False)
        print("推流与 opi-control 已恢复")


if __name__ == "__main__":
    main()
