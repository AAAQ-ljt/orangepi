"""摄像头封装：低延迟 + FFmpeg 推流服务占用处理。"""
from __future__ import annotations

import subprocess
import contextlib
from typing import Iterator

import cv2

FFMPEG_SERVICES = [
    "ffmpeg-stream.service",
    "ffmpeg-stream-sub.service",
]

CAMERA_IDS = [0, 2]
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


@contextlib.contextmanager
def camera_exclusive() -> Iterator[None]:
    """在打开摄像头前停止 FFmpeg，退出后恢复。"""
    try:
        for svc in FFMPEG_SERVICES:
            subprocess.run(["systemctl", "stop", svc], check=False)
        yield
    finally:
        for svc in FFMPEG_SERVICES:
            subprocess.run(["systemctl", "start", svc], check=False)


class Camera:
    def __init__(self, device_id: int = 0):
        self.device_id = device_id
        self.cap = None

    def open(self) -> bool:
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        # 低延迟：只保留最新一帧
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return self.cap.isOpened()

    def read(self):
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
