"""摄像头独占守卫：打开摄像头前后自动停止 / 恢复 ffmpeg 推流服务。

背景（AGENTS.md §1.2）：香橙派上的 ffmpeg 推流服务会长期占用摄像头设备，
不先停服务就直接 `cv2.VideoCapture` 会报 `device or resource busy`。

用法：
    from vision.camera_guard import camera_exclusive
    with camera_exclusive():
        cap = cv2.VideoCapture(0, cv2.CAP_V4L)
        ...

无论正常退出、异常、还是 Ctrl+C，都会恢复推流服务。
"""
from __future__ import annotations

import contextlib
import subprocess
import time
from typing import Iterable, Iterator, List

FFMPEG_SERVICES: List[str] = [
    "ffmpeg-stream.service",
    "ffmpeg-stream-sub.service",
]

# 停止/恢复推流后等待服务让出/拿回设备的秒数
DEFAULT_SETTLE_S = 1.0


def stop_ffmpeg(services: Iterable[str] = FFMPEG_SERVICES, settle_s: float = DEFAULT_SETTLE_S) -> None:
    """停止推流服务并等待设备释放。"""
    for svc in services:
        subprocess.run(["systemctl", "stop", svc], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if settle_s > 0:
        time.sleep(settle_s)


def start_ffmpeg(services: Iterable[str] = FFMPEG_SERVICES) -> None:
    """恢复推流服务。"""
    for svc in services:
        subprocess.run(["systemctl", "start", svc], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


@contextlib.contextmanager
def camera_exclusive(services: Iterable[str] = FFMPEG_SERVICES,
                     settle_s: float = DEFAULT_SETTLE_S,
                     restore: bool = True) -> Iterator[None]:
    """进入即停推流，退出即恢复（含异常路径）。"""
    stop_ffmpeg(services, settle_s=settle_s)
    try:
        yield
    finally:
        if restore:
            start_ffmpeg(services)
