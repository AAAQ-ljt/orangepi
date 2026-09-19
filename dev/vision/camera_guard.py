"""摄像头独占守卫：打开摄像头前后自动停止 / 恢复 ffmpeg 推流服务。

背景（AGENTS.md §1.2）：香橙派上的 ffmpeg 推流服务会长期占用摄像头设备，
不先停服务就直接 `cv2.VideoCapture` 会报 `device or resource busy`。

用法：
    from vision.camera_guard import camera_exclusive, open_camera
    with camera_exclusive():
        cap = open_camera(2, 640, 480)          # 等设备真的空闲再开，带重试
        if cap is None:
            ...
        ...

无论正常退出、异常、还是 Ctrl+C，都会恢复推流服务。

2026-09-19 实测补充（两个真踩到的坑）：
1. `systemctl stop` 返回**不等于**设备已释放：ffmpeg 可能正卡在往媒体服务器写数据，
   要过一两秒才真正退出 → 紧接着 `cv2.VideoCapture` 会 `can't open camera by index`。
   所以这里按 **/proc 扫描设备占用者**、等它真的空出来，超时才上 SIGKILL。
2. 被 stop 掉的服务如果不 `reset-failed`，看门狗（stream-watch）可能把它当"崩溃"重启，
   反过来抢走摄像头 → 停完顺手 reset-failed。
"""
from __future__ import annotations

import contextlib
import os
import subprocess
import time
from typing import Iterable, Iterator, List

FFMPEG_SERVICES: List[str] = [
    "ffmpeg-stream.service",
    "ffmpeg-stream-sub.service",
]

# 停止/恢复推流后等待服务让出/拿回设备的秒数
DEFAULT_SETTLE_S = 1.0
STOP_TIMEOUT_S = 6.0        # 停服务后等设备释放的上限（超时上 SIGKILL）


def _systemctl(*args: str) -> int:
    """调用 systemctl；没有 systemctl 的机器（开发用的 Windows）安全返回非零。"""
    try:
        return subprocess.run(["systemctl", *args], check=False,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
    except (FileNotFoundError, OSError):
        return 1


def _is_active(service: str) -> bool:
    return _systemctl("is-active", "--quiet", service) == 0


def device_holders(index: int) -> List[int]:
    """哪些进程正打开着 /dev/video<index>（纯 /proc 扫描，不依赖 fuser）。"""
    dev = f"/dev/video{index}"
    holders: List[int] = []
    try:
        pids = [p for p in os.listdir("/proc") if p.isdigit()]
    except OSError:
        return holders                      # 非 Linux 环境（开发机）直接返回空
    for pid in pids:
        fd_dir = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.readlink(f"{fd_dir}/{fd}") == dev:
                    holders.append(int(pid))
                    break
            except OSError:
                continue
    return holders


def wait_device_free(index: int, timeout_s: float = STOP_TIMEOUT_S,
                     poll_s: float = 0.2) -> bool:
    """等到 /dev/video<index> 没有进程占用；超时返回 False。"""
    deadline = time.time() + timeout_s
    while True:
        if not device_holders(index):
            return True
        if time.time() >= deadline:
            return False
        time.sleep(poll_s)


def stop_ffmpeg(services: Iterable[str] = FFMPEG_SERVICES,
                settle_s: float = DEFAULT_SETTLE_S,
                timeout_s: float = STOP_TIMEOUT_S) -> None:
    """停止推流服务，并**确认它们真的释放了设备**才返回。"""
    services = list(services)
    for svc in services:
        _systemctl("stop", svc)
    deadline = time.time() + timeout_s
    for svc in services:
        while time.time() < deadline and _is_active(svc):
            time.sleep(0.2)
        if _is_active(svc):                 # 卡在写网络 / 不响应 SIGTERM → 硬杀
            _systemctl("kill", "--signal=SIGKILL", svc)
            time.sleep(0.5)
        _systemctl("reset-failed", svc)     # 别让看门狗把"我们停的"当成"它崩了"
    if settle_s > 0:
        time.sleep(settle_s)


def start_ffmpeg(services: Iterable[str] = FFMPEG_SERVICES) -> None:
    """恢复推流服务。"""
    for svc in services:
        _systemctl("start", svc)


def open_camera(index: int, width: int = 640, height: int = 480, timeout_s: float = 8.0):
    """打开摄像头（带重试）：推流刚停时设备可能还没释放，等它空出来再开。

    返回 cv2.VideoCapture；打不开返回 None（并打印占用者，便于定位是谁拿着设备）。
    """
    import cv2

    deadline = time.time() + timeout_s
    while True:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return cap
        cap.release()
        if time.time() >= deadline:
            holders = device_holders(index)
            extra = f"，占用它的进程: {holders}" if holders else ""
            print(f"[CAM] 打不开 /dev/video{index}（等了 {timeout_s:.0f}s）{extra}")
            if holders:
                print("[CAM] 提示：推流服务没退干净。先看 systemctl status ffmpeg-stream{,-sub}.service；"
                      "仍占用就 systemctl kill -s SIGKILL 它")
            return None
        time.sleep(0.4)


@contextlib.contextmanager
def camera_exclusive(services: Iterable[str] = FFMPEG_SERVICES,
                     settle_s: float = DEFAULT_SETTLE_S,
                     restore: bool = True) -> Iterator[None]:
    """进入即停推流（并等设备真的释放），退出即恢复（含异常路径）。"""
    stop_ffmpeg(services, settle_s=settle_s)
    try:
        yield
    finally:
        if restore:
            start_ffmpeg(services)
