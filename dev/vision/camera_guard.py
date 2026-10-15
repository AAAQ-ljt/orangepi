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
from typing import Iterable, Iterator, List, Optional

from config import settings

FFMPEG_SERVICES: List[str] = [
    "ffmpeg-stream.service",
    "ffmpeg-stream-sub.service",
]

# 每路推流服务对应的摄像头设备 index（主摄 video0 / 副摄 video2）
# （2026-09-23 代码审查 B6：stop_ffmpeg 承诺"确认设备已释放"，但从不真的查设备占用）
SERVICE_DEVICE_INDEX = {
    "ffmpeg-stream.service": 0,
    "ffmpeg-stream-sub.service": 2,
}

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
        # 服务停了 ≠ 设备已释放（ffmpeg 可能还在退），按 /proc 确认占用者清空再返回
        # （2026-09-23 代码审查 B6：原来只轮询 is-active + 固定 sleep，docstring 承诺没兑现）
        dev_index = SERVICE_DEVICE_INDEX.get(svc)
        if dev_index is not None:
            remain = max(0.5, deadline - time.time())
            if not wait_device_free(dev_index, timeout_s=remain):
                print(f"[CAM] ⚠️ 停掉 {svc} 后 /dev/video{dev_index} 仍被占用："
                      f"{device_holders(dev_index)}（继续执行，打开摄像头时会重试）")
    if settle_s > 0:
        time.sleep(settle_s)


def start_ffmpeg(services: Iterable[str] = FFMPEG_SERVICES) -> None:
    """恢复推流服务。"""
    for svc in services:
        _systemctl("start", svc)


# ------------------------------------------------------------------ 画面裁剪
def crop_frame(frame, line_y: Optional[int] = None):
    """裁掉画面底部（车自己的车盖）：**保留 y < line_y 的行**，返回顶部切片。

    为什么要有它（2026-10-15 现场实测，logs/lane_20261015_*）
    --------------------------------------------------------
    固定机位下画面底部是**车自己的车盖**：它的上沿（实测 y≈415~463、过渡带最低 y≈392）
    在发车闸门的 ROI 里形成"暗蓝灰"连通域（HSV≈(101~105, 87~91, 87~91)），面积 0.059~0.135
    压着 0.06 阈值抖动 → 两次"没放板就发车"。它同样是任何亮/暗结构检测的污染源。
    裁掉 = 从所有消费方（闸门/检测/存图）一次性拆掉这类污染，比在每个消费方里各写一条
    "排除底部"更省事、也不会漏。

    `line_y=None` 取 `settings.FRAME_CROP_LINE_Y`；≤0 或 ≥ 画面高（小图/配置离谱）→ 原样返回。
    裁剪只切底部 —— 图像原点仍是左上角，所有"以 y 为坐标"的检测档参数（ROI/前瞻带/
    y_ref）**完全不受影响**（2026-10-15 实拍 12/12 帧读数逐位一致）。
    """
    if line_y is None:
        line_y = getattr(settings, "FRAME_CROP_LINE_Y", 0)
    line_y = int(line_y or 0)
    if line_y <= 0 or frame is None:
        return frame
    if line_y >= frame.shape[0]:       # 小图/配置离谱：不裁，别把画面裁没
        return frame
    return frame[:line_y]


def open_camera(index: int, width: int = 640, height: int = 480,
                timeout_s: float = 8.0, fourcc: str = None):
    """打开摄像头（带重试）：推流刚停时设备可能还没释放，等它空出来再开。

    `fourcc`（如 "MJPG"）可选：部分 USB 摄像头 MJPG 模式帧率明显高于默认
    YUYV（2026-10-09 T6，明天现场前后对比 FPS 再定默认值）。

    返回 cv2.VideoCapture；打不开返回 None（并打印占用者，便于定位是谁拿着设备）。
    """
    import cv2

    deadline = time.time() + timeout_s
    while True:
        cap = cv2.VideoCapture(index, cv2.CAP_V4L)
        if cap.isOpened():
            if fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            # ★ 帧率（2026-10-15 小车实测，指令级）：
            #   `CAP_PROP_BUFFERSIZE=1` 会把出帧率**砍半**——同一颗下摄、同一帧画面：
            #      BUFFERSIZE=1 → 14.1fps；不设（驱动默认 4）→ 28.0fps；BUFFERSIZE=4 → 28.5fps
            #   （单缓冲时驱动必须等应用把 buffer 放回才能抓下一帧；ffmpeg 走同一设备是 29.8fps。）
            #   代价权衡：默认 4 缓冲最多压 ~4 帧（≈130ms）滞后；我们读得比出帧快，
            #   队列基本是空的，所以"永远拿最新帧"的意图仍然成立。
            # ⚠️ 别再把它设回 1；FPS 必须显式要 30，否则协商到的捕获间隔可能是模组默认值。
            cap.set(cv2.CAP_PROP_FPS, 30)
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
