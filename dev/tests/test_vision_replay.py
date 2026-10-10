"""vision_main 离线回放端到端测试（无摄像头/无硬件，2026-10-09 T0/T1）。

起真实 vision_main 子进程 + `--replay-frames` 假摄像头序列，从 UDP 收包断言：
  1. 发车边沿（板帧→空帧）→ `start_released` 置位、开场 `board_blocked` 为真；
  2. 放行后 lane 通道（`--lane-method ref`）给出 `center_x`（A 路径循迹闭环的离线验证）；
  3. 回放放完进程自己正常退出（rc=0），不残留子进程。

数据安全：只往 127.0.0.1:随机端口发包，不碰摄像头/推流/PCA9685；
`--replay-frames` 会跳过 `camera_exclusive()`（nullcontext），本地无 systemctl 也不报错。

跑法：
    cd dev && PYTHONPATH=. python tests/test_vision_replay.py
"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

DEV = Path(__file__).resolve().parent.parent


def _board_frame() -> np.ndarray:
    """合成蓝板帧：画面中央一大块纯蓝（进 StartGate 判据①面积占比/②主导度）。"""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[100:420, 100:560] = (200, 0, 0)      # BGR 纯蓝
    return img


def _lane_frame() -> np.ndarray:
    """合成赛道帧：中性灰底 + 两条斜白线（lane_ref Canny+Hough 应能给中心）。

    线形约束（离线验证过）：
    1. 斜率必须落进 0.25~2.0 闸门（(150,480)→(280,100) 那种 |k|=2.9 会被滤掉）；
    2. 前瞻带（y≈199）处的配对宽度要 ≥120（过窄配不上对，不给中心）。
    底色不能带蓝色分量：StartGate 的"蓝色主导度"对红棕底会误判"有板"永不发车。
    """
    img = np.full((480, 640, 3), (70, 70, 70), dtype=np.uint8)
    cv2.line(img, (40, 480), (320, 80), (255, 255, 255), 6)
    cv2.line(img, (600, 480), (320, 80), (255, 255, 255), 6)
    return img


def _imwrite_unicode(path: Path, img: np.ndarray) -> None:
    """cv2.imwrite 遇中文路径静默失败（与 imread 同款 Unicode 病），用 imencode+tofile。"""
    ok, buf = cv2.imencode(".jpg", img)
    assert ok, "imencode 失败"
    buf.tofile(str(path))


def _write_frames(directory: Path, n_board: int = 8, n_lane: int = 30) -> None:
    for i in range(n_board):
        _imwrite_unicode(directory / f"board_{i:04d}.jpg", _board_frame())
    for i in range(n_lane):
        _imwrite_unicode(directory / f"lane_{i:04d}.jpg", _lane_frame())


def _recv(port: int, n_packets: int, timeout_s: float = 25.0) -> list:
    """收 UDP 包直到 n_packets 或超时。"""
    msgs = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    sock.settimeout(0.2)
    end = time.time() + timeout_s
    while len(msgs) < n_packets and time.time() < end:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        try:
            msgs.append(json.loads(data.decode("utf-8")))
        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
    sock.close()
    return msgs


def _run_vision(frames_dir: Path, port: int, status_file: Path) -> subprocess.Popen:
    env = dict(os.environ)
    env["PYTHONPATH"] = str(DEV)
    cmd = [sys.executable, str(DEV / "vision" / "vision_main.py"),
           "--replay-frames", str(frames_dir), "--replay-once",
           "--udp-port", str(port), "--camera", "2", "--gate-camera", "0",
           "--status-file", str(status_file), "--lane-method", "ref"]
    return subprocess.Popen(cmd, cwd=str(DEV), env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def test_replay_full_flow_with_chinese_path():
    """离线全流程：发车边沿 → 放行 → 循迹中心 → 回放结束自动退出。

    用**中文目录名**顺带覆盖 imread_unicode 路径（cv2.imread 在 Windows 读不了中文路径）。
    """
    with tempfile.TemporaryDirectory(prefix="回放测试") as tmp:
        d = Path(tmp)
        _write_frames(d)
        port = 39000 + (os.getpid() % 500)
        proc = _run_vision(d, port, d / "status.json")
        try:
            msgs = _recv(port, n_packets=36)
            out, _ = proc.communicate(timeout=30)
        finally:
            if proc.poll() is None:
                proc.kill()
        rc = proc.returncode
        assert rc == 0, f"回放结束应自己退出 rc=0，实际 {rc}\n{out[-2000:]}"
        assert msgs, "一条 UDP 都没收到（vision_main 没起来或没发包）"

        assert any(m.get("board_blocked") for m in msgs[:12]), "开场应看到蓝板"
        released = [m for m in msgs if m.get("start_released")]
        assert released, "板帧→空帧的边沿应触发 start_released"
        lane = [m for m in msgs if m.get("center_x") is not None]
        assert lane, f"放行后 lane 通道应给 center_x（实际 {len(lane)}/{len(msgs)} 帧有中心）"


if __name__ == "__main__":
    test_replay_full_flow_with_chinese_path()
    print("test_vision_replay: all passed")