"""摄像头独占守卫（camera_guard）单元测试：不碰真实摄像头。

覆盖 2026-09-19 实车踩到的两个坑：
  1. `systemctl stop` 返回 ≠ 设备已释放 → 紧接着开摄像头会 `can't open camera by index`。
     修法 = 按 /proc 扫占用者、等它空出来，并有超时兜底；
  2. 停掉的服务要 `reset-failed`，否则看门狗可能把它当"崩溃"重启、反过来抢摄像头。

这些函数在非 Linux（开发机）上必须**安全空转**（不能因为读不到 /proc 就炸）。

跑法：
    cd dev && PYTHONPATH=. python tests/test_camera_guard.py
"""
from __future__ import annotations

import os
import time

from vision.camera_guard import (FFMPEG_SERVICES, device_holders, open_camera,
                                 stop_ffmpeg)


def test_device_holders_is_safe_and_well_typed():
    """没有 /proc（开发机）时必须返回空列表；有 /proc（车端）时返回 pid 列表。

    注意：**车端跑这个测试时推流进程正占着摄像头**，所以不能断言"一定是空"——
    这条测试只保证"不抛异常 + 类型正确"。
    """
    holders = device_holders(2)
    assert isinstance(holders, list)
    assert all(isinstance(p, int) for p in holders)
    if not os.path.isdir("/proc"):          # 开发机（Windows）没有 procfs
        assert holders == []


def test_wait_device_free_returns_immediately_when_no_holders():
    from vision.camera_guard import wait_device_free as wdf
    assert wdf(97, timeout_s=0.5) is True, "不存在的设备没有占用者，应立即返回"


def test_wait_device_free_times_out(monkeypatch=None):
    """有占用者时按超时返回 False（这里用打桩模拟"一直有人在用"）。"""
    import vision.camera_guard as cg
    real = cg.device_holders
    cg.device_holders = lambda idx: [12345]          # 假装设备一直被占
    try:
        t0 = time.time()
        assert cg.wait_device_free(2, timeout_s=0.4, poll_s=0.1) is False
        assert 0.3 <= time.time() - t0 < 2.0, "应在超时附近返回"
    finally:
        cg.device_holders = real


def test_stop_ffmpeg_never_raises_without_systemctl():
    """没有 systemctl（开发机）时只当"没停成"，绝不抛异常。

    ⚠️ 2026-10-15 修正：**这个测试原来会真的 `systemctl stop` 掉两路推流**——
    在车上跑 `run_tests.sh` 会把图传整个停掉且不恢复（实测：跑完两路都 inactive，
    现场图传空档）。所以这里把 `_systemctl` 换成记录器：既不碰真服务，又能验证
    "对每个推流服务都发了 stop"（等价于 systemctl 缺失时"静默失败"的语义）。
    """
    import vision.camera_guard as cg

    calls = []
    real = cg._systemctl
    cg._systemctl = lambda *a: (calls.append(a), 1)[1]      # 模拟"没有 systemctl"（返回非零）
    try:
        stop_ffmpeg(services=FFMPEG_SERVICES, settle_s=0.0, timeout_s=0.2)
    finally:
        cg._systemctl = real
    stops = [c for c in calls if c and c[0] == "stop"]
    assert len(stops) == len(FFMPEG_SERVICES), f"应对每个服务都尝试 stop，实际 {calls}"


def test_open_camera_returns_none_for_missing_device():
    """打不开的摄像头索引 → 返回 None（并且不会卡太久）。"""
    cap = open_camera(97, 320, 240, timeout_s=0.6)
    assert cap is None


def test_crop_frame_removes_bottom_band():
    """画面裁剪（2026-10-15）：保留 y < settings.FRAME_CROP_LINE_Y 的行，只切底部。

    语义要钉住：
      · 默认（settings 值）→ 高度 = 384（保留 0~383 行），且**顶部原样**（行 0 不动）；
      · line_y=0 → 原样返回（同一对象，不做拷贝）；
      · line_y ≥ 画面高（小图/配置离谱）→ 原样返回，绝不能把画面裁没。
    """
    import numpy as np

    from config import settings
    from vision.camera_guard import crop_frame

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:384] = (10, 20, 30)
    frame[384:] = (99, 99, 99)                  # "车盖"区
    frame[0, 0] = (7, 8, 9)                     # 顶部标记行（在填充之后设，别被覆盖）
    out = crop_frame(frame)
    assert out.shape[0] == settings.FRAME_CROP_LINE_Y, f"高度不对：{out.shape}"
    assert out.shape[1] == 640 and out.shape[2] == 3
    assert tuple(out[0, 0]) == (7, 8, 9), "必须保留顶部（原点不能动）"
    assert not (out == 99).all(), "车盖区必须已被裁掉"
    assert crop_frame(frame, 0) is frame, "line_y=0 应原样返回"
    small = np.zeros((96, 640, 3), dtype=np.uint8)
    assert crop_frame(small) is small, "小图（高 < 裁剪线）应原样返回，别裁空"
    assert crop_frame(None) is None


if __name__ == "__main__":
    test_device_holders_is_safe_and_well_typed()
    test_wait_device_free_returns_immediately_when_no_holders()
    test_wait_device_free_times_out()
    test_stop_ffmpeg_never_raises_without_systemctl()
    test_open_camera_returns_none_for_missing_device()
    test_crop_frame_removes_bottom_band()
    print("test_camera_guard: all passed")
