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

import time

from vision.camera_guard import (FFMPEG_SERVICES, device_holders, open_camera,
                                 stop_ffmpeg, wait_device_free)


def test_device_holders_is_safe_without_procfs():
    """开发机（Windows）没有 /proc：必须返回空列表而不是抛异常。"""
    assert device_holders(2) == []


def test_wait_device_free_returns_immediately_when_no_holders():
    t0 = time.time()
    assert wait_device_free(2, timeout_s=1.0) is True
    assert time.time() - t0 < 0.5, "没有占用者时应立即返回"


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
    """没有 systemctl（开发机）时只当"没停成"，绝不抛异常。"""
    stop_ffmpeg(services=FFMPEG_SERVICES, settle_s=0.0, timeout_s=0.2)


def test_open_camera_returns_none_for_missing_device():
    """打不开的摄像头索引 → 返回 None（并且不会卡太久）。"""
    cap = open_camera(97, 320, 240, timeout_s=0.6)
    assert cap is None


if __name__ == "__main__":
    test_device_holders_is_safe_without_procfs()
    test_wait_device_free_returns_immediately_when_no_holders()
    test_wait_device_free_times_out()
    test_stop_ffmpeg_never_raises_without_systemctl()
    test_open_camera_returns_none_for_missing_device()
    print("test_camera_guard: all passed")
