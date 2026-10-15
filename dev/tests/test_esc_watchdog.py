#!/usr/bin/env python3
"""电调看门狗（esc_watchdog）单元测试：不碰硬件（--dry-run）、不碰电机。

覆盖 2026-10-15 现场事故的机上防线：
  · 喂心跳 → 不许触发；
  · 停喂 → 必须在超时后**触发**（写安全值 + 退出 2）；
  · 主控正常收尾（删心跳文件）→ 静默退出 0；
  · 一直没见过心跳（主控没跑起来）→ 退出 0，绝不碰硬件；
  · WatchdogClient 的 start/feed/stop 生命周期（心跳文件建→喂→删、子进程收掉）。

跑法：cd dev && PYTHONPATH=. python tests/test_esc_watchdog.py
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

WATCHDOG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "scripts", "esc_watchdog.py")


def _spawn(hb: str, timeout_s: float, grace: float = 10.0):
    return subprocess.Popen([sys.executable, "-u", WATCHDOG, "--heartbeat", hb,
                             "--timeout", str(timeout_s), "--start-grace", str(grace),
                             "--dry-run"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def test_keeps_quiet_while_fed() -> None:
    hb = os.path.join(tempfile.mkdtemp(prefix="wd_"), "hb")
    open(hb, "w").write("hb\n")
    p = _spawn(hb, 0.8)
    try:
        t0 = time.time()
        while time.time() - t0 < 1.5:                    # 喂 1.5s（>timeout）
            os.utime(hb)
            time.sleep(0.05)
        assert p.poll() is None, "喂着心跳却触发了看门狗"
    finally:
        p.kill()
        p.wait(timeout=3)


def test_trips_when_heartbeat_stops() -> None:
    hb = os.path.join(tempfile.mkdtemp(prefix="wd_"), "hb")
    open(hb, "w").write("hb\n")
    p = _spawn(hb, 0.5)
    t0 = time.time()
    while time.time() - t0 < 0.8:                        # 先喂一会儿（让它进入监视）
        os.utime(hb)
        time.sleep(0.05)
    out, _ = p.communicate(timeout=6)                    # 停止喂 → 应触发
    assert p.returncode == 2, f"应触发（退出码 2），实得 {p.returncode}\n{out}"
    assert "强制安全值" in out, f"触发时必须打印醒目告警：{out}"


def test_clean_exit_when_heartbeat_removed() -> None:
    hb = os.path.join(tempfile.mkdtemp(prefix="wd_"), "hb")
    open(hb, "w").write("hb\n")
    p = _spawn(hb, 5.0)                                  # timeout 很长，避免误触发
    t0 = time.time()
    while time.time() - t0 < 0.6:
        os.utime(hb)
        time.sleep(0.05)
    os.unlink(hb)                                        # 主控正常收尾
    out, _ = p.communicate(timeout=6)
    assert p.returncode == 0, f"正常收尾应静默退出 0，实得 {p.returncode}\n{out}"


def test_never_fires_without_first_heartbeat() -> None:
    """主控根本没跑起来（心跳文件一直不在）→ 到 grace 就退出，绝不写硬件。"""
    hb = os.path.join(tempfile.mkdtemp(prefix="wd_"), "nope")
    p = _spawn(hb, 0.3, grace=0.8)
    out, _ = p.communicate(timeout=6)
    assert p.returncode == 0, f"没见过心跳应退出 0，实得 {p.returncode}\n{out}"
    assert "强制安全值" not in out, f"没见过心跳不该写硬件：{out}"


def test_client_lifecycle() -> None:
    """WatchdogClient：start 建心跳并拉起子进程；feed 更新；stop 删心跳 + 收掉子进程。"""
    from scripts.esc_watchdog import WatchdogClient
    wd = WatchdogClient(timeout_s=5.0, tag="test", grace_s=5.0, dry_run=True)
    wd.start()
    try:
        assert os.path.isfile(wd.path), "start 应建出心跳文件"
        t1 = os.stat(wd.path).st_mtime
        time.sleep(0.05)
        wd.feed()
        assert os.stat(wd.path).st_mtime >= t1, "feed 应更新心跳"
        assert wd.proc is not None and wd.proc.poll() is None, "子进程应活着"
    finally:
        wd.stop()
    assert not os.path.exists(wd.path), "stop 应删掉心跳"
    assert wd.proc is not None and wd.proc.poll() is not None, "stop 应收掉子进程"


def main() -> int:
    funcs = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v) and getattr(v, "__module__", "") == __name__]
    failed = 0
    for fn in funcs:
        try:
            fn()
            print(f"  ✅ {fn.__name__}")
        except Exception as exc:                          # noqa: BLE001
            failed += 1
            import traceback
            print(f"  ❌ {fn.__name__}: {exc}")
            traceback.print_exc()
    print(f"[test_esc_watchdog] {len(funcs) - failed}/{len(funcs)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
