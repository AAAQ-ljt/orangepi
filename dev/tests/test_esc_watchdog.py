#!/usr/bin/env python3
"""电调看门狗（esc_watchdog）单元测试：不碰硬件（--dry-run）、不碰电机。

覆盖 2026-10-15 现场事故的机上防线：
  · 喂心跳 → 不许触发；
  · 停喂 → 必须在超时后**触发**（写安全值 + 退出 2）；
  · 主控正常收尾（删心跳文件）→ 静默退出 0；
  · 一直没见过心跳（主控没跑起来）→ 退出 0，绝不碰硬件；
  · WatchdogClient 的 start/feed/stop 生命周期（心跳文件建→喂→删、子进程收掉）。

2026-10-17 增补（安全网自己不能被误杀）：
  · SIGINT/SIGHUP（操作员 Ctrl+C / 关会话断线）→ 必须**活着**，随后停喂仍要触发；
  · WatchdogClient 起的子进程在新会话（setsid）里、输出落自己的日志（不继承会被打断的管道）；
  · SIGTERM 必须**留着**（主控收尾就是 terminate 收它）。

跑法：cd dev && PYTHONPATH=. python tests/test_esc_watchdog.py
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

WATCHDOG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "scripts", "esc_watchdog.py")

_SKIPPED: list = []          # 跳过的用例要单独报出来，不许混进"通过"里


def _linux_only(why: str) -> bool:
    """信号语义/进程会话这些要 Linux 才算数（Windows 上 os.kill 是另一套）。"""
    if sys.platform.startswith("linux"):
        return True
    _SKIPPED.append(why)
    return False


def _spawn(hb: str, timeout_s: float, grace: float = 10.0):
    return subprocess.Popen([sys.executable, "-u", WATCHDOG, "--heartbeat", hb,
                             "--timeout", str(timeout_s), "--start-grace", str(grace),
                             "--dry-run"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def _feed_for(hb: str, seconds: float) -> None:
    t0 = time.time()
    while time.time() - t0 < seconds:
        os.utime(hb)
        time.sleep(0.05)


def _session_of(pid: int) -> int:
    """读 /proc/<pid>/stat 的会话号（第 6 字段）——用来验"它在新会话里"。"""
    with open(f"/proc/{pid}/stat", "r", encoding="utf-8", errors="replace") as fh:
        tail = fh.read().rsplit(") ", 1)[-1].split()
    return int(tail[3])          # tail = state,ppid,pgrp,session,… ⇒ session 是第 4 个


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


def test_survives_int_and_hup_then_still_trips() -> None:
    """★ 2026-10-17：操作员的 Ctrl+C / 关会话断线（SIGINT/SIGHUP）**不能**把安全网带走。

    为什么：主循环卡在 I2C/写盘时，操作员按 Ctrl+C 只置了标志、循环读不到——那 1 秒后
    "看门狗写 1500us"就是**唯一**的救命一手；如果信号顺手把看门狗也杀了，车就带着最后的
    脉宽一直跑（10-15 事故形态）。这条钉住：INT/HUP 打不动它，之后停喂心跳仍必须触发。
    """
    if not _linux_only("test_survives_int_and_hup_then_still_trips（需要 Linux 信号语义）"):
        print("      ⏭️ 跳过 test_survives_int_and_hup_then_still_trips（车上会真跑）")
        return
    hb = os.path.join(tempfile.mkdtemp(prefix="wd_sig_"), "hb")
    open(hb, "w").write("hb\n")
    p = _spawn(hb, 0.5)
    try:
        _feed_for(hb, 0.8)                                 # 先让它进入监视状态
        os.kill(p.pid, signal.SIGINT)                      # = 终端 Ctrl+C
        os.kill(p.pid, signal.SIGHUP)                      # = 关 SSH / 关 5G 断线
        time.sleep(0.3)
        assert p.poll() is None, "看门狗被 SIGINT/SIGHUP 杀掉了 —— 安全网在最需要它的场景里消失"
    except Exception:
        p.kill()
        p.wait(timeout=3)
        raise
    out, _ = p.communicate(timeout=6)                      # 停喂心跳 → 应触发
    assert p.returncode == 2, f"停喂后应触发（退出码 2），实得 {p.returncode}\n{out}"
    assert "强制安全值" in out, f"触发时必须打印醒目告警：{out}"


def test_term_still_stops_watchdog() -> None:
    """★ SIGTERM 必须**留着**：主控正常收尾就是 `terminate()` 收它（忽略了 TERM 就会变硬杀）。"""
    if not _linux_only("test_term_still_stops_watchdog（需要 Linux 信号语义）"):
        print("      ⏭️ 跳过 test_term_still_stops_watchdog（车上会真跑）")
        return
    hb = os.path.join(tempfile.mkdtemp(prefix="wd_term_"), "hb")
    open(hb, "w").write("hb\n")
    p = _spawn(hb, 5.0)                                    # 超时长一些，避免误触发
    _feed_for(hb, 0.5)
    p.terminate()                                          # = WatchdogClient.stop()
    out, _ = p.communicate(timeout=6)
    assert p.returncode == -signal.SIGTERM, \
        f"应被 SIGTERM 终止（returncode=-15），实得 {p.returncode}\n{out}"
    assert "强制安全值" not in out, "TERM 是收尾信号，不该触发写安全值"


def test_client_detaches_and_logs() -> None:
    """★ WatchdogClient 起的子进程：**独立会话**（终端的 Ctrl+C/HUP 到不了）+ 输出进自己的日志。

    这两件事是配套的：会话隔离防"按键/断线误杀"，日志文件防"它继承了我们那条 `| tee` 管道、
    操作员一断线它下一次 print 就 BrokenPipeError 自杀"。日志顺带是复盘证据（🛑 那几行）。
    """
    if not _linux_only("test_client_detaches_and_logs（需要 /proc 与会话语义）"):
        print("      ⏭️ 跳过 test_client_detaches_and_logs（车上会真跑）")
        return
    from scripts.esc_watchdog import WatchdogClient
    tmp = tempfile.mkdtemp(prefix="wd_sess_")
    log = os.path.join(tmp, "esc_watchdog.log")
    wd = WatchdogClient(timeout_s=0.5, tag="testsess", grace_s=5.0, dry_run=True)
    wd.start(log_path=log)
    try:
        assert wd.proc is not None
        assert wd.proc.stdout is None, "不该用管道接它的输出（我们的管道会先断）"
        _feed_for(wd.path, 0.4)
        sess = _session_of(wd.proc.pid)
        assert sess == wd.proc.pid, f"看门狗没在新会话里：session={sess} pid={wd.proc.pid}"
        os.unlink(wd.path)                                 # 模拟"主控正常收尾"
        code = wd.proc.wait(timeout=4)
        assert code == 0, f"删心跳后应静默退出 0，实得 {code}"
    finally:
        wd.stop()
    with open(log, "r", encoding="utf-8", errors="replace") as fh:
        txt = fh.read()
    assert "[WATCHDOG]" in txt and "心跳文件已删除" in txt, \
        f"看门狗的输出应落进自己的日志文件：{txt!r}"


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
    print(f"[test_esc_watchdog] {len(funcs) - len(_SKIPPED) - failed}/{len(funcs) - len(_SKIPPED)} passed"
          + (f"，{len(_SKIPPED)} 个跳过未验证：" + "；".join(_SKIPPED) if _SKIPPED else ""))
    if _SKIPPED:
        print("      ⚠️ 跳过的用例**没有**验证过——车上（Linux）跑一遍才算数")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
