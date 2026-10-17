#!/usr/bin/env python3
"""电调看门狗：主控卡死 / 被杀 / 未走到收尾时，强行把电调拉回 1500us（停车）+ 舵机回中。

为什么必须有它（2026-10-15 现场事故，两次丢车都要断电才停）
----------------------------------------------------------
PCA9685 会**一直输出最后一次写进去的脉宽** —— 所以只要循迹进程不能再写了：
  · 相机 UVC 卡死 → `cap.read()` 永久阻塞（下摄的已知故障，重启是标准恢复手段）；
  · 进程被 OOM / SIGKILL 杀掉，或异常没走到收尾；
  · 主循环被 I2C / SD 写卡住；
车就会**带着最后的油门前冲，而 Ctrl+C 完全没用**（信号只给主循环设标志，主循环已经不动了）。
这一层保护**必须独立于主控进程**，所以它是单独一个进程：

    python3 -u /root/dev/scripts/esc_watchdog.py --heartbeat /dev/shm/lane_hb --timeout 1.0

状态机（三条，别改乱）
----------------------
  ① 等心跳文件出现并**被更新过至少一次**（最多 --start-grace 秒；没等到 → 退出 0，不碰硬件）
  ② 监视：距上次更新 > --timeout → 写 ESC=`ESC_STOP_US` + 舵机 `SERVO_CENTER_ANGLE` →
     打印醒目告警 → 退出 2（**只跳一次**：再往后主控继续乱写就由人来断电）
  ③ 心跳文件被删掉（主控正常收尾）→ 静默退出 0

配合方式（lane_final_test.py 已内置）：主控**开跑前**建心跳文件并拉起本进程，
**每帧** `os.utime(心跳)` 喂一次（/dev/shm 上微秒级，不写 SD），收尾时先停车、再删心跳。

★ 它自己不能被误杀（2026-10-17）
------------------------------
它是"主控死了/卡住了"时的**唯一**退路，所以有三条自保（任一都别改坏）：
  1. **独立会话**（主控用 `start_new_session=True` 起它）：终端的 Ctrl+C、以及关 SSH 时
     PTY 关闭产生的 SIGHUP，都到不了它 —— 否则"操作员按 Ctrl+C / 断线"会连兜底一起带走
     （主循环卡在 I2C/写盘时，那 1 秒后本会写 1500us 的一手就没了：10-15 事故形态）；
  2. **显式忽略 SIGINT/SIGHUP/SIGQUIT**（`SIG_IGN`）：防"按 PID/进程组直接 kill"的那些发法
     （`kill -INT -<pgid>` 之类）。**SIGTERM 必须留着** —— 主控正常收尾就是用 `terminate()` 收它；
  3. **自己绝不抛**：所有打印走 `_say()`（管道断了也不抛），主循环整体再包一层 try
     （异常只记一行、继续值守）——安全网进程的职责就是"活到心跳停的那一刻"。

本地/无硬件自测：
    python3 esc_watchdog.py --heartbeat /tmp/hb --timeout 0.5 --dry-run      # 只打印不写硬件
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:                                                   # 本地无 repo 配置时也允许导入
    from config import settings                        # noqa: F401
    from config import site as _site                   # 触发 site.yaml 覆盖（舵机中位）
    _ = _site
except Exception:                                      # pragma: no cover
    settings = None                                    # type: ignore[assignment]

DEFAULT_TIMEOUT_S = 1.0        # 心跳超时（秒）：正常 25~30 帧/s，1s ≈ 丢掉 25~30 帧
DEFAULT_GRACE_S = 60.0         # 等"第一次心跳"的上限（含标定/开摄像头的时间）
POLL_S = 0.05
FALLBACK_ESC_STOP_US = 1500.0  # 兜底（理论上永远是 settings.ESC_STOP_US）
FALLBACK_SERVO_CENTER = 90.0


def _say(msg: str) -> None:
    """打印**绝不能抛**（管道断了 / 日志写不了 都不许影响守卫）——安全网的第一原则是别死。"""
    try:
        print(msg, flush=True)
    except Exception:                                  # noqa: BLE001
        pass


def _heartbeat_path(tag: str = "lane") -> str:
    """心跳文件路径：优先 /dev/shm（内存盘，不磨损 SD），没有就退回系统临时目录。"""
    for root in ("/dev/shm", tempfile.gettempdir()):
        if os.path.isdir(root) and os.access(root, os.W_OK):
            return os.path.join(root, f"esc_watchdog_{tag}_{os.getpid()}.hb")
    return os.path.join(tempfile.gettempdir(), f"esc_watchdog_{tag}.hb")


def _force_safe(dry_run: bool, why: str) -> int:
    """写安全值：电调 1500us + 舵机中位。"""
    stop_us = getattr(settings, "ESC_STOP_US", FALLBACK_ESC_STOP_US)
    center = getattr(settings, "SERVO_CENTER_ANGLE", FALLBACK_SERVO_CENTER)
    _say(f"[WATCHDOG] 🛑 心跳停了（{why}）→ 强制安全值："
         f"ESC={stop_us:.0f}us、舵机={center:.1f}°")
    if dry_run:
        _say("[WATCHDOG] (dry-run) 不写硬件")
        return 2
    try:
        from hardware.pca9685 import PCA9685
        max_us = getattr(settings, "ESC_DEBUG_MAX_US", 1600)
        pca = PCA9685(esc_max_us=int(max_us))
        pca.init()                                     # init 内部 center_all()
        pca.write_us(pca.CH_ESC, stop_us)
        pca.write_us(pca.CH_STEERING, pca.angle_to_us(center))
        _say("[WATCHDOG] ✅ 已写入安全值")
        return 2
    except Exception as exc:                           # noqa: BLE001
        _say(f"[WATCHDOG] ❌ 写硬件失败（{exc}）——请立刻断电！")
        return 3


def _ignore_stop_signals() -> None:
    """★ 安全网自己不能被误杀（见模块 docstring）：忽略 INT/HUP/QUIT，**保留 SIGTERM**。

    为什么单独成函数：本地/无硬件也 import 得动，且测试可以直接调它验语义。
    """
    for name in ("SIGINT", "SIGHUP", "SIGQUIT"):
        sig = getattr(signal, name, None)
        if sig is None:                                # Windows 没有 SIGHUP/SIGQUIT
            continue
        try:
            signal.signal(sig, signal.SIG_IGN)
        except (ValueError, OSError):
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description="电调看门狗（独立进程）")
    ap.add_argument("--heartbeat", required=True, help="主控每帧更新的心跳文件")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    ap.add_argument("--start-grace", type=float, default=DEFAULT_GRACE_S)
    ap.add_argument("--dry-run", action="store_true", help="只打印，不写硬件（本地测试）")
    args = ap.parse_args()

    # ★ 第一件事：把自己从"操作员的按键/断线"里摘出来（INT/HUP/QUIT 忽略；TERM 留着收尾用）
    _ignore_stop_signals()

    # 尽量别被 OOM 杀掉（本进程很小，OOM 应该先挑主控）
    try:
        with open("/proc/self/oom_score_adj", "w") as fh:
            fh.write("-1000")
    except Exception:                                   # noqa: BLE001
        pass

    hb = args.heartbeat
    t0 = time.monotonic()
    seen = 0.0
    last = None
    while True:
        # ★ 整圈再包一层：任何意外都只记一行、继续值守（安全网"死了"比"抛异常"严重得多）
        try:
            try:
                mtime = os.stat(hb).st_mtime
            except OSError:
                # 文件不在：还没建（等）或主控正常收尾时删掉了（走）
                # ⚠️ 这里必须用 seen（monotonic）比，别用 last（那是文件 mtime，墙上时钟）
                if seen and time.monotonic() - seen > args.timeout:
                    _say("[WATCHDOG] 心跳文件已删除（主控正常收尾）→ 退出")
                    return 0
                if not seen and time.monotonic() - t0 > args.start_grace:
                    _say("[WATCHDOG] 一直没见过心跳（主控没跑起来？）→ 退出，不碰硬件")
                    return 0
                time.sleep(POLL_S)
                continue
            if last is None or mtime > last:
                last, seen = mtime, time.monotonic()
                continue
            if time.monotonic() - seen > args.timeout:
                return _force_safe(args.dry_run, f"超时 {args.timeout:.1f}s 没有更新")
            time.sleep(POLL_S)
        except Exception as exc:                       # noqa: BLE001 —— 守卫绝不能死
            _say(f"[WATCHDOG] ⚠️ 内部异常（记一行，继续值守）：{exc}")
            time.sleep(POLL_S)


class WatchdogClient:
    """主控侧：建心跳 + 拉起看门狗进程 + 每帧喂 + 收尾收掉。"""

    def __init__(self, timeout_s: float = DEFAULT_TIMEOUT_S, tag: str = "lane",
                 grace_s: float = DEFAULT_GRACE_S, dry_run: bool = False) -> None:
        self.timeout_s = float(timeout_s)
        self.path = _heartbeat_path(tag)
        self.grace_s = float(grace_s)
        self.dry_run = bool(dry_run)
        self.proc: subprocess.Popen | None = None
        self.log_path = ""
        self._log_fh = None

    def start(self, log_path: str = "") -> None:
        """拉起看门狗。log_path = 它自己的日志（默认系统临时目录）——**绝不能让它继承我们的
        stdout**：那通常是 `| tee` 的管道，操作员会话一断，它下一次 print 就会抛
        BrokenPipeError 把自己搞死（安全网恰好死在最需要它的时候）。

        `start_new_session=True` = 独立会话：终端的 Ctrl+C 与关 SSH 的 SIGHUP 都到不了它；
        它在脚本里还会再显式忽略 INT/HUP（见 `_ignore_stop_signals`）。
        """
        with open(self.path, "w") as fh:
            fh.write("hb\n")
        os.utime(self.path)
        cmd = [sys.executable, "-u",
               os.path.join(os.path.dirname(os.path.abspath(__file__)), "esc_watchdog.py"),
               "--heartbeat", self.path, "--timeout", f"{self.timeout_s:.2f}",
               "--start-grace", f"{self.grace_s:.0f}"]
        if self.dry_run:
            cmd.append("--dry-run")
        self.log_path = log_path or os.path.join(tempfile.gettempdir(), "esc_watchdog.log")
        try:
            self._log_fh = open(self.log_path, "ab", buffering=0)
            out = self._log_fh
        except OSError:                                  # 打不开就丢弃输出，但绝不继承管道
            self._log_fh = None
            out = subprocess.DEVNULL
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=out,
                                     stderr=subprocess.STDOUT, start_new_session=True)
        print(f"[LANE] 电调看门狗已拉起（超时 {self.timeout_s:.1f}s 不喂心跳就强制停车；"
              f"pid={self.proc.pid}；独立会话；日志={self.log_path}）")

    def feed(self) -> None:
        try:
            os.utime(self.path)
        except OSError:
            pass

    def stop(self) -> "int | None":
        """收尾：先删心跳（告诉它主控已安全停车），再收掉进程。

        返回子进程退出码（2/3 = 它触发过并写了安全值 → 调用方据此告警），
        正常收掉时通常是 0/None。
        """
        try:
            os.unlink(self.path)
        except OSError:
            pass
        code = None
        if self.proc is not None:
            try:
                self.proc.terminate()                    # SIGTERM 是"留着"的那个信号（没被忽略）
                code = self.proc.wait(timeout=2.0)
            except Exception:                            # noqa: BLE001
                try:
                    self.proc.kill()
                    code = self.proc.wait(timeout=2.0)
                except Exception:                        # noqa: BLE001
                    pass
            print(f"[LANE] 电调看门狗已退出（退出码 {code}）")
        if self._log_fh is not None:
            try:
                self._log_fh.close()
            except Exception:                            # noqa: BLE001
                pass
            self._log_fh = None
        return code


if __name__ == "__main__":
    sys.exit(main())
