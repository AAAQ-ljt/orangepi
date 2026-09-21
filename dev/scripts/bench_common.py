"""台架脚本的公共安全层：让"跑电机"这件事在任何退出路径下都不会留尾巴。

从 `bench_board_test.py` 里抽出来的（那份已经实机验证过），三件事：

1. **MotorSession**：进去时停 `opi-control` 让出 PCA9685、初始化并解锁电调、云台回中；
   出来时（正常/异常/Ctrl+C/关会话）电调归零、舵机回中、释放 PCA9685、重启 opi-control；
2. **restore_remote_stage()**：恢复后**逐项验证**（opi-control + 两路图传推流）并打印 ✅/❌，
   幂等，可被信号处理、finally、atexit 重复调用；
3. **install_signal_guard()**：把 INT/TERM/**HUP(关会话)**/QUIT/ABRT 统一收敛到"置标志"，
   并忽略 SIGPIPE（串口/管道断开不要直接杀进程）。

用法：
    from scripts.bench_common import MotorSession, restore_remote_stage, install_signal_guard
    state = {"stopped": False, "driver": None, "restored": False}
    install_signal_guard(state)
    atexit.register(restore_remote_stage, state, "atexit")
    with MotorSession(speed_us_max=1560) as session:      # 不用电机就 MotorSession(None)
        ...
"""
from __future__ import annotations

import atexit
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import settings

STREAM_SERVICES = ["ffmpeg-stream.service", "ffmpeg-stream-sub.service"]
PAN_CENTER = TILT_CENTER = 90
NEUTRAL_US = 1500.0


def svc_active(name: str) -> bool:
    """服务是否 active（在没有 systemctl 的机器上——比如开发用的 Windows——安全返回 False）。"""
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", name],
                              stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0
    except (FileNotFoundError, OSError):
        return False


def svc_start(name: str) -> None:
    """拉起服务；**收尾路径绝不能抛异常**（没有 systemctl / 启动失败都只当"没起来"）。"""
    try:
        subprocess.run(["systemctl", "start", name], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (FileNotFoundError, OSError) as exc:
        print(f"[BENCH]   ⚠️ 无法执行 systemctl（{exc}）")


def svc_stop(name: str) -> None:
    """停服务（同样是"尽力而为"，失败只提示不抛）。"""
    try:
        subprocess.run(["systemctl", "stop", name], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (FileNotFoundError, OSError) as exc:
        print(f"[BENCH]   ⚠️ 无法执行 systemctl（{exc}）")


def restore_remote_stage(state: dict, reason: str = "") -> None:
    """停电机 + 恢复**远程控制阶段**并逐项验证（幂等；任何退出路径都调它）。"""
    if state.get("restored"):
        return
    state["restored"] = True
    print(f"\n[BENCH] 收尾{f'（{reason}）' if reason else ''}：")
    driver = state.get("driver")
    if driver is not None:
        try:
            driver.safe_stop()      # 电调 1500us、舵机回中
            driver.shutdown()       # 释放 PCA9685
            print("[BENCH]   电调已归零、舵机回中、PCA9685 已释放")
        except Exception as exc:    # 收尾阶段不能再抛异常
            print(f"[BENCH]   ⚠️ 释放驱动异常：{exc}")
    if state.get("used_motor"):
        svc_start("opi-control.service")
        time.sleep(2)
        checks = [("opi-control.service", "远程控制（网页摇杆/键盘）")]
        checks += [(s, "图传推流") for s in STREAM_SERVICES]
        all_ok = True
        for svc, desc in checks:
            ok = svc_active(svc)
            all_ok &= ok
            print(f"[BENCH]   {'✅' if ok else '❌'} {svc:28s} {desc} = {'active' if ok else '未启动'}")
        if all_ok:
            print("[BENCH] ✅ 已恢复**远程控制阶段**（可以正常遥控/看图传）")
        else:
            print("[BENCH] ⚠️ 有服务没起来，请手动执行："
                  "systemctl start opi-control ffmpeg-stream ffmpeg-stream-sub")
    else:
        for svc in STREAM_SERVICES:
            ok = svc_active(svc)
            print(f"[BENCH]   {'✅' if ok else '❌'} {svc:28s} 图传推流 = {'active' if ok else '未启动'}")
        print("[BENCH] ✅ 未改动远程控制阶段（本次只读摄像头）")


def install_signal_guard(state: dict) -> None:
    """Ctrl+C / kill / 关会话 / 异常终止 → 只置标志，由主循环退出后统一收尾。"""
    def _on_signal(signum, _frame):
        print(f"\n[BENCH] 收到信号 {signum}，准备收尾…")
        state["stopped"] = True

    for name in ("SIGINT", "SIGTERM", "SIGHUP", "SIGQUIT", "SIGABRT"):
        sig = getattr(signal, name, None)      # Windows 没有 SIGHUP/SIGQUIT
        if sig is None:
            continue
        try:
            signal.signal(sig, _on_signal)
        except (ValueError, OSError):
            pass
    if hasattr(signal, "SIGPIPE"):
        try:
            signal.signal(signal.SIGPIPE, signal.SIG_IGN)
        except (ValueError, OSError):
            pass
    atexit.register(restore_remote_stage, state, "atexit")


class MotorSession:
    """`with MotorSession(...)` —— 进去接管 PCA9685，出来必恢复远程控制阶段。

    enabled=False（或 speed_us_max=None）时只做空操作：不碰 opi-control。
    """

    def __init__(self, state: dict, enabled: bool = True, speed_us_max: float = None):
        self.state = state
        self.enabled = enabled
        self.speed_us_max = speed_us_max or settings.ESC_DEBUG_MAX_US
        self.driver = None

    def __enter__(self):
        if not self.enabled:
            self.state["used_motor"] = False
            return None
        from control.driver import Driver
        print("[BENCH] 停止 opi-control（暂时让出 PCA9685；退出时会自动恢复）")
        svc_stop("opi-control.service")
        time.sleep(1.5)
        driver = Driver(real=True, esc_max_us=int(max(self.speed_us_max, settings.ESC_DEBUG_MAX_US)))
        driver.arm()
        self.driver = driver
        self.state["driver"] = driver
        self.state["used_motor"] = True
        pca = driver.pca
        # 云台回中 + 电调先给中位（完成解锁），否则云台朝向会让测试不可复现
        pca.set_pan_angle(PAN_CENTER)
        pca.set_tilt_angle(TILT_CENTER)
        pca.set_esc_percent(0)
        time.sleep(1.0)
        return pca

    def __exit__(self, exc_type, exc, tb):
        restore_remote_stage(self.state, "正常结束")
        return False
