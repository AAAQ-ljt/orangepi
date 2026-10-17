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
        _say(f"[BENCH]   ⚠️ 无法执行 systemctl（{exc}）")


def svc_stop(name: str) -> None:
    """停服务（同样是"尽力而为"，失败只提示不抛）。"""
    try:
        subprocess.run(["systemctl", "stop", name], check=False,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (FileNotFoundError, OSError) as exc:
        _say(f"[BENCH]   ⚠️ 无法执行 systemctl（{exc}）")


def _say(msg: str) -> None:
    """打印**绝不抛**：收尾链里每一步都要印字，而 stdout 可能是一条已经断掉的管道
    （2026-10-17 现场：`python3 … | tee` 跑，^C 先把 tee 打死 → 我们的 print 抛 BrokenPipeError
    → 收尾第一步就崩，电调归零被整个跳过 ⇒ 车带着最后一次脉宽跑）。"""
    try:
        print(msg, flush=True)
    except Exception:                                  # noqa: BLE001
        pass


def restore_remote_stage(state: dict, reason: str = "", on_motor_safe=None) -> None:
    """停电机 + 恢复**远程控制阶段**并逐项验证（幂等；任何退出路径都调它）。

    ★ 顺序铁律（2026-10-17 事故）：**先写硬件、后印字**。原来第一行是 `print`，stdout 一断
    （^C 打死 `| tee`）就直接抛出去，`driver.safe_stop()` 根本没执行 ⇒ PCA9685 保持最后一次脉宽
    ⇒ 车一直跑，现场只能断电。现在任何一条打印都不许挡在"电调归零"前面。
    并把"电机是否确实归零"记进 `state["motor_safe"]`，供调用方决定**要不要解除看门狗**。

    `on_motor_safe`：电调这一步一做完就立刻回调（**在恢复服务的 2s 等待之前**）。调用方用它
    "电调已安全 ⇒ 现在可以解除看门狗"——否则停喂心跳 + 服务恢复要 2s 多，看门狗会在**成功的收尾里
    被触发**（2026-10-17 现场就报了一次假的「看门狗触发过（主控卡死/被杀）」）。
    """
    if state.get("restored"):
        return
    state["restored"] = True

    # ---- ① 硬件先行：电调 1500us、舵机回中、释放 PCA9685
    driver = state.get("driver")
    motor_safe = True
    stop_err = ""
    if driver is not None:
        try:
            driver.safe_stop()      # 电调 1500us、舵机回中
            driver.shutdown()       # 释放 PCA9685
        except Exception as exc:    # 收尾阶段不能再抛异常
            motor_safe = False
            stop_err = str(exc)
    state["motor_safe"] = motor_safe
    if on_motor_safe is not None:               # ★ 电调已处置（成/败都通知）→ 立刻决定看门狗去留
        try:
            on_motor_safe(motor_safe)
        except Exception as exc:                # noqa: BLE001
            _say(f"[BENCH]   ⚠️ on_motor_safe 回调异常（继续收尾）：{exc}")

    # ---- ② 之后才是打印（每一条都走 _say，断管也不抛）
    _say(f"\n[BENCH] 收尾{f'（{reason}）' if reason else ''}：")
    if driver is not None:
        if motor_safe:
            _say("[BENCH]   电调已归零、舵机回中、PCA9685 已释放")
        else:
            _say(f"[BENCH]   ⚠️⚠️ 电调归零**失败**：{stop_err}")
            _say("[BENCH]   ⚠️⚠️ 电机可能还在转！→ 看门狗 1s 内会强写 1500us；"
                 "**别断电**，等它 / 手边留断电开关")
    if state.get("used_motor"):
        svc_start("opi-control.service")
        time.sleep(2)
        if not svc_active("opi-control.service"):      # ★ 给一次重试：别的进程可能在这 2s 里把它停了
            _say("[BENCH]   ⚠️ opi-control 第一次没起来 → 重试一次")
            svc_start("opi-control.service")
            time.sleep(1)
        checks = [("opi-control.service", "远程控制（网页摇杆/键盘）")]
        checks += [(s, "图传推流") for s in STREAM_SERVICES]
        all_ok = True
        for svc, desc in checks:
            ok = svc_active(svc)
            all_ok &= ok
            _say(f"[BENCH]   {'✅' if ok else '❌'} {svc:28s} {desc} = {'active' if ok else '未启动'}")
        if all_ok:
            _say("[BENCH] ✅ 已恢复**远程控制阶段**（可以正常遥控/看图传）")
        else:
            _say("[BENCH] ⚠️ 有服务没起来，请手动执行："
                  "systemctl start opi-control ffmpeg-stream ffmpeg-stream-sub")
        # ★ 2026-10-15 现场提醒：opi-control 一旦起来就**接管 PCA9685** —— 如果谁的
        #   手机/网页正停在摇杆页（或键盘按住），车会直接按那个输入动起来，
        #   而我们这边的 Ctrl+C 已经没用了（脚本已经退出，不是它在开车）。
        _say("[BENCH] ⚠️ 注意：远程控制已恢复 = 网页摇杆/键盘**现在能直接让车动** —— "
             "别把手机/网页停在摇杆页；要车绝对不动就断电或 systemctl stop opi-control")
    else:
        for svc in STREAM_SERVICES:
            ok = svc_active(svc)
            _say(f"[BENCH]   {'✅' if ok else '❌'} {svc:28s} 图传推流 = {'active' if ok else '未启动'}")
        _say("[BENCH] ✅ 未改动远程控制阶段（本次只读摄像头）")


def install_signal_guard(state: dict) -> None:
    """Ctrl+C / kill / 关会话 / 异常终止 → 只置标志，由主循环退出后统一收尾。"""
    def _on_signal(signum, _frame):
        _say(f"\n[BENCH] 收到信号 {signum}，准备收尾…")
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
