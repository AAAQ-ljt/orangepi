#!/usr/bin/env python3
"""小车端录视频：副摄（下摄 /dev/video2）→ /root/dev/video/*.mp4。

设计目标（用户 2026-10-15 提的用法：SSH 上去跑开始、再跑结束）

    python3 /root/dev/video/record.py start      # 开始录（自动停副摄推流、等设备释放）
    python3 /root/dev/video/record.py start -d 300   # 或限时 300s，到点自己停
    # ... 录完在同一个窗口 Ctrl+C，或者另开一个 SSH 跑：
    python3 /root/dev/video/record.py stop       # 结束录（自动恢复副摄推流）

`start` 默认**前台**占着这个 SSH 会话（Ctrl+C 即停，看着日志最直观）；
要长录、SSH 连接不稳（手机热点、图传掉线）就加 `--detach`：命令立刻返回、
录制进程 setsid 脱离终端，断线不停，之后用 `stop` 结束。

安全与兜底（AGENTS.md §1.2/§1.4）：
* 停/恢复推流一律走 `vision.camera_guard`（循迹/台架/采集/探针同一路径）：停完等
  /dev/video2 真被释放、顺手 reset-failed —— 否则流收 SIGTERM 后落在 failed，
  stream-watch 看门狗"只救 failed"会把刚停掉的推流重新拉起、抢走摄像头（2026-10-12 踩过）；
* 任何退出路径（Ctrl+C / SSH 断线 / 异常 / stop 命令）都收敛到同一处 finally：停 worker →
  恢复推流 → 清会话文件 → 收掉 watchdog；
* 再叠两层兜底：watchdog 子进程（录制主进程被 SIGKILL 时替它恢复推流）+ `stop`/`status`/
  `restore` 会捡起陈旧会话文件把推流恢复 —— "推流被忘在停着"的状态几乎不可能长期存在；
* 每 30s 在终端打一次心跳（已录时长 + 文件 MB），不会对着黑屏猜。

用法汇总（车上，需 root —— 要调 systemctl）：

    python3 /root/dev/video/record.py start
    python3 /root/dev/video/record.py start --name lane_1015        # 存 video/lane_1015/
    python3 /root/dev/video/record.py start --duration 300          # 300s 自动停
    python3 /root/dev/video/record.py start --detach                # 后台录，断线不停
    python3 /root/dev/video/record.py stop
    python3 /root/dev/video/record.py status                        # 看状态（含异常兜底）
    python3 /root/dev/video/record.py restore                       # 应急：把副摄推流拉起来

文件名 `cam2_<日期>_<时间>.mp4`（cam2 = /dev/video2）；mp4 带 `+frag_keyframe`，
断电/进程被硬杀也仍可播。视频是本目录的运行产物，按 .gitignore 不入库。
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

# ---------------------------------------------------------------- 固定配置
CAM_DEVICE = 2                     # 副摄 = 下摄（AGENTS.md §6：巡线固定 camera 2）
CAM_SERVICE = "ffmpeg-stream-sub.service"   # 占着 /dev/video2 的那路推流
VIDEO_DIR = os.path.dirname(os.path.abspath(__file__))    # 视频就存本目录（dev/video/）
SESSION_PATH = os.path.join(VIDEO_DIR, ".recording_session")   # start/stop 之间的会话文件
FFMPEG_BIN = "/usr/bin/ffmpeg"
FFPROBE_BIN = "/usr/bin/ffprobe"
WIDTH, HEIGHT, FPS = 640, 480, 30     # 与推流服务、采集脚本同参数（MJPG）
WATCHDOG_GRACE_S = 20                 # watchdog 发现"主进程没了"后再等一会儿才恢复推流
WATCHDOG_MAX_S = 6 * 3600             # watchdog 最长守 6h（漏网也由 status/restore 兜）
HEARTBEAT_S = 30.0                    # 录制中终端心跳间隔


# ---------------------------------------------------------------- 基础工具
def log(msg: str) -> None:
    """一律经这里打印：stdout 被 SSH 断线写坏（EPIPE/IOError）也不能让进程炸掉。

    这个函数是"所有退出路径都恢复推流"的前提之一——终端没了，代码还要跑完 finally。
    """
    try:
        print(msg, flush=True)
    except Exception:
        pass


# 本地开发机（Windows）导入 camera_guard 也行：里边 systemctl 缺失时安全返回失败
sys.path.insert(0, os.path.dirname(VIDEO_DIR))
try:                                                    # pragma: no cover - 环境分支
    from vision.camera_guard import (                   # type: ignore
        start_ffmpeg,
        stop_ffmpeg,
        device_holders,
        FFMPEG_SERVICES,
    )
    _GUARD_OK = True
except Exception as exc:                                # noqa: BLE001
    _GUARD_OK = False
    FFMPEG_SERVICES = [CAM_SERVICE]
    log(f"[REC] ⚠️ 导入 vision.camera_guard 失败（{exc}），退化为自带实现")

    def start_ffmpeg(services=None, **_) -> None:       # type: ignore
        for svc in (services or [CAM_SERVICE]):
            subprocess.run(["systemctl", "start", svc], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def stop_ffmpeg(services=None, **_) -> None:        # type: ignore
        for svc in (services or [CAM_SERVICE]):
            subprocess.run(["systemctl", "stop", svc], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)

    def device_holders(index: int):                     # type: ignore
        return []


def session_payload(path: str = SESSION_PATH) -> dict:
    """读会话文件；不存在返回 {}（只用 key=value，手写解析，不引 json 依赖）。"""
    if not os.path.isfile(path):
        return {}
    data: dict = {}
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as fh:
            for line in fh:
                line = line.strip()
                if not line or "=" not in line:
                    continue
                key, _, val = line.partition("=")
                data[key.strip()] = val.strip()
    except OSError as exc:
        log(f"[REC] ⚠️ 读会话文件失败 {path}: {exc}")
        return {}
    return data


def process_exists(pid: int) -> bool:
    """这个 pid 还活着吗（跨平台：os.kill(pid,0)，Linux/Windows 都行）。

    注意**不要**在这里排除自己（`pid == os.getpid()`）：会话文件里记的父进程 pid
    常常就是当前进程自己，排除掉会把活跃会话误判成"残留"（2026-10-15 踩过）。
    """
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except (OSError, AttributeError):
        return False


def is_our_pid(pid: int, markers=("record.py", "_worker")) -> bool:
    """这个 pid 是活着的、而且命令行确实是我们这个脚本吗（防 pid 复用误杀）。

    没有 /proc 的环境（开发机 Windows）退化为"进程存在即认"——
    宁可多问一句也不要误杀，车上为 Linux，/proc 核验始终生效。
    """
    if not process_exists(pid):
        return False
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            cmdline = fh.read().decode("utf-8", "replace")
    except OSError:
        return True                                     # 无 /proc：只按"活着"判定
    return all(m in cmdline for m in markers)


def is_worker_alive(pid: int) -> bool:
    return is_our_pid(pid, ("record.py", "_worker"))


# ---------------------------------------------------------------- 会话文件
def write_session(payload: dict) -> None:
    tmp = SESSION_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write("pid={pid}\n".format(**payload))
        fh.write("worker_pid={worker_pid}\n".format(**payload))
        fh.write("watchdog_pid={watchdog_pid}\n".format(**payload))
        fh.write("file={file}\n".format(**payload))
        fh.write(f"device={CAM_DEVICE}\n")
        fh.write("service={service}\n".format(**payload))
        fh.write("active_before={active_before}\n".format(**payload))
        fh.write("started={started}\n".format(**payload))
        fh.write(f"duration={payload.get('duration', 0)}\n")
        fh.write(f"ppid={os.getpid()}\n")               # worker 用它判断父进程还在不在
    os.replace(tmp, SESSION_PATH)


def clear_session() -> None:
    try:
        os.remove(SESSION_PATH)
    except FileNotFoundError:
        pass
    except OSError as exc:
        log(f"[REC] ⚠️ 删会话文件失败: {exc}")


# ---------------------------------------------------------------- 陈旧会话兜底
def service_active(svc: str) -> bool:
    try:
        rc = subprocess.run(["systemctl", "is-active", "--quiet", svc], check=False,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
        return rc == 0
    except (FileNotFoundError, OSError):
        return False


def maybe_stale_restore(verbose: bool = True) -> bool:
    """会话文件在、但录制进程已经没了 → 说明上次是异常退出的。

    顺手把推流恢复回去并清掉残留会话文件。什么都没做返回 False。
    """
    data = session_payload()
    if not data:
        return False
    pid = int(data.get("pid", "0") or 0)
    if is_worker_alive(int(data.get("worker_pid", "0") or 0)) or is_our_pid(pid):
        return False                                    # 还活着，那是真在录
    svc = data.get("service", CAM_SERVICE)
    if verbose:
        log(f"[REC] ⚠️ 发现上次录制残留（pid={pid} 已不在），恢复推流 {svc} 并清理")
    try:
        start_ffmpeg(services=[svc])
    except Exception as exc:                            # noqa: BLE001
        log(f"[REC] ⚠️ 恢复推流失败: {exc}")
    clear_session()
    return True


# ---------------------------------------------------------------- watchdog
def _watchdog_main(parent_pid: int) -> int:
    """兜底 2：录制主进程被 SIGKILL（来不及跑 finally）时，替它恢复推流。

    正常结束时主进程会 SIGTERM 掉本进程；只有"没人来杀我"才真去恢复。
    """
    stop_flag = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.set())
    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())
    seen_session = os.path.isfile(SESSION_PATH)
    deadline = time.time() + WATCHDOG_MAX_S
    while not stop_flag.is_set() and time.time() < deadline:
        if process_exists(parent_pid):
            time.sleep(2.0)
            continue
        # 主进程没了：等一小会儿（它可能正在 finally 里收尾），再看会话文件还在不在
        time.sleep(WATCHDOG_GRACE_S)
        if stop_flag.is_set():
            break
        if not os.path.isfile(SESSION_PATH):
            break                                       # 正常收尾已清掉会话 → 无需动作
        seen_session = True
        log("[REC][watchdog] 录制进程已消失但推流可能还停着 → 恢复推流")
        try:
            start_ffmpeg(services=[CAM_SERVICE])
        except Exception as exc:                        # noqa: BLE001
            log(f"[REC][watchdog] 恢复失败: {exc}")
            break
        clear_session()
        break
    if not seen_session and os.path.isfile(SESSION_PATH):
        clear_session()
    return 0


def _spawn_watchdog(parent_pid: int) -> subprocess.Popen:
    """watchdog 的日志一律丢弃：--detach 退出的 start 若把 stdout 管道留给它，
    SSH 那条命令会一直挂着不返回（2026-10-15）。"""
    cmd = [sys.executable, os.path.abspath(__file__), "_watchdog", str(parent_pid)]
    return subprocess.Popen(
        cmd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


# ---------------------------------------------------------------- 录制进程（worker）
def build_ffmpeg_cmd(filename: str, device: int = CAM_DEVICE) -> list:
    """直读 v4l2 MJPG → H.264，不经过 cv2（帧率/画质与推流服务同口径）。

    `+frag_keyframe+empty_moov` 是断电保险：mp4 头部先写着，机器被断电/进程被硬杀，
    文件也仍然可播（普通 mp4 会把 moov 原子留到收尾，硬杀就废了）。
    """
    return [
        FFMPEG_BIN, "-hide_banner", "-nostdin", "-loglevel", "warning", "-y",
        "-thread_queue_size", "32",
        "-f", "v4l2", "-input_format", "mjpeg",
        "-video_size", f"{WIDTH}x{HEIGHT}", "-framerate", str(FPS),
        "-i", f"/dev/video{device}",
        "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
        "-crf", "23", "-pix_fmt", "yuv420p",
        "-g", str(FPS), "-keyint_min", str(FPS), "-sc_threshold", "0",
        "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
        filename,
    ]


def _pump_stderr(proc: subprocess.Popen, stop_flag: threading.Event) -> None:
    """把 ffmpeg 的 stderr 逐行转出来（跑在后台线程，绝不阻塞主控制流）。"""
    try:
        for raw in iter(proc.stderr.readline, b""):
            if stop_flag.is_set():
                break
            line = raw.decode("utf-8", "replace").strip()
            if line:
                log(f"[ffmpeg] {line[:200]}")
    except Exception:                                   # noqa: BLE001
        pass                                                # 退出阶段管道被关很正常


def _worker_main(filename: str, duration: float) -> int:
    """真正录像的进程：拍完（或被 TERM/INT）→ 收尾文件 → 恢复推流。"""
    stop_flag = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop_flag.set())
    signal.signal(signal.SIGINT, lambda *_: stop_flag.set())
    # 父进程负责把 SIGINT/SIGHUP 转成对 worker 的 SIGTERM；这里跟着终端信号死也无妨
    if hasattr(signal, "SIGHUP"):                       # Windows 没有 SIGHUP
        signal.signal(signal.SIGHUP, lambda *_: stop_flag.set())

    if not os.path.exists(f"/dev/video{CAM_DEVICE}"):
        log(f"[REC] ✗ 设备 /dev/video{CAM_DEVICE} 不存在——摄像头没插好？")
        return 3
    if not os.access(f"/dev/video{CAM_DEVICE}", os.R_OK):
        log(f"[REC] ✗ 没有读 /dev/video{CAM_DEVICE} 的权限（要 root 跑）")
        return 3

    cmd = build_ffmpeg_cmd(filename)
    proc: subprocess.Popen | None = None
    try:
        proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL,
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        t0 = time.time()
        # 等它确认真的在录（设备被占用/参数不被支持时 ffmpeg 会秒退）
        while proc.poll() is None and time.time() - t0 < 4.0:
            time.sleep(0.2)
        if proc.poll() is not None:
            log(f"[REC] ✗ ffmpeg 启动即退出（code={proc.returncode}）——设备被占用或参数不支持")
            return 4

        _pump_stderr(proc, stop_flag)
        log(f"[REC] ● 录制中：{filename}")
        log(f"[REC]   摄像头 /dev/video{CAM_DEVICE}（副摄）{WIDTH}x{HEIGHT}@{FPS}，"
            f"{'到点自动停' if duration > 0 else 'Ctrl+C 或 `record.py stop` 结束'}")
        beat = time.time()
        while not stop_flag.is_set():
            rc = proc.poll()
            if rc is not None:
                log(f"[REC] ⚠️ ffmpeg 自行退出（code={rc}）——检查日志里最后一行的报错")
                return 4 if rc != 0 else 0
            if duration > 0 and time.time() - t0 >= duration:
                log(f"[REC] 到点（{duration:.0f}s），自动停止")
                break
            if time.time() - beat >= HEARTBEAT_S:
                ppid = int(session_payload().get("ppid", "0") or 0)
                if ppid > 0 and not process_exists(ppid):
                    log("[REC] 父进程（start 那条命令）已不在，停止录制并恢复推流")
                    break
                mb = os.path.getsize(filename) / 1048576.0 if os.path.isfile(filename) else 0.0
                log(f"[REC]   …已录 {time.time() - t0:.0f}s / {mb:.1f}MB")
                beat = time.time()
            stop_flag.wait(0.2)
        return 0
    finally:
        if proc is not None and proc.poll() is None:
            try:
                proc.send_signal(signal.SIGTERM)        # ffmpeg 收到 TERM 会正常收尾 mp4
            except Exception:                           # noqa: BLE001
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                log("[REC] ⚠️ ffmpeg 10s 没退，SIGKILL 收尾")
                proc.kill()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    pass
        # 文件收尾 + 结果自检
        size = os.path.getsize(filename) if os.path.isfile(filename) else 0
        if size > 0:
            info = probe_video(filename)
            log(f"[REC] ✓ 已保存 {filename}（{size / 1048576.0:.1f}MB{info}）")
        else:
            log(f"[REC] ✗ 没写出文件：{filename}")


def probe_video(path: str) -> str:
    """ffprobe 报时长/帧数；失败就返回空串（不影响主流程）。"""
    try:
        out = subprocess.run(
            [FFPROBE_BIN, "-v", "error", "-select_streams", "v:0",
             "-show_entries", "stream=nb_frames,avg_frame_rate:format=duration",
             "-of", "default=nw=1:nk=1", path],
            check=False, capture_output=True, text=True, timeout=20).stdout.split()
        if len(out) >= 3:
            return f"，{out[0]} 帧 @ {out[1]}fps，{float(out[2]):.1f}s"
    except Exception:                                   # noqa: BLE001
        pass
    return ""


# ---------------------------------------------------------------- start
def cmd_start(args) -> int:
    maybe_stale_restore()

    data = session_payload()
    if data and (is_worker_alive(int(data.get("worker_pid", "0") or 0))
                 or is_our_pid(int(data.get("pid", "0") or 0))):
        log(f"[REC] ✗ 已经在录了（pid={data.get('pid')}，文件 {data.get('file')}）")
        log("[REC]   先 `record.py stop` 再开始新的")
        return 1

    if args.name:
        base = args.name if os.path.isabs(args.name) else os.path.join(VIDEO_DIR, args.name)
        if os.path.commonpath([os.path.realpath(base), os.path.realpath(VIDEO_DIR)]) \
                != os.path.realpath(VIDEO_DIR):
            log("[REC] ✗ --name 只能在本脚本目录（dev/video）里建子目录")
            return 2
        out_dir = base
    else:
        out_dir = VIDEO_DIR
    try:
        os.makedirs(out_dir, exist_ok=True)
    except OSError as exc:
        log(f"[REC] ✗ 建目录失败 {out_dir}: {exc}")
        return 2

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(out_dir, f"cam{CAM_DEVICE}_{stamp}.mp4")

    active_before = service_active(CAM_SERVICE)
    if not active_before:
        holders = device_holders(CAM_DEVICE)
        log(f"[REC] ⚠️ {CAM_SERVICE} 本来就没在跑；如果 /dev/video{CAM_DEVICE} 打不开，"
            f"占用者可能是 {holders or '别的程序'}")

    log(f"[REC] 开始录制 → {filename}")
    stop_ffmpeg(services=[CAM_SERVICE])                 # ★ 停推流、等设备真的释放
    holders = device_holders(CAM_DEVICE)
    if holders:
        log(f"[REC] ⚠️ /dev/video{CAM_DEVICE} 仍被 {holders} 占用——录像可能失败")

    worker = subprocess.Popen(
        [sys.executable, os.path.abspath(__file__), "_worker", filename, str(args.duration)],
        stdin=subprocess.DEVNULL, start_new_session=True,
    )
    write_session({
        "pid": os.getpid(), "worker_pid": worker.pid, "watchdog_pid": 0,
        "file": filename, "service": CAM_SERVICE,
        "active_before": int(active_before), "started": stamp, "duration": args.duration,
    })
    try:
        watchdog = _spawn_watchdog(os.getpid())
        payload = session_payload()
        payload["watchdog_pid"] = watchdog.pid
        write_session(payload)
    except Exception as exc:                            # noqa: BLE001
        watchdog = None
        log(f"[REC] ⚠️ watchdog 起不来（不影响录制）: {exc}")

    stop = threading.Event()
    got_signal = {"signame": ""}

    def _on_signal(signum, _frame):
        got_signal["signame"] = signal.Signals(signum).name
        stop.set()

    for signame in ("SIGINT", "SIGTERM", "SIGHUP"):     # SSH 断线=HUP，也按"停止"处理
        sig = getattr(signal, signame, None)            # Windows 无 SIGHUP
        if sig is None:
            continue
        try:
            signal.signal(sig, _on_signal)
        except (OSError, ValueError):
            pass

    # 前台守到录制结束（Ctrl+C 停、--duration 到点停、或另开 SSH 跑 stop）。
    # --detach 时下面立刻返回，由 worker + watchdog 自己收尾。
    rc_worker = 0
    try:
        while not stop.is_set():
            rc = worker.poll()
            if rc is not None:
                rc_worker = rc
                if rc != 0:
                    log(f"[REC] ⚠️ 录制子进程提前退出（code={rc}）——多半是设备打不开，"
                        f"或 /dev/video{CAM_DEVICE} 被别的程序占着")
                break
            if args.detach:
                break
            stop.wait(0.3)
        if stop.is_set() and got_signal["signame"]:
            log(f"[REC] 收到 {got_signal['signame']}，停止录制…")
        elif args.detach and worker.poll() is None:
            log(f"[REC] 已在后台录制（pid={os.getpid()}）。停止："
                f"python3 {os.path.relpath(os.path.abspath(__file__))} stop")
    finally:
        # ★ 唯一的一处收尾：停 worker → 恢复推流 → 清理会话/watchdog
        if worker.poll() is None:
            worker.send_signal(signal.SIGTERM)
            try:
                worker.wait(timeout=20)
            except subprocess.TimeoutExpired:
                log("[REC] ⚠️ 录制子进程 20s 没收尾，SIGKILL")
                worker.kill()
                try:
                    worker.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    pass
        start_ffmpeg(services=[CAM_SERVICE])
        if service_active(CAM_SERVICE):
            log(f"[REC] 已恢复推流 {CAM_SERVICE}")
        else:
            log(f"[REC] ⚠️ {CAM_SERVICE} 没起来——查 systemctl status {CAM_SERVICE}")
        clear_session()
        if watchdog is not None and watchdog.poll() is None:
            watchdog.terminate()
            try:
                watchdog.wait(timeout=5)
            except subprocess.TimeoutExpired:
                watchdog.kill()

    size = os.path.getsize(filename) if os.path.isfile(filename) else 0
    if size > 0:
        log(f"[REC] 停止完成。视频：{filename}（{size / 1048576.0:.1f}MB）")
        return rc_worker
    log(f"[REC] ✗ 没有录到内容：{filename}（0 字节）——看上面 worker 的报错")
    return rc_worker or 1


# ---------------------------------------------------------------- stop / status / restore
def kill_process_group(pid: int, sig: int) -> None:
    """优先按"进程组"发信号（父进程自己 setsid 起的），不支持时退化为单进程。"""
    try:
        os.killpg(pid, sig)                             # Windows 没有 killpg
        return
    except AttributeError:
        pass
    except (ProcessLookupError, PermissionError):
        pass
    os.kill(pid, sig)


def cmd_stop(_args) -> int:
    data = session_payload()
    if not data:
        if maybe_stale_restore():
            return 0
        log(f"[REC] 没有正在进行的录制（{SESSION_PATH} 不存在）")
        if not service_active(CAM_SERVICE):
            log(f"[REC] ⚠️ 但 {CAM_SERVICE} 也没在跑——要手动恢复就 `record.py restore`")
        return 1

    pid = int(data.get("pid", "0") or 0)
    if not is_our_pid(pid):
        log(f"[REC] ⚠️ 会话里的 pid={pid} 已经不在了（上次异常退出？）→ 恢复推流并清理")
        maybe_stale_restore()
        return 0

    log(f"[REC] 通知录制进程 {pid} 停止…（文件 {data.get('file')}）")
    try:
        kill_process_group(pid, signal.SIGTERM)
    except OSError as exc:
        log(f"[REC] ✗ 杀不掉 {pid}: {exc}")
        return 1

    deadline = time.time() + 30                          # 等它收尾 mp4 + 恢复推流
    while time.time() < deadline:
        if not os.path.exists(SESSION_PATH):
            break
        time.sleep(0.5)
    if not os.path.isfile(SESSION_PATH):
        log("[REC] ✓ 录制已停止，推流已恢复")
        return 0
    log("[REC] ⚠️ 30s 没等到收尾，再发一次 TERM")
    try:
        os.killpg(pid, signal.SIGTERM)
    except OSError:
        pass
    return 1


def cmd_restore(_args) -> int:
    """应急恢复：把副摄推流拉起来（不清会话文件——那可能是别的会话在录）。"""
    if maybe_stale_restore():
        return 0
    data = session_payload()
    if data and (is_worker_alive(int(data.get("worker_pid", "0") or 0))
                 or is_our_pid(int(data.get("pid", "0") or 0))):
        log("[REC] 有录制正在进行，不动推流（要停就 `record.py stop`）")
        return 1
    start_ffmpeg(services=[CAM_SERVICE])
    ok = service_active(CAM_SERVICE)
    log(f"[REC] {'✓' if ok else '✗'} {CAM_SERVICE} "
        f"{'已在跑' if ok else '没起来——查 systemctl status'}")
    return 0 if ok else 1


def cmd_status(_args) -> int:
    data = session_payload()
    active = service_active(CAM_SERVICE)
    holders = device_holders(CAM_DEVICE)
    log(f"[REC] 推流 {CAM_SERVICE}: {'active' if active else 'inactive'}")
    log(f"[REC] /dev/video{CAM_DEVICE} 占用者: {holders or '无'}")
    if not data:
        if maybe_stale_restore():
            return 0
        log("[REC] 没有录制会话")
        return 0
    pid = int(data.get("pid", "0") or 0)
    worker_alive = is_worker_alive(int(data.get("worker_pid", "0") or 0))
    if is_our_pid(pid) or worker_alive:
        log(f"[REC] 录制进行中：pid={pid}，文件 {data.get('file')}，"
            f"开始于 {data.get('started')}")
        if data.get("duration") and float(data.get("duration", 0)) > 0:
            log(f"[REC]   到点自动停：{data.get('duration')}s")
        return 0
    log("[REC] ⚠️ 会话文件在但进程已不在（异常退出）→ 自动恢复并清理")
    maybe_stale_restore()
    return 0


# ---------------------------------------------------------------- 入口
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="小车副摄（下摄）录像：自动停/恢复推流，存 dev/video/",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：\n"
               "  python3 /root/dev/video/record.py start\n"
               "  python3 /root/dev/video/record.py start --name lane_1015 --duration 300\n"
               "  python3 /root/dev/video/record.py stop\n"
               "  python3 /root/dev/video/record.py status\n")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("start", help="开始录制（停副摄推流、释放设备，前台守到结束）")
    s.add_argument("--name", "-n", default=None,
                   help="存到 dev/video/<name>/ 子目录（默认直接存 dev/video/）")
    s.add_argument("--duration", "-d", type=float, default=0.0,
                   help="录制秒数，到点自动停（0=不限，靠 Ctrl+C / stop 结束）")
    s.add_argument("--detach", action="store_true",
                   help="后台录制：本命令立刻返回（SSH 断线也不停，用 stop 结束）")
    s.set_defaults(func=cmd_start)

    sub.add_parser("stop", help="结束录制并恢复推流").set_defaults(func=cmd_stop)
    sub.add_parser("status", help="看当前是否在录 / 推流状态（含异常兜底恢复）") \
       .set_defaults(func=cmd_status)
    sub.add_parser("restore", help="应急：把副摄推流拉起来（不碰正在进行的录制）") \
       .set_defaults(func=cmd_restore)
    return p


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "_worker":                    # 内部用法，不进 argparse
        return _worker_main(argv[1], float(argv[2]))
    if argv and argv[0] == "_watchdog":
        return _watchdog_main(int(argv[1]))
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
