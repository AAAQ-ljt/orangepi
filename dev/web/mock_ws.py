#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地假后端 —— 不需要车、不需要摄像头，就能把「驾驶模式」整条链路跑起来。

    python dev/web/mock_ws.py                                  # 用仓库里的官方页副本
    python dev/web/mock_ws.py --page F:/smart_car/newCarDOC/orangepiCode/var/www/orangepi-rc/index.html
    python dev/web/mock_ws.py --port 8090 --quiet

它做三件事（全部只用标准库）：
    1. 静态服务：`/` 返回官方 index.html（**运行时注入**那行 <script src="/rc-drive.js">，
       不改原文件）；`/rc-drive.js` 返回仓库里的主脚本；`/health` 返回假状态。
    2. `/ws`：手写 RFC6455 握手 + 帧编解码的假 WebSocket。
    3. 假后端状态机：**照抄车端 opi-control-app.py 的语义**（arm 要求油门为 0、estop 挡解锁、
       drive 未解锁就报错、drive 里 configure 行程、gimbal 的 tilt = 180 - tilt、8s failsafe、
       5Hz telemetry），并打印收到的每条消息，方便逐项核对协议。

⚠️ 它只是本地联调工具，不要拿它当车端行为依据（真正的权威是 /root/opi-control/opi-control-app.py）。
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from deploy_rc import patch_html  # noqa: E402  （同目录，复用同一份补丁逻辑）

WEB_DIR = Path(__file__).resolve().parent
DEFAULT_PAGE = WEB_DIR / "index.html"
JS_PATH = WEB_DIR / "rc-drive.js"

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
FAILSAFE_TIMEOUT = 8.0  # 与车端一致
TELEMETRY_INTERVAL = 0.2

CLI_OPTS = {"quiet": False, "verbose_drive": False}

# --------------------------------------------------------------------------
# 假后端状态（字段名与车端 state 字典对齐）
# --------------------------------------------------------------------------
ST = {
    "armed": False,
    "estop": False,
    "steering": 90,
    "steer_center": 90,
    "throttle": 0,
    "throttle_us": 1500,
    "throttle_min_us": 1300,
    "throttle_center_us": 1500,
    "throttle_max_us": 1600,
    "pan": 90,
    "tilt": 90,
    "battery_v": 15.6,
    "signal": 78,
    "last_cmd_ts": 0.0,
}
STATS: dict[str, int] = {}
STATS_LOCK = threading.Lock()
_print_lock = threading.Lock()


def _say(text: str) -> None:
    if CLI_OPTS["quiet"]:
        return
    with _print_lock:
        print(text, flush=True)


def _bump(kind: str) -> None:
    with STATS_LOCK:
        STATS[kind] = STATS.get(kind, 0) + 1


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def calc_throttle_us(value: int) -> int:
    value = clamp(int(value), -100, 100)
    c = ST["throttle_center_us"]
    if value == 0:
        return c
    if value > 0:
        return int(round(c + (value / 100.0) * (ST["throttle_max_us"] - c)))
    return int(round(c + (value / 100.0) * (c - ST["throttle_min_us"])))


def configure_throttle_range(min_us: int, center_us: int, max_us: int) -> None:
    ST["throttle_min_us"] = int(min_us)
    ST["throttle_center_us"] = int(center_us)
    ST["throttle_max_us"] = int(max_us)


def set_steering(angle: int) -> None:
    center = ST.get("steer_center", 90)
    ST["steering"] = clamp(int(angle), center - 30, center + 30)


def set_throttle(value: int) -> None:
    value = clamp(int(value), -100, 100)
    ST["throttle"] = value
    ST["throttle_us"] = calc_throttle_us(value)


def arm_system(enabled: bool):
    if ST["estop"] and enabled:
        _say("[CTRL] estop active, cannot arm")
        return None
    if enabled and ST["throttle"] != 0:
        _say("[WARN] throttle not zero, cannot arm")
        return False
    ST["armed"] = enabled
    if not enabled:
        set_throttle(0)
    _say(f"[CTRL] armed -> {enabled}")
    return True


def emergency_stop() -> None:
    ST["estop"] = True
    ST["armed"] = False
    set_throttle(0)
    set_steering(ST.get("steer_center", 90))
    _say("[CTRL] EMERGENCY STOP!")


def apply_failsafe() -> None:
    set_throttle(0)
    set_steering(ST.get("steer_center", 90))
    ST["armed"] = False
    _say("[SAFE] failsafe triggered（8s 没收到前端消息）")


# --------------------------------------------------------------------------
# WebSocket 帧编解码（RFC6455 最小实现）
# --------------------------------------------------------------------------
def ws_accept_key(key: str) -> str:
    digest = hashlib.sha1((key + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def ws_encode(payload: bytes, opcode: int = 0x1) -> bytes:
    head = bytearray([0x80 | opcode])
    n = len(payload)
    if n < 126:
        head.append(n)
    elif n < 65536:
        head.append(126)
        head += struct.pack(">H", n)
    else:
        head.append(127)
        head += struct.pack(">Q", n)
    return bytes(head) + payload


def ws_read_frame(rfile):
    """返回 (fin, opcode, data)；连接结束返回 None。"""
    hdr = rfile.read(2)
    if len(hdr) < 2:
        return None
    b1, b2 = hdr[0], hdr[1]
    fin = bool(b1 & 0x80)
    opcode = b1 & 0x0F
    masked = bool(b2 & 0x80)
    length = b2 & 0x7F
    if length == 126:
        length = struct.unpack(">H", rfile.read(2))[0]
    elif length == 127:
        length = struct.unpack(">Q", rfile.read(8))[0]
    mask = rfile.read(4) if masked else None
    data = rfile.read(length) if length else b""
    if mask:
        data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    return fin, opcode, data


class Conn:
    """一条已升级的 WS 连接：写操作加锁（读线程 + 遥测线程都会写）。"""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.lock = threading.Lock()
        self.alive = True

    def send_json(self, obj: dict) -> bool:
        if not self.alive:
            return False
        try:
            with self.lock:
                self.sock.sendall(ws_encode(json.dumps(obj, ensure_ascii=False).encode("utf-8")))
            return True
        except OSError:
            self.alive = False
            return False

    def send_control(self, opcode: int, payload: bytes = b"") -> None:
        try:
            with self.lock:
                self.sock.sendall(ws_encode(payload, opcode))
        except OSError:
            self.alive = False


# --------------------------------------------------------------------------
# 消息处理（与车端 handle_message 对齐）
# --------------------------------------------------------------------------
_last_drive_log = {"t": 0.0, "key": ""}


def handle_message(conn: Conn, msg: dict) -> None:
    kind = str(msg.get("type"))
    _bump(kind)
    ST["last_cmd_ts"] = time.time()

    if kind == "ping":
        conn.send_json({"type": "ack", "msg": "pong"})
        return
    if kind == "set_volume":
        conn.send_json({"type": "ack", "msg": f"volume set to {msg.get('volume', 50)}%"})
        return
    if kind == "arm":
        enabled = bool(msg.get("enabled", False))
        ok = arm_system(enabled)
        if not ok:
            conn.send_json({"type": "error", "msg": "estop active"})
            return
        conn.send_json({"type": "ack", "msg": f"armed={enabled}"})
        return
    if kind == "clear_estop":
        ST["estop"] = False
        _say("[CTRL] estop cleared")
        conn.send_json({"type": "ack", "msg": "estop cleared"})
        return
    if kind == "estop":
        emergency_stop()
        conn.send_json({"type": "ack", "msg": "estop triggered"})
        return
    if kind == "drive":
        if ST["estop"]:
            conn.send_json({"type": "error", "msg": "estop active"})
            return
        if not ST["armed"]:
            conn.send_json({"type": "error", "msg": "system not armed"})
            return
        configure_throttle_range(
            int(msg.get("throttle_min_us", ST["throttle_min_us"])),
            int(msg.get("throttle_center_us", ST["throttle_center_us"])),
            int(msg.get("throttle_max_us", ST["throttle_max_us"])),
        )
        ST["steer_center"] = int(msg.get("steer_center", ST.get("steer_center", 90)))
        set_steering(int(msg.get("steering", ST["steer_center"])))
        set_throttle(int(msg.get("throttle", 0)))
        line = (
            "[drive] steering={} steer_center={} throttle={}% ({}us) range={}/{}/{}".format(
                ST["steering"],
                ST["steer_center"],
                ST["throttle"],
                ST["throttle_us"],
                ST["throttle_min_us"],
                ST["throttle_center_us"],
                ST["throttle_max_us"],
            )
        )
        now = time.time()
        if CLI_OPTS["verbose_drive"] or line != _last_drive_log["key"] or now - _last_drive_log["t"] > 0.5:
            _last_drive_log["t"] = now
            _last_drive_log["key"] = line
            _say(line)
        conn.send_json({"type": "ack", "msg": "drive updated"})
        return
    if kind == "gimbal":
        pan = clamp(int(msg.get("pan", 90)), 0, 180)
        tilt = clamp(int(msg.get("tilt", 90)), 0, 180)
        ST["pan"] = pan
        ST["tilt"] = 180 - tilt  # ★ 车端 set_gimbal 就是这么写的（反转的是 tilt）
        _say(f"[gimbal] 收到 pan={pan} tilt={tilt} → 硬件 pan={ST['pan']} tilt={ST['tilt']}")
        conn.send_json({"type": "ack", "msg": "gimbal updated"})
        return
    _say(f"[WS] 未知消息类型: {kind}")
    conn.send_json({"type": "error", "msg": f"unknown type: {kind}"})


def telemetry_loop(conn: Conn) -> None:
    while conn.alive:
        now = time.time()
        if ST["armed"] and (now - ST["last_cmd_ts"] > FAILSAFE_TIMEOUT):
            apply_failsafe()
        conn.send_json({
            "type": "telemetry",
            "armed": ST["armed"],
            "estop": ST["estop"],
            "steering": ST["steering"],
            "throttle": ST["throttle"],
            "pan": ST["pan"],
            "tilt": ST["tilt"],
            "battery_v": ST["battery_v"],
            "signal": ST["signal"],
            "ts": now,
        })
        time.sleep(TELEMETRY_INTERVAL)


# --------------------------------------------------------------------------
# HTTP 处理
# --------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "rc-mock/1.0"
    page_bytes = b""
    page_name = ""

    def log_message(self, fmt, *args):  # 默认日志太吵
        if not CLI_OPTS["quiet"]:
            sys.stderr.write("[http] %s %s\n" % (self.address_string(), fmt % args))

    def _send(self, code: int, body: bytes, ctype: str, extra: dict | None = None) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send(200, self.page_bytes, "text/html; charset=utf-8")
            return
        if path == "/rc-drive.js":
            if not JS_PATH.exists():
                self._send(404, b"rc-drive.js missing", "text/plain")
                return
            self._send(200, JS_PATH.read_bytes(), "application/javascript; charset=utf-8")
            return
        if path == "/health":
            body = json.dumps({"ok": True, "state": ST, "stats": STATS}, ensure_ascii=False).encode()
            self._send(200, body, "application/json; charset=utf-8")
            return
        if path == "/ws" and (self.headers.get("Upgrade", "").lower() == "websocket"):
            self._ws_upgrade()
            return
        self._send(404, b"not found", "text/plain")

    def _ws_upgrade(self):
        key = self.headers.get("Sec-WebSocket-Key")
        if not key:
            self._send(400, b"missing Sec-WebSocket-Key", "text/plain")
            return
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", ws_accept_key(key))
        self.end_headers()
        self.close_connection = True  # 之后这条连接归我们管
        _say("[WS] client connected")
        conn = Conn(self.connection)
        threading.Thread(target=telemetry_loop, args=(conn,), daemon=True).start()
        try:
            buf = bytearray()
            while True:
                frame = ws_read_frame(self.rfile)
                if frame is None:
                    break
                fin, opcode, data = frame
                if opcode == 0x8:  # close
                    break
                if opcode == 0x9:  # ping → pong
                    conn.send_control(0xA, data)
                    continue
                if opcode == 0xA:  # pong
                    continue
                if opcode in (0x1, 0x2, 0x0):
                    buf += data
                    if not fin:
                        continue
                    text = bytes(buf).decode("utf-8", errors="replace")
                    buf = bytearray()
                    try:
                        msg = json.loads(text)
                    except ValueError:
                        _say(f"[WS] 非法 JSON: {text[:120]}")
                        continue
                    if not CLI_OPTS["quiet"] and msg.get("type") not in ("drive", "ping", "gimbal"):
                        _say(f"[WS MSG] {msg}")
                    handle_message(conn, msg)
        except OSError as exc:
            _say(f"[WS] error: {exc}")
        finally:
            conn.alive = False
            _say("[WS] client disconnected")


def main() -> int:
    parser = argparse.ArgumentParser(description="驾驶模式本地假后端（静态页 + 假 /ws）")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--page", default=str(DEFAULT_PAGE), help="官方 index.html 路径（运行时注入脚本行）")
    parser.add_argument("--quiet", action="store_true", help="少打印")
    parser.add_argument("--verbose-drive", action="store_true", help="每一帧 drive 都打印（默认 0.5s 或变化时打印）")
    args = parser.parse_args()

    CLI_OPTS["quiet"] = args.quiet
    CLI_OPTS["verbose_drive"] = args.verbose_drive

    page = Path(args.page)
    if not page.exists():
        print(f"[err] 找不到官方页：{page}", file=sys.stderr)
        return 2
    html = page.read_bytes().decode("utf-8")
    patched, changed = patch_html(html)
    if not changed:
        print("[info] 该页面里已经有补丁标记（照原样提供，不重复插入）")
    Handler.page_bytes = patched.encode("utf-8")
    Handler.page_name = str(page)

    if not JS_PATH.exists():
        print(f"[err] 缺少 {JS_PATH}", file=sys.stderr)
        return 2

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[OK] 假后端已启动：http://{args.host}:{args.port}/")
    print(f"     官方页来源：{Handler.page_name}")
    print(f"     主脚本：{JS_PATH}（改完直接刷新浏览器即可，no-store）")
    print("     Ctrl+C 结束（会打印收包统计）")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    finally:
        server.server_close()
        with STATS_LOCK:
            print("[stats] 收到的消息统计：" + (json.dumps(STATS, ensure_ascii=False) if STATS else "（空）"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
