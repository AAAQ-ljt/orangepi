"""串口控制台助手 —— 通过 USB-TTL 串口登录小车并执行命令（免交互）。

用途：当网络（WiFi/5G/隧道）都断了，用串口救场。小车调试串口是
`serial-getty@ttyFIQ0`，波特率 **1500000**，登录 `root/orangepi`。

用法:
    python scripts/serial_car.py "hostname; ip -br a"          # 执行命令并回显
    python scripts/serial_car.py --timeout 40 "journalctl -u car-dial -n 30 --no-pager"
    python scripts/serial_car.py --port COM8 --baud 1500000 "uname -a"
    python scripts/serial_car.py --raw                          # 只打印串口输出 30s（人工观察/看启动日志）

实现要点：
- 用 `echo <<<标记>>>` 包住命令，靠标记判断"这条命令执行完了"，不用猜提示符；
- 自动识别 login:/Password: 并登录；
- 每次都新开会话，退出即关串口（避免占用 COM8 影响人工调试）。
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

PROMPT_MARK = "<<<CAR_CMD_DONE>>>"
DEFAULT_PORT = "COM8"
DEFAULT_BAUD = 1500000
USER = "root"
PASSWORD = "orangepi"


class SerialConsole:
    def __init__(self, port: str, baud: int, user: str = USER, password: str = PASSWORD,
                 open_timeout: float = 10.0):
        self.ser = serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N",
                                 stopbits=1, timeout=0.2, write_timeout=5)
        self.user = user
        self.password = password
        self.logged_in = False
        time.sleep(0.3)
        self._drain(1.0)

    # ------------------------------------------------------------------ 内部
    def _read_available(self, timeout: float) -> str:
        buf = []
        end = time.time() + timeout
        while time.time() < end:
            n = self.ser.in_waiting
            if n:
                buf.append(self.ser.read(n).decode("utf-8", errors="replace"))
            else:
                time.sleep(0.05)
        return "".join(buf)

    def _drain(self, timeout: float = 0.5) -> str:
        return self._read_available(timeout)

    def _send_line(self, line: str) -> None:
        self.ser.write((line + "\n").encode())
        self.ser.flush()

    # ------------------------------------------------------------------ 登录
    def ensure_login(self, timeout: float = 10.0) -> bool:
        self._send_line("")
        end = time.time() + timeout
        seen = ""
        while time.time() < end:
            seen += self._read_available(0.4)
            low = seen.lower()
            if "login:" in low and "password:" not in low.split("login:")[-1]:
                self._send_line(self.user)
                time.sleep(0.4)
            elif "password:" in low:
                self._send_line(self.password)
                time.sleep(1.2)
            elif seen.rstrip().endswith("#") or seen.rstrip().endswith("$"):
                self.logged_in = True
                return True
        return self.logged_in

    # ------------------------------------------------------------------ 执行
    def run(self, cmd: str, timeout: float = 25.0) -> str:
        if not self.logged_in and not self.ensure_login():
            raise RuntimeError("串口登录失败：确认波特率 1500000、串口没被别的程序占用")
        self._drain(0.3)
        # 用标记包住命令：先回显一个开始标记，命令结束再回显结束标记
        wrapped = f"echo '{PROMPT_MARK}START'; {cmd}; echo '{PROMPT_MARK}END'"
        self._send_line(wrapped)
        buf = ""
        end = time.time() + timeout
        while time.time() < end:
            buf += self._read_available(0.3)
            if f"{PROMPT_MARK}END" in buf:
                break
        # 去掉标记行与首尾噪声
        lines = [ln for ln in buf.replace("\r", "").split("\n")
                 if PROMPT_MARK not in ln]
        out = "\n".join(lines).strip("\n")
        return out

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description="串口控制台助手")
    ap.add_argument("command", nargs="?", help="要执行的命令（可带分号串多条）")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--timeout", type=float, default=25.0)
    ap.add_argument("--raw", action="store_true", help="只打印串口原始输出（看启动日志/人工观察）")
    args = ap.parse_args()

    if not args.command and not args.raw:
        ap.error("要么给命令，要么加 --raw")

    try:
        console = SerialConsole(args.port, args.baud)
    except serial.SerialException as exc:
        print(f"[serial] 打开 {args.port} 失败：{exc}", file=sys.stderr)
        print("[serial] 可能原因：串口被其他程序占用（关掉你的串口终端再试）", file=sys.stderr)
        return 1

    try:
        if args.raw:
            print(f"[serial] 只读模式，{args.timeout:.0f}s …")
            sys.stdout.write(console._read_available(args.timeout))
            return 0
        print(console.run(args.command, timeout=args.timeout))
    finally:
        console.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
