"""串口推送文件 —— 不依赖网络，把本地文件写进小车。

用途：网络全断（或 SSH 通道被本地网络限制）时，仍能部署代码/配置。

用法:
    python scripts/serial_push.py 本地文件 /root/dev/xxx [--mode 755]
    python scripts/serial_push.py --verify-only 本地文件 /root/dev/xxx   # 只比对 sha256

原理：base64 分块写入车上 /tmp/serial_push.b64（每块一条串口命令，避免长行被吞），
      再在车上解码落到目标路径（自动 sudo），最后比对 sha256 确认一字不差。
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import posixpath
import re
import sys

from serial_car import SerialConsole

TMP_B64 = "/tmp/serial_push.b64"

# Git Bash(MSYS) 会把 "/root/x" 这类参数自动转成 "E:/Program Files/Git/root/x"，
# 函数把这些"被污染的"远端路径纠正回来（比要求每个人记得加 MSYS_NO_PATHCONV=1 更省心）
REMOTE_ROOTS = ("/root/", "/etc/", "/opt/", "/var/", "/tmp/", "/usr/", "/run/", "/home/", "/srv/")


def normalize_remote(path: str) -> str:
    if path.startswith("/") and not path.startswith("//"):
        return path
    norm = path.replace("\\", "/")
    for root in REMOTE_ROOTS:
        idx = norm.find(root)
        if idx > 0:
            fixed = norm[idx:]
            print(f"[push] ⚠️ 检测到 MSYS 路径转换，已纠正：{path} → {fixed}")
            return fixed
    return norm


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def push(console: SerialConsole, local: str, remote: str, mode: str, chunk: int,
         binary: bool = False) -> bool:
    remote = normalize_remote(remote)
    # 先刷新 sudo 凭据（否则各条 sudo 命令会停在密码提示上，把串口会话卡死）
    console.run(f"echo {console.password} | sudo -S -v", timeout=25)
    with open(local, "rb") as fh:
        raw = fh.read()
    if not binary:
        # 统一 LF：文本脚本在 Linux 上带 CR 会炸。二进制（tar.gz 等）必须加 --binary，
        # 否则这种替换会随机破坏内容（校验一定失败）。
        raw = raw.replace(b"\r\n", b"\n")
    b64 = base64.b64encode(raw).decode()
    local_hash = sha256(raw)
    print(f"[push] {local} → {remote}（{len(raw)} 字节，base64 {len(b64)}，分 {(len(b64)+chunk-1)//chunk} 块）")

    console.run(f"sudo rm -f {TMP_B64}; sudo touch {TMP_B64} && sudo chmod 666 {TMP_B64}", timeout=20)
    for i in range(0, len(b64), chunk):
        piece = b64[i:i + chunk]
        console.run(f"printf %s {piece} >> {TMP_B64}", timeout=30)
        if (i // chunk) % 5 == 0:
            print(f"  ... {min(i+chunk, len(b64))}/{len(b64)}", flush=True)

    remote_dir = posixpath.dirname(remote)
    cmd = (f"sudo mkdir -p {remote_dir} && base64 -d {TMP_B64} | sudo tee {remote} >/dev/null "
           f"&& sudo chmod {mode} {remote} && sudo rm -f {TMP_B64} "
           f"&& echo HASH=$(sudo sha256sum {remote} | cut -d' ' -f1)")
    out = console.run(cmd, timeout=60)
    # 串口输出里混着提示符等噪声，用正则精确抓 HASH=...
    m = re.search(r"HASH=([0-9a-f]{64})", out)
    remote_hash = m.group(1) if m else ""
    if remote_hash == local_hash:
        print(f"[push] ✅ 校验通过（sha256 {local_hash[:12]}…）")
        return True
    print(f"[push] ❌ 校验失败：本地 {local_hash[:12]}… 远端 {remote_hash[:12] or '(空)'}")
    if not remote_hash:
        print(f"[push] 远端回显：{out.strip()[-200:]}")
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="串口推送文件到小车")
    ap.add_argument("local")
    ap.add_argument("remote")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=1500000)
    ap.add_argument("--mode", default="644", help="远端权限（脚本用 755）")
    ap.add_argument("--chunk", type=int, default=700, help="每块 base64 字符数")
    ap.add_argument("--binary", action="store_true",
                    help="二进制文件（tar.gz 等）：不做 CRLF→LF 替换，否则内容会被破坏")
    ap.add_argument("--verify-only", action="store_true")
    args = ap.parse_args()

    console = SerialConsole(args.port, args.baud)
    try:
        if args.verify_only:
            with open(args.local, "rb") as fh:
                data = fh.read()
            if not args.binary:
                data = data.replace(b"\r\n", b"\n")
            local_hash = sha256(data)
            remote = normalize_remote(args.remote)
            out = console.run(f"echo HASH=$(sudo sha256sum {remote} | cut -d' ' -f1)", timeout=30)
            m = re.search(r"HASH=([0-9a-f]{64})", out)
            remote_hash = m.group(1) if m else ""
            same = remote_hash == local_hash
            print(f"[verify] {'✅ 一致' if same else '❌ 不一致'} 本地 {local_hash[:12]}… 远端 {remote_hash[:12] or '(空)'}")
            return 0 if same else 1
        return 0 if push(console, args.local, args.remote, args.mode, args.chunk,
                         args.binary) else 1
    finally:
        console.close()


if __name__ == "__main__":
    sys.exit(main())
