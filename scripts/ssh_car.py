"""小车 SSH 助手 —— AI 自动连接香橙派小车使用(免交互).

用法:
    python scripts/ssh_car.py <命令...>              # 默认:走服务器转发(121.40.149.155:2222, 推荐)
    python scripts/ssh_car.py --direct <命令...>     # 备选:热点局域网直连(10.68.1.43:22)
    python scripts/ssh_car.py --timeout 60 <命令...> # 命令超时秒数(默认 30)

示例:
    python scripts/ssh_car.py uname -a
    python scripts/ssh_car.py --direct hostname -I
"""
import argparse
import sys

import paramiko

# 服务器转发(主): 公网服务器 → frp 隧道 → 小车 SSH
TUNNEL = dict(host="121.40.149.155", port=2222, user="root", pwd="orangepi")
# 热点直连(备): 手机热点局域网
DIRECT = dict(host="10.68.1.43", port=22, user="root", pwd="orangepi")


def main() -> None:
    parser = argparse.ArgumentParser(description="小车 SSH 助手")
    parser.add_argument("--direct", action="store_true", help="热点局域网直连(默认走服务器转发)")
    parser.add_argument("--timeout", type=float, default=30.0, help="命令超时秒数")
    parser.add_argument("cmd", nargs="*", help="要执行的命令(不加引号也行)")
    args = parser.parse_args()

    target = DIRECT if args.direct else TUNNEL
    cmd = " ".join(args.cmd) if args.cmd else "echo connected"

    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(
        target["host"],
        port=target["port"],
        username=target["user"],
        password=target["pwd"],
        timeout=15,
        banner_timeout=15,
        auth_timeout=15,
    )
    _, stdout, stderr = c.exec_command(cmd, timeout=args.timeout)
    out = stdout.read().decode(errors="replace")
    err = stderr.read().decode(errors="replace")
    rc = stdout.channel.recv_exit_status()
    sys.stdout.write(out)
    if err:
        sys.stdout.write("[stderr]\n" + err)
    sys.stdout.write(f"\n[exit code: {rc}]\n")
    c.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()