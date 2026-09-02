"""远程服务器 SSH 助手 —— AI 自动连接公网服务器(免交互).

用法:
    python scripts/ssh_server.py "要执行的命令"
    python scripts/ssh_server.py "uname -a" 20     # 可选第二参数:超时秒数(默认 30)

连接参数: 121.40.149.155 (root)—— 凭据由用户提供,注意保密。
"""
import sys

import paramiko

HOST = "121.40.149.155"
PORT = 22
USER = "root"
PWD = "Lyk12345678@@"


def main() -> None:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "echo connected"
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(
        HOST,
        port=PORT,
        username=USER,
        password=PWD,
        timeout=15,
        banner_timeout=15,
        auth_timeout=15,
    )
    _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
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