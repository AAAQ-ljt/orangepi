"""文件下载助手 —— 从目标机下载文件到本地(免交互).

用法:
    python scripts/ssh_get.py <远端绝对路径> <本地路径>        # 默认:小车(走服务器转发隧道)
    python scripts/ssh_get.py --direct <远端绝对路径> <本地路径> # 小车热点直连
    python scripts/ssh_get.py --server <远端绝对路径> <本地路径> # 公网服务器本身
"""
import argparse

import paramiko

TUNNEL = dict(host="121.40.149.155", port=2222, user="root", pwd="orangepi")
DIRECT = dict(host="10.68.1.43", port=22, user="root", pwd="orangepi")
SERVER = dict(host="121.40.149.155", port=22, user="root", pwd="Lyk12345678@@")


def main() -> None:
    parser = argparse.ArgumentParser(description="SSH 文件下载助手")
    parser.add_argument("--direct", action="store_true", help="小车走热点直连")
    parser.add_argument("--server", action="store_true", help="目标为公网服务器")
    parser.add_argument("remote", help="远端绝对路径")
    parser.add_argument("local", help="本地保存路径")
    args = parser.parse_args()

    if args.server:
        target = SERVER
    elif args.direct:
        target = DIRECT
    else:
        target = TUNNEL

    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(target["host"], port=target["port"], username=target["user"],
              password=target["pwd"], timeout=15, banner_timeout=15, auth_timeout=15)
    sftp = c.open_sftp()
    sftp.get(args.remote, args.local)
    sftp.close()
    print(f"[OK] {target['host']}:{args.remote} -> {args.local}")
    c.close()


if __name__ == "__main__":
    main()