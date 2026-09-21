"""文件下载助手 —— 从目标机下载文件到本地(免交互).

用法:
    python scripts/ssh_get.py <远端绝对路径> <本地路径>           # 默认:小车(走服务器转发隧道)
    python scripts/ssh_get.py -r <远端目录> <本地目录>            # 递归下载整个目录
    python scripts/ssh_get.py --direct <远端绝对路径> <本地路径>  # 小车热点直连
    python scripts/ssh_get.py --server <远端绝对路径> <本地路径>  # 公网服务器本身

示例:
    python scripts/ssh_get.py /root/dev/logs/lane_ref.csv .
    python scripts/ssh_get.py -r /root/dev/logs/lane_probe .
"""
import argparse
import os
import stat

import paramiko

from remote_path import normalize_remote

TUNNEL = dict(host="121.40.149.155", port=2222, user="root", pwd="orangepi")
DIRECT = dict(host="10.68.1.43", port=22, user="root", pwd="orangepi")
SERVER = dict(host="121.40.149.155", port=22, user="root", pwd="Lyk12345678@@")


def _pull_dir(sftp, remote: str, local: str) -> int:
    """把远端目录整棵拉下来，返回文件数（远端路径一律用 /，不做本地路径转换）。"""
    os.makedirs(local, exist_ok=True)
    n = 0
    for entry in sftp.listdir_attr(remote):
        r_path = remote.rstrip("/") + "/" + entry.filename
        l_path = os.path.join(local, entry.filename)
        if stat.S_ISDIR(entry.st_mode):
            n += _pull_dir(sftp, r_path, l_path)
        else:
            sftp.get(r_path, l_path)
            print(f"[OK] {r_path} -> {l_path}")
            n += 1
    return n


def main() -> None:
    parser = argparse.ArgumentParser(description="SSH 文件下载助手")
    parser.add_argument("-r", "--recursive", action="store_true", help="远端是目录，递归下载")
    parser.add_argument("--direct", action="store_true", help="小车走热点直连")
    parser.add_argument("--server", action="store_true", help="目标为公网服务器")
    parser.add_argument("remote", help="远端绝对路径")
    parser.add_argument("local", help="本地保存路径")
    args = parser.parse_args()
    args.remote = normalize_remote(args.remote)

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
    if args.recursive:
        n = _pull_dir(sftp, args.remote, args.local)
        print(f"[OK] {target['host']}:{args.remote} -> {args.local}（{n} 个文件）")
    else:
        sftp.get(args.remote, args.local)
        print(f"[OK] {target['host']}:{args.remote} -> {args.local}")
    sftp.close()
    c.close()


if __name__ == "__main__":
    main()