"""代码同步助手 —— 把本地 dev/ 镜像到小车 /root/dev/（免交互、增量、可预览）。

设计目标：**只传变化的文件**，不碰小车上的运行数据（数据集、模型、日志、测试图片）；
**并且绝不把「在车上改得更新的文件」悄悄覆盖掉**（2026-09-19 就这么丢过一次车端代码）。

用法:
    python scripts/ssh_sync.py --dry-run            # 预览：列出将要上传/删除的文件
    python scripts/ssh_sync.py                      # 同步 dev/ -> /root/dev/
    python scripts/ssh_sync.py --force              # 远端更新的文件也强行覆盖（确认过再用）
    python scripts/ssh_sync.py --delete             # 同时删除远端多余文件（谨慎！默认不删）
    python scripts/ssh_sync.py --include-models     # 连 *.rknn 一起传（默认跳过，通常用 U 盘/其他方式传）
    python scripts/ssh_sync.py --direct             # 热点直连（隧道不通时）
    python scripts/ssh_sync.py --src dev --dst /root/dev

⚠️ 同步是**单向的（本地 → 车）**：在车上直接改过的代码必须手动拉回仓库
   （`python scripts/ssh_get.py <远端文件> <本地路径>`），否则下次同步就是覆盖。

排除规则（可按需在 EXCLUDE_DIRS / EXCLUDE_SUFFIXES 里调整）:
    __pycache__、*.pyc、.git、logs/、img 下采集的测试图片、*.rknn/*.pt/*.onnx
"""
from __future__ import annotations

import argparse
import os
import posixpath
import sys
from typing import Dict, List, Tuple

import paramiko

TUNNEL = dict(host="121.40.149.155", port=2222, user="root", pwd="orangepi")
DIRECT = dict(host="10.68.1.43", port=22, user="root", pwd="orangepi")

EXCLUDE_DIRS = {"__pycache__", ".git", ".idea", ".vscode", "logs", "runs", "dataset", "datasets"}
EXCLUDE_SUFFIXES = (".pyc", ".pyo", ".log", ".mp4", ".avi", ".pid")
MODEL_SUFFIXES = (".rknn", ".pt", ".onnx", ".onnx.rknn")
# 车端本地数据/配置：每台车自己的状态，不能被仓库覆盖
# （wifi-ledger.conf 里是真实密码；site.yaml 是现场标定值）
LOCAL_ONLY_FILES = {"wifi-ledger.conf", "site.yaml"}
# img/ 下的采集目录（测试图片），不同步
IMG_DATA_PREFIXES = ("dev/img/test", "dev/img/direct_test")


def is_excluded(rel_posix: str, include_models: bool) -> bool:
    parts = rel_posix.split("/")
    if any(p in EXCLUDE_DIRS for p in parts):
        return True
    if rel_posix.endswith(EXCLUDE_SUFFIXES):
        return True
    if parts[-1] in LOCAL_ONLY_FILES:
        return True
    if not include_models and rel_posix.endswith(MODEL_SUFFIXES):
        return True
    if any(rel_posix.startswith(p) for p in IMG_DATA_PREFIXES):
        return True
    return False


def collect_local(src: str, include_models: bool) -> Dict[str, Tuple[str, int]]:
    """返回 {相对路径(posix): (本地绝对路径, 字节数)}。"""
    src = os.path.abspath(src)
    out: Dict[str, Tuple[str, int]] = {}
    for root, dirs, files in os.walk(src):
        dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
        for name in files:
            full = os.path.join(root, name)
            rel = os.path.relpath(full, src).replace("\\", "/")
            if is_excluded(f"dev/{rel}", include_models):
                continue
            try:
                out[rel] = (full, os.path.getsize(full))
            except OSError:
                continue
    return out


def remote_listing(sftp: paramiko.SFTPClient, base: str) -> Dict[str, Tuple[int, float]]:
    """递归列出远端文件 {相对路径: (字节数, mtime)}；目录不存在则返回空。"""
    out: Dict[str, Tuple[int, float]] = {}

    def walk(path: str, rel: str) -> None:
        try:
            entries = sftp.listdir_attr(path)
        except IOError:
            return
        for e in entries:
            child_rel = f"{rel}/{e.filename}" if rel else e.filename
            child = posixpath.join(path, e.filename)
            if e.st_mode and (e.st_mode & 0o040000):     # 目录
                if e.filename in EXCLUDE_DIRS:
                    continue
                walk(child, child_rel)
            else:
                out[child_rel] = (e.st_size or 0, float(e.st_mtime or 0))

    walk(base, "")
    return out


def content_equal(local_path: str, remote_bytes: bytes) -> bool:
    """内容是否一致；**行尾差异（LF↔CRLF）不算差异**。

    为什么需要：Windows 工作区里 git 会把 .py 换成 CRLF（每行多 1 字节），
    于是"按字节数比大小"的增量同步会认为**每个文件都变了**——2026-09-19 就是这样
    一次性上传 56 个文件、把车端更新的实现覆盖掉的。
    """
    try:
        with open(local_path, "rb") as fh:
            local_bytes = fh.read()
    except OSError:
        return False
    if local_bytes == remote_bytes:
        return True
    if b"\0" in local_bytes or b"\0" in remote_bytes:      # 二进制不按行尾比较
        return False
    return (local_bytes.replace(b"\r\n", b"\n")
            == remote_bytes.replace(b"\r\n", b"\n"))


def read_remote(sftp: paramiko.SFTPClient, path: str, limit: int = 4 << 20) -> bytes:
    """读远端文件内容（超过 limit 字节的文件不读，返回空串表示"没法比"）。"""
    try:
        with sftp.open(path, "rb") as fh:
            fh.prefetch()
            return fh.read(limit)
    except IOError:
        return b""


def ensure_remote_dir(sftp: paramiko.SFTPClient, path: str, cache: set) -> None:
    if path in cache or path in ("", "/"):
        return
    parent = posixpath.dirname(path.rstrip("/"))
    ensure_remote_dir(sftp, parent, cache)
    try:
        sftp.stat(path)
    except IOError:
        sftp.mkdir(path)
        print(f"  [mkdir] {path}")
    cache.add(path)


def main() -> int:
    parser = argparse.ArgumentParser(description="同步 dev/ 到小车")
    parser.add_argument("--src", default="dev", help="本地源目录（默认 dev）")
    parser.add_argument("--dst", default="/root/dev", help="远端目标目录（默认 /root/dev）")
    parser.add_argument("--direct", action="store_true", help="热点直连")
    parser.add_argument("--dry-run", action="store_true", help="只预览，不传输")
    parser.add_argument("--delete", action="store_true", help="删除远端多余文件（默认保留）")
    parser.add_argument("--include-models", action="store_true", help="连 *.rknn/*.pt/*.onnx 一起传")
    parser.add_argument("--force", action="store_true",
                       help="远端文件比本地新时也覆盖（默认跳过并告警，防止覆盖车上直接改的代码）")
    args = parser.parse_args()

    local = collect_local(args.src, args.include_models)
    target = DIRECT if args.direct else TUNNEL

    print(f"[SYNC] {args.src} -> {target['host']}:{args.dst}")
    print(f"[SYNC] 本地待同步文件 {len(local)} 个（模型={'含' if args.include_models else '跳过'}）")

    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(target["host"], port=target["port"], username=target["user"],
              password=target["pwd"], timeout=20, banner_timeout=20, auth_timeout=20)
    sftp = c.open_sftp()
    try:
        remote = remote_listing(sftp, args.dst)
        to_upload: List[str] = []
        skipped_newer: List[str] = []
        for rel, (full, size) in sorted(local.items()):
            info = remote.get(rel)
            if info is None:
                to_upload.append(rel)                      # 远端还没有
                continue
            r_size, r_mtime = info
            if r_size == size:
                continue                                   # 大小一致，先当没变
            dst_path = posixpath.join(args.dst, rel)
            if content_equal(full, read_remote(sftp, dst_path)):
                continue                                   # 只是行尾差异，别当改动
            if not args.force and r_mtime > os.path.getmtime(full) + 2:
                skipped_newer.append(rel)                  # 远端更新 → 疑似在车上直接改的
                continue
            to_upload.append(rel)
        # stale 必须同时套用 is_excluded：remote 是全量、local 是过滤后的，
        # 否则 --delete 会把「有意不同步的车端资源」当多余文件删掉
        # （models/*.rknn、config/site.yaml 标定值、wifi-ledger.conf —— 2026-09-23 代码审查 A2）。
        stale_all = [rel for rel in sorted(remote) if rel not in local]
        stale = [rel for rel in stale_all
                 if not is_excluded(f"dev/{rel}", args.include_models)]
        protected = [rel for rel in stale_all
                     if is_excluded(f"dev/{rel}", args.include_models)]

        if skipped_newer:
            print(f"[SYNC] ⚠️ 跳过 {len(skipped_newer)} 个「远端更新」的文件（不覆盖车上直接改的内容）：")
            for rel in skipped_newer:
                print(f"  [skip] {rel}   ← 要覆盖请加 --force；要保留请先 ssh_get 拉回仓库")

        if args.delete and protected:
            print(f"[SYNC] 远端 {len(protected)} 个文件受排除规则保护，--delete 不会动它们：")
            for rel in protected:
                print(f"  [keep] {rel}")

        if args.delete and stale:
            print(f"[SYNC] 将删除远端多余文件 {len(stale)} 个：")
            for rel in stale:
                print(f"  [del ] {rel}")

        if not to_upload and not (args.delete and stale):
            print("[SYNC] 已是最新，无需传输 ✅")
            return 0

        print(f"[SYNC] 需上传 {len(to_upload)} 个：")
        dir_cache: set = set()
        for rel in to_upload:
            full, size = local[rel]
            dst_path = posixpath.join(args.dst, rel)
            print(f"  [{'upd' if remote.get(rel) is not None else 'new'}] {rel} ({size} B)")
            if args.dry_run:
                continue
            ensure_remote_dir(sftp, posixpath.dirname(dst_path), dir_cache)
            sftp.put(full, dst_path)

        if args.delete and stale and not args.dry_run:
            for rel in stale:
                sftp.remove(posixpath.join(args.dst, rel))
        if args.dry_run:
            print("[SYNC] --dry-run：未做任何改动")
        else:
            print(f"[SYNC] 完成：上传 {0 if args.dry_run else len(to_upload)} 个文件 ✅")
    finally:
        sftp.close()
        c.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
