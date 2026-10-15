#!/usr/bin/env python3
"""全量哈希审计：把一棵 `dev/` 树的"代码文件内容指纹"打出来，用于**推车前后的漂移核查**。

为什么要有它（2026-10-11/12 两次踩坑）
-------------------------------------
并行会话/AI 会直接改车端 `/root/dev`，而 `ssh_sync` 的"远端更新"判断看的是 **mtime**——
车端时钟慢 12h（10-11 实测），所以 mtime 不可信：一次 `ssh_sync` 就把车上**更新的**
`lane_ref.py`/`lane_ref_test.py` 覆盖回了旧版（10-11 晚修的一整批：时域拒绝、`--show`、
target 兜底链、tag 列全都丢了）。正确姿势是**先比内容**：

    # 车端（经 ssh 跑）与本地各出一份清单，再 diff
    python scripts/ssh_car.py "python3 /root/dev/scripts/diag/repo_hash_audit.py" > /tmp/car.txt
    python dev/scripts/diag/repo_hash_audit.py dev > /tmp/local.txt
    diff <(sort /tmp/car.txt) <(sort /tmp/local.txt)

输出：每行 `<相对路径> <md5>`（**CRLF 归一后**计算，Windows 工作区与车端可比）。
只扫代码/配置类文件；跳过 `models/`、`logs/`、`img/`（采集数据）、`attic/`、`backup-*`、
`__pycache__`、`data/`。
"""
from __future__ import annotations

import hashlib
import os
import sys

EXTS = (".py", ".sh", ".yaml", ".yml", ".service", ".timer", ".conf", ".md", ".link", ".env", ".json")
SKIP_DIRS = {"logs", "models", "img", "attic", "data", "__pycache__", ".git"}
SKIP_PREFIX = ("backup-",)


def norm_hash(path: str) -> str:
    """md5（LF 归一）：Windows 工作区的 CRLF 与车端 LF 应当比出**同一个**指纹。"""
    h = hashlib.md5()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk.replace(b"\r\n", b"\n"))
    return h.hexdigest()


def audit(root: str) -> int:
    n = 0
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames
                       if d not in SKIP_DIRS and not d.startswith(SKIP_PREFIX)]
        for name in sorted(filenames):
            if not name.endswith(EXTS):
                continue
            full = os.path.join(dirpath, name)
            rel = os.path.relpath(full, root).replace(os.sep, "/")
            try:
                print(f"{rel} {norm_hash(full)}")
            except OSError as exc:
                print(f"# {rel}: {exc}", file=sys.stderr)
                continue
            n += 1
    print(f"# {n} files under {root}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(audit(sys.argv[1] if len(sys.argv) > 1 else "/root/dev"))
