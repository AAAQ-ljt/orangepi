"""远端路径纠正 —— 抵消 Git Bash(MSYS) 的自动路径转换。

Git Bash 会把命令行里的 `/root/x` 这类参数自动转成 `E:/Program Files/Git/root/x`，
于是远端报 `No such file`。把这段纠正逻辑抽出来给所有 ssh/serial 助手共用，
比要求每个人记得加 `MSYS_NO_PATHCONV=1` 更省心（两边都做，双保险）。
"""
from __future__ import annotations

REMOTE_ROOTS = ("/root/", "/etc/", "/opt/", "/var/", "/tmp/", "/usr/", "/run/", "/home/", "/srv/")


def normalize_remote(path: str) -> str:
    """把被 MSYS 污染的远端绝对路径纠正回 `/root/...` 形式；本来就正常就原样返回。"""
    if path.startswith("/") and not path.startswith("//"):
        return path
    norm = path.replace("\\", "/")
    for root in REMOTE_ROOTS:
        idx = norm.find(root)
        if idx > 0:
            fixed = norm[idx:]
            print(f"[path] ⚠️ 检测到 MSYS 路径转换，已纠正：{path} → {fixed}")
            return fixed
    return norm
