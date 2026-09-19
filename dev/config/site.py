"""现场标定值覆盖层：读 `config/site.yaml`，把同名键覆盖到 `config/settings` 上。

**为什么要有这一层**：
- 标定值（车道中心 `target_x`、阈值等）属于「这台车 + 这个摄像头安装」的本地数据，
  不该提交进 git，也不该被代码同步覆盖（`site.yaml` 已在 ssh_sync 的排除名单里）；
- 程序启动时**自动**加载，于是跑车只需要一条命令，不需要额外传 `--target-x`（竞速比赛没时间跑两步）。

用法（一般不用手动调）：
    from config import settings          # 导入时 config 包会自动 apply 覆盖
    python -c "from config import site; print(site.load())"     # 看当前生效的覆盖项

写入方式：`bench_test.py --calibrate N` 会自动把标定结果写进 `config/site.yaml`。
"""
from __future__ import annotations

import os
from typing import Any, Dict

SITE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "site.yaml")
_applied: Dict[str, Any] = {}


def load(path: str = None) -> Dict[str, Any]:
    """读取 site.yaml（不存在或格式错时返回空 dict，绝不因此让程序起不来）。"""
    path = path or SITE_FILE
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        return {k: v for k, v in data.items() if not str(k).startswith("#")}
    except FileNotFoundError:
        return {}
    except Exception as exc:
        print(f"[site] 读取 {path} 失败（忽略）：{exc}")
        return {}


def save(values: Dict[str, Any], path: str = None) -> str:
    """把值合并写入 site.yaml（保留已有键），返回文件路径。"""
    path = path or SITE_FILE
    merged = load(path)
    merged.update({k: v for k, v in values.items() if v is not None})
    lines = ["# 现场标定值（车端本地数据：不进 git、不参与代码同步）",
             "# 由 scripts/bench_test.py --calibrate 写入；同名键会覆盖 config/settings.py",
             ""]
    for key in sorted(merged):
        lines.append(f"{key}: {merged[key]}")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    return path


def apply(path: str = None) -> Dict[str, Any]:
    """把 site.yaml 的键覆盖到 settings（**大小写不敏感**：yaml 里写 target_x 也能覆盖 TARGET_X）。

    只覆盖 settings 里已存在的名字，防止拼写错误悄悄生效。
    """
    global _applied
    from config import settings
    values = load(path)
    applied: Dict[str, Any] = {}
    for key, value in values.items():
        name = key if hasattr(settings, key) else str(key).upper()
        if hasattr(settings, name):
            setattr(settings, name, value)
            applied[key] = value
        else:
            print(f"[site] 忽略未知配置项：{key}（settings.py 里没有同名常量，可能是拼写错误）")
    _applied = applied
    if applied:
        print(f"[site] 已应用现场标定值：{applied}")
    return applied


def applied() -> Dict[str, Any]:
    """当前已生效的覆盖项（调试/状态显示用）。"""
    return dict(_applied)
