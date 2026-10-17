#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rc-drive.js 静态自查（在 `node --check` 之外补一刀）。

为什么需要它：2026-10-16 踩过一个坑——一次编辑把换行吃掉了，
`const RCD_SOFT_LIMIT_DEG = 30;` 被并进上一行的注释里，于是它**根本没被声明**。
`node --check` 只看语法，这种"声明被注释吞掉"照样通过；直到浏览器里点进驾驶模式才炸
（ReferenceError → 日志空白、手柄一个包都发不出去）。

跑法：
    python dev/web/check_js.py            # 有 node 就顺带做语法检查
    python dev/web/check_js.py --no-node  # 只用正则那几项
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

JS = Path(__file__).resolve().parent / "rc-drive.js"


def strip_comments(src: str) -> str:
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    return re.sub(r"//[^\n]*", "", src)


def check_undeclared(src: str) -> list[str]:
    """用了但没声明的 RCD_* / rcd* 标识符（这次的坑）。"""
    code = strip_comments(src)
    problems = []
    for prefix in ("RCD_", "rcd"):
        declared = set(re.findall(r"\b(?:const|let|var|function)\s+(" + prefix + r"\w*)", code))
        declared |= set(re.findall(r"\bfunction\s+(" + prefix + r"\w*)", code))
        declared |= set(re.findall(r"\((\w*(?:RCD|rcd)\w*)\)\s*=>", code))  # 箭头函数参数
        used = set(re.findall(r"\b(" + prefix + r"\w+)\b", code))
        # 属性名/字符串里的不算
        used -= set(re.findall(r"\.(" + prefix + r"\w+)", code))
        missing = sorted(u for u in used - declared if not u.endswith("_"))
        if prefix == "rcd":
            # rcd* 里排除掉 "rcd-xxx" 这类被引号包住的 id/类名
            missing = [m for m in missing if ('"' + m) not in src and ("'" + m) not in src]
        problems += [f"用了但没声明: {m}" for m in missing]
    return problems


def check_unused_functions(src: str) -> list[str]:
    fns = re.findall(r"function (rcd\w+)\s*\(", src)
    return [f"声明了但没调用: {f}" for f in fns if len(re.findall(r"\b" + f + r"\b", src)) < 2]


def check_ids(src: str) -> list[str]:
    """getElementById("rcd-xxx") 里的 id，必须在本文件里被创建过
    （要么是 markup 里的 id="rcd-xxx"，要么是 createElement 后 .id = "rcd-xxx"）。"""
    code = strip_comments(src)
    # 只认 getElementById("rcd-xxx") 这种"整串字面量"；拼接（"rcd-cfg-" + key）不算
    wanted = set(re.findall(r'getElementById\(\s*"(rcd-[\w-]+)"\s*\)', code))
    created = set(re.findall(r'id="(rcd-[\w-]+)"', src))
    created |= set(re.findall(r'\.id\s*=\s*"(rcd-[\w-]+)"', src))
    created |= set(re.findall(r'\.id\s*=\s*"(rcd-cfg-)"', src))
    problems = []
    for i in sorted(wanted):
        if i in created:
            continue
        if i.startswith("rcd-cfg-"):  # 配置输入框：id 由前缀拼接生成
            if any(c.startswith("rcd-cfg-") for c in created):
                continue
        problems.append(f'getElementById("{i}") 找不到对应 id="…" / .id = "…"')
    return problems


def check_node_syntax(target: Path = JS) -> list[str]:
    node = shutil.which("node")
    if not node:
        print("[skip] 没有 node，跳过语法检查")
        return []
    proc = subprocess.run([node, "--check", str(target)], capture_output=True, text=True)
    if proc.returncode != 0:
        return ["node --check 语法检查失败:\n" + (proc.stderr or "").strip()]
    print("[ok] node --check 语法检查通过")
    return []


def main() -> int:
    parser = argparse.ArgumentParser(description="rc-drive.js 静态自查")
    parser.add_argument("path", nargs="?", default=str(JS), help="要检查的 js（默认 rc-drive.js；可指向副本做反向验证）")
    parser.add_argument("--no-node", action="store_true", help="跳过 node 语法检查")
    args = parser.parse_args()

    target = Path(args.path)
    if not target.exists():
        print(f"[err] 找不到 {target}", file=sys.stderr)
        return 2
    src = target.read_text(encoding="utf-8")

    problems = []
    if not args.no_node:
        problems += check_node_syntax(target)
    problems += check_undeclared(src)
    problems += check_unused_functions(src)
    problems += check_ids(src)

    for p in problems:
        print("[FAIL] " + p)
    if problems:
        print(f"\n{len(problems)} 处问题（{target.name}）")
        return 1
    print("[ok] 未声明标识符 / 未调用函数 / getElementById 的 id 一致性 都通过")
    print(f"[ok] {target.name}: {len(src.splitlines())} 行")
    return 0


if __name__ == "__main__":
    sys.exit(main())
