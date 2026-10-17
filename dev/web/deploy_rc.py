#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""驾驶模式部署/还原 —— 往官方遥控网页 index.html 注入一行 <script src="/rc-drive.js">.

    python dev/web/deploy_rc.py --dry-run     # 预览：改哪个文件、插哪一行（尽量拉车端文件核对，拉不到就用本地副本）
    python dev/web/deploy_rc.py               # 备份原件 → 插一行 → 上传 index.html + rc-drive.js
    python dev/web/deploy_rc.py --restore     # 用备份还原官方 index.html（备份找不到则退化为"就地剥掉补丁"）
    python dev/web/deploy_rc.py --local-page 某份/index.html   # 完全离线：只改本地这份（本地联调用）

幂等：index.html 里已经有 `smartcar-rc-drive` 标记就只补传 rc-drive.js，不重复插入。
只动 /var/www/orangepi-rc/ 下的两个文件；不碰 nginx / systemd / opi-control / frp。

⚠️ 这是往**官方资产**里插一行（AGENTS.md §1.2/§9）：执行前需队长点头。有一键还原。
⚠️ 官方若跑一次 sync_to_car.py，会用它的镜像覆盖掉这一行 → 重跑本脚本即可（补丁幂等）。
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
WEB_DIR = Path(__file__).resolve().parent
LOCAL_JS = WEB_DIR / "rc-drive.js"
LOCAL_INDEX_COPY = WEB_DIR / "index.html"  # 仓库里留的官方页副本（离线预演用）

# 车端路径
CAR_INDEX = "/var/www/orangepi-rc/index.html"
CAR_JS = "/var/www/orangepi-rc/rc-drive.js"
CAR_WEB_DIR = "/var/www/orangepi-rc"
BACKUP_GLOB = "/root/dev/backup-*/index.html"

MARKER = "smartcar-rc-drive"  # 幂等标记；部署后 `curl -s .../ | grep -c smartcar-rc-drive` 期望 1
COMMENT = "<!-- %s: 驾驶模式（由 dev/web/deploy_rc.py 注入，勿手改） -->" % MARKER
SCRIPT_TAG = '<script src="/rc-drive.js"></script>'
BACKUP_PREFIX = "backup-"  # 车端备份目录前缀（与 AGENTS.md §3.1 一致）


# --------------------------------------------------------------------------
# 纯函数（可离线单测，见 dev/tests/test_web_deploy.py）
# --------------------------------------------------------------------------
def detect_eol(text: str) -> str:
    """按文件现有内容判断换行风格（官方页是 CRLF，插进去的行要跟着走）。"""
    return "\r\n" if "\r\n" in text else "\n"


def is_patched(html: str) -> bool:
    return MARKER in html


def patch_html(html: str) -> tuple[str, bool]:
    """在 </body> 前插入标记行 + script 标签。已打过补丁则原样返回。

    按行插入（而不是按字符偏移），这样 strip_patch 能把这两行原样剥掉、
    与原件逐字节一致——否则会连带吃掉 </body> 那一行的缩进。
    """
    if is_patched(html):
        return html, False
    eol = detect_eol(html)
    lines = html.split(eol)

    target = None
    for tag in ("</body>", "</html>"):
        for i in range(len(lines) - 1, -1, -1):
            if tag in lines[i]:
                target = i
                break
        if target is not None:
            break
    if target is None:
        raise ValueError("没找到 </body> 或 </html>，不是一份完整网页？")

    line = lines[target]
    indent = line[: len(line) - len(line.lstrip())]  # 跟 </body> 对齐
    lines[target:target] = [indent + COMMENT, indent + SCRIPT_TAG]
    return eol.join(lines), True


def strip_patch(html: str) -> tuple[str, bool]:
    """把补丁那两行剥掉（--restore 找不到备份时的退路）。"""
    if not is_patched(html):
        return html, False
    eol = detect_eol(html)
    keep = []
    for line in html.split(eol):
        stripped = line.strip()
        if MARKER in line or stripped == SCRIPT_TAG:
            continue
        keep.append(line)
    return eol.join(keep), True


# --------------------------------------------------------------------------
# 本地 / 车端 I-O
# --------------------------------------------------------------------------
def read_text(path: Path) -> str:
    return path.read_bytes().decode("utf-8")


def write_text(path: Path, text: str) -> None:
    """二进制写：保证 CRLF 不被平台换行转换悄悄改掉。"""
    path.write_bytes(text.encode("utf-8"))


def _env() -> dict:
    env = dict(os.environ)
    env["MSYS_NO_PATHCONV"] = "1"  # Git Bash 路径转换（AGENTS.md §6）
    return env


def _run(script: str, *args: str, check: bool = True) -> subprocess.CompletedProcess:
    cmd = [sys.executable, str(REPO_ROOT / "scripts" / script), *args]
    print(f"[run] {' '.join(cmd[1:])}", flush=True)
    return subprocess.run(cmd, env=_env(), check=check, text=True, capture_output=True)


def ssh_cmd(command: str, check: bool = True) -> str:
    """在车端执行一条命令（走 frp 隧道，只读或建备份目录用）。

    ⚠️ scripts/ssh_car.py 会在 stdout **末尾附一行 `[exit code: N]`**，必须滤掉，
    否则把它当路径用会得到一条带尾巴的"路径"（2026-10-16 踩到：--restore 直接 ENOENT）。
    """
    proc = _run("ssh_car.py", command, check=check)
    raw = proc.stdout or ""
    lines = [ln for ln in raw.splitlines() if not ln.strip().startswith("[exit code:")]
    out = "\n".join(lines).strip()
    if out:
        print(out)
    if proc.stderr and proc.stderr.strip():
        print("[stderr] " + proc.stderr.strip(), file=sys.stderr)
    return out


def ssh_pull(remote: str, local: Path) -> bool:
    """拉文件；失败时把可能留下的半截/空文件删掉（别让它被当成有效下载）。"""
    local.parent.mkdir(parents=True, exist_ok=True)
    try:
        _run("ssh_get.py", remote, str(local))
    except subprocess.CalledProcessError as exc:
        print(f"[warn] 拉取失败 {remote}: {(exc.stderr or '').strip() or exc}", file=sys.stderr)
        _drop(local)
        return False
    if not local.exists() or local.stat().st_size == 0:
        print(f"[warn] 拉下来的 {local} 是空文件，按失败处理", file=sys.stderr)
        _drop(local)
        return False
    return True


def _drop(path: Path) -> None:
    try:
        if path.exists():
            path.unlink()
    except OSError:
        pass


def ssh_push(local: Path, remote: str) -> bool:
    try:
        _run("ssh_put.py", str(local), remote)
        return True
    except subprocess.CalledProcessError as exc:
        print(f"[err] 上传失败 {local} -> {remote}: {(exc.stderr or '').strip() or exc}", file=sys.stderr)
        return False


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------
def plan_and_report(html: str, origin: str) -> tuple[str, bool]:
    patched = is_patched(html)
    print(f"[info] 页面来源：{origin}")
    print(f"[info] 已有 {MARKER} 标记：{'是（跳过插入）' if patched else '否（将插入）'}")
    if patched:
        return html, False
    new_html, _ = patch_html(html)
    eol = "CRLF" if detect_eol(html) == "\r\n" else "LF"
    print(f"[info] 换行风格：{eol}；插入位置：最后一个 </body> 之前")
    print("[info] 将插入：")
    for line in (COMMENT, SCRIPT_TAG):
        print("        " + line)
    return new_html, True


def do_local_page(path: Path, dry_run: bool) -> int:
    if not path.exists():
        print(f"[err] 找不到 {path}", file=sys.stderr)
        return 2
    html = read_text(path)
    new_html, changed = plan_and_report(html, str(path))
    if dry_run or not changed:
        return 0
    write_text(path, new_html)
    print(f"[OK] 已写入 {path}")
    return 0


def do_deploy(dry_run: bool) -> int:
    if not LOCAL_JS.exists():
        print(f"[err] 主脚本不存在：{LOCAL_JS}", file=sys.stderr)
        return 2

    cache = WEB_DIR / ".car-index.cache.html"
    got = ssh_pull(CAR_INDEX, cache)
    if got:
        html = read_text(cache)
        origin = f"车端 {CAR_INDEX}"
    elif LOCAL_INDEX_COPY.exists():
        html = read_text(LOCAL_INDEX_COPY)
        origin = f"本地副本 {LOCAL_INDEX_COPY}（车端拉取失败，仅供预览）"
    else:
        print("[err] 车端拉不到、本地也没有副本，无法继续", file=sys.stderr)
        return 2

    new_html, changed = plan_and_report(html, origin)
    if dry_run:
        print("[dry-run] 没有改任何东西。")
        return 0

    if changed:
        if got:
            stamp = _dt.datetime.now().strftime("%Y%m%d")
            backup_dir = f"/root/dev/{BACKUP_PREFIX}{stamp}"
            # cp -n：同一天重复部署不会覆盖第一份原件
            ssh_cmd(f"mkdir -p {backup_dir} && cp -n {CAR_INDEX} {backup_dir}/index.html && ls -l {backup_dir}/index.html")
            if not ssh_pull(f"{backup_dir}/index.html", WEB_DIR / ".car-index.backup.html"):
                print("[warn] 备份校验没拉到，继续上传（备份命令已执行）", file=sys.stderr)
        else:
            print("[warn] 车端拉取失败，但将按本地副本的内容上传（有风险，请先确认车端在线）", file=sys.stderr)
        write_text(cache, new_html)
        if not ssh_push(cache, CAR_INDEX):
            return 3
    else:
        print("[info] index.html 无需改动，只补传 rc-drive.js")

    if not ssh_push(LOCAL_JS, CAR_JS):
        return 3
    print("[OK] 部署完成。核对：")
    print("    curl -sI http://121.40.149.155:8080/            # 期望 200")
    print(f"    curl -s  http://121.40.149.155:8080/ | grep -c {MARKER}   # 期望 1")
    return 0


def do_restore(dry_run: bool, from_backup: bool = False) -> int:
    """还原官方 index.html。

    默认口径 = **就地剥掉补丁那两行**（拉当前页 → 删标记行 → 上传）：只撤销我们加的东西，
    其余内容一字不动。这是最安全的还原方式。

    `--from-backup` 才用 `/root/dev/backup-*/index.html` 里的原件——⚠️ 那些备份是**不同日期**
    的快照（实测 0915 那份 74875 字节、当前页 79018 字节），拿旧的覆盖会把后来别人对官方页的
    改动一起回退掉，所以默认不用它，用它时会先把差异打出来。
    """
    tmp = WEB_DIR / ".car-index.restore.html"
    if from_backup:
        listing_raw = ssh_cmd(f"ls -1dt {BACKUP_GLOB} 2>/dev/null | head -1", check=False)
        listing = listing_raw.splitlines()[0].strip() if listing_raw.splitlines() else ""
        if not listing:
            print("[err] 车端没找到任何备份，改用默认口径（就地剥补丁）", file=sys.stderr)
            return do_restore(dry_run, from_backup=False)
        print(f"[info] 使用备份：{listing}")
        if not ssh_pull(listing, tmp):
            return 3
        backup_html = read_text(tmp)
        current = WEB_DIR / ".car-index.current.html"
        if ssh_pull(CAR_INDEX, current):
            cur_html = read_text(current)
            b_size = len(backup_html.encode("utf-8"))
            c_size = len(cur_html.encode("utf-8"))
            print(f"[warn] 备份 {b_size} 字节 vs 当前页 {c_size} 字节（差 {c_size - b_size:+d}）"
                  "——差异会被一起回退，确认后再执行")
            _drop(current)
    else:
        if not ssh_pull(CAR_INDEX, tmp):
            return 3
        html = read_text(tmp)
        html, changed = strip_patch(html)
        if not changed:
            print(f"[info] 当前页本来就没有 {MARKER} 补丁，无需还原")
            _drop(tmp)
            return 0
        write_text(tmp, html)
        print(f"[info] 已剥掉补丁两行（剩 {len(html.encode('utf-8'))} 字节），其余内容一字未动")

    if dry_run:
        print(f"[dry-run] 将用 {tmp} 覆盖 {CAR_INDEX}")
        _drop(tmp)
        return 0
    if not ssh_push(tmp, CAR_INDEX):
        return 3
    _drop(tmp)
    print("[OK] 已还原官方 index.html（rc-drive.js 保留在服务器上，未被引用即无副作用）")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="驾驶模式部署/还原（往官方 index.html 注入一行）")
    parser.add_argument("--dry-run", action="store_true", help="只预览，不改车端")
    parser.add_argument("--restore", action="store_true", help="还原官方 index.html（默认=就地剥掉补丁两行）")
    parser.add_argument(
        "--from-backup",
        action="store_true",
        help="还原时改用 /root/dev/backup-*/index.html 的原件（会一并回退别人后来的改动，谨慎）",
    )
    parser.add_argument(
        "--local-page",
        metavar="INDEX_HTML",
        help="完全离线：只对这份本地 index.html 打补丁（不连车）",
    )
    args = parser.parse_args()

    if args.local_page:
        return do_local_page(Path(args.local_page), args.dry_run)
    if args.restore:
        return do_restore(args.dry_run, from_backup=args.from_backup)
    return do_deploy(args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
