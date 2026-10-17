#!/usr/bin/env python3
"""stop_run.sh 测试（不碰硬件、不碰电机；只测"停"的逻辑）。

覆盖 2026-10-16 现场教训：Ctrl+C 到不了车时，要用一个**能明确调用**的停车命令。
  · 语法/用法：`bash -n`；`--park` 未实现时必须**明确报错**（绝不假装停了）；
  · 没有目标在跑时是空操作（不碰任何硬件）；
  · **Linux 上**（车上/CI）：假目标进程跑起来，验证
      ① 会收 SIGTERM 的目标 → 走"温和停"（进程自己退出，输出「全部自己退出了」）；
      ② 忽略 SIGTERM 的目标 → 宽限到点后升到 SIGKILL（输出「SIGKILL」），并提示看门狗兜底。
    这两条验证不了 SIGTERM 语义的平台（Windows/MSYS：TerminateProcess 无法被忽略）会跳过。

跑法：cd dev && PYTHONPATH=. python tests/test_stop_run.py      （车上：bash scripts/run_tests.sh 也会跑到）
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DEV = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(DEV, "scripts", "stop_run.sh")
IS_LINUX = sys.platform.startswith("linux")


class _Skipped(Exception):
    """本平台跑不了（缺 pgrep / 不是 Linux）——**要单独报出来**，不许混进"通过"。"""


def _need(cond: bool, why: str) -> None:
    if not cond:
        raise _Skipped(why)


def _env(root: str, **extra: str) -> dict:
    env = dict(os.environ)
    env["CAR_DEV_ROOT"] = root
    env.update(extra)
    return env


def _fake_root(ignore_term: bool) -> str:
    """造一个假的 CAR_DEV_ROOT：里面有 scripts/lane_final_test.py（假目标）+ logs/。"""
    root = tempfile.mkdtemp(prefix="stoprun_")
    os.makedirs(os.path.join(root, "scripts"), exist_ok=True)
    os.makedirs(os.path.join(root, "logs"), exist_ok=True)
    trap = ("import signal\nsignal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
            if ignore_term else
            "import signal\n"
            "def _bye(*_a):\n    open(__file__ + '.term', 'w').write('term\\n')\n    raise SystemExit(0)\n"
            "signal.signal(signal.SIGTERM, _bye)\n")
    with open(os.path.join(root, "scripts", "lane_final_test.py"), "w", encoding="utf-8") as fh:
        fh.write("import time\n" + trap + "time.sleep(120)\n")
    return root


def _run(args: list[str], root: str, timeout: float = 30.0, **extra: str):
    return subprocess.run(["bash", SCRIPT] + args, env=_env(root, **extra),
                          capture_output=True, text=True, timeout=timeout)


def _spawn_fake(root: str):
    """跑假目标（进程命令行里带 …/scripts/lane_final_test.py，供 pgrep -f 命中）。"""
    return subprocess.Popen([sys.executable, os.path.join(root, "scripts", "lane_final_test.py")],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def test_syntax() -> None:
    r = subprocess.run(["bash", "-n", SCRIPT], capture_output=True, text=True)
    assert r.returncode == 0, f"bash -n 报错：{r.stderr}"


def test_no_crlf() -> None:
    """脚本里不能有 CR（回车字符）——血泪教训：CRLF 的 .sh 推上车会让 set -euo pipefail 报错、脚本当场死。"""
    raw = open(SCRIPT, "rb").read()
    assert bytes([13]) not in raw, (
        "stop_run.sh 里有 CR：推车前必须转 LF（否则车上 set -euo pipefail 会直接报错）")


def test_park_not_implemented() -> None:
    """--park 必须明确报错、什么都不做（免得现场以为按了就自动停车）。"""
    root = tempfile.mkdtemp(prefix="stoprun_")
    try:
        r = _run(["--park"], root)
        assert r.returncode == 2, f"--park 应退出 2，实得 {r.returncode}\n{r.stdout}"
        assert "还没实现" in r.stdout, f"必须说清楚没实现：{r.stdout}"
        assert "什么都没做" in r.stdout, f"必须说清楚什么都没做：{r.stdout}"
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_no_target_is_noop() -> None:
    """没有目标在跑 → 空操作、退出 0、不碰硬件。"""
    _need(shutil.which("pgrep") is not None, "需要 pgrep（车上/CI 有）")
    root = tempfile.mkdtemp(prefix="stoprun_")
    try:
        r = _run([], root)
        assert r.returncode == 0, f"空操作应退出 0，实得 {r.returncode}\n{r.stdout}\n{r.stderr}"
        assert "没有我们的跑动进程在运行" in r.stdout, r.stdout
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_status_lists_target() -> None:
    """--status 只读：列出在跑的目标、不动它。"""
    _need(IS_LINUX and shutil.which("pgrep") is not None, "需要 Linux 的 pgrep")
    root = _fake_root(ignore_term=False)
    p = _spawn_fake(root)
    try:
        time.sleep(1.0)
        r = _run(["--status"], root)
        assert r.returncode == 0, r.stdout
        assert "lane_final_test.py" in r.stdout, f"--status 应列出目标：{r.stdout}"
        assert p.poll() is None, "--status 不该动进程"
    finally:
        p.kill()
        p.wait(timeout=5)
        shutil.rmtree(root, ignore_errors=True)


def test_stops_graceful_target() -> None:
    """目标会收 SIGTERM → 走温和停（进程自己退出，不留硬杀痕迹）。"""
    _need(IS_LINUX and shutil.which("pgrep") is not None, "需要 Linux 的 pgrep/SIGTERM 语义")
    root = _fake_root(ignore_term=False)
    p = _spawn_fake(root)
    try:
        time.sleep(1.0)
        r = _run([], root, timeout=30)
        assert r.returncode == 0, f"应退出 0，实得 {r.returncode}\n{r.stdout}\n{r.stderr}"
        assert "全部自己退出了" in r.stdout, f"应走温和停：{r.stdout}"
        assert os.path.exists(os.path.join(root, "scripts", "lane_final_test.py.term")), \
            "目标没收到 SIGTERM（是被硬杀的吗？）"
        assert p.poll() is not None, "目标应该已经退出"
    finally:
        if p.poll() is None:
            p.kill()
        shutil.rmtree(root, ignore_errors=True)


def test_escalates_to_kill() -> None:
    """目标忽略 SIGTERM → 宽限到点后升到 SIGKILL，并提示看门狗兜底。"""
    _need(IS_LINUX and shutil.which("pgrep") is not None, "需要 Linux 的 pgrep/SIGTERM 语义")
    root = _fake_root(ignore_term=True)
    p = _spawn_fake(root)
    try:
        time.sleep(1.0)
        r = _run([], root, timeout=30, STOP_RUN_WAIT_S="1")
        assert "SIGKILL" in r.stdout, f"应升级到 SIGKILL：{r.stdout}"
        assert "看门狗" in r.stdout, f"硬杀路径必须提示看门狗/断电：{r.stdout}"
        time.sleep(0.5)
        assert p.poll() is not None, "目标应已被强杀"
    finally:
        if p.poll() is None:
            p.kill()
        shutil.rmtree(root, ignore_errors=True)


def test_refuses_without_pgrep() -> None:
    """没有 pgrep 时必须明确报错，不能静默当"没人跑"。"""
    _need(not IS_LINUX, "Linux 上必有 pgrep；这条测的是缺 procps 时的失败模式")
    root = tempfile.mkdtemp(prefix="stoprun_")
    try:
        r = subprocess.run(["bash", SCRIPT, "--status"],
                           env={**_env(root), "PATH": tempfile.gettempdir()},
                           capture_output=True, text=True, timeout=20)
        assert "找不到 pgrep" in (r.stdout + r.stderr), \
            f"缺 pgrep 时应明确报错：rc={r.returncode}\n{r.stdout}\n{r.stderr}"
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_file_mode_touches_stop_file() -> None:
    """★ 2026-10-17：`--file` = 走**停止文件通道**（首选）——先 touch 停止文件，没人响应再升级 TERM。

    停止文件是"不依赖 PID/命令行特征"的通道（主控主循环每帧查它）；这里用假目标（不读该文件）
    验证两件事：① 文件确实被创建（通道发出去了）；② 宽限到点后照旧升级 SIGTERM（假目标据此退出）。
    """
    _need(shutil.which("pgrep") is not None, "需要 Linux 的 pgrep")
    _need(IS_LINUX, "需要 Linux 的 pgrep/SIGTERM 语义")
    root = _fake_root(ignore_term=False)
    stop_file = os.path.join(root, "lane_stop")
    p = _spawn_fake(root)
    try:
        time.sleep(0.3)
        r = _run(["--file", "--wait", "1"], root, STOP_RUN_FILE=stop_file)
        assert os.path.exists(stop_file), "--file 应 touch 停止文件（首选通道）"
        assert "停止文件" in r.stdout, r.stdout
        assert os.path.exists(os.path.join(root, "scripts", "lane_final_test.py.term")), \
            f"文件通道没人响应时应升级到 TERM（假目标据此退出）：\n{r.stdout}"
    finally:
        p.kill()
        p.wait(timeout=3)
        shutil.rmtree(root, ignore_errors=True)


def main() -> int:
    funcs = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v) and getattr(v, "__module__", "") == __name__]
    failed = skipped = 0
    skip_names: list[str] = []
    for fn in funcs:
        try:
            fn()
            print(f"  ✅ {fn.__name__}")
        except _Skipped as exc:
            skipped += 1
            skip_names.append(f"{fn.__name__}（{exc}）")
            print(f"  ⏭️  {fn.__name__}: 跳过（{exc}）")
        except Exception as exc:                          # noqa: BLE001
            failed += 1
            import traceback
            print(f"  ❌ {fn.__name__}: {exc}")
            traceback.print_exc()
    ran = len(funcs) - skipped
    msg = f"[test_stop_run] {ran - failed}/{ran} passed"
    if skipped:
        msg += f"，{skipped} 个跳过未验证：" + "；".join(skip_names)
    print(msg)
    if skipped:
        print("      ⚠️ 跳过的用例**没有**验证过——车上（Linux）跑一遍才算数")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
