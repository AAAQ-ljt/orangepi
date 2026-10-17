"""驾驶模式部署脚本（dev/web/deploy_rc.py）的补丁/还原逻辑单测。

要验证的是"往官方 index.html 插一行"这件事**只多一行、可幂等、可完整还原**：
    patch_html 幂等（重复部署不重复插入）→ strip_patch 能剥干净 → 与原件逐字节相同。

跑法：
    cd dev && PYTHONPATH=. python tests/test_web_deploy.py
"""
from __future__ import annotations

import sys
from pathlib import Path

WEB_DIR = Path(__file__).resolve().parents[1] / "web"
sys.path.insert(0, str(WEB_DIR))

import deploy_rc  # noqa: E402

OFFICIAL_COPY = WEB_DIR / "index.html"  # 仓库里的官方页副本


def _sample(eol: str = "\r\n") -> str:
    return eol.join([
        "<html>",
        "  <body>",
        "    <h1>hi</h1>",
        "  </body>",
        "</html>",
        "",
    ])


def test_detect_eol():
    assert deploy_rc.detect_eol(_sample("\r\n")) == "\r\n"
    assert deploy_rc.detect_eol(_sample("\n")) == "\n"


def test_patch_inserts_before_body_and_keeps_eol():
    html = _sample("\r\n")
    new, changed = deploy_rc.patch_html(html)
    assert changed is True
    assert deploy_rc.is_patched(new)
    # 插在 </body> 之前
    assert new.index(deploy_rc.MARKER) < new.index("</body>")
    # 换行风格没被改掉：不该出现"裸 LF"
    assert "\n" not in new.replace("\r\n", "")
    # 除了插入的两行，其余内容一字不改
    assert deploy_rc.strip_patch(new)[0] == html


def test_patch_is_idempotent():
    once, changed1 = deploy_rc.patch_html(_sample())
    twice, changed2 = deploy_rc.patch_html(once)
    assert changed1 is True and changed2 is False
    assert twice == once, "已打补丁的页面不该被再次插入"


def test_patch_requires_body_or_html_tag():
    try:
        deploy_rc.patch_html("just a string")
    except ValueError:
        return
    raise AssertionError("没有 </body>/</html> 时应当抛 ValueError，避免把文件写坏")


def test_strip_patch_is_noop_when_clean():
    html = _sample()
    same, changed = deploy_rc.strip_patch(html)
    assert changed is False and same == html


def test_real_official_page_roundtrip():
    """拿真官方页做一次完整往返：插一行 → 只有一个标记 → 剥掉后逐字节还原。"""
    if not OFFICIAL_COPY.exists():
        print("[skip] 没有官方页副本，跳过真页往返测试")
        return
    original = OFFICIAL_COPY.read_bytes().decode("utf-8")
    patched, changed = deploy_rc.patch_html(original)
    assert changed is True
    # 部署后的自检口径：grep -c smartcar-rc-drive 期望 1
    assert patched.count(deploy_rc.MARKER) == 1
    assert patched.count(deploy_rc.SCRIPT_TAG) == 1
    # 只多了一行注释 + 一行 script（按行数核对）
    assert len(patched.splitlines()) == len(original.splitlines()) + 2
    # 还原后与原件逐字节一致（CRLF 也不能变）
    restored, _ = deploy_rc.strip_patch(patched)
    assert restored.encode("utf-8") == original.encode("utf-8")
    print("[OK] 官方页往返测试通过：插入 2 行、标记 1 处、还原逐字节一致")


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"[PASS] {fn.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"[FAIL] {fn.__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"[ERROR] {fn.__name__}: {type(exc).__name__}: {exc}")
    print(f"\n{len(fns) - failed}/{len(fns)} 通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
