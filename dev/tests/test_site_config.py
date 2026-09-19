"""现场标定覆盖层（config/site.yaml）单元测试。

要验证的是"标定一次、以后一条命令跑车"这条链路：
    --calibrate 写 site.yaml → 任何模块 import config.settings 时自动生效 → 不需要 --target-x

跑法：
    cd dev && PYTHONPATH=. python tests/test_site_config.py
"""
from __future__ import annotations

import os
import tempfile

from config import settings, site


def test_missing_file_is_noop():
    """没有 site.yaml 时不能报错（实验室/CI 上就是这种情况）。"""
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "nope.yaml")
        assert site.load(path) == {}
        assert site.apply(path) == {}
    # settings 保持默认值
    assert isinstance(settings.TARGET_X, float)


def test_save_then_apply_overrides_settings():
    from config import settings as st
    original = st.TARGET_X
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "site.yaml")
        site.save({"target_x": 377.5}, path)
        applied = site.apply(path)
        try:
            assert applied.get("target_x") == 377.5
            assert st.TARGET_X == 377.5, "site.yaml 应覆盖 settings.TARGET_X"
        finally:
            st.TARGET_X = original          # 还原，别污染同一进程里的其它测试


def test_unknown_key_is_ignored_with_warning():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "site.yaml")
        site.save({"no_such_setting": 123}, path)
        applied = site.apply(path)          # 未知键只警告，不写入
        assert "no_such_setting" not in applied
        assert applied == {}


def test_save_merges_and_preserves_existing():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "site.yaml")
        site.save({"target_x": 350.0}, path)
        site.save({"lane_kp": 0.5}, path)   # 第二次写不能把第一次的丢掉
        data = site.load(path)
        assert data.get("target_x") == 350.0 and data.get("lane_kp") == 0.5


def test_apply_is_idempotent():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "site.yaml")
        site.save({"target_x": 360.0}, path)
        site.apply(path)
        site.apply(path)
        assert settings.TARGET_X == 360.0


if __name__ == "__main__":
    test_missing_file_is_noop()
    test_save_then_apply_overrides_settings()
    test_unknown_key_is_ignored_with_warning()
    test_save_merges_and_preserves_existing()
    test_apply_is_idempotent()
    print("test_site_config: all passed")
