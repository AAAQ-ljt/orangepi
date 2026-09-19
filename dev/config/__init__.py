"""config 包：导入时自动应用现场标定值（config/site.yaml → 覆盖 settings 同名常量）。

这样"标定一次、以后一条命令跑车"才能成立：
    from config import settings     # 这一行之前，site.yaml 的覆盖已经生效
"""
from __future__ import annotations

from config import site as _site

# 自动应用现场标定（文件不存在就什么都不做）
try:
    _site.apply()
except Exception as _exc:      # 覆盖层出问题绝不能拖垮主程序
    print(f"[config] 应用 site.yaml 失败（忽略）：{_exc}")
