#!/usr/bin/env bash
# 兼容旧用法：等价于 `car-mode.sh auto "$@"`
#
# 推荐直接用统一入口：
#   bash /root/dev/scripts/car-mode.sh auto            # 默认 DRY-RUN（电机不动，仅验证视觉/状态机）
#   bash /root/dev/scripts/car-mode.sh auto --real     # 真正自主跑车（会要你输入 GO 确认）
#   bash /root/dev/scripts/car-mode.sh status          # 查看当前模式
#
# 本文件只作为旧文档 / 肌肉记忆的入口保留。
set -euo pipefail
exec "$(dirname "$(readlink -f "$0")")/car-mode.sh" auto "$@"
