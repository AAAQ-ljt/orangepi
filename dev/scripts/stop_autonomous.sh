#!/usr/bin/env bash
# 兼容旧用法：等价于 `car-mode.sh manual`
#
# 推荐直接用统一入口：bash /root/dev/scripts/car-mode.sh manual
# 本文件只作为旧文档 / 肌肉记忆的入口保留。
set -euo pipefail
exec "$(dirname "$(readlink -f "$0")")/car-mode.sh" manual "$@"
