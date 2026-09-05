#!/usr/bin/env bash
# 小车控制端启动脚本
# 默认 dry-run；需要真实硬件时加 --real
set -euo pipefail
cd /root/dev
export PYTHONPATH=/root/dev
exec python3 main.py "$@"
