#!/usr/bin/env bash
# 硬件测试包装：会动电机/舵机，先停 opi-control 避免两个程序抢 I2C，结束后恢复。
# 用法: bash run_hardware_test.sh <命令...>
set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "用法: $0 <要执行的测试命令...>"
  exit 1
fi

echo "[HW] stopping opi-control to release PCA9685"
systemctl stop opi-control.service || true
sleep 1

cleanup() {
  echo "[HW] restoring opi-control"
  systemctl start opi-control.service || true
  sleep 1
}
trap cleanup EXIT

echo "[HW] running: $*"
"$@"
