#!/usr/bin/env bash
# 确保 PCA9685 被安全初始化：启动 opi-control，舵机回中、电调 1500us。
set -euo pipefail

echo "[SAFE] starting opi-control.service to initialize PCA9685"
systemctl start opi-control.service || true
sleep 2

if systemctl is-active --quiet opi-control.service; then
  echo "[SAFE] opi-control is active, PCA9685 should be safe (steer center, ESC 1500us)"
else
  echo "[SAFE] WARNING: opi-control is NOT active, PCA9685 may not be initialized!"
  exit 1
fi
