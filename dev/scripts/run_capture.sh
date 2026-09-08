#!/usr/bin/env bash
# 拍照任务包装：确保安全状态 -> 停 ffmpeg -> 运行拍照 -> 恢复 ffmpeg
set -euo pipefail

# 1. 确保手动安全模式（opi-control 保持 PCA9685 安全）
bash "$(dirname "$0")/safe_pwm_init.sh"

# 2. 停止推流，释放摄像头
echo "[CAPTURE] stopping ffmpeg"
systemctl stop ffmpeg-stream.service || true
systemctl stop ffmpeg-stream-sub.service || true

cleanup() {
  echo "[CAPTURE] restoring ffmpeg"
  systemctl start ffmpeg-stream.service || true
  systemctl start ffmpeg-stream-sub.service || true
}
trap cleanup EXIT

# 3. 运行拍照脚本
echo "[CAPTURE] running capture_dataset.py"
python3 /root/dev/scripts/capture_dataset.py "$@"
