#!/usr/bin/env bash
# 录像任务包装：确保安全状态 -> 停 ffmpeg -> 运行录像 -> 恢复 ffmpeg
set -euo pipefail

bash "$(dirname "$0")/safe_pwm_init.sh"

echo "[VIDEO] stopping ffmpeg"
systemctl stop ffmpeg-stream.service || true
systemctl stop ffmpeg-stream-sub.service || true

cleanup() {
  echo "[VIDEO] restoring ffmpeg"
  systemctl start ffmpeg-stream.service || true
  systemctl start ffmpeg-stream-sub.service || true
}
trap cleanup EXIT

echo "[VIDEO] running record_video.py"
python3 /root/dev/scripts/record_video.py "$@"
