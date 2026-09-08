#!/usr/bin/env bash
# 摄像头0（/dev/video0）拍照/录像包装脚本。
# 用法:
#   bash img/camera0.sh photo [args...]
#   bash img/camera0.sh video [args...]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
DEVICE=0
SERVICE="ffmpeg-stream.service"

# 确保上电默认安全模式
bash "$ROOT_DIR/scripts/safe_pwm_init.sh"

# 关闭对应推流服务，释放摄像头
echo "[CAM0] stopping $SERVICE"
systemctl stop "$SERVICE" || true

cleanup() {
  echo "[CAM0] restoring $SERVICE"
  systemctl start "$SERVICE" || true
}
trap cleanup EXIT

MODE="${1:-photo}"
shift || true

if [ "$MODE" = "video" ]; then
  echo "[CAM0] record_video.py --device $DEVICE"
  python3 "$ROOT_DIR/scripts/record_video.py" --device "$DEVICE" "$@"
else
  echo "[CAM0] capture_dataset.py --device $DEVICE"
  python3 "$ROOT_DIR/scripts/capture_dataset.py" --device "$DEVICE" "$@"
fi
