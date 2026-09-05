#!/usr/bin/env bash
# 电脑视觉端启动脚本：运行上一届 YOLO 视觉并发送 UDP 到小车
set -euo pipefail

# 可在环境变量中覆盖
VISION_DIR="${VISION_DIR:-yolo版本视觉识别/src}"
CAR_IP="${CAR_IP:-10.23.159.43}"
PORT="${PORT:-5000}"
MODEL="${MODEL:-model/best11sseg.pt}"
CAMERA="${CAMERA:-0}"

cd "$VISION_DIR"
exec python main.py \
  --camera "$CAMERA" \
  --model "$MODEL" \
  --udp-ip "$CAR_IP" \
  --udp-port "$PORT" \
  "$@"
