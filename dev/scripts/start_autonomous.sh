#!/usr/bin/env bash
# 进入自动驾驶模式：关闭手动/推流/语音服务，启动 vision + control。
# 使用前必须确保小车处于安全状态（轮子架空或空旷场地）。
set -euo pipefail

echo "[AUTO] stopping manual/streaming services"
systemctl stop opi-control.service || true
systemctl stop ffmpeg-stream.service || true
systemctl stop ffmpeg-stream-sub.service || true
systemctl stop talk-player.service || true
systemctl stop frpc.service || true
systemctl stop nginx.service || true
sleep 1

MODEL="${MODEL:-/root/dev/models/best11nseg.rknn}"
PORT="${PORT:-5000}"
MAX_US="${MAX_US:-1540}"

echo "[AUTO] starting vision_main.py"
PYTHONPATH=/root/dev nohup python3 -u /root/dev/vision/vision_main.py \
  --model "$MODEL" --udp-ip 127.0.0.1 --udp-port "$PORT" \
  > /tmp/vision_main.log 2>&1 &
VISION_PID=$!

echo "[AUTO] starting main.py --real --arm --max-us $MAX_US"
PYTHONPATH=/root/dev python3 -u /root/dev/main.py --real --arm --port "$PORT" --max-us "$MAX_US"

# main.py 退出后清理 vision
echo "[AUTO] main exited, stopping vision"
kill "$VISION_PID" 2>/dev/null || true
wait "$VISION_PID" 2>/dev/null || true
