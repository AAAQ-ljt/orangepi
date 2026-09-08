#!/usr/bin/env bash
# 退出自动驾驶，恢复手动模式。
set -euo pipefail

echo "[AUTO] stopping vision/control"
pkill -f "vision/vision_main.py" || true
pkill -INT -f "main.py --real" 2>/dev/null || pkill -f "main.py --real" || true
sleep 2
pkill -9 -f "vision/vision_main.py" || true
pkill -9 -f "main.py --real" || true

echo "[AUTO] restoring manual services"
systemctl start opi-control.service || true
systemctl start ffmpeg-stream.service || true
systemctl start ffmpeg-stream-sub.service || true
systemctl start talk-player.service || true
systemctl start frpc.service || true
systemctl start nginx.service || true
sleep 1
echo "[AUTO] manual mode restored"
