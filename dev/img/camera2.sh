#!/usr/bin/env bash
# 摄像头2（/dev/video2）拍照/录像包装脚本。
# 保存目录强制限制在 /root/dev/img 内。
# 用法:
#   bash img/camera2.sh photo --folder test1
#   bash img/camera2.sh photo --output /root/dev/img/test1
#   bash img/camera2.sh video --folder test1
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
IMG_ROOT="$SCRIPT_DIR"
DEVICE=2
SERVICE="ffmpeg-stream-sub.service"

resolve_output() {
  local output=""
  local folder=""
  local new_args=()
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --output)
        output="$2"; shift 2 ;;
      --folder|-f)
        folder="$2"; shift 2 ;;
      *)
        new_args+=("$1"); shift ;;
    esac
  done

  local final=""
  if [ -n "$output" ]; then
    case "$output" in
      "$IMG_ROOT"/*) final="$output" ;;
      *)
        echo "[CAM2] 错误: 输出路径必须在 $IMG_ROOT 内" >&2
        exit 1 ;;
    esac
  elif [ -n "$folder" ]; then
    case "$folder" in
      */*|*\\*|..*|.*)
        echo "[CAM2] 错误: 文件夹名不合法: $folder" >&2
        exit 1 ;;
    esac
    final="$IMG_ROOT/$folder"
  else
    final="$IMG_ROOT/capture"
  fi

  mkdir -p "$final"
  RESOLVED_ARGS=("${new_args[@]}" "--output" "$final")
}

bash "$ROOT_DIR/scripts/safe_pwm_init.sh"

echo "[CAM2] stopping $SERVICE"
systemctl stop "$SERVICE" || true

cleanup() {
  echo "[CAM2] restoring $SERVICE"
  systemctl start "$SERVICE" || true
}
trap cleanup EXIT

MODE="${1:-photo}"
shift || true

resolve_output "$@"

if [ "$MODE" = "video" ]; then
  echo "[CAM2] record_video.py --device $DEVICE ${RESOLVED_ARGS[*]}"
  python3 "$ROOT_DIR/scripts/record_video.py" --device "$DEVICE" "${RESOLVED_ARGS[@]}"
else
  echo "[CAM2] capture_dataset.py --device $DEVICE ${RESOLVED_ARGS[*]}"
  python3 "$ROOT_DIR/scripts/capture_dataset.py" --device "$DEVICE" "${RESOLVED_ARGS[@]}"
fi
