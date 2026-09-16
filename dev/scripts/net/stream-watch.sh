#!/usr/bin/env bash
# ============================================================================
# stream-watch.sh —— 图传推流看门狗
#
# 为什么需要它（2026-09-16 实测）：
#   ffmpeg 往媒体服务器 **推送** RTSP 时不会自动重连。网络一断，
#   推送连接就死了，但 ffmpeg 进程还活着、systemd 也认为服务正常——
#   结果"服务显示 active，服务器上却一条流都没有"，比赛时这是致命的。
#
# 本脚本定期检查每个推流服务**到媒体服务器的 TCP 连接是否还在**（按进程 PID 精确匹配），
# 不在就重启该服务；并带 60s 退避，避免网络长期不通时反复重启。
#
# 部署：stream-watch.service + stream-watch.timer（每 30s 跑一次）
# ============================================================================
set -uo pipefail

MEDIA_ENV="/etc/default/smartcar-media"
STATE_DIR="/run/stream-watch"
BACKOFF_S=60
SERVICES=(ffmpeg-stream.service ffmpeg-stream-sub.service)

mkdir -p "$STATE_DIR"

# 从 /etc/default/smartcar-media 解析出媒体服务器 host:port
[[ -f "$MEDIA_ENV" ]] || { echo "[watch] 找不到 $MEDIA_ENV，退出"; exit 0; }
MEDIA_URL=$(sed -n 's/^MEDIA_RTSP=//p' "$MEDIA_ENV" | head -1)
HOST=$(sed -n 's|.*@\([^:/]*\).*|\1|p' <<< "$MEDIA_URL")
PORT=$(sed -n 's|.*@[^:]*:\([0-9]*\).*|\1|p' <<< "$MEDIA_URL")
[[ -n "$HOST" && -n "$PORT" ]] || { echo "[watch] 解析媒体服务器地址失败：$MEDIA_URL"; exit 0; }

restart_service() {
  local svc="$1" reason="$2" stamp="$STATE_DIR/${svc%.service}.last"
  local now last
  now=$(date +%s)
  last=$(cat "$stamp" 2>/dev/null || echo 0)
  if (( now - last < BACKOFF_S )); then
    echo "[watch] $svc 需要重启（$reason），但距上次仅 $((now-last))s，退避中"
    exit 0
  fi
  echo "[watch] 重启 $svc：$reason"
  echo "$now" > "$stamp"
  systemctl restart "$svc" 2>/dev/null || echo "[watch] $svc 重启失败"
}

for svc in "${SERVICES[@]}"; do
  systemctl is-enabled --quiet "$svc" 2>/dev/null || continue     # 没启用的别管

  # 只救"崩了"的服务（state=failed）。
  # **绝不能救"被人主动停掉"的服务**：自动驾驶要停掉占用下摄的那路推流，
  # 拍照/台架脚本也会临时停推流 —— 看门狗若把它们拉起来，就会去抢摄像头，
  # 把视觉进程/采集脚本的摄像头抢走（2026-09-16 实测踩到）。
  if systemctl is-failed --quiet "$svc" 2>/dev/null; then
    restart_service "$svc" "服务处于 failed（崩溃）"
    continue
  fi
  systemctl is-active --quiet "$svc" || continue                  # inactive = 有人主动停的，不管

  pid=$(systemctl show -p MainPID --value "$svc" 2>/dev/null)
  if [[ -z "$pid" || "$pid" == "0" ]]; then
    restart_service "$svc" "拿不到主进程 PID"
    continue
  fi
  # 该进程到媒体服务器端口还有 ESTABLISHED 连接吗？（进程活着但流已死的情况）
  if ss -tnp state established "( dport = :$PORT )" 2>/dev/null | grep -q "pid=$pid"; then
    :
  else
    restart_service "$svc" "到 $HOST:$PORT 的推送连接已断（进程还活着但流已死）"
  fi
done

exit 0
