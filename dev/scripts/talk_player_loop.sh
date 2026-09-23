#!/bin/bash
# 语音下行循环：把网页"按住说话"的音频从自建媒体服务器拉下来，在本车喇叭播放
#
# 链路：浏览器 /talk_car0027/whip → 小车 nginx 反代 → 自建 mediamtx:8889
#        → 本脚本从 mediamtx:8554 拉 talk_car0027 → 喇叭(plughw:3,0)
#
# 说明：原版从官方服务器 82.157.204.126:17005 拉流，2026-09-15 全面切到自建服务器。
# 备份：/root/talk_player_loop.sh.bak-<日期>
# 2026-09-23 修复：服务器地址从 /etc/default/smartcar-media（car-media.env）读取，
#   换服务器只改一处（car-net.sh apply 会同步它），不再写死在脚本里（审查 D5）。

MEDIA_ENV="${MEDIA_ENV:-/etc/default/smartcar-media}"
[[ -f "$MEDIA_ENV" ]] || { echo "[talk] 找不到 $MEDIA_ENV，无法确定媒体服务器"; exit 1; }
MEDIA_URL=$(sed -n 's/^MEDIA_RTSP=//p' "$MEDIA_ENV" | head -1)
[[ -n "$MEDIA_URL" ]] || { echo "[talk] $MEDIA_ENV 里没有 MEDIA_RTSP"; exit 1; }

# MEDIA_RTSP=rtsp://car:pass@host:8554 → 拼上 talk 流名
TALK_URL="${MEDIA_URL%/}/talk_car0027"

while true; do
  /usr/bin/ffmpeg -rtsp_transport tcp -fflags nobuffer -flags low_delay \
    -i "$TALK_URL" \
    -af "aresample=async=1:min_hard_comp=0.100:first_pts=0" -f alsa plughw:3,0
  sleep 1
done
