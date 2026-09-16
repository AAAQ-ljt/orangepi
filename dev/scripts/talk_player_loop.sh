#!/bin/bash
# 语音下行循环：把网页"按住说话"的音频从自建媒体服务器拉下来，在本车喇叭播放
#
# 链路：浏览器 /talk_car0027/whip → 小车 nginx 反代 → 自建 mediamtx:8889
#        → 本脚本从 mediamtx:8554 拉 talk_car0027 → 喇叭(plughw:3,0)
#
# 说明：原版从官方服务器 82.157.204.126:17005 拉流，2026-09-15 全面切到自建服务器。
# 备份：/root/talk_player_loop.sh.bak-<日期>

while true; do
  /usr/bin/ffmpeg -rtsp_transport tcp -fflags nobuffer -flags low_delay \
    -i "rtsp://car:Ct7vL92xQm4@121.40.149.155:8554/talk_car0027" \
    -af "aresample=async=1:min_hard_comp=0.100:first_pts=0" -f alsa plughw:3,0
  sleep 1
done
