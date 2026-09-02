#!/bin/bash
# 下载 frp 0.39.0 (linux_amd64) 到 /tmp/frp.tar.gz —— 依次尝试 GitHub 加速镜像,最后直连
set -u
URLS=(
  "https://ghfast.top/https://github.com/fatedier/frp/releases/download/v0.39.0/frp_0.39.0_linux_amd64.tar.gz"
  "https://gh-proxy.com/https://github.com/fatedier/frp/releases/download/v0.39.0/frp_0.39.0_linux_amd64.tar.gz"
  "https://mirror.ghproxy.com/https://github.com/fatedier/frp/releases/download/v0.39.0/frp_0.39.0_linux_amd64.tar.gz"
  "https://github.moeyy.xyz/https://github.com/fatedier/frp/releases/download/v0.39.0/frp_0.39.0_linux_amd64.tar.gz"
  "https://github.com/fatedier/frp/releases/download/v0.39.0/frp_0.39.0_linux_amd64.tar.gz"
)
for u in "${URLS[@]}"; do
  echo "== try: $u"
  if wget -q --timeout=60 -O /tmp/frp.tar.gz "$u"; then
    sz=$(stat -c%s /tmp/frp.tar.gz)
    echo "DOWNLOAD-OK size=$sz"
    exit 0
  fi
done
echo "ALL-FAILED"
exit 1