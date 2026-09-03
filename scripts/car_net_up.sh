#!/bin/bash
# 5G QMI 拨号常驻 v4 — SIM8262E-M2 (APN=cmnet) — 按 libqmi 1.24 实际输出格式解析
echo 'Y' > /sys/class/net/wwan0/qmi/raw_ip 2>/dev/null || true
ip link set wwan0 up || true
sleep 2
qmicli -d /dev/cdc-wdm0 -p --wds-start-network="apn=cmnet,ip-type=4" --client-no-release-cid || exit 1
sleep 4
SETTINGS=$(qmicli -d /dev/cdc-wdm0 -p --wds-get-current-settings 2>&1)
echo "$SETTINGS"
IP=$(echo "$SETTINGS" | grep -oP 'IPv4 address:\s*\K[0-9.]+' | head -1)
GW=$(echo "$SETTINGS" | grep -oP 'IPv4 gateway address:\s*\K[0-9.]+' | head -1)
DNS=$(echo "$SETTINGS" | grep -oP 'IPv4 primary DNS:\s*\K[0-9.]+' | head -1)
if [ -z "$IP" ] || [ -z "$GW" ]; then
  echo "解析失败: IP=$IP GW=$GW,退出"
  exit 1
fi
ip addr flush dev wwan0
ip addr add "$IP/32" dev wwan0 || exit 1
# 蜂窝兜底路由:metric 1000 → WiFi(600)优先,WiFi 断自动切蜂窝; /32 需要 onlink
ip route replace default via "$GW" dev wwan0 metric 1000 onlink || true
if [ -n "$DNS" ]; then
  resolvectl dns wwan0 "$DNS" 2>/dev/null || echo "nameserver $DNS" >> /etc/resolv.conf
fi
echo "OK IP=$IP GW=$GW DNS=$DNS"
echo '=== 蜂窝连通性 ==='
ping -I wwan0 -c 2 -W 5 223.5.5.5 2>&1 | tail -2
curl --interface wwan0 -s --max-time 8 -o /dev/null -w 'CELL-HTTP %{http_code}\n' http://www.baidu.com || echo 'HTTP 未通'
exec sleep infinity