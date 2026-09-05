#!/bin/bash
# 一次性拨号验证 — SIM8262E-M2 (APN=cmnet)
echo 'Y' > /sys/class/net/wwan0/qmi/raw_ip 2>/dev/null || true
ip link set wwan0 up || true
sleep 2
echo '=== ① QMI 拨号 ==='
qmicli -d /dev/cdc-wdm0 -p --wds-start-network="apn=cmnet,ip-type=ipv4" --client-no-release-cid
sleep 3
ip addr flush dev wwan0
echo '=== ② DHCP ==='
dhclient -v -pf /run/dhclient-wwan0.pid wwan0 > /tmp/dh.log 2>&1 &
DH_PID=$!
sleep 12
echo '=== ③ wwan0 地址 ==='
ip addr show wwan0 | grep 'inet ' || echo '无 IP'
echo '=== ④ 蜂窝出口 ping ==='
ping -I wwan0 -c 2 -W 5 223.5.5.5 2>&1 | tail -2
echo '=== ⑤ 蜂窝出口 HTTP ==='
curl --interface wwan0 -s --max-time 8 -o /dev/null -w 'CELL-HTTP %{http_code} (%{time_total}s)\n' http://www.baidu.com || echo 'HTTP 失败'
echo 'ONCE-DIAL-END'