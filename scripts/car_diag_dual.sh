#!/bin/bash
# 双栈拨号诊断 — 看实际下发的 IP 配置 (IPv4/IPv6)
systemctl stop car-dial 2>/dev/null
sleep 2
echo "=== ① start-network ip-type=4,6 ==="
qmicli -d /dev/cdc-wdm0 -p --wds-start-network="apn=cmnet,ip-type=4,6" --client-no-release-cid
echo "=== ② current-settings ==="
sleep 6
qmicli -d /dev/cdc-wdm0 -p --wds-get-current-settings
echo "=== ③ 结束网络 ==="
qmicli -d /dev/cdc-wdm0 -p --wds-stop-network=18446744073709551615 2>/dev/null || true
echo "DIAG-END"