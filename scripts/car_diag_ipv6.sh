#!/bin/bash
# ① 翻旧账: ip-type=4 时 get-current-settings 的真实输出
echo '=== A. 之前 ip-type=4 的 settings 输出(从car-dial日志) ==='
journalctl -u car-dial --no-pager 2>/dev/null | grep -B1 -A8 'qmicli 网络配置' | tail -20
echo
# ② 纯 IPv6 拨号诊断
echo '=== B. ip-type=6 拨号 ==='
qmicli -d /dev/cdc-wdm0 -p --wds-start-network="apn=cmnet,ip-type=6" --client-no-release-cid
sleep 6
echo '=== C. IPv6 settings ==='
qmicli -d /dev/cdc-wdm0 -p --wds-get-current-settings
echo '=== D. 结束 ==='
qmicli -d /dev/cdc-wdm0 -p --wds-stop-network=18446744073709551615 2>/dev/null || true
echo 'IPV6-DIAG-END'