#!/bin/bash
# 积压排查脚本 — WiFi预配置 / SIM8202 / nginx / 9001 / 推流进程 (只读)
echo '===== ① WiFi 预配置(为何只连特定热点) ====='
nmcli -t connection show 2>/dev/null
echo '--- NetworkManager 连接文件 ---'
ls /etc/NetworkManager/system-connections/ 2>/dev/null
grep -hE 'ssid|psk' /etc/NetworkManager/system-connections/* 2>/dev/null | sed 's/psk=.*/psk=<hidden>/'
echo '--- 当前连接 ---'
nmcli -t -f NAME,DEVICE,TYPE connection show --active 2>/dev/null
echo
echo '===== ② SIM8202 模块枚举 ====='
lsusb
echo '--- modem 串口设备 ---'
ls /dev/ttyUSB* /dev/ttyACM* /dev/cdc-wdm* 2>/dev/null
echo '--- 拨号相关进程 ---'
ps aux | grep -iE 'ModemManager|pppd|qmi|mbim|wwan' | grep -v grep | head -5
echo
echo '===== ③ nginx 错误日志(502根因) ====='
tail -15 /var/log/nginx/error.log 2>/dev/null || echo '无 nginx error.log'
echo
echo '===== ④ 9001 uvicorn 服务 ====='
ps aux | grep uvicorn | grep -v grep
echo '--- 9001 的 systemd/supervisor 单元 ---'
systemctl list-units --state=running 2>/dev/null | grep -iE 'uvicorn|9001|orangepi|rc' | head -5
echo '--- 9001 最近输出(找systemd日志) ---'
journalctl --since '1 hour ago' 2>/dev/null | grep -iE 'uvicorn|9001|exception|error' | tail -8
echo
echo '===== ⑤ 推流进程 ====='
ps aux | grep -iE 'ffmpeg|gst|whipp|whep|webrtc|mjpg|rtsp' | grep -v grep | head -8
echo '===== 排查完成 ====='