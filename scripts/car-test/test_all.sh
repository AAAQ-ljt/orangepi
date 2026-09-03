#!/bin/bash
# ============================================================
# 小车部件测试 — Phase 0 只读自检 / Phase 1 摄像头抓帧
# 运行: bash /root/car-test/test_all.sh
# 输出: [PASS]/[FAIL]/[WARN] 每项一行, 结尾汇总
# 注意: 只读为主; 摄像头抓帧会自动停/恢复推流服务
# ============================================================
PASS=0; FAIL=0; WARN=0
ok()   { echo "[PASS] $1"; PASS=$((PASS+1)); }
bad()  { echo "[FAIL] $1"; FAIL=$((FAIL+1)); }
warn() { echo "[WARN] $1"; WARN=$((WARN+1)); }
tcp_ok() { timeout 3 bash -c "echo > /dev/tcp/$1/$2" 2>/dev/null && return 0; return 1; }

echo '########## T1 系统 ##########'
H=$(hostname)
[ "$H" = "orangepi5" ] && ok "主机名或别名: $H" || warn "主机名=$H"
echo "$(uname -r)" | grep -q rockchip && ok "内核: $(uname -r)" || warn "内核: $(uname -r)"
T=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
echo "     温度: $(( ${T:-0} / 1000 ))°C"
M=$(free -m | awk '/Mem/{print $2}')
[ "$M" -gt 3000 ] && ok "内存 ${M}MB" || warn "内存仅 ${M}MB"
echo "     磁盘剩余: $(df -h / | awk 'NR==2{print $4}')"
echo "     负载: $(uptime | grep -o 'load average.*')"

echo '########## T2 网络 ##########'
ip -br link | grep -q 'wlx' && ok "WiFi 网卡存在" || warn "无 WiFi 网卡"
ip link show wwan0 >/dev/null 2>&1 && ok "蜂窝网卡 wwan0 存在" || warn "无 wwan0"
ip link show eth0 >/dev/null 2>&1 && ok "有线网卡 eth0 存在" || warn "无 eth0"
ip route | grep -q '^default' && ok "默认路由: $(ip route | grep '^default' | head -1 | awk '{print $3, $5}')" || warn "无默认路由"
OUT=$(curl -s --max-time 8 ifconfig.me 2>/dev/null)
[ -n "$OUT" ] && ok "公网出口: $OUT" || warn "公网出口查询失败"

echo '########## T3 frp 隧道 ##########'
systemctl is-active frpc >/dev/null 2>&1 && ok "官方 frpc 服务" || bad "官方 frpc 未运行"
systemctl is-active frpc-ssh >/dev/null 2>&1 && ok "自建 frpc-ssh 服务" || bad "frpc-ssh 未运行"
tcp_ok 121.40.149.155 2222 && ok "服务器 2222 (SSH 隧道)" || warn "2222 不可达"
tcp_ok 121.40.149.155 8080 && ok "服务器 8080 (网页隧道)" || warn "8080 不可达"

echo '########## T4 关键服务 ##########'
for svc in nginx ffmpeg-stream ffmpeg-stream-sub; do
  systemctl is-active "$svc" >/dev/null 2>&1 && ok "$svc 运行中" || bad "$svc 未运行"
done
systemctl is-active car-dial >/dev/null 2>&1 && ok "car-dial (蜂窝拨号)" || warn "car-dial 未运行"
ss -lnt | grep -q ':9001' && ok "9001 uvicorn 监听中" || bad "9001 未监听 → 摇杆控制失灵"
python3 -c 'import adafruit_pca9685' 2>/dev/null && ok "adafruit_pca9685 库存在" || bad "缺 adafruit_pca9685 (9001 崩溃根因)"

echo '########## T5 摄像头设备 ##########'
for v in /dev/video0 /dev/video2; do
  [ -e "$v" ] && ok "$v 存在" || bad "$v 缺失"
done
v4l2-ctl --list-devices 2>/dev/null | grep -qi 'camera\|icspring' && ok "v4l2 枚举识别" || warn "v4l2 枚举异常"

echo '########## T6 I2C / PCA9685 ##########'
i2cdetect -y 5 2>/dev/null | grep -q '40' && ok "PCA9685 @0x40 (I2C5)" || bad "PCA9685 未探测到 (I2C5)"

echo '########## T7 传感器与串口节点 ##########'
ls /dev/ttyS* >/dev/null 2>&1 && ok "串口组存在: $(ls /dev/ttyS* | tr '\n' ' ')" || warn "无 ttyS*"
ls /dev/ttyUSB* 2>/dev/null | grep -q 'ttyUSB' && ok "5G 模块 ttyUSB 组: $(ls /dev/ttyUSB* | tr '\n' ' ')" || warn "无 ttyUSB*"
[ -e /dev/cdc-wdm0 ] && ok "QMI 口 cdc-wdm0 存在" || warn "无 cdc-wdm0"

echo '########## T8 5G 蜂窝 ##########'
ip -br addr show wwan0 2>/dev/null | grep -q '10\.' && ok "wwan0 已获 IP: $(ip -br addr show wwan0 | awk '{print $3}')" || warn "wwan0 无 IPv4"

echo '########## T9 推流进程 ##########'
N=$(pgrep -fc 'ffmpeg.*rtsp' 2>/dev/null)
[ -n "$N" ] && [ "$N" -ge 1 ] && ok "ffmpeg 推流进程 x$N" || bad "推流进程未运行"
pgrep -f 'cam_car0003' >/dev/null && ok "推流目标 cam_car0003" || warn "推流目标缺失/非 car0003"

echo '########## T10 摄像头抓帧 (自动停/恢复推流) ##########'
systemctl stop ffmpeg-stream.service 2>/dev/null
systemctl stop ffmpeg-stream-sub.service 2>/dev/null
sleep 1
FRAMES=0
for v in 0 2; do
  if ffmpeg -y -loglevel error -f v4l2 -input_format mjpeg -video_size 640x480 -i "/dev/video$v" -frames:v 3 "/tmp/cam${v}_test.jpg" 2>/dev/null && [ -s "/tmp/cam${v}_test.jpg" ]; then
    ok "video$v 抓帧成功 ($(stat -c%s /tmp/cam${v}_test.jpg)B)"
    FRAMES=$((FRAMES+1))
  else
    bad "video$v 抓帧失败"
  fi
done
systemctl restart ffmpeg-stream.service 2>/dev/null
systemctl restart ffmpeg-stream-sub.service 2>/dev/null
[ "$FRAMES" -ge 1 ] && ok "摄像头可用路数: $FRAMES/2" || bad "两路摄像头均不可用"

echo '########## 汇总 ##########'
echo "RESULT: PASS=$PASS FAIL=$FAIL WARN=$WARN"