#!/bin/bash
# 车端环境探测脚本(只读,安全) — 用法: bash /root/car_probe.sh
echo '===== ① 摄像头设备 ====='
ls -la /dev/video* 2>/dev/null || echo '无 /dev/video*'
v4l2-ctl --list-devices 2>/dev/null | head -15
echo
echo '===== ② I2C 总线与 PCA9685 扫描 ====='
i2cdetect -l 2>/dev/null
echo '--- i2cdetect -y 5 ---'
i2cdetect -y 5 2>/dev/null
echo
echo '===== ③ Python 环境 ====='
python3 -V
python3 -c 'import numpy; print("numpy", numpy.__version__)' 2>&1 | tail -1
python3 -c 'import cv2; print("opencv", cv2.__version__)' 2>&1 | tail -1
python3 -c 'import serial; print("pyserial", serial.__version__)' 2>&1 | tail -1
echo
echo '===== ④ 串口/USB 设备 ====='
ls /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
lsusb 2>/dev/null | head -10
echo
echo '===== ⑤ 系统资源 ====='
free -h | head -2
cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null | awk '{printf "CPU温度: %.1f°C\n", $1/1000}'
echo
echo '===== ⑥ RKNN 相关 ====='
ls /usr/local/lib/python3.8/dist-packages/ 2>/dev/null | grep -i rknn
find /root /home /usr/local -maxdepth 4 -iname '*rknn*' 2>/dev/null | head -10
echo '===== 探测完成 ====='