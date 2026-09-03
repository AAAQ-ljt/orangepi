#!/usr/bin/env python3
# SIM 深度诊断 v2 — CFUN 复位后等待 USB 重枚举再查询
import serial
import time
import os

PORT = '/dev/ttyUSB2'

def at(port, cmd, wait=1.5):
    try:
        s = serial.Serial(port, 115200, timeout=3)
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write((cmd + '\r').encode())
        time.sleep(wait)
        out = s.read(4096).decode(errors='replace').strip()
        s.close()
        return out
    except Exception as e:
        return f'<ERR {e}>'

def wait_port(port, seconds=40):
    for i in range(int(seconds / 2)):
        if os.path.exists(port):
            return True
        time.sleep(2)
    return False

print('=== ① 复位前状态确认 ===')
print('AT+CPIN? =>', at(PORT, 'AT+CPIN?'))

print('=== ② 模块软复位(USB 会重枚举,约等 20-40s) ===')
print('AT+CFUN=1,1 =>', at(PORT, 'AT+CFUN=1,1', 3.0))
print('等待串口重新出现...')
if not wait_port(PORT):
    print('❌ 40 秒内 /dev/ttyUSB2 未恢复,检查模块供电/枚举')
    exit(1)
print('串口已恢复 ✅')
time.sleep(5)  # 等模块内部初始化

print('=== ③ 复位后 SIM 检测 ===')
print('AT+CPIN? =>', at(PORT, 'AT+CPIN?', 2.0))
pin = at(PORT, 'AT+CPIN?', 2.0)
print('AT+CSQ =>', at(PORT, 'AT+CSQ'))
print('AT+COPS? =>', at(PORT, 'AT+COPS?'))

if 'READY' in pin.upper():
    print('=== ④ SIM 已识别! 网络注册确认 ===')
    print('AT+CEREG? =>', at(PORT, 'AT+CEREG?', 2.0))
    print('AT+CGDCONT? =>', at(PORT, 'AT+CGDCONT?', 2.0))
else:
    print('=== ④ SIM 仍未识别,探测槽位切换命令 ===')
    for c in ['AT+SSIM?', 'AT+SSIM=?', 'AT+CUSAT?', 'AT+SIMDET?', 'AT+CSMS?']:
        print(f'{c} =>', at(PORT, c, 1.2))
    print('--- 尝试切换 SIM2 槽后重查 ---')
    for c in ['AT+SSIM=1', 'AT+CUSAT=1']:
        r = at(PORT, c, 1.2)
        print(f'{c} => {r}')
        if 'OK' in r:
            time.sleep(4)
            print('切换后 AT+CPIN? =>', at(PORT, 'AT+CPIN?', 2.0))
            break