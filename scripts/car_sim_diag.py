#!/usr/bin/env python3
# SIM 卡深度诊断 — 模块复位、SIM 槽选择命令探测
import serial
import time
import subprocess

PORT = '/dev/ttyUSB2'

def at(port, cmd, wait=1.2):
    try:
        s = serial.Serial(port, 115200, timeout=2)
        time.sleep(0.2)
        s.reset_input_buffer()
        s.write((cmd + '\r').encode())
        time.sleep(wait)
        out = s.read(4096).decode(errors='replace').strip()
        s.close()
        return out
    except Exception as e:
        return f'<ERR {e}>'

print('=== 内核日志 SIM/卡相关 ===')
r = subprocess.run(['bash', '-c', "dmesg 2>/dev/null | grep -iE 'sim|card detect|ttyUSB' | tail -25"], capture_output=True, text=True, timeout=20)
print(r.stdout or '(dmesg 无权限或无匹配)')

print('=== ① 模块软复位(重新初始化,重读SIM) ===')
print('AT+CFUN=1,1 =>', at(PORT, 'AT+CFUN=1,1', 1.0))
time.sleep(6)
print('复位后 AT+CPIN? =>', at(PORT, 'AT+CPIN?', 1.5))

print('=== ② SIM 槽选择命令探测 ===')
for c in ['AT+SSIM?', 'AT+SSIM=?', 'AT+CUSAT?', 'AT+SIMDET?', 'AT+CSIM?']:
    print(f'{c} =>', at(PORT, c, 1.0))

print('=== ③ 复位后状态 ===')
print('AT+CSQ =>', at(PORT, 'AT+CSQ'))
print('AT+COPS? =>', at(PORT, 'AT+COPS?'))

print('=== ④ 尝试切 SIM2 槽再看 ===')
print('AT+SSIM=1 =>', at(PORT, 'AT+SSIM=1', 1.0))
time.sleep(3)
print('AT+CPIN? =>', at(PORT, 'AT+CPIN?', 1.5))