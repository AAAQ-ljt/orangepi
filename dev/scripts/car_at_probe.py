#!/usr/bin/env python3
# 5G 模块 AT 探测 — 找到 AT 口并查询 SIM 卡/信号/注册/APN 状态
import serial
import time

PORTS = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3', '/dev/ttyUSB4']
CMDS = ['ATI', 'AT+CPIN?', 'AT+CSQ', 'AT+COPS?', 'AT+CEREG?',
        'AT+CGDCONT?', 'AT+CGMM', 'AT+CUSBD?', 'AT+CGMR']

def probe(port):
    try:
        s = serial.Serial(port, 115200, timeout=2)
    except Exception as e:
        print(f'[{port}] 打开失败: {e}')
        return None
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write(b'ATE0\r')
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(b'AT\r')
    time.sleep(0.8)
    out = s.read(2048).decode(errors='replace')
    if 'OK' in out.upper():
        print(f'[{port}] [OK] AT 响应正常')
        return s
    print(f'[{port}] 无 AT 响应: {out!r}')
    s.close()
    return None

def full_query(s, port):
    for c in CMDS:
        try:
            s.write((c + '\r').encode())
            time.sleep(1.0)
            out = s.read(4096).decode(errors='replace').strip()
            print(f'[{port}] {c} => {out}')
        except Exception as e:
            print(f'[{port}] {c} 错误: {e}')

if __name__ == '__main__':
    port_ok = None
    for p in PORTS:
        s = probe(p)
        if s:
            port_ok = (p, s)
            break
    if port_ok:
        full_query(port_ok[1], port_ok[0])
        port_ok[1].close()
    else:
        print('未找到 AT 口，尝试 /dev/cdc-wdm0 需要 QMI 工具另行处理')