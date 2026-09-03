#!/usr/bin/env python3
# SIM 快速状态查询 — 只读,不复位
import serial
import time

PORT = '/dev/ttyUSB2'


def at(cmd, wait=2.0):
    try:
        s = serial.Serial(PORT, 115200, timeout=3)
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write((cmd + '\r').encode())
        time.sleep(wait)
        out = s.read(4096).decode(errors='replace').strip()
        s.close()
        return out.replace('\r', '')
    except Exception as e:
        return f'<ERR {e}>'


print('AT+CPIN? =>', at('AT+CPIN?'))
print('AT+CSQ   =>', at('AT+CSQ'))
print('AT+COPS? =>', at('AT+COPS?'))
print('AT+CEREG?=>', at('AT+CEREG?'))