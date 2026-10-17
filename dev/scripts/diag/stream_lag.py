#!/usr/bin/env python3
"""车端源端推流积压测量：把 ffmpeg 日志里的 time= 与进程已运行时长对比。

积压 = 进程已运行时长 − 已编码时长。>1s 说明输入队列在攒延迟（历史元凶）。
用法（车端）：python3 stream_lag.py
"""
import os
import re
import subprocess
import time


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout.strip()


def probe(log_path, dev_tag):
    if not os.path.exists(log_path):
        print(f"[{dev_tag}] 日志不存在: {log_path}")
        return
    last = sh(
        "tail -c 400000 %s | tr '\\r' '\\n' | grep -oE 'time=[0-9:.]+' | tail -1" % log_path
    )
    mtime = os.stat(log_path).st_mtime
    now = time.time()
    etimes = sh("ps -eo etimes,args | grep '%s' | grep -v grep | awk '{print $1}'" % dev_tag)

    m = re.match(r"time=(\d+):(\d+):([\d.]+)", last)
    print(f"[{dev_tag}] 最后 time={last!r}  日志写入距今 {now - mtime:.2f}s  etimes={etimes!r}")
    if not m or not etimes:
        print(f"[{dev_tag}] 数据不足，跳过")
        return
    stream_s = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
    elapsed_at_log = mtime - (now - float(etimes))
    print(
        f"[{dev_tag}] 进程运行 {elapsed_at_log:.2f}s / 已编码 {stream_s:.2f}s "
        f"=> 源端积压 {elapsed_at_log - stream_s:+.2f}s"
    )


if __name__ == "__main__":
    probe("/var/log/ffmpeg-stream.log", "/dev/video0")
    probe("/var/log/ffmpeg-stream-sub.log", "/dev/video2")
