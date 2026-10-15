#!/usr/bin/env python3
"""跑车时在浏览器里看"检测器的眼睛"：把最新会话的 ann_*.jpg 叠加图推成 MJPEG。

为什么不是 X11 / live_view（2026-10-11 操场实测结论）：
- 本车没有 X 服务器，X11 只能靠 ssh 转发；公网隧道一帧一整幅位图（640×480×4 ≈1.2MB），
  15fps ≈ 18MB/s，5G 上行撑不住，卡成幻灯片——X11 只适合热点直连；
- live_view.py 要自己开摄像头——摄像头独占，跟实跑脚本互斥，跑车时开不了；
- 实跑脚本每 --save-every-s（默认 1s）落一张叠加图 ann_*.jpg，本脚本只读文件把它
  推成 MJPEG：零摄像头占用、零控制回路影响，浏览器延迟 ≈1~2s，隧道/直连都流畅。

用法：
  车端：  sudo python3 /root/dev/scripts/diag/ann_live.py --http 8090
          （默认自动跟随 /root/dev/logs 下最新的 lane_* 会话；也可 --dir 指定）
  PC 侧另开一条终端做隧道：
          ssh -L 8090:127.0.0.1:8090 -p 2222 root@121.40.149.155
  浏览器： http://127.0.0.1:8090

发车前就可以开：起步自检/挡板阶段就有图（phase=stop 也落叠加图），不用等发车。
"""
from __future__ import annotations

import argparse
import glob
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def newest_ann(args) -> str:
    """最新一张叠加图：--dir 指定目录，否则自动跟 logs 下最新的 lane_* 会话。"""
    d = args.dir
    if not d:
        sessions = sorted(glob.glob(os.path.join(args.logs, "lane_*")))
        if not sessions:
            return ""
        d = sessions[-1]
    anns = glob.glob(os.path.join(d, "ann_*.jpg"))
    return max(anns, key=os.path.getmtime) if anns else ""


class Handler(BaseHTTPRequestHandler):
    args = None                                   # main() 里注入

    def log_message(self, *_a) -> None:           # 静音：别每帧刷一行日志
        pass

    def do_GET(self) -> None:
        if self.path != "/stream":
            html = ("<!doctype html><title>ann live</title>"
                    "<body style='margin:0;background:#111;text-align:center'>"
                    "<img src='/stream' style='max-width:100%'></body>").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        last_path, last_size = "", -1
        try:
            while True:
                path = newest_ann(self.args)
                if path:
                    try:
                        size = os.path.getsize(path)
                        if path != last_path or size != last_size:
                            time.sleep(0.15)      # 等脚本把这张写完，避免读到半张
                            if os.path.getsize(path) == size:
                                with open(path, "rb") as fh:
                                    data = fh.read()
                                self.wfile.write(
                                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                                    b"Content-Length: " + str(len(data)).encode()
                                    + b"\r\n\r\n" + data + b"\r\n")
                                self.wfile.flush()
                                last_path, last_size = path, size
                    except OSError:
                        pass                      # 文件正被轮换/读 half：下一轮再来
                time.sleep(self.args.interval)
        except (BrokenPipeError, ConnectionResetError):
            pass                                  # 浏览器关了/刷新了：正常退出


def main() -> int:
    ap = argparse.ArgumentParser(description="ann_*.jpg → MJPEG（浏览器看检测器叠加图）")
    ap.add_argument("--http", type=int, default=8090, help="监听端口（默认 8090）")
    ap.add_argument("--logs", default="/root/dev/logs", help="会话根目录")
    ap.add_argument("--dir", default="", help="直接指定会话目录（默认自动跟最新 lane_*）")
    ap.add_argument("--interval", type=float, default=0.5, help="扫新帧间隔秒（默认 0.5）")
    args = ap.parse_args()
    Handler.args = args
    src = args.dir or f"{args.logs}/lane_*（自动跟最新）"
    print(f"[ANN] MJPEG 服务 http://0.0.0.0:{args.http}  源：{src}")
    ThreadingHTTPServer(("0.0.0.0", args.http), Handler).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
