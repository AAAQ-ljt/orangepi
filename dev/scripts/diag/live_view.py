#!/usr/bin/env python3
"""循迹实况视图（推车调试用）：把 lane_ref 检测器**当前这一帧**看到了什么实时画出来。

干什么用
--------
手推着车走一遍赛道，边推边看代码到底把哪两条线当车道线、前瞻带压在哪几行、
误差是多少、什么时候掉成"单侧兜底" —— 直接看出"为什么丢线 / 为什么锁歪"，
不用等回到电脑前翻存图（本车没有显示器，这是唯一能"实时看"的途径）。

叠加图用的就是 `scripts/lane_final_test.py`（最终循迹脚本）里的同一个 `annotate()`，
**和实跑存下来的 ann_*.jpg 逐像素同款**，所以现场看到什么、跑起来就是什么。

两种显示方式
------------
1. **HTTP MJPEG（推荐）**：`--http 8090` → 浏览器看。
   跨公网走隧道时，X11 每帧要传整幅位图，会卡；MJPEG 只传 JPEG，顺畅得多。
2. **X11 窗口**：`--show`。本车**没有 Xorg**，必须靠 SSH 的 X11 转发
   （MobaXterm 自带 X 服务器；车端 sshd 已 `X11Forwarding yes`、有 `xauth`、
   cv2 是带 QT5 GUI 的构建）。跑之前确认 `echo $DISPLAY` 非空。

安全
----
**完全不碰电机**：本脚本不 import driver / bench_common，不写 PCA9685，只读摄像头。
所以推车时用它没有失控风险。摄像头独占由 `camera_exclusive()` 保证
（自动停/恢复两路推流，异常退出也会恢复）。

用法
----
    # 车端（推荐，浏览器看）
    sudo python3 /root/dev/scripts/diag/live_view.py --http 8090
    # Windows 上再开一条端口转发（MobaXterm 的 Local Terminal 里敲），
    # 然后浏览器打开 http://127.0.0.1:8090
    ssh -L 8090:127.0.0.1:8090 -p 2222 root@121.40.149.155

    # 或者 X11 窗口（MobaXterm 里直接跑，要求 $DISPLAY 能连通）
    sudo python3 /root/dev/scripts/diag/live_view.py --show

    # 现场试不同 ROI / 前瞻带（不写回 site.yaml，纯临时）
    sudo python3 .../live_view.py --http 8090 --roi-top 0.35 --roi-bottom 0.75

按键（仅 X11 窗口模式）
    q / ESC 退出      s 存一帧到 --save-dir      d 切换「叠加图 / 原始画面」
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import cv2   # noqa: E402

from config import settings                                  # noqa: E402
from vision.camera_guard import camera_exclusive, open_camera  # noqa: E402
from vision.start_gate import StartGate                      # noqa: E402

# ★ 2026-10-14：检测/叠加图都换成**最终循迹脚本**（lane_ref_test.py 已归档）——
#   现场看到的 = 实跑脚本用的，同一套代码同一个叠加图。
from scripts.lane_final_test import LaneFinalDetector, annotate   # noqa: E402


class LatestFrame:
    """最新一帧的 JPEG（HTTP 线程读、主循环写；用锁保证不撕裂）。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg = None                 # type: Optional[bytes]
        self._seq = 0

    def put(self, jpeg) -> None:
        with self._lock:
            self._jpeg = jpeg
            self._seq += 1

    def get(self) -> Tuple[Optional[bytes], int]:
        with self._lock:
            return self._jpeg, self._seq


LATEST = LatestFrame()

_PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>循迹实况</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px/1.5 monospace}
img{width:100%;max-width:960px;display:block;margin:0 auto}</style></head>
<body><h3 style="text-align:center">循迹实况（lane_ref 检测器叠加图）</h3>
<img src="/stream.mjpg" alt="stream"></body></html>"""


class MJPEGHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):        # 别刷屏
        pass

    def do_GET(self):                 # noqa: N802
        if self.path in ("/", "/index.html"):
            body = _PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path not in ("/stream.mjpg", "/live.mjpg"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_seq = -1
        try:
            while True:
                jpeg, seq = LATEST.get()
                if jpeg is None or seq == last_seq:
                    time.sleep(0.02)
                    continue
                last_seq = seq
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass                      # 浏览器关页面 = 正常


def start_http(port: int) -> None:
    srv = ThreadingHTTPServer(("0.0.0.0", port), MJPEGHandler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[LIVE] HTTP MJPEG 已在 0.0.0.0:{port} 监听：浏览器打开 http://127.0.0.1:{port}")


def main() -> int:
    ap = argparse.ArgumentParser(description="循迹实况视图（只读摄像头，绝不碰电机）")
    ap.add_argument("--camera", type=int, default=2, help="2=下摄（默认）/ 0=云台主摄")
    ap.add_argument("--show", action="store_true", help="用 X11 窗口显示（需要 DISPLAY）")
    ap.add_argument("--http", type=int, default=0, help="另开一个 MJPEG HTTP 端口（推荐）")
    ap.add_argument("--width", type=int, default=settings.IMG_W)
    ap.add_argument("--height", type=int, default=settings.IMG_H)
    ap.add_argument("--fourcc", default=None, help="如 MJPG（本车默认已是 MJPG，一般不用给）")
    ap.add_argument("--fps", type=float, default=0.0,
                    help="限制推流/显示的帧率（0=不限；隧道窄时给 8~10）")
    ap.add_argument("--view-scale", type=float, default=1.0,
                    help="显示/推流前把画面缩小多少（X11 转发卡时用 0.5 —— 数据量降到 1/4）")
    ap.add_argument("--quality", type=int, default=70, help="JPEG 质量（默认 70，越小越省流量）")
    ap.add_argument("--print-every", type=int, default=15, help="终端每 N 帧打一行读数")
    ap.add_argument("--save-dir", default="", help="按 s 键存帧的目录（X11 模式）")
    # ---- 检测参数：默认全取 lane_final_test 的标定档；这里给的都是临时覆盖，不写回任何文件 ----
    ap.add_argument("--roi-top", type=int, default=None, help="检测带上沿（行号，默认 5）")
    ap.add_argument("--roi-bottom", type=int, default=None, help="检测带下沿（行号，默认 200）")
    ap.add_argument("--look-top", type=int, default=None, help="前瞻带上沿（行号，默认 24）")
    ap.add_argument("--look-bottom", type=int, default=None, help="前瞻带下沿（行号，默认 165）")
    ap.add_argument("--target-x", type=float, default=None, help="期望中心 x（像素，默认 330）")
    ap.add_argument("--ridge-min", type=float, default=None, help="脊分下限（默认 7.0）")
    args = ap.parse_args()

    if not args.show and not args.http:
        args.http = 8090
        print("[LIVE] 没给 --show / --http，默认开 MJPEG 8090")

    if args.show and not os.environ.get("DISPLAY"):
        print("[LIVE] ⚠️ $DISPLAY 为空 → X11 用不了（本车没有 Xorg，需要 SSH 的 X11 转发）。")
        print("[LIVE]    改用 --http 8090，或在 MobaXterm 里确认 X11-forwarding 已启用。")
        if not args.http:
            return 2

    over = {k: v for k, v in (("roi_top", args.roi_top), ("roi_bottom", args.roi_bottom),
                              ("look_top", args.look_top), ("look_bottom", args.look_bottom),
                              ("target_x", args.target_x), ("ridge_min", args.ridge_min))
            if v is not None}
    det = LaneFinalDetector(fps=30.0, **over)
    gate = StartGate()

    print(f"[LIVE] 摄像头 /dev/video{args.camera}  {args.width}x{args.height}")
    print(f"[LIVE] 检测带 y={det.y_top}~{det.y_bot}  前瞻带 y={det.look_top}~{det.look_bot}  "
          f"y_ref={det.y_ref}  |k| {det.k_min}~{det.k_max}  目标点 x={det.target_x:.0f}")
    print("[LIVE] 图例：红=左线 / 蓝=右线 / 橙=检测带 / 青=前瞻带 / 绿竖线=目标点 / 黄圈=中心")
    print("[LIVE] 纯视觉，不驱动电机 —— 推车走一遍，看它在哪一帧掉成'单侧'。")

    if args.http:
        start_http(args.http)
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    state = {"stopped": False}

    def _on_signal(_signum, _frame):
        print("\n[LIVE] 收到停止信号，收尾…")
        state["stopped"] = True

    import signal
    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        sig = getattr(signal, name, None)
        if sig is not None:
            try:
                signal.signal(sig, _on_signal)
            except (ValueError, OSError):
                pass

    frames = 0
    t0 = time.time()
    last_put = 0.0
    raw_mode = False
    saved = 0
    rc = 0
    try:
        with camera_exclusive():
            cap = open_camera(args.camera, args.width, args.height, fourcc=args.fourcc)
            if cap is None:
                print("[LIVE] 摄像头打不开")
                return 2
            try:
                while not state["stopped"]:
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        time.sleep(0.02)
                        continue
                    r = det.detect(frame)
                    gs = gate.update(frame)
                    frames += 1
                    fps = frames / max(1e-6, time.time() - t0)

                    if args.print_every and frames % args.print_every == 0:
                        print(f"[LIVE] #{frames} fps={fps:.1f} "
                              f"c={'-' if r.center_x is None else f'{r.center_x:.0f}'} "
                              f"e={'-' if r.error is None else f'{r.error:+.0f}'} "
                              f"q={r.quality:.2f} L={'-' if r.left_x is None else f'{r.left_x:.0f}'} "
                              f"R={'-' if r.right_x is None else f'{r.right_x:.0f}'} "
                              f"n={r.n_l}/{r.n_r} "
                              f"{'双侧' if r.both_sides else '单侧'}"
                              f"{'  板#' if gs.blocked else ''}")

                    need_vis = args.show or args.http
                    if need_vis:
                        now = time.time()
                        if args.fps <= 0 or (now - last_put) >= (1.0 / args.fps):
                            last_put = now
                            vis = frame if raw_mode else annotate(
                                frame, r, det, "live",
                                extra=f"fps={fps:.1f} roi={det.y_top}~{det.y_bot}")
                            if args.view_scale != 1.0:
                                # X11 转发的是**原始位图**，缩一半数据量就降到 1/4 —— 卡的时候先用这个
                                vis = cv2.resize(vis, None, fx=args.view_scale, fy=args.view_scale,
                                                 interpolation=cv2.INTER_AREA)
                            if args.http:
                                LATEST.put(cv2.imencode(
                                    ".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])[1].tobytes())
                            if args.show:
                                cv2.imshow("lane live (q=quit d=raw s=save)", vis)
                                key = cv2.waitKey(1) & 0xFF
                                if key in (ord("q"), 27):
                                    state["stopped"] = True
                                elif key == ord("d"):
                                    raw_mode = not raw_mode
                                    print(f"[LIVE] 显示模式：{'原始画面' if raw_mode else '叠加图'}")
                                elif key == ord("s") and args.save_dir:
                                    p = os.path.join(args.save_dir, f"live_{saved:04d}.jpg")
                                    cv2.imwrite(p, vis)
                                    saved += 1
                                    print(f"[LIVE] 已存 {p}")
            finally:
                cap.release()
    finally:
        if args.show:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        print("[LIVE] 结束（推流已自动恢复）")
    return rc


if __name__ == "__main__":
    sys.exit(main())
