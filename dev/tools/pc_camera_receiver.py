"""电脑端接收小车直传摄像头帧，运行 YOLO，显示窗口，并 UDP 发给小车控制端。

用法：
    python tools/pc_camera_receiver.py --model path/to/best11sseg.pt --udp-ip 10.68.1.43 --udp-port 5000 --show
"""
from __future__ import annotations

import argparse
import pathlib
import socket
import struct
import sys
import time

VISION_SRC = pathlib.Path(
    r"D:\5g\orangepi\2025年比赛资料\智能车代码包+软件操作手册\yolo版本视觉+控制代码\yolo版本视觉识别\src"
)
sys.path.insert(0, str(VISION_SRC))

import cv2  # noqa: E402

from config import Config  # noqa: E402
from lane_follower import LaneMidpointTracker  # noqa: E402
from yolo_adapter import YOLOSegmentationAdapter  # noqa: E402
from udp_sender import DetectionStateSender  # noqa: E402
from image_preprocessor import preprocess  # noqa: E402
from blue_card_handler import BlueCardHandler  # noqa: E402


def recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed")
        buf += chunk
    return buf


def main() -> int:
    parser = argparse.ArgumentParser(description="PC camera receiver + YOLO")
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9002)
    parser.add_argument("--model", type=pathlib.Path, required=True)
    parser.add_argument("--udp-ip", default="10.68.1.43")
    parser.add_argument("--udp-port", type=int, default=5000)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.listen, args.port))
    server.listen(1)
    print(f"[RECV] listening on {args.listen}:{args.port}, waiting for car...")

    conn, addr = server.accept()
    print(f"[RECV] car connected: {addr}")
    conn.settimeout(5)

    adapter = YOLOSegmentationAdapter(model_path=args.model)
    tracker = LaneMidpointTracker()
    sender = DetectionStateSender(target_ip=args.udp_ip, target_port=args.udp_port)
    blue_card = BlueCardHandler(roi=(0, 0, 640, 480), debug=False)

    start = time.time()
    frames = 0
    try:
        while True:
            if args.duration > 0 and (time.time() - start) >= args.duration:
                print(f"[RECV] duration reached, {frames} frames")
                break

            try:
                size_data = recv_exact(conn, 4)
            except (ConnectionError, socket.timeout) as exc:
                print(f"[RECV] connection error: {exc}")
                break

            size = struct.unpack(">I", size_data)[0]
            if size <= 0 or size > 5_000_000:
                print(f"[RECV] bad frame size {size}")
                continue

            jpg = recv_exact(conn, size)
            frame = cv2.imdecode(np_frombuffer(jpg), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            state = tracker.detection_state
            state.is_board = blue_card.detect(frame)
            processed = preprocess(frame)
            results = adapter.predict(processed)
            annotated, info = tracker.process_frame(processed, results, adapter.class_names)
            state = tracker.detection_state

            frames += 1
            print(f"[RECV] #{frames} center={state.center_x} "
                  f"zebra={state.has_zebra_crossing} cone={state.blue_cone_count} "
                  f"A={state.has_sign_a} B={state.has_sign_b} "
                  f"L={state.has_left_sign} R={state.has_right_sign} "
                  f"board={state.is_board}")

            sender.send_detection_state(state)

            if args.show:
                cv2.putText(annotated,
                            f"L={state.left_x} R={state.right_x} C={state.center_x} "
                            f"zebra={int(state.has_zebra_crossing)} cone={state.blue_cone_count} "
                            f"A={int(state.has_sign_a)} B={int(state.has_sign_b)} "
                            f"Lsign={int(state.has_left_sign)} Rsign={int(state.has_right_sign)} "
                            f"board={int(state.is_board)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("car camera direct", annotated)
                key = cv2.waitKey(1)
                if key == ord("q"):
                    print("[RECV] stopped by user")
                    break
    except KeyboardInterrupt:
        print("[RECV] interrupted")
    finally:
        if args.show:
            cv2.destroyAllWindows()
        sender.close()
        conn.close()
        server.close()
    return 0


def np_frombuffer(data: bytes):
    import numpy as np
    return np.frombuffer(data, dtype=np.uint8)


if __name__ == "__main__":
    raise SystemExit(main())
