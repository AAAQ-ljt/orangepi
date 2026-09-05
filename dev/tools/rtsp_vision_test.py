"""通过小车 RTSP 图传在电脑上运行 YOLO 视觉，并把结果 UDP 发回小车控制端。

用法：
    python tools/rtsp_vision_test.py --model path/to/best11sseg.pt --udp-ip 10.68.1.43 --udp-port 5000 --duration 30 --show
"""
from __future__ import annotations

import argparse
import pathlib
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


def main() -> int:
    parser = argparse.ArgumentParser(description="RTSP vision test")
    parser.add_argument("--rtsp", default="rtsp://82.157.204.126:17005/cam_car0027")
    parser.add_argument("--model", type=pathlib.Path, required=True)
    parser.add_argument("--udp-ip", default="10.68.1.43")
    parser.add_argument("--udp-port", type=int, default=5000)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--show", action="store_true", help="显示 OpenCV 预览窗口")
    args = parser.parse_args()

    print(f"[RTSP] opening {args.rtsp}")
    cap = cv2.VideoCapture(args.rtsp, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("ERROR: cannot open RTSP stream")
        return 1

    adapter = YOLOSegmentationAdapter(model_path=args.model)
    tracker = LaneMidpointTracker()
    sender = DetectionStateSender(target_ip=args.udp_ip, target_port=args.udp_port)
    blue_card = BlueCardHandler(roi=(0, 0, 640, 480), debug=False)

    start = time.time()
    frames = 0
    fail_count = 0
    try:
        while True:
            if args.duration > 0 and (time.time() - start) >= args.duration:
                print(f"[RTSP] duration reached, {frames} frames")
                break

            ret, frame = cap.read()
            if not ret or frame is None:
                fail_count += 1
                if fail_count >= 50:
                    print("[RTSP] too many frame read failures, exit")
                    break
                time.sleep(0.1)
                continue
            fail_count = 0

            state = tracker.detection_state
            state.is_board = blue_card.detect(frame)
            processed = preprocess(frame)
            results = adapter.predict(processed)
            annotated, info = tracker.process_frame(processed, results, adapter.class_names)
            state = tracker.detection_state

            frames += 1
            print(f"[RTSP] #{frames} center={state.center_x} "
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
                cv2.imshow("car camera vision", annotated)
                key = cv2.waitKey(1)
                if key == ord("q"):
                    print("[RTSP] stopped by user")
                    break
    except KeyboardInterrupt:
        print("[RTSP] interrupted")
    finally:
        if args.show:
            cv2.destroyAllWindows()
        sender.close()
        cap.release()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
