"""场景化视觉测试：用上一届训练图片按顺序发送 UDP，验证控制端 FSM。

在电脑端运行（需要 ultralytics 环境）：
    python tools/scenario_vision_test.py \
      --data-dir "D:/5g/orangepi/dev/tests/yolo 2.yolov8/train/images" \
      --labels-dir "D:/5g/orangepi/dev/tests/yolo 2.yolov8/train/labels" \
      --model "path/to/best11sseg.pt" \
      --udp-ip 10.68.1.43 \
      --udp-port 5000
"""
from __future__ import annotations

import argparse
import pathlib
import sys
import time

# 上一届视觉代码目录
VISION_SRC = pathlib.Path(
    r"D:\5g\orangepi\2025年比赛资料\智能车代码包+软件操作手册\yolo版本视觉+控制代码\yolo版本视觉识别\src"
)
sys.path.insert(0, str(VISION_SRC))

from config import Config  # noqa: E402
from lane_follower import LaneMidpointTracker  # noqa: E402
from yolo_adapter import YOLOSegmentationAdapter  # noqa: E402
from udp_sender import DetectionStateSender  # noqa: E402
from image_preprocessor import preprocess  # noqa: E402
from blue_card_handler import BlueCardHandler  # noqa: E402

CLASS_NAMES = [
    "lane_change_sign_left",
    "lane_change_sign_right",
    "left_lane",
    "obstacle_cone_blue",
    "obstacle_cone_yellow",
    "parking_sign_A",
    "parking_sign_B",
    "right_lane",
    "zebra_crossing",
]


def find_image_with_class(labels_dir: pathlib.Path, images_dir: pathlib.Path,
                          class_id: int) -> pathlib.Path | None:
    for label_path in sorted(labels_dir.glob("*.txt")):
        for line in label_path.read_text(encoding="utf-8", errors="ignore").splitlines():
            parts = line.split()
            if parts and int(parts[0]) == class_id:
                image_path = images_dir / (label_path.stem + ".png")
                if image_path.exists():
                    return image_path
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Scenario vision UDP test")
    parser.add_argument("--data-dir", type=pathlib.Path, required=True)
    parser.add_argument("--labels-dir", type=pathlib.Path, required=True)
    parser.add_argument("--model", type=pathlib.Path, required=True)
    parser.add_argument("--udp-ip", default="10.68.1.43")
    parser.add_argument("--udp-port", type=int, default=5000)
    parser.add_argument("--delay", type=float, default=1.0)
    args = parser.parse_args()

    # 场景顺序：normal -> zebra -> normal -> cone -> normal -> parking A
    scenarios = [
        ("normal", None),
        ("zebra", 8),
        ("normal", None),
        ("cone", 3),   # obstacle_cone_blue
        ("normal", None),
        ("parking_A", 5),
    ]

    # 为每个需要特定类的场景找一张图
    image_plan = []
    for name, class_id in scenarios:
        if class_id is None:
            # normal 随便选一张包含 left_lane/right_lane 的图
            img = find_image_with_class(args.labels_dir, args.data_dir, 2)
            if img is None:
                img = next(iter(sorted(args.data_dir.glob("*.png"))), None)
            image_plan.append((name, img))
        else:
            img = find_image_with_class(args.labels_dir, args.data_dir, class_id)
            image_plan.append((name, img))

    for name, img in image_plan:
        if img is None:
            print(f"[SCENARIO] skip {name}: no image found")
            continue

        print(f"[SCENARIO] {name}: {img.name}")

    # 初始化视觉组件
    adapter = YOLOSegmentationAdapter(model_path=args.model)
    tracker = LaneMidpointTracker()
    sender = DetectionStateSender(target_ip=args.udp_ip, target_port=args.udp_port)
    blue_card = BlueCardHandler(roi=(0, 0, 640, 480), debug=False)

    try:
        for name, img in image_plan:
            if img is None:
                continue
            frame = preprocess(cv2.imread(str(img)))
            state = tracker.detection_state
            state.is_board = blue_card.detect(frame)
            results = adapter.predict(frame)
            tracker.process_frame(frame, results, adapter.class_names)
            state = tracker.detection_state

            print(f"[SCENARIO] send {name}: center={state.center_x} "
                  f"zebra={state.has_zebra_crossing} cone={state.blue_cone_count} "
                  f"A={state.has_sign_a} B={state.has_sign_b}")
            sender.send_detection_state(state)
            time.sleep(args.delay)
    finally:
        sender.close()

    return 0


if __name__ == "__main__":
    import cv2  # noqa: E402
    raise SystemExit(main())
