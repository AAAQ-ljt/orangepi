"""电脑视觉环境验证脚本。

用法：
    python tools/verify_vision_model.py --model path/to/best11sseg.pt --source image.jpg
"""
from __future__ import annotations

import argparse
import sys

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify PC YOLO vision environment")
    parser.add_argument("--model", required=True, help="YOLO 权重路径")
    parser.add_argument("--source", required=True, help="测试图片/视频路径")
    parser.add_argument("--conf", type=float, default=0.3)
    parser.add_argument("--imgsz", type=int, default=640)
    args = parser.parse_args()

    print(f"[VISION] loading model: {args.model}")
    model = YOLO(args.model)
    results = model.predict(source=args.source, conf=args.conf, imgsz=args.imgsz, verbose=False)

    for r in results:
        names = r.names
        boxes = r.boxes
        if boxes is not None and len(boxes) > 0:
            for b in boxes:
                cls = int(b.cls[0])
                conf = float(b.conf[0])
                print(f"  {names[cls]:<24} conf={conf:.3f}")
        else:
            print("  no objects detected")

    return 0


if __name__ == "__main__":
    sys.exit(main())
