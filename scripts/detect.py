"""
YOLO11 推理脚本 — 用训练好的权重对 图片/视频/摄像头 检测

用法（conda 环境 yolo11）：
    python scripts/detect.py --source <图片|视频|文件夹|0摄像头>
    python scripts/detect.py --source test.jpg --weights runs/smartcar/yolo11s/weights/best.pt --conf 0.35
    python scripts/detect.py --source 0 --save            # 摄像头实时检测并保存

说明：
    - 小车端(OrangePi5)部署时，ultralytics 推理仅用于原型验证；
      正式上车的模型建议转 RKNN 后走 NPU 推理（见项目 README 建议）。
"""
import argparse

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLO11 推理")
    parser.add_argument("--weights", default="runs/smartcar/yolo11s/weights/best.pt",
                        help="训练好的权重（best.pt）")
    parser.add_argument("--source", required=True,
                        help="输入：图片 / 视频 / 文件夹 / 摄像头编号(0,1,...)")
    parser.add_argument("--conf", type=float, default=0.35, help="置信度阈值")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--save", action="store_true", help="保存带标注的结果图/视频")
    parser.add_argument("--show", action="store_true", help="弹窗实时显示（需本地桌面）")
    args = parser.parse_args()

    model = YOLO(args.weights)

    results = model.predict(
        source=args.source,
        conf=args.conf,
        imgsz=args.imgsz,
        save=args.save,
        show=args.show,
        project="runs/detect",
        name="inference",
    )

    for r in results:
        names = r.names
        boxes = r.boxes
        if boxes is not None and len(boxes) > 0:
            for b in boxes:
                cls = int(b.cls[0])
                conf = float(b.conf[0])
                print(f"  {names[cls]:<18} conf={conf:.3f} xyxy={[round(x,1) for x in b.xyxy[0].tolist()]}")
        else:
            print("  未检测到目标")


if __name__ == "__main__":
    main()