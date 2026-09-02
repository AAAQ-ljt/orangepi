"""
YOLO11 训练脚本 — 5G 智能车竞赛视觉目标检测

用法（在 conda 环境 yolo11 中）：
    conda activate yolo11
    python scripts/train.py            # 默认用 yolo11s 微调
    python scripts/train.py --model yolo11m.pt --epochs 150 --batch 12

说明：
    - 首次运行会自动下载 yolo11s.pt 预训练权重（约 20MB）
    - RTX 5070 (8GB)：yolo11s 建议 batch 16~32；yolo11m 建议 8~12
    - 训练产物输出到 runs/smartcar/<name>/，best.pt 即最优权重
"""
import argparse

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLO11 训练")
    parser.add_argument("--model", default="weights/yolo11s.pt",
                        help="预训练权重，如 weights/yolo11n.pt / yolo11s.pt / yolo11m.pt")
    parser.add_argument("--data", default="datasets/smartcar/smartcar.yaml",
                        help="数据集 yaml 路径")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16,
                        help="根据显存调整：8GB 显存 yolo11s 用 16~32")
    parser.add_argument("--device", default="0", help="GPU 编号，0 = 第一块")
    parser.add_argument("--patience", type=int, default=30,
                        help="早停耐心值，0 表示关闭")
    parser.add_argument("--name", default="yolo11s", help="实验名（输出目录名）")
    args = parser.parse_args()

    model = YOLO(args.model)

    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        patience=args.patience,
        project="runs/smartcar",
        name=args.name,
        pretrained=True,          # 加载预训练权重微调，收敛更快、数据少时更稳
        optimizer="auto",
        amp=True,
        plots=True,               # 输出混淆矩阵、PR 曲线等
        val=True,
        seed=0,
    )

    # 训练结束后打印最佳权重路径
    import pathlib
    best = pathlib.Path("runs") / "smartcar" / args.name / "weights" / "best.pt"
    print(f"\n[OK] 训练完成，最佳权重: {best}")


if __name__ == "__main__":
    main()