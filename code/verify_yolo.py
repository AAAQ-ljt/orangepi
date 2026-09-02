"""YOLOv11 环境与模型权重验证脚本.

用途:
1. 打印环境信息(python / torch / cuda / ultralytics / opencv)
2. 加载 2025 权重 best11nseg.pt 跑一次推理,验证链路可用
3. 输出检测结果并保存标注图到 code\\runs\\verify\\

用法(激活环境后):
    conda activate E:\\yolov11\\env
    python verify_yolo.py [--model <路径>] [--image <路径>] [--device 0|cpu] [--conf 0.25]

说明:
- 默认 device=0(GPU)。无 NVIDIA GPU 时请加 --device cpu
- 默认测试图是 2025 资料的停车标志模板,模型未在类似场景训练,
  检测数为 0 属于正常现象,本脚本验证的是"环境+权重+推理链路"。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent  # D:\5g\orangepi
DEFAULT_MODEL = (
    REPO_ROOT / "2025年比赛资料" / "智能车代码包+软件操作手册"
    / "yolo版本视觉+控制代码" / "yolo版本视觉识别" / "model" / "best11nseg.pt"
)
DEFAULT_IMAGE = (
    REPO_ROOT / "2025年比赛资料" / "智能车代码包+软件操作手册"
    / "opencv版本视觉+控制代码" / "A.png"
)


def main() -> int:
    parser = argparse.ArgumentParser(description="验证 YOLOv11 环境与权重")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="模型权重路径")
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE, help="测试图像路径")
    parser.add_argument("--device", default="0", help="推理设备:0=GPU, cpu=CPU")
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    args = parser.parse_args()

    # ---- 环境信息 ----
    import torch
    import cv2
    from ultralytics import YOLO, __version__ as ultralytics_version

    print("=" * 56)
    print(f"python      : {sys.version.split()[0]}")
    print(f"torch       : {torch.__version__}")
    print(f"cuda        : {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        props = torch.cuda.get_device_properties(0)
        print(f"gpu         : {props.name} ({props.total_memory / 2**30:.1f} GiB)")
    print(f"ultralytics : {ultralytics_version}")
    print(f"opencv      : {cv2.__version__}")
    print("=" * 56)

    if not args.model.exists():
        print(f"[FAIL] 模型不存在: {args.model}")
        return 1
    if not args.image.exists():
        print(f"[FAIL] 图像不存在: {args.image}")
        return 1

    # ---- 加载权重 ----
    model = YOLO(str(args.model))
    print(f"model       : {args.model.name} ({len(model.names)} 类)")
    print(f"classes     : {model.names}")

    # ---- 推理 ----
    device = args.device
    if device != "cpu" and not torch.cuda.is_available():
        print(f"[warn] CUDA 不可用(torch={torch.__version__}),自动回退 device=cpu")
        device = "cpu"
    t0 = time.perf_counter()
    results = model.predict(str(args.image), device=device, conf=args.conf, verbose=False)
    dt_ms = (time.perf_counter() - t0) * 1000
    r = results[0]
    print("-" * 56)
    print(f"inference   : {dt_ms:.1f} ms (device={device}, 图 {r.orig_shape[1]}x{r.orig_shape[0]})")
    print(f"detections  : {len(r.boxes)}")
    for b in r.boxes:
        cls, conf = int(b.cls[0]), float(b.conf[0])
        box = [round(v, 1) for v in b.xyxy[0].tolist()]
        print(f"  - {model.names[cls]:<24} conf={conf:.3f} box={box}")
    if len(r.boxes) == 0:
        print("  (0 个检测属正常:默认测试图非赛道场景,本脚本只验证链路)")

    # ---- 保存标注图 ----
    out = REPO_ROOT / "code" / "runs" / "verify" / f"{args.image.stem}_result.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    r.save(filename=str(out))
    print(f"result      : {out}")
    print("[OK] 环境与权重验证通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
