#!/usr/bin/env python3
"""RKNN 转换脚本：ONNX -> RKNN (RK3588)。

用法：
    # INT8 量化（推荐，需先准备校准集 dataset.txt）
    python convert_rknn.py --onnx best.onnx --output best_int8.rknn \
        --platform rk3588 --quantize --dataset dataset.txt

    # 不出 INT8 时走 FP16（精度几乎无损，但板端慢 2~3 倍）
    python convert_rknn.py --onnx best.onnx --output best_fp16.rknn --no-quantize

    # 对拍掉点严重时，换更精细的量化算法
    python convert_rknn.py --onnx best.onnx --output best_int8.rknn \
        --quantize --dataset dataset.txt --quantized-algorithm mmse

⚠️ 只能在 **x86 Linux** 上运行（Windows 与 RK3588 板子都不支持 rknn-toolkit2），
   见 doc/数据与模型方案.md §7.3。

关于 mean/std（**不要随手改**）：
    车端 `vision/rknn_detector.py` 的 preprocess 产出的是 **uint8 0~255** 的 RGB，
    而 YOLO 的 ONNX 期望输入是 **0~1**。RKNN 必须在图内完成这次归一化，所以
    `mean_values=[[0,0,0]] / std_values=[[255,255,255]]` 是**必需**的，不是可选优化。
    漏掉它模型会全程输出垃圾（拿全 0~255 当 0~1 用）。
    若哪天车端改成直接喂 float 0~1，这里要同步改成 mean=0 / std=1。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List

from rknn.api import RKNN


def _parse_triple(text: str, name: str) -> List[List[float]]:
    """把 '0,0,0' 解析成 [[0.0, 0.0, 0.0]]（RKNN 要的是二维）。"""
    try:
        vals = [float(x) for x in text.replace(" ", "").split(",")]
    except ValueError:
        raise SystemExit(f"[RKNN] --{name} 格式错误，应为逗号分隔的三个数，例如 255,255,255")
    if len(vals) != 3:
        raise SystemExit(f"[RKNN] --{name} 需要三个值，收到 {len(vals)} 个")
    return [vals]


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert ONNX to RKNN (RK3588)")
    parser.add_argument("--onnx", required=True, help="输入 ONNX 模型路径")
    parser.add_argument("--output", default="model.rknn", help="输出 RKNN 路径")
    parser.add_argument("--platform", default="rk3588", help="目标平台")
    parser.add_argument("--quantize", dest="quantize", action="store_true",
                        help="INT8 量化（需要 --dataset 提供校准集）")
    parser.add_argument("--no-quantize", dest="quantize", action="store_false",
                        help="不做 INT8 量化，导出 FP16 模型")
    parser.set_defaults(quantize=True)
    parser.add_argument("--dataset", default="dataset.txt",
                        help="INT8 量化校准数据列表（每行一个图片绝对路径）")
    parser.add_argument("--mean", default="0,0,0",
                        help="输入归一化均值，默认 0,0,0（配合 std=255 即 /255）")
    parser.add_argument("--std", default="255,255,255",
                        help="输入归一化标准差，默认 255,255,255")
    parser.add_argument("--quantized-dtype", default="asymmetric_quantized-8",
                        help="量化类型，RK3588 用 asymmetric_quantized-8（w8a8）")
    parser.add_argument("--quantized-algorithm", default="normal",
                        choices=["normal", "mmse", "kl_divergence"],
                        help="量化算法：normal 快；mmse 精度更好但慢（对拍掉点时可换）")
    parser.add_argument("--quantized-method", default="channel",
                        choices=["layer", "channel"],
                        help="量化粒度：channel 逐通道，精度优于 layer")
    parser.add_argument("--optimization-level", type=int, default=3, choices=[0, 1, 2, 3],
                        help="图优化等级，3 为最高")
    args = parser.parse_args()

    if not Path(args.onnx).is_file():
        print(f"[RKNN] 找不到 ONNX 文件: {args.onnx}")
        return 1
    if args.quantize and not Path(args.dataset).is_file():
        print(f"[RKNN] 找不到校准集列表: {args.dataset}\n"
              f"       它每行应是一个图片的**绝对路径**；不量化请显式加 --no-quantize")
        return 1

    rknn = RKNN(verbose=True)

    print(f"[RKNN] config platform={args.platform} quantize={args.quantize}")
    print(f"[RKNN]   mean={args.mean} std={args.std} "
          f"dtype={args.quantized_dtype} algo={args.quantized_algorithm} "
          f"method={args.quantized_method} opt={args.optimization_level}")
    if rknn.config(
        target_platform=args.platform,
        mean_values=_parse_triple(args.mean, "mean"),
        std_values=_parse_triple(args.std, "std"),
        quantized_dtype=args.quantized_dtype,
        quantized_algorithm=args.quantized_algorithm,
        quantized_method=args.quantized_method,
        optimization_level=args.optimization_level,
    ) != 0:
        print("[RKNN] config failed")
        return 1

    print(f"[RKNN] load onnx {args.onnx}")
    if rknn.load_onnx(model=args.onnx) != 0:
        print("[RKNN] load_onnx failed")
        return 1

    print(f"[RKNN] build do_quantization={args.quantize}")
    if rknn.build(do_quantization=args.quantize,
                  dataset=args.dataset if args.quantize else None) != 0:
        print("[RKNN] build failed")
        return 1

    print(f"[RKNN] export {args.output}")
    if rknn.export_rknn(args.output) != 0:
        print("[RKNN] export failed")
        return 1

    rknn.release()
    print(f"[OK] {args.output} generated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
