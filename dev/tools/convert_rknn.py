#!/usr/bin/env python3
"""RKNN 转换脚本：ONNX -> RKNN (RK3588)。

用法：
    python convert_rknn.py --onnx best11sseg.onnx --output best11sseg.rknn [--platform rk3588] [--quantize]
"""
from __future__ import annotations

import argparse
import sys

from rknn.api import RKNN


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert ONNX to RKNN")
    parser.add_argument("--onnx", required=True, help="输入 ONNX 模型路径")
    parser.add_argument("--output", default="model.rknn", help="输出 RKNN 路径")
    parser.add_argument("--platform", default="rk3588", help="目标平台")
    parser.add_argument("--quantize", action="store_true", help="是否 INT8 量化（需要 dataset.txt）")
    parser.add_argument("--dataset", default="dataset.txt", help="INT8 量化校准数据列表")
    args = parser.parse_args()

    rknn = RKNN(verbose=True)

    print(f"[RKNN] config platform={args.platform}")
    if rknn.config(target_platform=args.platform) != 0:
        print("[RKNN] config failed")
        return 1

    print(f"[RKNN] load onnx {args.onnx}")
    if rknn.load_onnx(model=args.onnx) != 0:
        print("[RKNN] load_onnx failed")
        return 1

    print(f"[RKNN] build do_quantization={args.quantize}")
    if rknn.build(do_quantization=args.quantize, dataset=args.dataset if args.quantize else None) != 0:
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
