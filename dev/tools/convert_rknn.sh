#!/usr/bin/env bash
# RKNN 一键转换脚本（ModelScope GPU 环境 / Ubuntu22.04 / Python3.10）
# 用法:
#   bash convert_rknn.sh <onnx文件> [输出rknn]
set -euo pipefail

ONNX="${1:?用法: bash convert_rknn.sh <onnx文件> [输出rknn]}"
OUTPUT="${2:-${ONNX%.onnx}.rknn}"
PLATFORM="${PLATFORM:-rk3588}"

echo "=============================="
echo " RKNN 转换"
echo " ONNX    : $ONNX"
echo " OUTPUT  : $OUTPUT"
echo " PLATFORM: $PLATFORM"
echo "=============================="

echo "[1/4] 安装 rknn-toolkit2"
pip install --upgrade pip
pip install rknn-toolkit2

echo "[2/4] 检查 ONNX 文件"
if [ ! -f "$ONNX" ]; then
  echo "错误: 找不到 $ONNX"
  exit 1
fi

echo "[3/4] 执行转换"
python convert_rknn.py --onnx "$ONNX" --output "$OUTPUT" --platform "$PLATFORM"

echo "[4/4] 完成"
ls -lh "$OUTPUT"
