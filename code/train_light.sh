#!/usr/bin/env bash
# 红绿灯 3 类模型训练脚本（2026-10-09 v11 定案：模型轨=仅灯）
#
# 类别（顺序=YOLO id，与 config/model_profile.yaml 的 light3cls 一致）：
#   0 traffic_light_off / 1 traffic_light_red / 2 traffic_light_green
#
# 数据（务必先看 doc/补拍清单-2026-10-10.md）：
#   现有红绿灯 571 张按灯态重标 + 现场补拍灯灭态；按片段整组切分 8:1:1 → datasets/light3cls/
#   结构：datasets/light3cls/{images,labels}/{train,val} + data.yaml（参考 code/ 下说明）
#
# 训练机：5070 Ti（队友侧）。⚠️ 本脚本只到 ONNX；RKNN 转换由队友负责（2026-10-09 已对齐）。
#
# 用法（在训练机上）：
#   bash code/train_light.sh <datasets/light3cls/data.yaml>

set -euo pipefail

DATA="${1:?用法: bash code/train_light.sh <data.yaml>}"
YOLO_DIR="${YOLO_DIR:-yolov5}"            # 官方 13 号案例的 yolov5 工程目录
IMG=640
BATCH=8
EPOCHS="${EPOCHS:-200}"                   # 官方教程 200；50 轮即可达 conf 0.7~0.9

echo "==> 训练（yolov5 $YOLO_DIR, data=$DATA, img=$IMG, batch=$BATCH, epochs=$EPOCHS）"
python "$YOLO_DIR/train.py" \
    --weights "$YOLO_DIR/weights/yolov5s.pt" \
    --data "$DATA" \
    --img "$IMG" --batch "$BATCH" --epochs "$EPOCHS" \
    --device 0 --workers 0

BEST="runs/train/exp/weights/best.pt"
[ -f "$BEST" ] || BEST="$(ls -d runs/train/exp*/ | tail -1)weights/best.pt"
echo "==> 导出 ONNX（opset12, nms off）: $BEST"
python "$YOLO_DIR/export.py" \
    --weights "$BEST" --img "$IMG" --batch 1 --include onnx --opset 12

echo
echo "==> 下一步（队友侧）：ONNX → RKNN（rknn_model_zoo yolov5 例程改路径），"
echo "    目标 rk3588 / 480×640 / FP16 优先（INT8+校准集可选）；产出 best_light3cls.rknn"
echo "    上板后融合规则见 doc/开发方案-2026-10-08.md §五（HSV 主 + 模型一致才动作）。"