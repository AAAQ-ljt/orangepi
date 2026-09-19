#!/usr/bin/env python3
"""模型自检探针：确认 RKNN 模型在车上能跑、有多少类、每个类各检出什么。

野外调试最常用的三件事（都不动电机）：

1. **模型能不能加载 / 输入输出长什么样**（NHWC? 多少类？）；
2. **每个类 id 实际检出什么** —— profile 里类名对不上时，用这个工具对着已知目标
   （斑马线 / 蓝板 / 锥桶 / 停车区）一照，就知道哪个 id 是哪一类；
3. 存一张画了框和类 id 的图，回来复盘。

用法：

    # 用一张已有图片
    python3 scripts/diag/model_probe.py --model models/best_4cls_640x480_fp16.rknn --image /tmp/probe_frame.jpg

    # 现场用摄像头（会临时停推流，退出自动恢复）
    sudo python3 scripts/diag/model_probe.py --model models/best_4cls_640x480_fp16.rknn --camera 2 --frames 5

    # 顺便套某个 profile 看统一元素名（profile 不对就先别加）
    python3 scripts/diag/model_probe.py --model ... --profile legacy --image x.jpg
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import cv2
import numpy as np

from config import settings
from vision.elements import load_profile
from vision.postprocess import parse_elements
from vision.rknn_detector import DEFAULT_INPUT_SIZE, RKNNYoloDet


def _describe_outputs(outs) -> int:
    """打印每路输出形状，返回检测头的通道数（= 4 + 类别数）。"""
    nc_total = 0
    for i, o in enumerate(outs):
        a = np.asarray(o)
        print(f"    输出[{i}] shape={a.shape} dtype={a.dtype} "
              f"min={float(a.min()):.3f} max={float(a.max()):.3f}")
        if a.ndim == 3 and a.shape[1] > 4:
            nc_total = max(nc_total, int(a.shape[1]))
    return nc_total


def _per_class(outs, nc: int, conf_thr: float, topk: int = 3) -> None:
    """不依赖 profile，直接按通道看每个类 id 的最高分（用来认类别）。"""
    det = None
    for o in outs:
        a = np.asarray(o)
        if a.ndim == 3 and a.shape[1] > 4:
            det = a[0]
            break
    if det is None:
        print("    找不到检测头（(1, 4+nc, N)）")
        return
    n_cls = min(nc - 4, det.shape[0] - 4)
    scores = det[4:4 + n_cls]
    boxes = det[:4].T                    # (N,4) cx,cy,w,h
    print(f"    类别数（按输出通道推断）= {n_cls}")
    print("    每个类 id 的最高分（>阈值才算检出）：")
    for c in range(n_cls):
        s = scores[c]
        idx = int(np.argmax(s))
        n_hit = int((s >= conf_thr).sum())
        cx, cy, bw, bh = (float(v) for v in boxes[idx])
        print(f"      id={c}: 最高分 {float(s[idx]):.3f} "
              f"（>阈值 {n_hit} 个）"
              f"  最高分那框: 中心=({cx:.0f},{cy:.0f}) 尺寸=({bw:.0f}x{bh:.0f})")


def run_on_frame(frame, model, profile, conf_thr: float, scale: float, verbose: bool):
    x = model.preprocess(frame)
    outs = model.infer(x)
    nc = _describe_outputs(outs)
    if verbose and nc:
        _per_class(outs, nc, conf_thr)
    elements = parse_elements(outs, profile, conf_threshold=conf_thr,
                              box_transform=model.restore) if profile else []
    if profile:
        print(f"    profile={profile.name} → 元素 {len(elements)} 个: "
              f"{[(e.name, e.color, round(e.conf, 2)) for e in elements]}")
    vis = frame.copy()
    # 直接按 id 画（不依赖 profile），便于野外认类别
    det = None
    for o in outs:
        a = np.asarray(o)
        if a.ndim == 3 and a.shape[1] > 4:
            det = a[0]
            break
    if det is not None:
        n_cls = det.shape[0] - 4
        boxes = np.stack([det[0] - det[2] / 2, det[1] - det[3] / 2,
                          det[0] + det[2] / 2, det[1] + det[3] / 2], axis=1)
        boxes = model.restore(boxes)
        for c in range(n_cls):
            s = det[4 + c]
            for i in np.where(s >= conf_thr)[0]:
                x1, y1, x2, y2 = (int(v * scale) for v in boxes[i])
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(vis, f"id{c} {float(s[i]):.2f}", (x1, max(12, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return vis, elements


def main() -> int:
    ap = argparse.ArgumentParser(description="模型自检探针（不动电机）")
    ap.add_argument("--model", required=True, help="RKNN 模型路径")
    ap.add_argument("--profile", default="", help="model_profile.yaml 里的 profile 名（可选）")
    ap.add_argument("--image", default="", help="用一张图片（不碰摄像头）")
    ap.add_argument("--camera", type=int, default=-1, help="用摄像头（会临时停推流）")
    ap.add_argument("--frames", type=int, default=5, help="摄像头抓几帧（默认 5）")
    ap.add_argument("--conf", type=float, default=0.25, help="置信度阈值（默认 0.25）")
    ap.add_argument("--out", default="/root/dev/logs/model_probe", help="存图目录")
    ap.add_argument("--imgsz", default="", help="模型输入 (h,w)，默认 480,640")
    args = ap.parse_args()

    imgsz = DEFAULT_INPUT_SIZE
    if args.imgsz:
        h, w = (int(v) for v in args.imgsz.replace("x", ",").split(",")[:2])
        imgsz = (h, w)
    profile = load_profile(args.profile) if args.profile else None
    os.makedirs(args.out, exist_ok=True)

    print(f"[MODEL] {args.model}  输入={imgsz}  profile={profile.name if profile else '（未套）'}")
    model = RKNNYoloDet(args.model, imgsz)
    if not model.load():
        print("[MODEL] ❌ 模型加载失败（路径对不对？转换时是否失败？）")
        return 1
    print("[MODEL] ✅ 模型已加载")

    def handle(frame, name: str) -> None:
        vis, elements = run_on_frame(frame, model, profile, args.conf, 1.0, verbose=True)
        p = os.path.join(args.out, f"model_{name}.jpg")
        cv2.imwrite(p, vis)
        print(f"    存图：{p}")

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"[MODEL] 读不到图片 {args.image}")
            return 1
        handle(img, "image")
        model.release()
        return 0

    if args.camera < 0:
        print("[MODEL] 需要 --image 或 --camera")
        return 2

    from vision.camera_guard import camera_exclusive, open_camera
    with camera_exclusive():
        cap = open_camera(args.camera, settings.IMG_W, settings.IMG_H)
        if cap is None:
            return 1
        for i in range(args.frames):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            handle(frame, f"{i:02d}")
        cap.release()
    model.release()
    print(f"[MODEL] 完成，图在 {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
