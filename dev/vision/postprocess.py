"""YOLO11-seg RKNN 后处理（小车端）。

先实现基于检测框的简化版后处理，后续再补 mask 精修车道线。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List

import numpy as np

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

CONF_THRESHOLD = 0.25
IOU_THRESHOLD = 0.45
IMG_SIZE = 640


@dataclass
class Detection:
    class_id: int
    conf: float
    xyxy: tuple
    center_x: float = 0.0
    center_y: float = 0.0


@dataclass
class PerceptionResult:
    left_x: int = 0
    right_x: int = 638
    center_x: float = 320.0
    blue_cone_count: int = 0
    yellow_cone_count: int = 0
    has_left_sign: bool = False
    has_right_sign: bool = False
    has_sign_a: bool = False
    has_sign_b: bool = False
    has_zebra_crossing: bool = False
    is_barrier: bool = False
    detections: List[Detection] = field(default_factory=list)


def xywh2xyxy(x: np.ndarray) -> np.ndarray:
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2
    y[..., 1] = x[..., 1] - x[..., 3] / 2
    y[..., 2] = x[..., 0] + x[..., 2] / 2
    y[..., 3] = x[..., 1] + x[..., 3] / 2
    return y


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> List[int]:
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_threshold]
    return keep


def _parse_detection_output(det: np.ndarray, nc: int) -> tuple:
    """det shape: (1, 4+nc, 8400) 或 (1, 4+nc+nm, 8400) 自动识别。"""
    det = det[0]  # (C, N)
    C, N = det.shape
    if C >= 4 + nc:
        boxes = det[:4].T  # (N,4)
        scores = det[4:4+nc].T  # (N,nc)
        return boxes, scores
    raise ValueError(f"unexpected detection output shape: {det.shape}")


def _parse_mask_output(mask: np.ndarray, nm: int) -> np.ndarray:
    """mask shape: (1, nm, H, W) 或 (1, H, W, nm)。"""
    if len(mask.shape) == 4:
        if mask.shape[1] == nm:
            return mask[0]  # (nm,H,W)
        if mask.shape[-1] == nm:
            return np.transpose(mask[0], (2, 0, 1))
    raise ValueError(f"unexpected mask output shape: {mask.shape}")


def postprocess(outputs: List[np.ndarray], nc: int = 9,
                conf_threshold: float = CONF_THRESHOLD,
                iou_threshold: float = IOU_THRESHOLD,
                img_size: int = IMG_SIZE) -> PerceptionResult:
    # 自动识别哪个是检测输出，哪个是 mask 原型
    det = None
    mask_proto = None
    for out in outputs:
        arr = np.asarray(out)
        if arr.ndim == 3:
            # 可能是 (1, C, N)
            if arr.shape[1] > 4:
                det = arr
        elif arr.ndim == 4:
            mask_proto = arr

    if det is None:
        raise ValueError("no detection output found")

    boxes, scores = _parse_detection_output(det, nc)
    boxes = xywh2xyxy(boxes)

    result = PerceptionResult()
    all_dets = []
    for class_id in range(nc):
        cls_scores = scores[:, class_id]
        idx = np.where(cls_scores >= conf_threshold)[0]
        if len(idx) == 0:
            continue
        cand_boxes = boxes[idx]
        cand_scores = cls_scores[idx]
        keep = nms(cand_boxes, cand_scores, iou_threshold)
        for k in keep:
            box = cand_boxes[k]
            conf = float(cand_scores[k])
            cx = float((box[0] + box[2]) / 2)
            cy = float((box[1] + box[3]) / 2)
            d = Detection(class_id=class_id, conf=conf,
                          xyxy=tuple(float(v) for v in box),
                          center_x=cx, center_y=cy)
            all_dets.append(d)
            result.detections.append(d)

    # 简化车道线：用 left_lane / right_lane 检测框中心近似
    lefts = [d.center_x for d in all_dets if d.class_id == 2 and d.conf >= conf_threshold]
    rights = [d.center_x for d in all_dets if d.class_id == 7 and d.conf >= conf_threshold]
    if lefts:
        result.left_x = int(np.mean(lefts))
    if rights:
        result.right_x = int(np.mean(rights))
    result.center_x = (result.left_x + result.right_x) / 2.0

    for d in all_dets:
        name = CLASS_NAMES[d.class_id]
        if name == "obstacle_cone_blue":
            result.blue_cone_count += 1
        elif name == "obstacle_cone_yellow":
            result.yellow_cone_count += 1
        elif name == "lane_change_sign_left":
            result.has_left_sign = True
        elif name == "lane_change_sign_right":
            result.has_right_sign = True
        elif name == "parking_sign_A":
            result.has_sign_a = True
        elif name == "parking_sign_B":
            result.has_sign_b = True
        elif name == "zebra_crossing":
            result.has_zebra_crossing = True

    return result


def result_to_message(result: PerceptionResult, timestamp: float) -> Dict[str, Any]:
    return {
        "timestamp": timestamp,
        "left_x": result.left_x,
        "right_x": result.right_x,
        "center_x": result.center_x,
        "blue_cone_count": result.blue_cone_count,
        "yellow_cone_count": result.yellow_cone_count,
        "has_left_sign": result.has_left_sign,
        "has_right_sign": result.has_right_sign,
        "has_sign_a": result.has_sign_a,
        "has_sign_b": result.has_sign_b,
        "has_zebra_crossing": result.has_zebra_crossing,
        "is_barrier": False,
        "traffic_light_state": None,
    }
