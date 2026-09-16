"""模型输出 → 元素列表（解码 + NMS + 统一语义）。

本文件只负责"把张量变成框"，**语义映射交给 `vision/elements.py` 的 profile**：
换模型（上一届 → 我们自己训练的）不需要动这里，只改 `config/model_profile.yaml`。
"""
from __future__ import annotations

from typing import List, Optional

import numpy as np

from vision.elements import Element, ModelProfile, elements_from_boxes, load_profile

IOU_THRESHOLD = 0.45


def xywh2xyxy(x: np.ndarray) -> np.ndarray:
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2
    y[..., 1] = x[..., 1] - x[..., 3] / 2
    y[..., 2] = x[..., 0] + x[..., 2] / 2
    y[..., 3] = x[..., 1] + x[..., 3] / 2
    return y


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> List[int]:
    """单类别 NMS。"""
    if boxes.size == 0:
        return []
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
        order = order[order[1:][iou <= iou_threshold]]
    return keep


def _find_detection_tensor(outputs: List[np.ndarray]) -> np.ndarray:
    """在 RKNN 的多路输出里找出检测头 (1, 4+nc, N)。"""
    for out in outputs:
        arr = np.asarray(out)
        if arr.ndim == 3 and arr.shape[1] > 4:
            return arr
    raise ValueError(f"no detection output found: {[np.asarray(o).shape for o in outputs]}")


def parse_elements(outputs: List[np.ndarray], profile: ModelProfile,
                   conf_threshold: Optional[float] = None,
                   iou_threshold: float = IOU_THRESHOLD) -> List[Element]:
    """解码 + 逐类 NMS → 元素列表（统一语义）。"""
    det = _find_detection_tensor(outputs)[0]        # (C, N)
    nc = profile.num_classes
    if det.shape[0] < 4 + nc:
        raise ValueError(f"类别数与 profile 不符：输出 {det.shape[0] - 4} 类，profile {nc} 类")
    boxes = xywh2xyxy(det[:4].T)                    # (N,4)
    scores = det[4:4 + nc].T                        # (N,nc)
    conf_thr = float(profile.conf if conf_threshold is None else conf_threshold)

    picked_boxes, picked_scores = [], []
    for class_id in range(nc):
        cls_scores = scores[:, class_id]
        idx = np.where(cls_scores >= conf_thr)[0]
        if idx.size == 0:
            continue
        for k in nms(boxes[idx], cls_scores[idx], iou_threshold):
            picked_boxes.append(boxes[idx][k])
            picked_scores.append((class_id, float(cls_scores[idx][k])))

    if not picked_boxes:
        return []
    return elements_from_boxes(np.asarray(picked_boxes),
                               np.asarray(picked_scores, dtype=float).reshape(-1, 2),
                               profile)


def postprocess(outputs: List[np.ndarray],
                profile: Optional[ModelProfile] = None,
                conf_threshold: Optional[float] = None,
                iou_threshold: float = IOU_THRESHOLD) -> List[Element]:
    """兼容旧调用名的入口：不给 profile 就用默认（legacy，上一届模型）。"""
    prof = profile if profile is not None else load_profile()
    return parse_elements(outputs, prof, conf_threshold=conf_threshold, iou_threshold=iou_threshold)
