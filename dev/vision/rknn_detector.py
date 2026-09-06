"""RKNN YOLO11 分割检测器（小车端）。"""
from __future__ import annotations

import cv2
import numpy as np
from rknnlite.api import RKNNLite


class RKNNYoloSeg:
    def __init__(self, model_path: str):
        self.model_path = model_path
        self.rknn = None

    def load(self) -> bool:
        self.rknn = RKNNLite()
        if self.rknn.load_rknn(self.model_path) != 0:
            return False
        if self.rknn.init_runtime() != 0:
            return False
        return True

    def infer(self, frame_bgr: np.ndarray) -> list:
        if self.rknn is None:
            raise RuntimeError("RKNN model not loaded")
        img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 640))
        img = np.expand_dims(img, axis=0).astype(np.uint8)
        outputs = self.rknn.inference(inputs=[img])
        return outputs

    def release(self) -> None:
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None
