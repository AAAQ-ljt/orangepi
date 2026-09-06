"""RKNN YOLO11 分割检测器（小车端，使用 NPU AUTO 多核）。"""
from __future__ import annotations

import cv2
import numpy as np
from rknnlite.api import RKNNLite


class RKNNYoloSeg:
    def __init__(self, model_path: str, img_size: int = 640):
        self.model_path = model_path
        self.img_size = img_size
        self.rknn = None

    def load(self) -> bool:
        self.rknn = RKNNLite()
        if self.rknn.load_rknn(self.model_path) != 0:
            return False
        # NPU_AUTO：由运行时自动调度 NPU 核心
        if self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO) != 0:
            return False
        return True

    def preprocess(self, frame_bgr: np.ndarray) -> np.ndarray:
        img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (self.img_size, self.img_size))
        img = np.expand_dims(img, axis=0).astype(np.uint8)
        return img

    def infer(self, img: np.ndarray) -> list:
        if self.rknn is None:
            raise RuntimeError("RKNN model not loaded")
        return self.rknn.inference(inputs=[img])

    def release(self) -> None:
        if self.rknn is not None:
            self.rknn.release()
            self.rknn = None
