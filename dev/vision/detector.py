"""YOLO/RKNN 检测器骨架。"""
from __future__ import annotations

from typing import Any, Dict, Optional


class Detector:
    def __init__(self, model_path: Optional[str] = None):
        self.model_path = model_path
        self.model = None

    def load(self) -> None:
        # TODO: 加载 RKNN / YOLO 模型
        raise NotImplementedError("Detector.load() 待实现")

    def infer(self, frame) -> Dict[str, Any]:
        # TODO: 推理并返回后处理所需的原始结果
        raise NotImplementedError("Detector.infer() 待实现")
