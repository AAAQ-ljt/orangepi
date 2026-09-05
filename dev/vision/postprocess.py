"""视觉后处理骨架：将模型输出转为 VisionMessage。"""
from __future__ import annotations

from typing import Any, Dict

from common.protocol import VisionMessage


def postprocess(raw: Dict[str, Any], timestamp: float) -> VisionMessage:
    # TODO: 根据模型输出提取车道线、锥桶、红绿灯、斑马线、挡板、停车区
    return VisionMessage(timestamp=timestamp)
