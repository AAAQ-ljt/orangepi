"""UDP JSON 消息协议定义。

vision 进程 -> control 进程 使用该协议通信。
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional

VISION_PORT = 9100          # vision 进程监听/发送端口
CONTROL_PORT = 9101         # control 进程监听端口（预留）

# 消息类型
MSG_VISION = "vision"
MSG_CONTROL = "control"
MSG_HEARTBEAT = "heartbeat"


@dataclass
class VisionMessage:
    """视觉进程发给控制进程的结构化感知结果。"""
    timestamp: float = 0.0
    lane_center: Optional[float] = None      # 归一化中线位置，-1..1
    lane_visible: bool = False
    objects: List[Dict[str, Any]] = field(default_factory=list)
    # objects 示例:
    # {"type": "cone", "x": 0.3, "y": 0.5, "conf": 0.85}
    # {"type": "traffic_light", "state": "red", "conf": 0.9}
    # {"type": "crosswalk", "visible": True}
    # {"type": "blue_barrier", "visible": True}
    # {"type": "parking", "choice": "A"}

    def to_json(self) -> Dict[str, Any]:
        return {"type": MSG_VISION, **asdict(self)}


def decode_message(data: Dict[str, Any]) -> VisionMessage:
    """从 UDP JSON dict 构造 VisionMessage。"""
    fields = {k: v for k, v in data.items() if k != "type"}
    return VisionMessage(**fields)
