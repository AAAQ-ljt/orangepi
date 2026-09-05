"""UDP JSON 消息协议：视觉 -> 控制。

同时兼容上一届 YOLO 视觉端字段和 2026 新增字段。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class PerceptionMessage:
    """统一感知消息。"""

    timestamp: float = 0.0
    left_x: Optional[int] = None
    right_x: Optional[int] = None
    center_x: Optional[float] = None
    blue_cone_count: int = 0
    yellow_cone_count: int = 0
    has_left_sign: bool = False
    has_right_sign: bool = False
    has_sign_a: bool = False
    has_sign_b: bool = False
    has_zebra_crossing: bool = False
    is_barrier: bool = False
    traffic_light_state: Optional[str] = None
    raw: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PerceptionMessage":
        """从 UDP JSON dict 构造，兼容缺失字段。"""

        def _int(key: str, default=None):
            value = data.get(key, default)
            try:
                return int(value)
            except (TypeError, ValueError):
                return default

        def _float(key: str, default=None):
            value = data.get(key, default)
            try:
                return float(value)
            except (TypeError, ValueError):
                return default

        def _bool(key: str, default=False) -> bool:
            value = data.get(key, default)
            if isinstance(value, bool):
                return value
            if isinstance(value, str):
                return value.strip().lower() in ("true", "1", "yes", "on")
            return bool(value)

        left_x = _int("left_x")
        right_x = _int("right_x")
        center_x = _float("center_x")
        if center_x is None and left_x is not None and right_x is not None:
            center_x = (left_x + right_x) / 2.0

        traffic = data.get("traffic_light_state") or data.get("traffic_light")
        if isinstance(traffic, str):
            traffic = traffic.strip().lower()

        return cls(
            timestamp=_float("timestamp", 0.0),
            left_x=left_x,
            right_x=right_x,
            center_x=center_x,
            blue_cone_count=_int("blue_cone_count", 0),
            yellow_cone_count=_int("yellow_cone_count", 0),
            has_left_sign=_bool("has_left_sign"),
            has_right_sign=_bool("has_right_sign"),
            has_sign_a=_bool("has_sign_a"),
            has_sign_b=_bool("has_sign_b"),
            has_zebra_crossing=_bool("has_zebra_crossing", False) or _bool("is_rxd_flag", False),
            is_barrier=_bool("is_barrier", False) or _bool("is_board", False),
            traffic_light_state=traffic,
            raw=dict(data),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "left_x": self.left_x,
            "right_x": self.right_x,
            "center_x": self.center_x,
            "blue_cone_count": self.blue_cone_count,
            "yellow_cone_count": self.yellow_cone_count,
            "has_left_sign": self.has_left_sign,
            "has_right_sign": self.has_right_sign,
            "has_sign_a": self.has_sign_a,
            "has_sign_b": self.has_sign_b,
            "has_zebra_crossing": self.has_zebra_crossing,
            "is_barrier": self.is_barrier,
            "traffic_light_state": self.traffic_light_state,
        }
