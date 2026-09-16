"""UDP JSON 消息协议：视觉 -> 控制。

契约（改动必须走变更记录，见 AGENTS.md §5.1）：
- 观测量 `center_x / left_x / right_x` 均为**像素**，基于 640 宽画面；
- `lane_confidence` 为本帧扫线置信度 0~1，控制端仲裁层用它决定是否降级；
- `board_blocked` = 画面里检测到发车挡板（发车信号用它做**边沿触发**）；
- `start_released` = 视觉侧边沿检测器给出的"挡板已移开"闩锁（控制端仍会自行去抖复核）；
- 兼容旧字段：`is_barrier` 视同 `board_blocked`，`has_zebra_crossing` 兼容 `is_rxd_flag`。
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
    lane_confidence: Optional[float] = None
    blue_cone_count: int = 0
    yellow_cone_count: int = 0
    has_left_sign: bool = False
    has_right_sign: bool = False
    has_sign_a: bool = False
    has_sign_b: bool = False
    has_zebra_crossing: bool = False
    board_blocked: bool = False
    start_released: bool = False
    traffic_light_state: Optional[str] = None
    raw: Dict[str, Any] = field(default_factory=dict)

    # ---- 兼容旧代码：is_barrier 语义等同 board_blocked ----
    @property
    def is_barrier(self) -> bool:
        return self.board_blocked

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PerceptionMessage":
        """从 UDP JSON dict 构造，兼容缺失字段与 null。"""

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

        board_blocked = _bool("board_blocked", False) or _bool("is_barrier", False)

        confidence = _float("lane_confidence")
        if confidence is None:
            # 没有置信度字段时：有 center_x 就当作可信（旧版视觉端行为）
            confidence = 1.0 if center_x is not None else 0.0

        return cls(
            timestamp=_float("timestamp", 0.0),
            left_x=left_x,
            right_x=right_x,
            center_x=center_x,
            lane_confidence=confidence,
            blue_cone_count=_int("blue_cone_count", 0),
            yellow_cone_count=_int("yellow_cone_count", 0),
            has_left_sign=_bool("has_left_sign"),
            has_right_sign=_bool("has_right_sign"),
            has_sign_a=_bool("has_sign_a"),
            has_sign_b=_bool("has_sign_b"),
            has_zebra_crossing=_bool("has_zebra_crossing", False) or _bool("is_rxd_flag", False),
            board_blocked=board_blocked,
            start_released=_bool("start_released", False),
            traffic_light_state=traffic,
            raw=dict(data),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "left_x": self.left_x,
            "right_x": self.right_x,
            "center_x": self.center_x,
            "lane_confidence": self.lane_confidence,
            "blue_cone_count": self.blue_cone_count,
            "yellow_cone_count": self.yellow_cone_count,
            "has_left_sign": self.has_left_sign,
            "has_right_sign": self.has_right_sign,
            "has_sign_a": self.has_sign_a,
            "has_sign_b": self.has_sign_b,
            "has_zebra_crossing": self.has_zebra_crossing,
            "board_blocked": self.board_blocked,
            "start_released": self.start_released,
            "traffic_light_state": self.traffic_light_state,
        }
