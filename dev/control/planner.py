"""决策规划骨架：根据视觉消息生成控制指令。"""
from __future__ import annotations

from common.protocol import VisionMessage


class Planner:
    def plan(self, vision: VisionMessage):
        # TODO: 根据视觉结果输出期望转向/油门
        return {"steering": 0.0, "throttle": 0.0}
