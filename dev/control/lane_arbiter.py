"""车道仲裁层（主方案 §3.1.4）。

两路观测量（扫线通道 A / 分割通道 C）输出同构的 `center_x + confidence`，
本层按置信度统一调度，决定"用谁 / 降油门 / 靠上一帧维持 / 要求停车"：

| 条件 | 动作 |
|---|---|
| A 置信度正常 | 采用 A.center_x |
| A 连续 N 帧低置信度 | 降油门（throttle_scale），改用最近一次有效值维持 |
| 维持超过 HOLD_S | 标记 degraded，继续维持 |
| 超过 STOP_S 仍无有效信息 | should_stop=True（上层电调归零） |

说明：主方案里的"IMU 航向短时维持"要等 IMU 真正接上（`hardware/imu.py` 目前是空壳，
`read_yaw()` 恒返回 0）再做，**现在不做假实现**。

本模块只依赖两个浮点数，不 import 视觉层的数据类（分层见 AGENTS.md §5.1）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from config import settings


@dataclass
class ArbiterOutput:
    center_x: float
    confidence: float
    source: str            # "scan"（用本帧）/ "hold"（用历史值维持）/ "none"（从未有过有效值）
    throttle_scale: float
    degraded: bool         # 是否处于降级（低置信度）状态
    should_stop: bool      # 是否要求停车（长时间无有效信息）
    held_s: float = 0.0    # 已维持时长（秒）


class LaneArbiter:
    def __init__(self,
                 conf_thresh: float = None,
                 low_conf_frames: int = None,
                 hold_s: float = None,
                 stop_s: float = None,
                 hold_throttle_scale: float = None,
                 target_x: float = None):
        self.conf_thresh = float(settings.ARBITER_CONF_THRESH
                                 if conf_thresh is None else conf_thresh)
        self.low_conf_frames = int(settings.ARBITER_LOW_CONF_FRAMES
                                   if low_conf_frames is None else low_conf_frames)
        self.hold_s = float(settings.ARBITER_HOLD_S if hold_s is None else hold_s)
        self.stop_s = float(settings.ARBITER_STOP_S if stop_s is None else stop_s)
        self.hold_throttle_scale = float(settings.ARBITER_HOLD_THROTTLE_SCALE
                                         if hold_throttle_scale is None else hold_throttle_scale)
        self.target_x = float(settings.TARGET_X if target_x is None else target_x)
        self.reset()

    def reset(self) -> None:
        self._low_frames = 0
        self._last_center: Optional[float] = None
        self._last_conf = 0.0
        self._last_valid_t: Optional[float] = None

    def update(self, center_x: Optional[float], confidence: Optional[float],
               now: float) -> ArbiterOutput:
        conf = 0.0 if confidence is None else float(confidence)
        has_value = center_x is not None

        if has_value and conf >= self.conf_thresh:
            self._low_frames = 0
            self._last_center = float(center_x)
            self._last_conf = conf
            self._last_valid_t = now
            return ArbiterOutput(center_x=float(center_x), confidence=conf, source="scan",
                                 throttle_scale=1.0, degraded=False, should_stop=False)

        # 低置信度：连续 N 帧才真正降级（防抖）
        self._low_frames += 1
        if self._last_center is None:
            # 从未有过有效值：直接要求停车（没有任何可用的横向参考）
            return ArbiterOutput(center_x=self.target_x, confidence=0.0, source="none",
                                 throttle_scale=0.0, degraded=True, should_stop=True)

        held_s = 0.0 if self._last_valid_t is None else max(0.0, now - self._last_valid_t)
        degraded = self._low_frames >= self.low_conf_frames
        should_stop = held_s > self.stop_s

        scale = self.hold_throttle_scale if degraded else 1.0
        if should_stop:
            scale = 0.0
        return ArbiterOutput(center_x=self._last_center, confidence=self._last_conf,
                             source="hold",
                             throttle_scale=scale, degraded=degraded, should_stop=should_stop,
                             held_s=held_s)
