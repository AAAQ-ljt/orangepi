"""误差滤波：中位数离群剔除 + 越新权重越大的加权平均。

移植自官方上届 `yolo版本…/HardWare/ErrorFilter.h`（那个包里质量最高的几个小模块之一）：
- 先用历史窗口中位数判断当前值是否为离群点，是则**直接返回上次有效值且不入队**；
- 否则入队，并按"越新权重越大"做加权平均。

比一阶低通更适合处理视觉偶发跳变（跳变被整段丢弃，而不是被平滑进控制量）。
"""
from __future__ import annotations

from collections import deque
from typing import Deque, Optional

import numpy as np

from config import settings


class ErrorFilter:
    """滑动窗口误差滤波器（有状态）。"""

    def __init__(self, window: int = None, outlier: float = None):
        self.window = int(settings.ERROR_FILTER_WINDOW if window is None else window)
        self.outlier = float(settings.ERROR_FILTER_OUTLIER if outlier is None else outlier)
        self._history: Deque[float] = deque(maxlen=self.window)
        self._last: Optional[float] = None

    def reset(self) -> None:
        """切换状态 / 重新起步时必须调用（官方漏了这条，代价是切状态时输出踢腿）。"""
        self._history.clear()
        self._last = None

    @property
    def last(self) -> Optional[float]:
        return self._last

    def update(self, value: float) -> float:
        value = float(value)
        if not np.isfinite(value):
            return self._last if self._last is not None else 0.0

        if len(self._history) >= 2:
            median = float(np.median(self._history))
            if abs(value - median) > self.outlier:
                # 离群：不入队，沿用上次有效值
                return self._last if self._last is not None else median

        self._history.append(value)
        errors = np.asarray(self._history, dtype=float)
        weights = np.arange(1, errors.size + 1, dtype=float)   # 越新权重越大
        self._last = float(np.sum(errors * weights) / np.sum(weights))
        return self._last
