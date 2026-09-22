"""误差滤波：中位数离群剔除 + 越新权重越大的加权平均。

移植自官方上届 `yolo版本…/HardWare/ErrorFilter.h`（那个包里质量最高的几个小模块之一）：
- 先用历史窗口中位数判断当前值是否为离群点，是则**直接返回上次有效值且不入队**；
- 否则入队，并按"越新权重越大"做加权平均。

比一阶低通更适合处理视觉偶发跳变（跳变被整段丢弃，而不是被平滑进控制量）。

★ 2026-09-22 补的一条（官方没有，代价很大）
------------------------------------------------
原版的"离群就沿用旧值"有个致命边界：**当误差真的阶跃到新水平时，新值永远相对旧中位数是离群点，
于是滤波器会永久冻结**。实验室落地实测（`lane_20260922_171817`）：原始误差连续 1.6s 在
+58~+164px，而滤波后一直卡在 −29.6/+3.0 —— 车实际上偏了十几厘米，控制器却以为只有几像素，
**跑偏保护（读的也是滤波值）也因此不触发**。
修法：连续被拒 `max_rejects` 次就认定"世界真的变了"（不是抖动）→ 清窗口、接受新值。
"""
from __future__ import annotations

from collections import deque
from typing import Deque, Optional

import numpy as np

from config import settings


class ErrorFilter:
    """滑动窗口误差滤波器（有状态）。"""

    def __init__(self, window: int = None, outlier: float = None,
                 max_rejects: int = None):
        self.window = int(settings.ERROR_FILTER_WINDOW if window is None else window)
        self.outlier = float(settings.ERROR_FILTER_OUTLIER if outlier is None else outlier)
        self.max_rejects = int(getattr(settings, "ERROR_FILTER_MAX_REJECTS", 4)
                               if max_rejects is None else max_rejects)
        self._history: Deque[float] = deque(maxlen=self.window)
        self._last: Optional[float] = None
        self._rejects = 0

    def reset(self) -> None:
        """切换状态 / 重新起步时必须调用（官方漏了这条，代价是切状态时输出踢腿）。"""
        self._history.clear()
        self._last = None
        self._rejects = 0

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
                self._rejects += 1
                if self._rejects < max(1, self.max_rejects):
                    # 离群：不入队，沿用上次有效值
                    return self._last if self._last is not None else median
                # ★ 连续被拒到上限 → 不是抖动，是误差真的换了一个level：清窗口接受它
                self._history.clear()
                self._rejects = 0
            else:
                self._rejects = 0

        self._history.append(value)
        errors = np.asarray(self._history, dtype=float)
        weights = np.arange(1, errors.size + 1, dtype=float)   # 越新权重越大
        self._last = float(np.sum(errors * weights) / np.sum(weights))
        return self._last
