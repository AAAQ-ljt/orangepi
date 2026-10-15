"""帧率口径：把"15fps 现场口径"的增益/窗口换算到**实测帧率**（别在任何地方写死帧率）。

为什么要有这个模块（2026-10-12）
--------------------------------
下摄的**真实出帧率是变的**：亮环境 ≈30fps（MJPG，`cam_fps_probe.py` 实测 29.8）、
暗环境 AE 拉长曝光掉到 9~16fps（10-12 定案：曝光 33/60/120ms → 30/16.7/8.3 三档台阶）。
而循迹的增益、滤波窗口、防抖帧数都是 **2026-09~10 在 15fps 下现场整定的**：

  · 参考实现的 PID **不带 dt**（`Ki·Σe`、`Kd·Δe/帧`）：帧率一变，每秒积分动作与等效
    连续阻尼跟着变 —— 15→30fps 是"阻尼减半 + 积分动作翻倍"，手感直接换一个人；
  · 滤波器窗口、防抖帧数按**帧**计数：帧率翻倍 → 秒级语义减半（0.27s 的"拒收防抖"变 0.13s）。

所以全项目约定（唯一口径，别再各写各的）：

  ★ **所有登记数字都是 REF_FPS = 15fps 口径**（现场 A/B 表照旧写这套数字），
    运行时按实测帧率折算，保证"任意帧率下物理行为一致"：
      · 积分：每秒动作不变 → 30fps 下逐帧等效 ki = 登记值 × 15/30 = 0.5 倍；
      · 微分：等效连续阻尼 kd/REF_FPS 不变 → 30fps 下逐帧等效 kd = 登记值 × 2；
      · 窗口/防抖帧数：秒级语义不变 → 30fps 下帧数 ×2（`frames_at()`）。

现场只需要记住一件事：**参数按 15fps 口径给，代码自己折算**。
`--fps N` 可以把口径钉死（离线对照、排障、复现历史会话用）；默认按实测帧率自动走。
"""
from __future__ import annotations

from typing import Optional

REF_FPS = 15.0                      # ★ 登记口径（= 现场整定时的帧率，见 doc/开发文档.md）
FPS_MIN, FPS_MAX = 3.0, 60.0        # 估计器钳位：离谱值不许改控制律
DT_MIN, DT_MAX = 1.0 / 240.0, 0.5   # 单步 dt 参与控制律前先钳位（掉帧/卡顿不炸控制律）
DT_SANE_MAX = 10.0                  # 超过它视为"停顿/时间戳异常"，不参与帧率估计


def frames_at(frames: float, scale: float, lo: int = 1, hi: int = 600) -> int:
    """把"登记口径（15fps）的帧数"换算成当前帧率下的帧数（保持秒级语义）。

    scale = 实测帧率 / REF_FPS（见 `FrameRate.scale`）。结果钳位在 [lo, hi]：
    窗口类参数不该因为帧率估算抽风变成 0 帧或几千帧。
    """
    return int(max(int(lo), min(int(hi), round(float(frames) * float(scale)))))


class FrameRate:
    """实测帧率估计（相邻帧 dt 的指数滑动平均）——控制律 dt 口径的唯一来源。

    用法（运行循环里）::

        rate = FrameRate()
        ...
        fps = rate.update_now(now)          # 每帧喂时间戳
        pid.step(err, rate.dt)              # dt=None（还没样本）时退回逐帧口径
        err_filter.set_fps(fps)             # 窗口按帧率折算（内部幂等，可每帧调）

    设计取舍：
      · **EMA 而不是滑动窗口**：要的是"当前帧率"，不是历史平均；α=0.2 约 5 帧收敛，
        亮暗切换时 ~0.3s 内跟上，不会因为一次掉帧就大幅改增益；
      · **异常 dt 不入队**：<=1ms（时间戳精度问题）、>10s（停顿/休眠）直接忽略，
        否则会把增益瞬间拉偏；
      · `dt` 属性给出**已经钳位**的 dt（[1/240, 0.5]s）：真掉帧时按 0.5s 算，
        而不是让积分项一次性吃下 5 秒的误差。
    """

    def __init__(self, alpha: float = 0.2, default: float = REF_FPS) -> None:
        self.alpha = float(alpha)
        self.default = float(default)
        self.pinned: Optional[float] = None
        self._dt: Optional[float] = None
        self._last_t: Optional[float] = None
        self.n = 0                  # 参与估计的样本数（0 = 还没样本 → 用 default）

    def reset(self) -> None:
        self._dt = None
        self._last_t = None
        self.n = 0

    def pin(self, fps: float) -> float:
        """钉死帧率口径（离线对照/排障/复现历史会话）。返回钉住的 fps。"""
        self.pinned = float(max(FPS_MIN, min(FPS_MAX, float(fps))))
        return self.fps

    def update_now(self, now: float) -> float:
        """喂一个绝对时间戳（秒）。第一帧只记时间，不产生样本。返回当前 fps 估计。"""
        now = float(now)
        if self._last_t is not None:
            self.update_dt(now - self._last_t)
        self._last_t = now
        return self.fps

    def update_dt(self, dt: float) -> float:
        """喂一帧的 dt（秒）。异常 dt 不参与估计。返回当前 fps 估计。"""
        d = float(dt)
        if not (d > 1e-3) or not (d < DT_SANE_MAX):
            return self.fps
        d = max(DT_MIN, min(DT_MAX, d))
        self._dt = d if self._dt is None else (self.alpha * d + (1.0 - self.alpha) * self._dt)
        self.n += 1
        return self.fps

    @property
    def fps(self) -> float:
        if self.pinned is not None:
            return self.pinned
        if self._dt is None or self._dt <= 0:
            return self.default
        return float(max(FPS_MIN, min(FPS_MAX, 1.0 / self._dt)))

    @property
    def dt(self) -> Optional[float]:
        """当前参与控制律的 dt（秒）；None = 还没有样本（调用方退回逐帧口径）。"""
        if self.pinned is not None:
            return 1.0 / self.pinned
        return self._dt

    @property
    def scale(self) -> float:
        """相对登记口径的倍率（30fps → 2.0）：`frames_at()` 用的就是它。"""
        return self.fps / REF_FPS
