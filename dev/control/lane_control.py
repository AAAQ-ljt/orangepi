"""循迹控制层：参考实现版 PID + 自适应速度 + "丢线阶梯"。

出处
----
oldCode/src/control/control.cpp::Control_FollowTrail()   控制链路（本模块主体）
    pid = kp*e + ki*∫e + kd*Δe      （kp=0.15, ki=0.01, kd=0.12，误差用**像素**）
    angle = 90 − pid，限幅 ±15°
    angle = 0.7*新 + 0.3*旧          （输出平滑，抑制舵机抖动）
    |e| < 5 加速 / < 15 正常 / 否则减速（自适应速度）
oldCode/src/config/config.cpp                            kp/kd/min_angle/max_angle 的出处
oldCode/src/state_machine/*.cpp                          "看到再决策"的防抖思路（3 帧投票）

为什么和参考实现不完全一样（改动都在《循迹脚本交接.md》§4.4 登记）
----------------------------------------------------------------
1. **减速档有死区下限**：我们的电调 1545us 才起转（参考那台车的标定未知），
   基准 1560 时照抄"−20"会算出 1540 —— 那不是"减速"，是"停车"；
2. **默认不提速**（参考是 +15）：用户 2026-09-20 实测 1575us 太快；
3. **转向符号可配**（`STEER_SIGN`）：参考写的 `90 − pid` 是它那台车的方向，
   我们这边哪边是左是右由 `site.yaml` 的 `steer_sign` 决定（已实测）；
4. **丢线阶梯**（本模块新增）：参考实现"单侧就代边界、两侧都没有就继续用上一帧"，
   没有任何超时兜底。我们照《具体实施方案》§3.1.4 的仲裁层设计补上：
   有读数 → 跟线；丢线 0.3~0.8s → 沿用上一次有效中心（误差按时间衰减回中位）；
   丢线超过 1~1.5s → 电调归零停车。宁可停，也不要"带着垃圾读数继续跑"。
5. **帧率不再锁死 15**（2026-10-12）：`PidRef` 的积分/微分/平滑全部按实测 dt 折算
   （登记口径仍是 15fps，见 `control/rate.py`）；三个护栏的"样本数门槛"改成
   **时间跨度门槛** —— 帧率翻倍时旧写法等于"观测 0.13s 就敢判画龙/跑偏"。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from config import settings
from control.rate import DT_MAX, DT_MIN, REF_FPS

# ---------------------------------------------------------------- 控制默认值
# ★ 参考实现的标定基准：oldCode 的画面是 **320×240**（`launch.cpp`:
#   `capture.set(CAP_PROP_FRAME_WIDTH, 320)`），kp/kd 是**按 320 宽的像素误差**整定的。
#   我们的误差是 640 宽的像素 —— 同一个物理横偏在我们这儿像素数是它的 2 倍，
#   所以直接抄数字 = 等效增益翻倍。`gains_for()` 负责把这件事说清楚，别在代码里偷偷混用。
REF_GAIN_WIDTH = 320
KP_REF, KI_REF, KD_REF = 0.15, 0.01, 0.12   # oldCode/src/config/config.cpp（320 宽口径）
KP0, KI0, KD0 = KP_REF, KI_REF, KD_REF      # 默认仍按参考原数字给（见 gains_for 的 mode）
INTEGRAL_LIMIT = 50.0                # 参考实现：max_integral = 50
# ★ 微分项输入（Δerror/帧）钳位：参考实现没有，但 2026-09-23 操场实测必须加 ——
#   检测器弱对帧能把误差一帧跳 50~140px（p90=53px），kd=0.12 会把 100px 跳变放大成 12° 满舵，
#   车被自己的读数甩动。正常跟线 Δe p50=9px，所以 ±20px 的钳位不影响真运动。
DERIV_CLAMP_PX = 20.0
LIMIT_DEG0 = 15.0                    # 参考实现：angle_limit = 15
SMOOTH0 = 0.7                        # 参考实现：smooth_factor = 0.7
SPEED_FAST_US, SPEED_SLOW_US = 0.0, -10.0   # 参考是 +200/-300（它自己的 PWM 量纲）
SPEED_FLOOR_MARGIN_US = 5.0          # 减速/维持档下限 = 死区 + 这个余量
ERR_FAST_PX, ERR_SLOW_PX = 5.0, 15.0  # 参考实现的自适应速度分档
NEUTRAL_US = float(settings.ESC_STOP_US)


def gains_for(width: int, mode: str = "literal") -> Tuple[float, float, float]:
    """把参考实现的 PID 增益换成"我们这张画面"口径的**登记值**（15fps 口径）。

    mode:
      literal —— 直接抄参考数字（0.15/0.01/0.12）；
      width   —— 按画面宽度等比换算（320/width，kp）。
    ★ 这里返回的是 **15fps 登记口径**的逐帧增益（= 2026-09/10 现场整定的那套数字，
      现场 A/B 表照旧写它）；**帧率折算不在这里做** —— `PidRef.step(e, dt)` 按实测
      dt 自动折算（见 `control/rate.py` 的说明与 `PidRef` docstring），所以：
        15fps：ki=0.02、kd=0.12（就是下面这两个数字）
        30fps：逐帧等效自动变成 ki=0.01、kd=0.24（**阻尼/积分动作的物理量不变**）
      旧版把 15fps 的 ×2 硬写在 ki 里，等于把帧率锁死在 15 —— 2026-10-12 帧率放开后
      撤掉（`doc/开发文档.md` 有这次修正的账）。
      kp：与帧率无关，两种帧率下都是 0.15。
    """
    if mode == "width" and width > 0:
        scale = float(REF_GAIN_WIDTH) / float(width)
        return (KP_REF * scale, KI_REF * scale * 2.0, KD_REF * scale)
    return KP_REF, KI_REF * 2.0, KD_REF


class PidRef:
    """参考实现的 PID（像素误差口径，输出舵机**角度**，带输出平滑）。

    ★ 帧率口径（2026-10-12 重做，本类最要紧的一件事）
    -------------------------------------------------
    参考实现**没有 dt**：积分是"每帧累加"（`Ki·Σe`）、微分是"每帧差"（`Kd·Δe`）。
    它的行为只在"控制频率固定"时才自洽 —— 而我们的下摄帧率**是会变的**
    （亮环境 ≈30fps、暗环境 9~16fps，见 `doc/开发文档.md` 的帧率结论）。
    所以本类提供两条路径：

      · `step(e)`       —— **登记口径逐帧**（不带 dt）：与老代码逐位一致，
                           留给 parking_test 这类老调用方，行为随帧率漂（老问题）；
      · `step(e, dt)`   —— **按实测 dt 折算**：把 15fps 登记口径折成连续时间等价形式，
                           任意帧率下物理行为一致：
           积分项 = ki × REF_FPS × ∫e·dt     （每秒积分动作不变）
           微分项 = kd / REF_FPS × (de/dt)   （等效连续阻尼 kd/REF_FPS 不变）
           Δe 钳位按 px/s（= 登记口径的 ±DERIV_CLAMP_PX/帧）、平滑系数按时间常数折算。

      两条路径在 dt = 1/REF_FPS 时**数学等价**（`tests/test_lane_rate.py` 有用例钉住），
      差别只有一处：dt 路径在 `reset()` 后的第一帧不产生微分（老逐帧路径会把
      "上一帧误差 = 0"当真、给一记假踢腿 —— 与 `prime()` 要治的是同一个病）。

    ⚠️ 别在同一个实例上混用两种调用（两条路径各留各的"上一帧误差"与积分历史）。
    """

    def __init__(self, kp: float = KP0, ki: float = KI0, kd: float = KD0,
                 limit_deg: float = LIMIT_DEG0, smooth: float = SMOOTH0,
                 sign: float = 1.0, center: float = 90.0,
                 ref_fps: float = REF_FPS) -> None:
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.limit_deg = float(limit_deg)
        self.smooth = float(smooth)
        self.sign = float(sign)      # +1：角度增大=右转（与 settings.STEER_SIGN 同口径）
        # ★ 中位不再写死 90：本项目舵机 90°≠机械直行（2026-09-23 操场实测：固定 90°
        #   车自己绕大弯）。循迹必须以 settings.SERVO_CENTER_ANGLE 为中位。
        self.center = float(center)
        self.ref_fps = float(ref_fps) or REF_FPS   # 登记口径（现场整定时的帧率）
        self.integral = 0.0
        self.last_error = 0.0
        self.last_angle: Optional[float] = None
        self.last_dt: Optional[float] = None       # 最近一次参与控制律的 dt（日志用）
        self._prev_dt_error: Optional[float] = None   # dt 路径专用的"上一帧误差"

    # ------------------------------------------------------------ dt 口径折算
    @property
    def ki_per_s(self) -> float:
        """连续时间口径的积分增益（登记值 × REF_FPS）。"""
        return self.ki * self.ref_fps

    @property
    def kd_per_s(self) -> float:
        """连续时间口径的微分增益（登记值 / REF_FPS）。"""
        return self.kd / self.ref_fps

    @property
    def deriv_clamp_per_s(self) -> float:
        """微分输入的速率钳位（px/s）：等价于登记口径的 ±DERIV_CLAMP_PX / 帧。"""
        return DERIV_CLAMP_PX * self.ref_fps

    @property
    def integral_limit_s(self) -> float:
        """积分钳位（px·s）：等价于登记口径的 ±INTEGRAL_LIMIT（逐帧累加）。"""
        return INTEGRAL_LIMIT / self.ref_fps

    def frame_equivalent(self, fps: float) -> Tuple[float, float]:
        """给日志/现场用：在 fps 帧率下本实例的**逐帧等效** ki/kd。

        （15fps → 就是登记值本身；30fps → ki 减半、kd 加倍。现场 A/B 表按登记口径写，
        这一对数只是让人一眼看懂"30fps 下代码实际在用多少"。）
        """
        f = max(1e-6, float(fps))
        return (self.ki * self.ref_fps / f, self.kd * f / self.ref_fps)

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0
        self.last_angle = None
        self.last_dt = None
        self._prev_dt_error = None

    def prime(self, error: float) -> None:
        """把上一帧误差预置为当前值：下一帧的微分项从 0 起跳。

        使用场景（10-11 修）：`--single-hold` 每帧丢线帧都 reset、恢复双侧帧时
        `last_error=0` → 微分 = error−0 = 全量误差，kd 大时是一记假"踢腿"。
        """
        self.last_error = float(error)
        self._prev_dt_error = float(error)

    # ------------------------------------------------------------ 控制律
    def step(self, error: float, dt: Optional[float] = None) -> float:
        """一步控制律。`dt` 不给 = 登记口径逐帧（兼容老调用方）；给 dt = 按实际帧率折算。

        单位：`error` 是**像素**（640 宽画面口径）；返回舵机角度（度）。
        """
        error = float(error)
        if dt is None:
            return self._step_frame(error)
        return self._step_dt(error, float(dt))

    def _step_frame(self, error: float) -> float:
        """登记口径逐帧（= 老行为，逐位不变）。"""
        self.integral += error
        self.integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, self.integral))
        delta = max(-DERIV_CLAMP_PX, min(DERIV_CLAMP_PX, error - self.last_error))
        pid = self.kp * error + self.ki * self.integral + self.kd * delta
        self.last_error = error
        # 参考实现是 `90 - pid`；这里接上 settings.STEER_SIGN 让它和 planner 同口径：
        #   sign=+1（角度增大=右转）→ 90 + pid；sign=-1 → 90 - pid（= 参考实现原式）
        angle = self.center + self.sign * pid
        angle = max(self.center - self.limit_deg, min(self.center + self.limit_deg, angle))
        if self.last_angle is None:
            self.last_angle = angle
        else:                                                # 输出平滑，抑制舵机抖动
            angle = self.smooth * angle + (1.0 - self.smooth) * self.last_angle
            self.last_angle = angle
        return angle

    def _step_dt(self, error: float, dt: float) -> float:
        """按实测 dt 折算的控制律（任意帧率下与 15fps 登记口径物理等价）。"""
        if not (dt > 0.0) or dt != dt:             # dt=0/NaN（时间戳坏了）→ 退回逐帧，别除零
            return self._step_frame(error)
        d = max(DT_MIN, min(DT_MAX, dt))
        self.last_dt = d

        # 积分（连续时间口径 + 钳位）：15fps 的 ki=0.02 在这里自动等于"每秒 0.3"
        self.integral += error * d
        lim = self.integral_limit_s
        self.integral = max(-lim, min(lim, self.integral))

        # 微分：先钳"速率"（px/s）再乘增益 —— 与逐帧口径的 ±20px/帧 等价
        if self._prev_dt_error is None:
            rate = 0.0                              # reset 后第一帧不踢腿（见类 docstring）
        else:
            rate = (error - self._prev_dt_error) / d
        rate = max(-self.deriv_clamp_per_s, min(self.deriv_clamp_per_s, rate))
        self._prev_dt_error = error

        pid = self.kp * error + self.ki_per_s * self.integral + self.kd_per_s * rate
        angle = self.center + self.sign * pid
        angle = max(self.center - self.limit_deg, min(self.center + self.limit_deg, angle))
        if self.last_angle is None:
            self.last_angle = angle
        else:
            alpha = self._smooth_alpha(d)
            angle = alpha * angle + (1.0 - alpha) * self.last_angle
            self.last_angle = angle
        return angle

    def _smooth_alpha(self, dt: float) -> float:
        """把"每帧平滑系数"折成任意 dt 下等效的系数（时间常数不变）。

        登记口径下 `smooth` 是"每 1/15s 一步"的 EMA；dt 变成 1/30s 时用
        `1−(1−smooth)^(dt·REF_FPS)` = 1−√0.3 ≈ 0.452，平滑时间常数才不变。
        """
        a = float(self.smooth)
        if a >= 1.0:
            return 1.0
        a = max(1e-6, a)
        n = max(1e-6, float(dt) * self.ref_fps)
        return float(1.0 - (1.0 - a) ** n)


def adaptive_pulse(error: float, base_us: float,
                   fast: float = SPEED_FAST_US, slow: float = SPEED_SLOW_US) -> float:
    """参考实现的自适应速度：误差小加速、误差大减速（单位 us）。

    减速档有硬下限（死区 + 余量）：基准 1560 时 1560-20=1540 已低于死区 1545，
    那样"减速"会变成"停车"（用户 2026-09-20 把巡线速度降到 1560 后才出现这个风险）。
    """
    e = abs(float(error))
    if e < ERR_FAST_PX:
        us = base_us + fast
    elif e < ERR_SLOW_PX:
        us = base_us
    else:
        us = base_us + slow
    return max(us, float(settings.ESC_DEADBAND_US) + SPEED_FLOOR_MARGIN_US)


def floor_pulse(base_us: float, slow: float = SPEED_SLOW_US) -> float:
    """"减速档"的脉宽（维持/大误差时用），同样带死区下限。"""
    return max(float(base_us) + float(slow),
               float(settings.ESC_DEADBAND_US) + SPEED_FLOOR_MARGIN_US)


@dataclass
class LossState:
    """丢线阶梯的当前状态。"""
    phase: str          # fresh（本帧有有效读数）/ hold（沿用上次）/ lost（超时）
    since_s: float      # 距最后一次有效读数的秒数（fresh 时为 0）
    decay: float        # 维持期间的误差衰减系数：1 → 0（越久越回中位）

    @property
    def driving(self) -> bool:
        return self.phase != "lost"


class LossGuard:
    """丢线阶梯（《具体实施方案》§3.1.4 的落地，可单测）。

    规则：
      · 本帧有有效读数（质量达标）→ fresh，计时清零；
      · 没有 → 距上次有效 < `hold_s`：hold，误差按 `1 − t/hold_s` 衰减（舵角慢慢回中位）；
      · 距上次有效 ≥ `hold_s` → lost（调用方应停车）。
    `never_valid` 用于起步瞬间：还没见过一次线时，调用方按"探路（acquire）"处理。
    """

    def __init__(self, hold_s: float = 0.6) -> None:
        self.hold_s = float(hold_s)
        self._last_valid_t: Optional[float] = None
        self._ever_valid = False

    def reset(self) -> None:
        self._last_valid_t = None
        self._ever_valid = False

    @property
    def ever_valid(self) -> bool:
        return self._ever_valid

    def update(self, now: float, valid: bool) -> LossState:
        if valid:
            self._last_valid_t = float(now)
            self._ever_valid = True
            return LossState("fresh", 0.0, 1.0)
        if self._last_valid_t is None:
            return LossState("hold", float("inf"), 0.0)
        since = max(0.0, float(now) - self._last_valid_t)
        if since < self.hold_s:
            return LossState("hold", since, max(0.0, 1.0 - since / max(1e-6, self.hold_s)))
        return LossState("lost", since, 0.0)


class OscillationGuard:
    """画龙（S 弯）检测：舵角在窗口内来回打满幅的次数太多 = 控制失稳。

    2026-09-23 操场实测：误差 ±100~300 疯狂翻转，舵机 75°↔105°（限幅）来回甩，
    跑偏保护（连续计时）凑不满 → 车 S 弯直到冲出去。用"舵角大翻转次数"直接抓画龙：
    每次从"大角度一侧"翻到"另一侧"记一次，窗口内 ≥ flips 次就判失控停车。

    ★ 2026-10-12：判据前的"样本够不够"从**帧数**（原来 ≥4 帧）改成**时间跨度**
    （≥ 窗口的 1/4 = 0.5s）。帧数门槛在帧率翻倍时等于"只看了 0.13s 就敢判画龙"，
    而现在下摄帧率是 9~30 会变的（见 `control/rate.py`）。
    """

    MIN_SPAN_FRAC = 0.25      # 判定前至少要观测到窗口的这个比例（秒级语义，与帧率无关）

    def __init__(self, window_s: float = 2.0, flips: int = 3, amp_deg: float = 12.0) -> None:
        self.window_s = float(window_s)
        self.flips = int(flips)
        self.amp_deg = float(amp_deg)     # |angle-90| 超过它才算"大幅"
        self._recent: list = []           # [(t, angle)]

    def reset(self) -> None:
        self._recent.clear()

    def update(self, now: float, angle: float, center: float = 90.0) -> bool:
        """返回 True = 判定画龙失控，要求停车（幅度相对舵机中位）。"""
        cutoff = float(now) - self.window_s
        self._recent = [(t, a) for t, a in self._recent if t >= cutoff]
        self._recent.append((float(now), float(angle)))
        if len(self._recent) < 4:
            return False
        if self._recent[-1][0] - self._recent[0][0] < self.MIN_SPAN_FRAC * self.window_s:
            return False          # 样本够但时间跨度不足 → 别急着判（防高帧率误触发）
        # 数"完整来回"：符号翻转且幅度 ≥ amp → 每次翻符号记一次翻转
        signs = []
        for _t, a in self._recent:
            d = a - float(center)
            if abs(d) >= self.amp_deg:
                signs.append(1 if d > 0 else -1)
        flips_n = sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])
        return flips_n >= self.flips


class BigErrorGuard:
    """跑偏保护：误差**持续或高占比**很大 → 判定"锁错线/失控"，要求停车。

    参考实现没有这一层。2026-09-21 实验室实测：误差恒在 −33px、舵机一直往左打到
    冲出跑道，226 帧里误差从没回到 0 附近 —— 没有兜底就会一直跑下去。

    ★ 2026-10-12：振荡判据的"样本够不够"由帧数（原来 ≥6 帧）改成**时间跨度**
    （≥ 统计窗口的一半）。0.6 这个占比本身与帧率无关，但"6 帧"在不同帧率下
    对应 0.67s（9fps）~0.2s（30fps）—— 高帧率下等于没攒够证据就判，见 `control/rate.py`。
    """

    TALLY_SPAN_FRAC = 0.5     # 振荡判据要观测到统计窗口的这个比例（秒级语义）

    def __init__(self, err_px: float = 60.0, hold_s: float = 1.5,
                 frac_window_s: float = 1.5, frac: float = 0.6) -> None:
        self.err_px = float(err_px)
        self.hold_s = float(hold_s)
        self.frac_window_s = float(frac_window_s)   # 振荡判据的统计窗口
        self.frac = float(frac)                     # 窗口内大误差帧占比超过它 → 触发
        self._since: Optional[float] = None
        self._recent: list = []                     # [(t, big?)] 滚动

    def reset(self) -> None:
        self._since = None
        self._recent.clear()

    def update(self, now: float, error: Optional[float],
               raw_error: Optional[float] = None) -> bool:
        """返回 True = 本次触发（调用方应立刻停车）。

        `error` 是送进控制器的（滤波后）值，`raw_error` 是**原始**误差。
        ★ 两个都要看：2026-09-22 落地实测发现只盯滤波值会被"滤波冻结"骗过去——
        原始误差连续 4s 在 ≥40px，而滤波后一直卡在 30px 以下，跑偏保护形同虚设。
        持续时间足够长（hold_s)本身就能排除单帧毛刺，所以取两者里的大者。

        ★ 2026-09-23 操场实测补"振荡判据"：S 弯时误差 ±100~300 疯狂翻符号，
        连续 1.5s ≥阈值永远凑不满 → 保护从不触发。窗口内大误差帧**占比**超阈值
        也触发（0.6 = 窗口里六成的观测都是大误差，必是失控不是转弯）。
        """
        if self.hold_s <= 0:
            return False
        vals = [abs(float(v)) for v in (error, raw_error) if v is not None]
        big = bool(vals) and max(vals) >= self.err_px
        if not big:
            self._since = None
        else:
            if self._since is None:
                self._since = float(now)
            if (float(now) - self._since) >= self.hold_s:
                return True
        # 振荡判据：滚动窗口（要攒够**时间**证据，不只是样本数）
        cutoff = float(now) - self.frac_window_s
        self._recent = [(t, b) for t, b in self._recent if t >= cutoff]
        self._recent.append((float(now), big))
        span = (self._recent[-1][0] - self._recent[0][0]) if len(self._recent) >= 2 else 0.0
        if self.frac_window_s > 0 and len(self._recent) >= 4 \
                and span >= self.TALLY_SPAN_FRAC * self.frac_window_s:
            frac_big = sum(1 for _, b in self._recent if b) / float(len(self._recent))
            if frac_big >= self.frac:
                return True
        return False
