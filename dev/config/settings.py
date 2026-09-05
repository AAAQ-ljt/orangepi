"""控制端参数配置。

调试阶段默认把电调最大脉宽限制在 1540us，避免小车速度过快。
比赛前再调整为 full speed。
"""
from __future__ import annotations

# ---- 电调/电机 ----
ESC_STOP_US = 1500      # 停止脉宽
ESC_START_US = 1534     # 实测启动脉宽
ESC_DEBUG_MAX_US = 1540 # 调试阶段最大脉宽（安全限速）
ESC_MAX_US = 2000       # 比赛/全速最大脉宽
ESC_MIN_US = 1400       # 最大倒车脉宽

# ---- 舵机 ----
SERVO_CENTER_ANGLE = 90
SERVO_MIN_ANGLE = 60
SERVO_MAX_ANGLE = 120

# ---- 视觉/控制 ----
TARGET_X = 320.0
UDP_PORT = 5000
