#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
import asyncio
import json
import time
import subprocess
from typing import Dict, Any

from pwm_driver import PWMController

app = FastAPI()
pwm = PWMController()

# 全局状态字典，维护车辆当前的所有状态参数
state = {
    "armed": False,          # 解锁状态，True 才能控制动力
    "estop": False,          # 软件急停状态
    "steering": 90,          # 转向当前角度
    "steer_center": 90,      # 动态配置：舵机中值 (新增)
    "throttle": 0,           # 油门当前百分比 (-100 到 100)
    "throttle_us": 1500,     # 当前油门对应的实际脉宽 us
    "throttle_min_us": 1400, # 动态配置：油门后退最大脉宽
    "throttle_center_us": 1500, # 动态配置：油门停止中位脉宽
    "throttle_max_us": 1600, # 动态配置：油门前进最大脉宽
    "pan": 90,               # 云台当前水平角度
    "tilt": 90,              # 云台当前俯仰角度
    "battery_v": 15.6,       # 电池电压 (可外接 ADC 模块获取真实数据，当前为模拟)
    "signal": 78,            # 信号强度模拟值
    "last_cmd_ts": 0.0,      # "看门狗"时间戳：记录最后一次收到前端指令的时间
}

# 失控保护超时时间(秒)：超过 8 秒未收到前端心跳或指令，触发失控保护
FAILSAFE_TIMEOUT = 8.0

# 软件限位设计：防止前端发来的极端数据导致舵机打到底烧毁机械结构
STEERING_MIN = 60
STEERING_MAX = 120
PAN_MIN = 0
PAN_MAX = 180
TILT_MIN = 0
TILT_MAX = 180
THROTTLE_MIN = -100
THROTTLE_MAX = 100

def clamp(v, vmin, vmax):
    """辅助函数：将变量 v 限制在 [vmin, vmax] 范围内"""
    return max(vmin, min(vmax, v))

def configure_throttle_range(min_us: int, center_us: int, max_us: int):
    """更新底层的电调行程配置，并同步更新全局状态"""
    pwm.configure_esc_range(min_us, center_us, max_us)
    state["throttle_min_us"] = int(min_us)
    state["throttle_center_us"] = int(center_us)
    state["throttle_max_us"] = int(max_us)

def configure_steering_center(center: int):
    """更新底层的舵机中值配置，并同步更新全局状态 (新增)"""
    pwm.configure_steering_center(center)
    state["steer_center"] = int(center)

def calc_throttle_us(value: int) -> int:
    """逻辑计算：仅用于计算归一化油门对应的物理脉宽，用来回传给前端做 UI 展示"""
    value = clamp(int(value), THROTTLE_MIN, THROTTLE_MAX)
    center = state["throttle_center_us"]
    if value == 0:
        return center
    if value > 0:
        return int(round(center + (value / 100.0) * (state["throttle_max_us"] - center)))
    return int(round(center + (value / 100.0) * (center - state["throttle_min_us"])))

# ----------------- 硬件执行封装层 -----------------
def set_steering(angle: int):
    # 动态限位：基于调整后的中值左右各浮动 30 度 (替代原来硬编码的 STEERING_MIN 和 STEERING_MAX)
    center = state.get("steer_center", 90)
    vmin = center - 30
    vmax = center + 30
    
    angle = clamp(angle, vmin, vmax)
    
    state["steering"] = angle
    pwm.set_steering_angle(angle)
    print(f"[CTRL] steering -> {angle}", flush=True)

def set_throttle(value: int):
    value = clamp(int(value), THROTTLE_MIN, THROTTLE_MAX)
    state["throttle"] = value
    pwm.set_esc_normalized_percent(value)
    state["throttle_us"] = calc_throttle_us(value)
    print(f"[CTRL] throttle -> normalized={value} pwm={state['throttle_us']}us", flush=True)

def set_gimbal(pan: int, tilt: int):
    pan = clamp(pan, PAN_MIN, PAN_MAX)
    tilt = clamp(tilt, TILT_MIN, TILT_MAX)
    
      # ===================== 云台水平反转（左右反向） =====================
    tilt = 180 - tilt
    # =================================================================
    state["pan"] = pan
    state["tilt"] = tilt
    pwm.set_pan_angle(pan)
    pwm.set_tilt_angle(tilt)
    print(f"[CTRL] gimbal -> pan={pan}, tilt={tilt}", flush=True)
def set_opi_volume(volume: int):
    """调用 Linux 底层 amixer 命令调节香橙派物理音量"""
    volume = clamp(int(volume), 0, 100)
    try:
        # 香橙派默认的声卡主音量通常叫 'Master' 或 'Playback'
        subprocess.run(
            ["amixer", "sset", "Playback", f"{volume}%"], 
            check=True, 
            stdout=subprocess.DEVNULL, 
            stderr=subprocess.DEVNULL
        )
        print(f"[AUDIO] system volume -> {volume}%", flush=True)
    except Exception as e:
        print(f"[AUDIO] failed to set volume (is alsa-utils installed?): {e}", flush=True)    

# ----------------- 核心安全逻辑 -----------------
def arm_system(enabled: bool):
    """
    解锁(Arm)与上锁(Disarm)系统
    就像真车的启动键，未解锁时忽略油门指令。
    """
    if state["estop"] and enabled:
        print("[WARN] estop active, cannot arm", flush=True)
        return  # 处于急停状态，拒绝解锁
    
    # 防窜车保护：解锁瞬间如果油门不归零，拒绝解锁，防止车突然冲出去
    if enabled and state["throttle"] != 0:
        print("[WARN] throttle not zero, cannot arm")
        return False
        
    state["armed"] = enabled
    
    if not enabled:
        set_throttle(0) # 上锁时，无条件将油门切断
    print(f"[CTRL] armed -> {enabled}", flush=True)
    return True

def emergency_stop():
    """触发紧急停止：强制上锁，油门切断，方向回正(动态中点)"""
    state["estop"] = True
    state["armed"] = False
    set_throttle(0)
    set_steering(state.get("steer_center", 90))
    print("[CTRL] EMERGENCY STOP!", flush=True)

def clear_estop():
    """清除急停状态"""
    state["estop"] = False
    print("[CTRL] estop cleared", flush=True)

def apply_failsafe():
    """失控保护动作：一旦网络断开或延时过高触发此动作"""
    set_throttle(0)
    set_steering(state.get("steer_center", 90))
    state["armed"] = False
    print("[SAFE] failsafe triggered", flush=True)

# ----------------- FastAPI 生命周期与路由 -----------------
@app.on_event("shutdown")
def shutdown_event():
    # 程序关闭时，清理硬件引脚
    pwm.cleanup()

@app.get("/health")
async def health():
    """供外部检查系统状态的 HTTP 接口"""
    return JSONResponse({"ok": True, "state": state})

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """
    WebSocket 通信主入口
    用于接收高频控制指令，并下发车辆状态。
    """
    await ws.accept()
    print("[WS] client connected", flush=True)
    # 创建一个异步后台任务，专门用来周期性向前端发送车辆遥测数据
    telemetry_task = asyncio.create_task(send_telemetry(ws))

    try:
        while True:
            # 阻塞接收前端发来的 JSON 数据包
            raw = await ws.receive_text()
            msg = json.loads(raw)
            await handle_message(ws, msg)
    except WebSocketDisconnect:
        print("[WS] client disconnected", flush=True)
    except Exception as e:
        print(f"[WS] error: {e}", flush=True)
    finally:
        # 无论正常断开还是异常断开，都要取消发遥测数据的任务
        telemetry_task.cancel()

async def handle_message(ws: WebSocket, msg: Dict[str, Any]):
    """处理前端发来的各类指令包"""
    print("[WS MSG]", msg, flush=True)
    msg_type = msg.get("type")
    
    # 喂狗：只要收到合法消息，就刷新时间戳，防止触发失控保护
    state["last_cmd_ts"] = time.time()
    
    if msg_type == "set_volume":
        vol = msg.get("volume", 50)
        set_opi_volume(vol)
        await ws.send_json({"type": "ack", "msg": f"volume set to {vol}%"})
        return
    if msg_type == "ping":
        # 前端的心跳包，回复 ack 即可
        await ws.send_json({"type": "ack", "msg": "pong"})
        return

    if msg_type == "arm":
        enabled = bool(msg.get("enabled", False))
        ok = arm_system(enabled)
        if not ok:
            await ws.send_json({"type": "error", "msg": "estop active"})
            return
        await ws.send_json({"type": "ack", "msg": f"armed={enabled}"})
        return

    if msg_type == "clear_estop":
        clear_estop()
        await ws.send_json({"type": "ack", "msg": "estop cleared"})
        return

    if msg_type == "estop":
        emergency_stop()
        await ws.send_json({"type": "ack", "msg": "estop triggered"})
        return

    if msg_type == "drive":
        # 行驶控制指令
        if state["estop"]:
            await ws.send_json({"type": "error", "msg": "estop active"})
            return
        if not state["armed"]:
            await ws.send_json({"type": "error", "msg": "system not armed"})
            return

        # 提取控制量
        steering = int(msg.get("steering", state.get("steer_center", 90)))
        throttle = int(msg.get("throttle", 0))
        
        # 允许前端在发送指令的同时，微调电调和舵机的行程配置
        min_us = int(msg.get("throttle_min_us", state["throttle_min_us"]))
        center_us = int(msg.get("throttle_center_us", state["throttle_center_us"]))
        max_us = int(msg.get("throttle_max_us", state["throttle_max_us"]))
        steer_center = int(msg.get("steer_center", state.get("steer_center", 90)))

        configure_throttle_range(min_us, center_us, max_us)
        configure_steering_center(steer_center)

        # 执行动作
        set_steering(steering)
        set_throttle(throttle)
        await ws.send_json({"type": "ack", "msg": "drive updated"})
        return

    if msg_type == "gimbal":
        # 云台控制指令
        pan = int(msg.get("pan", 90))
        tilt = int(msg.get("tilt", 90))
        set_gimbal(pan, tilt)
        await ws.send_json({"type": "ack", "msg": "gimbal updated"})
        return

    await ws.send_json({"type": "error", "msg": f"unknown type: {msg_type}"})

async def send_telemetry(ws: WebSocket):
    """
    遥测(Telemetry)数据下发协程
    持续监控状态并主动向前端报告
    """
    while True:
        now = time.time()

        # 失控保护逻辑检测：如果解锁状态下超过 FAILSAFE_TIMEOUT 未收到前端消息
        if state["armed"] and (now - state["last_cmd_ts"] > FAILSAFE_TIMEOUT):
            apply_failsafe()

        # 发送当前状态，用于更新前端的 UI 显示面板
        await ws.send_json({
            "type": "telemetry",
            "armed": state["armed"],
            "estop": state["estop"],
            "steering": state["steering"],
            "throttle": state["throttle"],
            "pan": state["pan"],
            "tilt": state["tilt"],
            "battery_v": state["battery_v"],
            "signal": state["signal"],
            "ts": now
        })
        # 延时 0.2 秒，意味着遥测数据以 5Hz 的频率刷新
        await asyncio.sleep(0.2)