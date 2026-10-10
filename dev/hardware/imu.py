"""JY901S 九轴 IMU 驱动（2026-10-09 T5，照官方 imu.c + A10 文档移植，纯软件可测）。

协议（官方《10.IMU调试与数据分析》权威版本，穿越火线 demo imu.c 同源）：
- 帧长固定 11 字节：`0x55 CID D0 D1 D2 D3 D4 D5 D6 D7 SUM`；SUM = 前 10 字节
  无符号累加取低 8 位（回绕）；
- CID：0x51 加速度 / 0x52 陀螺仪 / 0x53 姿态角 / 0x54 磁力计；
- 小端 short：`val = (short)(b0 | b1 << 8)`；
- 换算（文档表 2，**权威口径**——⚠️ 与 imu.c 的 `/32768*180` 有 1.8 倍矛盾，以文档为准，
  上车后用"水平静止 az ≈ 1.00g、yaw 稳定"实测复核）：
    加速度 ×1/2048  → g（1g = 2048 LSB）
    陀螺仪 ×1/16    → °/s
    姿态角 ×0.01    → °（roll/pitch/yaw）
- 读前清输入缓冲（抢最新数据，与视觉 grab/retrieve 同思想）。

校准（imu_calibration 移植）：静止采 n 个完整帧求均值；`acc_bias_z = az_mean − 1.0`（减重力）；
陀螺三轴均值即零偏。静止判据：`|g| = sqrt(gx²+gy²+gz²) < 2 °/s`。

设计：协议解析做成 **纯函数 + 字节流状态机**（`JY901SStream`），不依赖硬件；
`IMU` 类负责串口/生命周期，单测用合成字节流覆盖（坏帧/半帧/校验失败重同步）。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

FRAME_LEN = 11
HEADER = 0x55
CID_ACC = 0x51
CID_GYRO = 0x52
CID_ANGLE = 0x53
CID_MAG = 0x54

# 换算系数（以官方"A10 文档表 2"为准，见模块 docstring）
ACC_SCALE = 1.0 / 2048.0       # g
GYRO_SCALE = 1.0 / 16.0        # °/s
ANGLE_SCALE = 0.01             # °


# ------------------------------------------------------------------ 纯函数层
def checksum(frame: bytes) -> int:
    """前 10 字节无符号累加，取低 8 位（溢出回绕）。"""
    return sum(frame[:10]) & 0xFF


def decode_payload(cid: int, payload: bytes) -> Dict[str, float]:
    """按 CID 解码 8 字节 payload → 物理量（原始 short 与换算后的值都在）。

    ⚠️ 符号口癖（JY901S 实机语义，官方文档实测例 333.36° 佐证）：
    roll/pitch 带符号（−180~180）；**yaw 无符号 0~360**（33336 LSB → 333.36°）。
    """
    raw = []
    for i in range(4):
        b0, b1 = payload[2 * i], payload[2 * i + 1]
        raw.append(b0 | (b1 << 8))
    vals = {f"raw{i}": float(v if v < 0x8000 else v - 0x10000) for i, v in enumerate(raw)}
    if cid == CID_ACC:      # ax, ay, az (g)
        return {"ax": vals["raw0"] * ACC_SCALE, "ay": vals["raw1"] * ACC_SCALE,
                "az": vals["raw2"] * ACC_SCALE, "temp": vals["raw3"], "cid": cid}
    if cid == CID_GYRO:     # gx, gy, gz (°/s)
        return {"gx": vals["raw0"] * GYRO_SCALE, "gy": vals["raw1"] * GYRO_SCALE,
                "gz": vals["raw2"] * GYRO_SCALE, "temp": vals["raw3"], "cid": cid}
    if cid == CID_ANGLE:    # roll, pitch (°, 带符号), yaw (°, 0~360)
        return {"roll": vals["raw0"] * ANGLE_SCALE, "pitch": vals["raw1"] * ANGLE_SCALE,
                "yaw": float(raw[2]) * ANGLE_SCALE, "temp": vals["raw3"], "cid": cid}
    return {"cid": cid, **vals}


@dataclass
class IMUData:
    """一轮完整数据（四通道都到齐才算——避免"加速度是新的、姿态角还是旧的"混合帧）。"""
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    flags: Dict[str, bool] = field(default_factory=dict)

    @property
    def gyro_norm(self) -> float:
        """|g| = sqrt(gx²+gy²+gz²)，°/s；< 2 视为静止。"""
        return (self.gx ** 2 + self.gy ** 2 + self.gz ** 2) ** 0.5

    def is_static(self, thresh: float = 2.0) -> bool:
        return self.gyro_norm < thresh


class JY901SStream:
    """字节流状态机：喂任意分段的原始字节，吐出完整帧（坏帧自动丢弃重同步）。

    用法：
        stream = JY901SStream()
        for chunk in raw_bytes_chunks:
            for frame in stream.feed(chunk):
                ...   # frame: (cid, payload_bytes)
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_seen = 0
        self.bad_checksum = 0

    def feed(self, chunk: bytes | bytearray):
        self._buf.extend(chunk)
        out = []
        while True:
            # 找帧头；帧头之前的一切都丢弃（最多丢 1 帧即重同步）
            idx = self._buf.find(bytes([HEADER]))
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < FRAME_LEN:
                break                      # 等后续字节
            frame = bytes(self._buf[:FRAME_LEN])
            del self._buf[:FRAME_LEN]
            cid = frame[1]
            if cid not in (CID_ACC, CID_GYRO, CID_ANGLE, CID_MAG):
                continue                   # 未知 CID：整帧丢弃
            if checksum(frame) != frame[10]:
                self.bad_checksum += 1
                continue                   # 校验失败：丢弃（帧头已移走，自动重同步）
            self.frames_seen += 1
            out.append((cid, frame[2:10]))
        return out


# ------------------------------------------------------------------ IMU 类（串口生命周期）
class IMU:
    """JY901S 串口 IMU。init() 后 read() 返回一轮完整数据；未接硬件时自动降级为
    全零（read_yaw 恒 0.0，与旧空壳行为一致，任何调用方都不会崩）。

    2026-10-09 T5：协议层纯函数可单测；串口接线/波特率（/dev/ttyS0, 9600）
    明天操场实机验证（yaw 稳定、静止 az≈1.00g）。
    """

    def __init__(self, device: str = "/dev/ttyS0", baudrate: int = 9600):
        self.device = device
        self.baudrate = baudrate
        self._ser = None
        self.stream = JY901SStream()
        self._data = IMUData()
        self._seen: Dict[int, bool] = {}
        self._bias: Dict[str, float] = {"gx": 0.0, "gy": 0.0, "gz": 0.0,
                                        "ax": 0.0, "ay": 0.0, "az": 0.0}
        self.yaw0: Optional[float] = None     # 初始航向（回正目标 = yaw0 + offset）

    def init(self) -> bool:
        try:
            import serial
        except ImportError:
            print("[IMU] 无 pyserial，IMU 不可用（read_yaw 恒 0.0）")
            return False
        try:
            self._ser = serial.Serial(self.device, self.baudrate, timeout=0.05)
            self._ser.flushInput()
            return True
        except (OSError, serial.SerialException) as exc:
            print(f"[IMU] 串口打开失败：{exc}（降级：read_yaw 恒 0.0）")
            self._ser = None
            return False

    def _drain_and_read(self) -> List[bytes]:
        if self._ser is None:
            return []
        try:
            self._ser.reset_input_buffer()        # 清缓冲抢最新（tcflush 语义）
            n = self._ser.in_waiting
            if n <= 0:
                return []
            return [self._ser.read(min(n, 256))]
        except OSError:
            return []

    def _consume(self, raw: bytes) -> Optional[Dict[str, float]]:
        """喂原始字节，收齐 acc+gyro+angle 后返回一轮最新数据（mag 非必需）。"""
        for cid, payload in self.stream.feed(raw):
            dec = decode_payload(cid, payload)
            if cid == CID_ACC:
                self._data.ax, self._data.ay = dec["ax"], dec["ay"]
                self._data.az = dec["az"]
            elif cid == CID_GYRO:
                self._data.gx, self._data.gy = dec["gx"], dec["gy"]
                self._data.gz = dec["gz"]
            elif cid == CID_ANGLE:
                self._data.roll, self._data.pitch = dec["roll"], dec["pitch"]
                self._data.yaw = dec["yaw"]
            self._seen[cid] = True
        if len(self._seen) >= 3:
            self._seen.clear()
            self._apply_bias()
            return self._data
        return None

    def _apply_bias(self) -> None:
        self._data.gx -= self._bias["gx"]
        self._data.gy -= self._bias["gy"]
        self._data.gz -= self._bias["gz"]
        self._data.az -= self._bias["az"]

    def read(self) -> Optional[IMUData]:
        """读一轮完整数据；未读到返回 None（调用方按 IMU 不可用降级）。"""
        for chunk in self._drain_and_read():
            got = self._consume(chunk)
            if got is not None:
                return got
        return None

    def calibrate(self, samples: int = 100) -> IMUData:
        """静止零偏标定：采 samples 个完整帧求均值；acc_bias_z 减掉重力 1g。"""
        accs, gyros = [], []
        deadline = time.time() + float(samples) / 50.0 * 4.0 + 5.0
        while len(accs) < samples and time.time() < deadline:
            d = self.read()
            if d is None:
                time.sleep(0.01)
                continue
            accs.append((d.ax, d.ay, d.az))
            gyros.append((d.gx, d.gy, d.gz))
        if not accs:
            print("[IMU] 校准：没读到数据，保持零偏为 0")
            return self._data
        n = len(accs)
        mx = sum(a[0] for a in accs) / n
        my = sum(a[1] for a in accs) / n
        mz = sum(a[2] for a in accs) / n
        self._bias = {"gx": sum(g[0] for g in gyros) / n,
                      "gy": sum(g[1] for g in gyros) / n,
                      "gz": sum(g[2] for g in gyros) / n,
                      "ax": mx, "ay": my, "az": mz - 1.0}   # acc_bias_z 减重力
        self.yaw0 = self._data.yaw
        print(f"[IMU] 零偏标定完成（{n} 样）：gyro=({self._bias['gx']:.3f},"
              f"{self._bias['gy']:.3f},{self._bias['gz']:.3f})°/s "
              f"acc_bias_z={self._bias['az']:.3f}g yaw0={self.yaw0:.2f}°")
        return self._data

    def read_yaw(self) -> float:
        """兼容旧接口：读到就用（含零偏），读不到/未接硬件返回 0.0。"""
        d = self.read()
        return 0.0 if d is None else float(d.yaw)

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None


# ------------------------------------------------------------------ 合成帧构造（测试/调试共用）
def make_frame(cid: int, shorts: List[int]) -> bytes:
    """按 JY901S 协议构造一帧（小端 short 列表 → 11 字节）。"""
    payload = bytearray()
    for v in shorts:
        v &= 0xFFFF
        payload += bytes([v & 0xFF, (v >> 8) & 0xFF])
    frame = bytes([HEADER, cid]) + bytes(payload)
    return frame + bytes([checksum(frame)])