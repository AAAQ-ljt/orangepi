"""JY901S IMU 协议单测（T5）：纯函数 + 合成字节流，无硬件。

覆盖：帧构造/校验和、四通道解码与换算、坏帧/半帧/字节分段重同步、
整轮数据（acc+gyro+angle）聚合、静止判据、零偏标定数学（喂假串口）。
跑法：cd dev && PYTHONPATH=. python tests/test_imu.py
"""
from __future__ import annotations


from hardware.imu import (CID_ACC, CID_ANGLE, CID_GYRO, HEADER, IMU, IMUData,
                          JY901SStream, checksum, decode_payload, make_frame)


def _round_trip(cid: int, shorts: list):
    """构造一帧 → 直接解码，验证换算系数。"""
    frame = make_frame(cid, shorts)
    assert len(frame) == 11
    assert checksum(frame) == frame[10]
    dec = decode_payload(cid, frame[2:10])
    return dec


def test_angle_scale():
    d = _round_trip(CID_ANGLE, [3456, -100, 33336, 0])
    assert abs(d["roll"] - 34.56) < 1e-9, "3456 LSB × 0.01 = 34.56°"
    assert abs(d["pitch"] - (-1.0)) < 1e-9
    assert abs(d["yaw"] - 333.36) < 1e-9, "33336 LSB × 0.01 = 333.36°（文档实测例）"


def test_acc_scale():
    d = _round_trip(CID_ACC, [2048, 0, -2048, 0])
    assert abs(d["ax"] - 1.0) < 1e-9, "2048 LSB = 1g"
    assert abs(d["az"] - (-1.0)) < 1e-9


def test_gyro_scale():
    d = _round_trip(CID_GYRO, [160, -320, 0, 0])
    assert abs(d["gx"] - 10.0) < 1e-9, "160 LSB / 16 = 10°/s"
    assert abs(d["gy"] - (-20.0)) < 1e-9


def test_stream_bad_checksum_then_resync():
    st = JY901SStream()
    good = make_frame(CID_ANGLE, [100, 0, 200, 0])
    bad = bytearray(good)
    bad[10] ^= 0xFF                       # 弄坏校验
    out = st.feed(bytes(bad))
    assert out == [], "坏校验帧应丢弃"
    assert st.bad_checksum == 1
    # 坏帧后紧接好帧：应正常重同步（帧头已随坏帧消费掉）
    out = st.feed(good)
    assert len(out) == 1 and out[0][0] == CID_ANGLE


def test_stream_resyncs_from_mid_frame():
    """从任意字节流中间进来（含垃圾前缀/半帧）：丢垃圾、找帧头、跨次喂完整解析。"""
    st = JY901SStream()
    good = make_frame(CID_GYRO, [1, 2, 3, 4])
    garbage = b"\x01\x02\x03"                 # 无帧头垃圾字节
    out = st.feed(garbage + good[:5])         # 垃圾 + 好帧前 5 字节
    assert out == [], "半帧不完整，等后续"
    out = st.feed(good[5:])                   # 补全
    assert len(out) == 1 and out[0][0] == CID_GYRO
    # 再来一个"坏 CID 帧 + 好帧"组合：坏帧被消费，好帧照常解析
    st2 = JY901SStream()
    bad_cid = bytes([HEADER, 0x99]) + b"\x00" * 8 + b"\x00"
    out = st2.feed(bad_cid + good)
    assert len(out) == 1 and out[0][0] == CID_GYRO


def test_imu_round_data_via_fake_serial():
    """喂一组 acc+gyro+angle 帧 → read() 应聚合出一轮完整数据。"""
    imu = IMU()
    # 模拟串口：直接把字节喂给 _consume（不碰 pyserial）
    imu._consume(make_frame(CID_ACC, [2048, 0, 2048, 0]))
    d = imu._consume(make_frame(CID_GYRO, [0, 0, 0, 0]))
    assert d is None, "只到两通道，不构成一轮"
    d = imu._consume(make_frame(CID_ANGLE, [0, 0, 100, 0]))
    assert d is not None, "acc+gyro+angle 到齐应出整轮"
    assert abs(d.az - 1.0) < 1e-6
    assert abs(d.yaw - 1.0) < 1e-6
    assert d.is_static(), "全零陀螺 → 静止"


def test_imu_read_yaw_without_hardware_is_zero():
    imu = IMU()
    assert imu.read_yaw() == 0.0, "无硬件/无串口时兼容旧行为恒 0"


def test_imu_calibration_mathematics():
    """零偏标定的数学：喂恒定"假读数"，校准后静止读数应归零（az 减重力）。"""
    imu = IMU()
    # 恒定"假数据"：gyro 恒 80 LSB（=5°/s 常量零偏），az 恒 1200 LSB（=0.586g）
    for _ in range(5):
        imu._consume(make_frame(CID_ACC, [0, 0, 1200, 0]))
        imu._consume(make_frame(CID_GYRO, [80, 0, 0, 0]))
        imu._consume(make_frame(CID_ANGLE, [0, 0, 100, 0]))
    imu._bias = {"gx": 5.0, "gy": 0.0, "gz": 0.0,
                 "ax": 0.0, "ay": 0.0, "az": 0.586 - 1.0}
    imu._consume(make_frame(CID_ACC, [0, 0, 1200, 0]))
    imu._consume(make_frame(CID_GYRO, [80, 0, 0, 0]))
    d = imu._consume(make_frame(CID_ANGLE, [0, 0, 100, 0]))
    assert abs(d.gx) < 1e-6, "零偏减掉后 gx 应归零"
    assert abs(d.az - 1.0) < 1e-3, "减掉 (az−1) 偏置后 az 应回到 1g（水平静止）"


def test_imu_data_static_judgement():
    d = IMUData(gx=1.0, gy=0.5, gz=0.2)
    assert d.is_static(), "|g|<2 静止"
    d2 = IMUData(gx=20.0, gy=0.0, gz=0.0)
    assert not d2.is_static(), "|g|≥2 非静止"


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"test_imu: all passed ({len(fns)})")