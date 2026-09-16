#!/usr/bin/env python3
"""小车视觉主进程：摄像头 → 扫线(每帧) + 发车检测 + 元素检测(可选) → UDP → 控制端。

设计要点（AGENTS.md §5.1）：
- **扫线每帧跑**（轻量），给控制环连续的横向观测量；
- 元素检测（RKNN）只提供**离散事件**，按 `--detect-every` 降频，没给 `--model` 时完全跳过；
- 发车判定用边沿触发的 `StartGate`，输出 `board_blocked` / `start_released`；
- 状态文件 `/tmp/smartcar_status.json` 便于 SSH 一眼看状态；日志按 `PRINT_EVERY_N` 降频。
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time

import cv2

from config import settings
from common.protocol import PerceptionMessage
from vision.camera_guard import camera_exclusive
from vision.lane_scan import LaneScanner
from vision.start_gate import StartGate


def _load_detector(model_path: str):
    from vision.rknn_detector import RKNNYoloSeg
    model = RKNNYoloSeg(model_path)
    if not model.load():
        return None
    return model


def _write_status(path: str, payload: dict) -> None:
    try:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False)
    except OSError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Car local vision (lane scan + start gate + optional RKNN)")
    parser.add_argument("--model", default=None,
                        help="RKNN 模型路径；不给则只跑扫线+发车检测")
    parser.add_argument("--udp-ip", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=settings.UDP_PORT)
    parser.add_argument("--camera", type=int, default=2,
                        help="摄像头 index：2=下摄（默认，巡线扫线用 /dev/video2），0=云台主摄 /dev/video0")
    parser.add_argument("--width", type=int, default=settings.IMG_W)
    parser.add_argument("--height", type=int, default=settings.IMG_H)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--detect-every", type=int, default=3,
                        help="每 N 帧做一次元素检测（扫线仍然每帧跑）")
    parser.add_argument("--no-lane", action="store_true", help="关闭扫线（仅用于对比测试）")
    parser.add_argument("--no-start-gate", action="store_true", help="关闭发车检测（仅用于调试）")
    parser.add_argument("--test-image", default=None, help="只测试单张图片后退出")
    parser.add_argument("--status-file", default=settings.STATUS_FILE)
    args = parser.parse_args()

    scanner = None if args.no_lane else LaneScanner()
    gate = None if args.no_start_gate else StartGate()

    # ---------------- 单图测试模式 ----------------
    if args.test_image:
        frame = cv2.imread(args.test_image)
        if frame is None:
            print(f"[VISION] cannot read {args.test_image}")
            return 1
        if scanner is not None:
            obs = scanner.scan(frame)
            print(f"[VISION] lane: center={obs.center_x:.1f} conf={obs.confidence:.2f} "
                  f"L={obs.left_x} R={obs.right_x} rows={obs.valid_rows}/{obs.total_rows}")
        if gate is not None:
            st = gate.update(frame)
            print(f"[VISION] start gate: blocked={st.blocked} blue={st.blue_ratio:.3f} edge={st.edge_density:.4f}")
        return 0

    # ---------------- 模型（可选） ----------------
    model = None
    postprocess = result_to_message = None
    if args.model:
        print(f"[VISION] loading RKNN model: {args.model}")
        model = _load_detector(args.model)
        if model is None:
            print("[VISION] model load failed; 继续以纯扫线模式运行")
        else:
            from vision.postprocess import postprocess, result_to_message  # noqa: F401

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame_count = 0
    loop_start = time.time()
    last_status_t = 0.0
    last_det = None
    gate_state = None
    gate_released = False

    print(f"[VISION] stopping ffmpeg to free camera, udp -> {args.udp_ip}:{args.udp_port}")
    try:
        with camera_exclusive():
            cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.isOpened():
                print("[VISION] camera open failed")
                return 1

            try:
                while True:
                    ret, frame = cap.read()
                    if not ret or frame is None:
                        time.sleep(0.05)
                        continue

                    t0 = time.time()
                    obs = scanner.scan(frame) if scanner is not None else None
                    # 发车检测只在等待发车阶段有意义：放行之后就不再算（省 ~8ms/帧）
                    if gate is not None and not gate_released:
                        gate_state = gate.update(frame)
                        if gate_state.released:
                            gate_released = True
                            print("[VISION] 发车检测完成（已放行），后续不再计算遮挡判据")

                    det = None
                    detect_every = max(1, args.detect_every)
                    if model is not None and frame_count % detect_every == 0:
                        img = model.preprocess(frame)
                        outputs = model.infer(img)
                        det = postprocess(outputs, conf_threshold=args.conf)
                        last_det = det
                    # 检测降频时，两次检测之间沿用上一次结果：
                    # 这样 FSM 的"连续 N 帧"才表示"目标持续存在"，而不是"刚好被检测到 N 次"
                    det = det or last_det

                    frame_count += 1
                    elapsed = time.time() - loop_start
                    fps = frame_count / elapsed if elapsed > 0 else 0.0
                    loop_ms = (time.time() - t0) * 1000.0

                    msg = PerceptionMessage(
                        timestamp=time.time(),
                        left_x=None if obs is None else obs.left_x,
                        right_x=None if obs is None else obs.right_x,
                        center_x=None if obs is None else obs.center_x,
                        lane_confidence=None if obs is None else obs.confidence,
                        blue_cone_count=0 if det is None else det.blue_cone_count,
                        yellow_cone_count=0 if det is None else det.yellow_cone_count,
                        has_left_sign=bool(det and det.has_left_sign),
                        has_right_sign=bool(det and det.has_right_sign),
                        has_sign_a=bool(det and det.has_sign_a),
                        has_sign_b=bool(det and det.has_sign_b),
                        has_zebra_crossing=bool(det and det.has_zebra_crossing),
                        board_blocked=(bool(gate_state and gate_state.blocked) and not gate_released),
                        start_released=gate_released,
                        traffic_light_state=None,
                    )
                    sock.sendto(json.dumps(msg.to_dict(), ensure_ascii=False).encode("utf-8"),
                                (args.udp_ip, args.udp_port))

                    now = time.time()
                    if now - last_status_t >= 0.5:
                        last_status_t = now
                        _write_status(args.status_file, {
                            "timestamp": now,
                            "fps": round(fps, 1),
                            "loop_ms": round(loop_ms, 1),
                            "center_x": None if obs is None else round(obs.center_x, 1),
                            "lane_confidence": None if obs is None else round(obs.confidence, 3),
                            "left_x": None if obs is None else obs.left_x,
                            "right_x": None if obs is None else obs.right_x,
                            "board_blocked": (bool(gate_state and gate_state.blocked)
                                              and not gate_released),
                            "start_released": gate_released,
                            "blue_ratio": None if gate_state is None else round(gate_state.blue_ratio, 3),
                            "detect_every": detect_every,
                        })

                    if frame_count % max(1, settings.PRINT_EVERY_N) == 0:
                        print(f"[VISION] #{frame_count} fps={fps:.1f} loop={loop_ms:.0f}ms "
                              f"center={None if obs is None else round(obs.center_x, 1)} "
                              f"conf={None if obs is None else round(obs.confidence, 2)} "
                              f"board={None if gate_state is None else gate_state.blocked} "
                              f"released={None if gate_state is None else gate_state.released}")
            except KeyboardInterrupt:
                print("\n[VISION] interrupted")
            finally:
                cap.release()
    finally:
        sock.close()
        if model is not None:
            model.release()
        print("[VISION] ffmpeg 推流已恢复")
    return 0


if __name__ == "__main__":
    sys.exit(main())
