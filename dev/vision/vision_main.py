#!/usr/bin/env python3
"""小车视觉主进程：摄像头 → 扫线(每帧) + 发车检测 + 元素检测(可选) → UDP → 控制端。

双摄分时复用（2026-09-16 实测确认的角色）：
- `/dev/video0`(index 0) = icspring = **云台主摄** → **发车阶段用它看蓝板**（板竖立在车前，下摄看不全）
- `/dev/video2`(index 2) = Global Shutter = **下摄** → **行车阶段用它扫线**（朝向赛道地面）

设计要点：
- 扫线每帧跑（轻量，给控制环连续观测量）；元素检测按 `--detect-every` 降频，没给 `--model` 时完全跳过；
- 发车判定用**边沿触发**的 `StartGate`（先见板 → 再见空），并给操作员**明确反馈**（日志/状态文件/可选提示音）；
- 元素语义由 `--profile` 决定（`config/model_profile.yaml`），换模型不改控制代码。
"""
from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time

import cv2

from config import settings
from common.protocol import PerceptionMessage
from vision.camera_guard import camera_exclusive
from vision.elements import load_profile
from vision.lane_scan import LaneScanner
from vision.postprocess import postprocess
from vision.start_gate import StartGate


def _beep(freq: int = 1000, duration_ms: int = 150) -> None:
    """短提示音（不阻塞）。用 lavfi 生成正弦，避免额外音频文件。"""
    try:
        subprocess.Popen(["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet",
                          "-f", "lavfi", "-i", f"sine=frequency={freq}:duration={duration_ms / 1000.0}"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        pass


class CameraSwitcher:
    """摄像头分时复用：需要哪路就开哪路（同一时刻只占一路）。"""

    def __init__(self, width: int, height: int):
        self.width, self.height = width, height
        self.cap: cv2.VideoCapture | None = None
        self.index: int | None = None

    def use(self, index: int) -> bool:
        if self.index == index and self.cap is not None and self.cap.isOpened():
            return True
        self.release()
        cap = cv2.VideoCapture(index, cv2.CAP_V4L)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            print(f"[VISION] ⚠️ 摄像头 index={index} 打不开")
            return False
        self.cap, self.index = cap, index
        print(f"[VISION] 切换到摄像头 index={index}")
        return True

    def read(self):
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        return frame if (ok and frame is not None) else None

    def release(self) -> None:
        if self.cap is not None:
            self.cap.release()
        self.cap, self.index = None, None


def _write_status(path: str, payload: dict) -> None:
    try:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False)
    except OSError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Car local vision (lane scan + start gate + elements)")
    parser.add_argument("--model", default=None, help="RKNN 模型路径；不给则只跑扫线+发车检测")
    parser.add_argument("--profile", default="legacy",
                        help="模型 profile（config/model_profile.yaml）：legacy=上一届模型, smartcar2026=我们自己的")
    parser.add_argument("--udp-ip", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=settings.UDP_PORT)
    parser.add_argument("--camera", type=int, default=2,
                        help="行车摄像头：2=下摄（默认，巡线扫线用 /dev/video2），0=云台主摄")
    parser.add_argument("--gate-camera", type=int, default=0,
                        help="发车阶段看蓝板的摄像头：0=云台主摄（默认），-1=与行车摄像头共用")
    parser.add_argument("--gate-beep", action="store_true", help="发车事件播短提示音（台架调试用）")
    parser.add_argument("--width", type=int, default=settings.IMG_W)
    parser.add_argument("--height", type=int, default=settings.IMG_H)
    parser.add_argument("--conf", type=float, default=None, help="元素检测置信度（默认取 profile 的值）")
    parser.add_argument("--detect-every", type=int, default=3,
                        help="每 N 帧做一次元素检测（扫线仍然每帧跑）")
    parser.add_argument("--no-lane", action="store_true", help="关闭扫线（仅用于对比测试）")
    parser.add_argument("--no-start-gate", action="store_true", help="关闭发车检测（仅用于调试）")
    parser.add_argument("--test-image", default=None, help="只测试单张图片后退出")
    parser.add_argument("--status-file", default=settings.STATUS_FILE)
    args = parser.parse_args()

    profile = load_profile(args.profile)
    scanner = None if args.no_lane else LaneScanner()
    gate = None if args.no_start_gate else StartGate()
    gate_camera = args.camera if args.gate_camera < 0 else args.gate_camera

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
            print(f"[VISION] start gate: blocked={st.blocked} "
                  f"蓝面积占比={st.metrics.area_ratio:.3f} 主导度={st.metrics.dominance:.1f}")
        if args.model:
            from vision.rknn_detector import RKNNYoloSeg
            model = RKNNYoloSeg(args.model)
            if model.load():
                elements = postprocess(model.infer(model.preprocess(frame)), profile,
                                                   box_transform=model.restore,
                                       conf_threshold=args.conf)
                print(f"[VISION] profile={profile.name} 元素 {len(elements)} 个: "
                      f"{[f'{e.name}({e.color}) {e.conf:.2f}' for e in elements]}")
                model.release()
        return 0

    # ---------------- 模型（可选） ----------------
    model = None
    if args.model:
        print(f"[VISION] loading RKNN model: {args.model}  (profile={profile.name}, {profile.num_classes} 类)")
        from vision.rknn_detector import RKNNYoloSeg
        model = RKNNYoloSeg(args.model)
        if not model.load():
            print("[VISION] model load failed; 继续以纯扫线模式运行")
            model = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frames = 0
    loop_start = time.time()
    last_status_t = 0.0
    last_elements = []
    gate_released = False
    gate_state = None

    print(f"[VISION] udp -> {args.udp_ip}:{args.udp_port}；行车摄像头={args.camera}，发车摄像头={gate_camera}")
    try:
        with camera_exclusive():
            cam = CameraSwitcher(args.width, args.height)
            try:
                while True:
                    # ---- 分时复用：发车阶段用云台摄，放行后切行车摄 ----
                    want = gate_camera if (gate is not None and not gate_released) else args.camera
                    if not cam.use(want):
                        time.sleep(0.5)
                        continue
                    frame = cam.read()
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    on_gate_camera = (want == gate_camera)

                    t0 = time.time()
                    # ---- 发车检测（只在等待发车阶段；放行后不再计算）----
                    if gate is not None and not gate_released:
                        gate_state = gate.update(frame)
                        if gate_state.released:
                            gate_released = True
                            if args.gate_beep:
                                _beep(1400, 250)
                            print("[VISION] ✅ 已放行：切到行车摄像头，开始巡线")
                    # ---- 扫线（行车摄像头才有意义）----
                    obs = None
                    if scanner is not None and not on_gate_camera:
                        obs = scanner.scan(frame)
                    # ---- 元素检测（降频；发车阶段不做，省 CPU）----
                    elements = last_elements
                    if model is not None and not on_gate_camera:
                        frames_every = max(1, args.detect_every)
                        if frames % frames_every == 0:
                            elements = postprocess(model.infer(model.preprocess(frame)), profile,
                                                   conf_threshold=args.conf,
                                                   box_transform=model.restore)
                            last_elements = elements
                    elif on_gate_camera:
                        elements = []
                    loop_ms = (time.time() - t0) * 1000.0

                    frames += 1
                    elapsed = time.time() - loop_start
                    fps = frames / elapsed if elapsed > 0 else 0.0

                    msg = PerceptionMessage(
                        timestamp=time.time(),
                        left_x=None if obs is None else obs.left_x,
                        right_x=None if obs is None else obs.right_x,
                        center_x=None if obs is None else obs.center_x,
                        lane_confidence=None if obs is None else obs.confidence,
                        blue_cone_count=sum(1 for e in elements if e.name == "cone" and e.color == "blue"),
                        yellow_cone_count=sum(1 for e in elements if e.name == "cone" and e.color == "yellow"),
                        has_sign_a=any(e.name == "parking_sign" and e.raw_name.endswith("_A") for e in elements),
                        has_sign_b=any(e.name == "parking_sign" and e.raw_name.endswith("_B") for e in elements),
                        has_zebra_crossing=any(e.name == "zebra" for e in elements),
                        board_blocked=(bool(gate_state and gate_state.blocked) and not gate_released),
                        start_released=gate_released,
                        traffic_light_state=None,       # 红绿灯本阶段占位（见 doc/自动驾驶开发方案.md §8）
                        elements=[e.to_dict() for e in elements],
                    )
                    sock.sendto(json.dumps(msg.to_dict(), ensure_ascii=False).encode("utf-8"),
                                (args.udp_ip, args.udp_port))

                    now = time.time()
                    if now - last_status_t >= 0.5:
                        last_status_t = now
                        _write_status(args.status_file, {
                            "timestamp": now, "fps": round(fps, 1), "loop_ms": round(loop_ms, 1),
                            "camera": cam.index, "on_gate_camera": on_gate_camera,
                            "center_x": None if obs is None else round(obs.center_x, 1),
                            "lane_confidence": None if obs is None else round(obs.confidence, 3),
                            "board_blocked": bool(gate_state and gate_state.blocked),
                            "start_released": gate_released,
                            "blue_area": None if gate_state is None else round(gate_state.metrics.area_ratio, 4),
                            "blue_dominance": None if gate_state is None else round(gate_state.metrics.dominance, 1),
                            "elements": [e.to_dict() for e in elements],
                        })

                    if frames % max(1, settings.PRINT_EVERY_N) == 0:
                        print(f"[VISION] #{frames} fps={fps:.1f} loop={loop_ms:.0f}ms cam={cam.index} "
                              f"center={None if obs is None else round(obs.center_x, 1)} "
                              f"conf={None if obs is None else round(obs.confidence, 2)} "
                              f"board={None if gate_state is None else gate_state.blocked} "
                              f"released={gate_released} elems={[e.name for e in elements]}")
            except KeyboardInterrupt:
                print("\n[VISION] interrupted")
            finally:
                cam.release()
    finally:
        sock.close()
        if model is not None:
            model.release()
        print("[VISION] ffmpeg 推流已恢复")
    return 0


if __name__ == "__main__":
    sys.exit(main())
