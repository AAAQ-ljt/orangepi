"""元素层：把模型输出翻译成**统一元素名**，让控制端与具体模型解耦。

为什么要有这一层：
- 上一届模型是 9 类实例分割（`zebra_crossing / obstacle_cone_blue / parking_sign_A …`），
  我们自己要训练的是 8 类检测（`crosswalk / obstacle_cone_blue / parking_area / blue_board …`）。
- 如果控制端直接认类 id，换模型就要改控制代码 —— 那不是解耦。
- 这里做映射：`模型类名 → 统一元素名`，profile 写在 `config/model_profile.yaml`，
  换模型只改配置（见 `doc/自动驾驶开发方案.md` §1）。

统一元素名（控制端只认这些）：
    zebra          斑马线
    cone           锥桶（颜色在 Element.color：blue/red/yellow）
    parking_sign   停车标志牌（上一届的 A/B 牌，仅 legacy profile 有）
    parking_area   黄胶布停车区外框（2026 方案）
    blue_board     蓝色挡板（2026 方案）
    light          红绿灯（本阶段不消费）
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

DEFAULT_PROFILE = "legacy"
PROFILE_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "config", "model_profile.yaml")

# 从类名里推颜色/状态（两套模型的命名都适用）
_COLOR_HINTS = ("blue", "red", "yellow", "green", "off")


@dataclass
class Element:
    """一个被识别出来的元素（统一语义）。"""

    name: str                                   # 统一元素名
    conf: float
    xyxy: Tuple[float, float, float, float]
    color: Optional[str] = None                 # blue / red / yellow / green / off
    raw_name: str = ""                          # 模型原始类名（调试/日志用）

    # ---- 常用几何量（都用像素，图像坐标系 y 向下）----
    @property
    def center_x(self) -> float:
        return (self.xyxy[0] + self.xyxy[2]) / 2.0

    @property
    def bottom_y(self) -> float:
        """接地线（框底边）：估距离用它，不要用几何中心（竖立物体会偏高）。"""
        return float(self.xyxy[3])

    @property
    def bottom_center(self) -> Tuple[float, float]:
        return self.center_x, self.bottom_y

    @property
    def width(self) -> float:
        return float(self.xyxy[2] - self.xyxy[0])

    @property
    def height(self) -> float:
        return float(self.xyxy[3] - self.xyxy[1])

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "conf": round(float(self.conf), 3),
            "xyxy": [round(float(v), 1) for v in self.xyxy],
            "color": self.color,
        }


def as_hw(value) -> Tuple[int, int]:
    """把各种写法的 imgsz 统一成 (h, w)。

    yaml 里人手写容易写成 `640`（方形）、`[480, 640]`（h,w）、`[640]`，
    检测器要的是 (h, w)，所以在这里收敛一次：
        as_hw(640) -> (640, 640)
        as_hw([480, 640]) -> (480, 640)
        as_hw([640]) -> (640, 640)
    """
    if isinstance(value, int):
        return int(value), int(value)
    seq = list(value)
    if len(seq) == 1:
        return int(seq[0]), int(seq[0])
    return int(seq[0]), int(seq[1])


@dataclass
class ModelProfile:
    name: str
    task: str                 # detect / segment
    imgsz: Tuple[int, int]    # (h, w)，统一由 as_hw() 规范化
    conf: float
    names: Dict[int, str]     # 类 id → 模型类名
    mapping: Dict[str, str]   # 模型类名 → 统一元素名（none 表示不使用）

    def element_name(self, class_id: int) -> Optional[str]:
        """类 id → 统一元素名；不使用/未知返回 None。"""
        raw = self.names.get(int(class_id))
        if raw is None:
            return None
        mapped = self.mapping.get(raw, "none")
        return None if mapped in ("", "none", None) else str(mapped)

    @property
    def num_classes(self) -> int:
        return len(self.names)


def load_profile(name: str = DEFAULT_PROFILE, path: str = PROFILE_FILE) -> ModelProfile:
    """从 yaml 读 profile；没装 pyyaml 时退回内置的 legacy/smartcar2026 定义。"""
    try:
        import yaml  # 车上/PC 上一般都有（ultralytics 依赖）
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        node = (data.get("profiles") or {}).get(name)
        if node:
            return ModelProfile(
                name=name,
                task=str(node.get("task", "detect")),
                imgsz=as_hw(node.get("imgsz", 640)),
                conf=float(node.get("conf", 0.25)),
                names={int(k): str(v) for k, v in (node.get("names") or {}).items()},
                mapping={str(k): str(v) for k, v in (node.get("map") or {}).items()},
            )
    except Exception as exc:      # yaml 缺失/格式错时不致命，走内置兜底
        print(f"[elements] 读取 profile 失败（{exc}），使用内置定义")
    return _builtin_profile(name)


def _builtin_profile(name: str) -> ModelProfile:
    """内置兜底：即使 yaml 丢了也能跑（与 config/model_profile.yaml 保持一致）。"""
    if name == "smartcar2026":
        return ModelProfile(
            name="smartcar2026", task="detect", imgsz=(480, 640), conf=0.25,
            names={0: "blue_board", 1: "crosswalk", 2: "traffic_light_off",
                   3: "traffic_light_red", 4: "traffic_light_green",
                   5: "obstacle_cone_blue", 6: "obstacle_cone_red", 7: "parking_area"},
            mapping={"blue_board": "blue_board", "crosswalk": "zebra",
                     "traffic_light_off": "light", "traffic_light_red": "light",
                     "traffic_light_green": "light", "obstacle_cone_blue": "cone",
                     "obstacle_cone_red": "cone", "parking_area": "parking_area"},
        )
    return ModelProfile(
        name="legacy", task="segment", imgsz=(640, 640), conf=0.25,
        names={0: "lane_change_sign_left", 1: "lane_change_sign_right",
               2: "left_lane", 3: "obstacle_cone_blue", 4: "obstacle_cone_yellow",
               5: "parking_sign_A", 6: "parking_sign_B", 7: "right_lane",
               8: "zebra_crossing"},
        mapping={"lane_change_sign_left": "none", "lane_change_sign_right": "none",
                 "left_lane": "none", "right_lane": "none",
                 "obstacle_cone_blue": "cone", "obstacle_cone_yellow": "cone",
                 "parking_sign_A": "parking_sign", "parking_sign_B": "parking_sign",
                 "zebra_crossing": "zebra"},
    )


def color_of(raw_name: str) -> Optional[str]:
    """从模型类名里推颜色/灯态（legacy 与 2026 两套命名都适用）。"""
    low = raw_name.lower()
    for hint in _COLOR_HINTS:
        if hint in low:
            return hint
    return None


def elements_from_boxes(boxes: np.ndarray, scores: np.ndarray,
                        profile: ModelProfile) -> List[Element]:
    """把 (N,4) 框 + (N,C) 分数 转成元素列表（调用前需自行做过 NMS）。"""
    out: List[Element] = []
    if boxes.size == 0 or scores.size == 0:
        return out
    for idx in range(boxes.shape[0]):
        class_id = int(scores[idx, 0])
        conf = float(scores[idx, 1])
        name = profile.element_name(class_id)
        if name is None:
            continue
        raw = profile.names.get(class_id, str(class_id))
        out.append(Element(name=name, conf=conf,
                           xyxy=(float(boxes[idx, 0]), float(boxes[idx, 1]),
                                 float(boxes[idx, 2]), float(boxes[idx, 3])),
                           color=color_of(raw), raw_name=raw))
    return out


def pick(elements: List[Element], name: str) -> List[Element]:
    """按统一元素名筛选。"""
    return [e for e in elements if e.name == name]


def nearest(elements: List[Element], name: str = None) -> Optional[Element]:
    """返回画面中**最近**的元素（框底边 y 最大 = 离车最近）；可限定元素名。"""
    cands = [e for e in elements if (name is None or e.name == name)]
    if not cands:
        return None
    return max(cands, key=lambda e: e.bottom_y)
