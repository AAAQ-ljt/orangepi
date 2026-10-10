"""停车实跑脚本（scripts/parking_test.py）的决策纯函数单测：不碰硬件。

覆盖（全部无硬件依赖）：
- side_verdict：蓝板在哪侧（板左/板右/压中线=center 不投票）
- park_target_x：目标半格横坐标
- LockVote：多帧投票锁定、center/反向清零、锁定后不切换
- arrived_verdict：蓝板宽度占比判"够近"
- probe_parking：合成图（黄区+蓝板）全量读数与判侧

跑法：cd dev && PYTHONPATH=. python tests/test_parking_logic.py
"""
from __future__ import annotations

import numpy as np
import cv2

from scripts.parking_test import (LockVote, arrived_verdict,
                                  park_target_x, probe_parking, side_verdict)


# ------------------------------------------------------------------ 判左右
def test_side_verdict_board_left_right_center():
    # 黄区：x 100~500（mid_x=300，span=400）；容差 = 400×0.15 = 60px
    assert side_verdict(150.0, 300.0, 400.0) == "left", "板底边中点 150 << 300 → 板在左"
    assert side_verdict(450.0, 300.0, 400.0) == "right", "450 >> 300 → 板在右"
    assert side_verdict(310.0, 300.0, 400.0) == "center", "|310−300|=10 < 60 → 压中线"
    assert side_verdict(250.0, 300.0, 400.0) == "center", "|250−300|=50 < 60 → 压中线"


def test_park_target_x_sides():
    assert abs(park_target_x("right", 300.0, 400.0) - 400.0) < 1e-6, "右半格=中点+宽/4"
    assert abs(park_target_x("left", 300.0, 400.0) - 200.0) < 1e-6, "左半格=中点−宽/4"
    assert abs(park_target_x("center", 300.0, 400.0) - 300.0) < 1e-6, "未判定先对准中线"


# ------------------------------------------------------------------ 投票锁定
def test_lock_vote_requires_consecutive_same_side():
    v = LockVote(need=3)
    assert v.update("left") is None
    assert v.update("left") is None
    assert v.update("left") == "left", "连续 3 帧同侧 → 锁定"


def test_lock_vote_center_and_flip_resets():
    v = LockVote(need=3)
    v.update("left")
    v.update("left")
    assert v.update("center") is None
    assert v.update("left") is None, "center 清零后要重新积累"
    v2 = LockVote(need=3)
    v2.update("left")
    v2.update("left")
    assert v2.update("right") is None
    assert v2.update("right") is None
    assert v2.update("right") == "right", "反向清零后重新积累也能锁定"


def test_lock_vote_no_flip_after_lock():
    v = LockVote(need=3)
    v.update("left")
    v.update("left")
    assert v.update("left") == "left"
    assert v.update("right") == "left", "锁定后不再切换（保守，防近距误判）"


# ------------------------------------------------------------------ 到位判据
def test_arrived_verdict():
    assert arrived_verdict(400.0, 640.0), "板宽 400/640 ≥ 0.55 → 够近"
    assert not arrived_verdict(200.0, 640.0), "板宽 200/640 < 0.55 → 还远"
    assert not arrived_verdict(400.0, 0.0), "画面宽 0（异常）不判到位"


# ------------------------------------------------------------------ 合成图全量读数
def _parking_scene(board_left: bool) -> np.ndarray:
    """停车区合成图：灰底 + 完整黄区矩形（x 100~540，板不遮挡中部）+ 蓝板立在近端底部。"""
    img = np.full((480, 640, 3), (70, 70, 70), dtype=np.uint8)
    cv2.rectangle(img, (100, 300), (540, 380), (0, 210, 220), -1)   # 黄区：mid_x=320, span=440
    bx = 140 if board_left else 500
    cv2.rectangle(img, (bx, 400), (bx + 100, 470), (200, 80, 30), -1)   # 蓝板（BGR 深蓝）
    return img


def test_probe_parking_reads_geometry():
    r = probe_parking(_parking_scene(board_left=True))
    assert r["yellow"], "黄区应检出"
    assert r["mid_x"] is not None and abs(r["mid_x"] - 320.0) < 2.0, f"mid_x={r['mid_x']}"
    assert r["span"] is not None and abs(r["span"] - 440.0) < 3.0
    assert r["board_x"] is not None and r["board_x"] < 240.0, "板在左半格"
    assert r["side"] == "left", "板在左 → side=left"


def test_probe_parking_board_right():
    r = probe_parking(_parking_scene(board_left=False))
    assert r["side"] == "right", "板在右 → side=right"


def test_probe_parking_no_elements():
    r = probe_parking(np.full((480, 640, 3), (70, 70, 70), np.uint8))
    assert not r["yellow"] and r["mid_x"] is None and r["board_x"] is None


# ------------------------------------------------------------------ 汇总
if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"PASS {fn.__name__}")
    print(f"test_parking_logic: all passed ({len(fns)})")