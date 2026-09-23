"""锥桶检测（cone_detect）单元测试：本地合成图 + 参考实现口径核对。

跑法：
    cd dev && PYTHONPATH=. python tests/test_cone_detect.py
"""
from __future__ import annotations

import cv2
import numpy as np

from vision.cone_detect import ConeDetector, ConeTracker, detect_cones_hsv


def _canvas() -> np.ndarray:
    """640×480 灰色地面（模拟塑胶跑道底色）。"""
    return np.full((480, 640, 3), (120, 120, 118), dtype=np.uint8)


def _draw_cone(img: np.ndarray, cx: int, bottom: int, w: int, h: int,
               color=(255, 120, 30)) -> None:
    """画一个锥桶：底部矩形 + 顶部小矩形（BGR 里给蓝色-红的分量）。"""
    x0, y0 = cx - w // 2, bottom - h
    cv2.rectangle(img, (x0, y0), (x0 + w, bottom), color, -1)


def test_empty_scene_no_cones():
    det = ConeDetector(method="hsv")
    assert det.detect(_canvas()) == []


def test_single_cone_detected():
    img = _canvas()
    _draw_cone(img, 300, 430, 90, 130)          # 中下部一侧
    cones = detect_cones_hsv(img)
    assert len(cones) >= 1, f"应检出锥桶，实际 {cones}"
    c = cones[0]
    assert abs(c.center_x - 300) < 25, f"中心应≈300，实际 {c.center_x:.0f}"
    assert c.bottom_y >= 400, "底边应接近绘制位置（估距用底边）"
    assert c.width >= 80 and c.height >= 120


def test_two_cones_both_returned():
    img = _canvas()
    _draw_cone(img, 220, 430, 80, 130)
    _draw_cone(img, 430, 440, 80, 120)
    cones = detect_cones_hsv(img)
    assert len(cones) == 2, f"两个锥桶都应返回，实际 {len(cones)}: {cones}"


def test_small_noise_filtered():
    img = _canvas()
    _draw_cone(img, 300, 430, 8, 10)            # 太小：应被 MIN_W/MIN_H 滤掉
    assert detect_cones_hsv(img) == []


def test_board_sized_blue_filtered():
    """近处大蓝板（挡板）不能被当锥桶：CONE_MAX_H 闸门。"""
    img = _canvas()
    _draw_cone(img, 320, 479, 400, 460)         # 接近整幅：像挡板贴脸
    assert detect_cones_hsv(img) == []


def test_tracker_debounce():
    tr = ConeTracker(enter_frames=3, exit_frames=1)
    img = _canvas()
    _draw_cone(img, 300, 430, 90, 130)
    cones = detect_cones_hsv(img)
    assert not tr.update(cones)                                # 第 1 帧：积累
    assert not tr.update(cones)                                # 第 2 帧：积累
    assert tr.update(cones) is True                            # 第 3 帧：成立
    assert tr.present is True
    assert not tr.update([])                                   # 消失 1 帧：退出
    assert tr.present is False


def test_model_channel_falls_back_on_local():
    """本地没有 rknnlite/NPU：model 通道应回退 hsv 而不是崩（权重路径随便给）。"""
    det = ConeDetector(method="model", model_path="/nonexistent/best4cls.rknn")
    assert det.method == "hsv", "无 NPU 环境应回退 hsv"
    img = _canvas()
    _draw_cone(img, 300, 430, 90, 130)
    cones = det.detect(img)
    assert len(cones) >= 1, "回退到 hsv 后应仍能检出合成锥桶"


def test_model_channel_requires_path():
    try:
        ConeDetector(method="model", model_path=None)
        assert False, "model 通道不带路径应报错"
    except ValueError:
        pass


if __name__ == "__main__":
    test_empty_scene_no_cones()
    test_single_cone_detected()
    test_two_cones_both_returned()
    test_small_noise_filtered()
    test_board_sized_blue_filtered()
    test_tracker_debounce()
    test_model_channel_falls_back_on_local()
    test_model_channel_requires_path()
    print("test_cone_detect: all passed")