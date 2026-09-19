"""扫线（LaneScanner）单元测试：合成赛道图，不碰硬件。

跑法：
    cd dev && PYTHONPATH=. python tests/test_lane_scan.py
"""
from __future__ import annotations

import numpy as np

from vision.lane_scan import LaneScanner

W, H = 640, 480


def _scanner(**kw) -> LaneScanner:
    """一律显式指定 target_x：**车端 config/site.yaml 会改默认 target_x**，
    单测必须与现场标定解耦（曾因此在车上红过一次）。"""
    return LaneScanner(target_x=320.0, **kw)


def _track_image(shift: int = 0, left: bool = True, right: bool = True,
                 thickness: int = 7) -> np.ndarray:
    """红棕跑道 + 两条竖直白线（默认关于画面中心对称，中线 = 320）。"""
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    frame[:, :, 2] = 130
    frame[:, :, 1] = 40
    frame[:, :, 0] = 30
    y0, y1 = int(H * 0.45), H - 16
    half = thickness // 2
    if left:
        x = 180 + shift
        frame[y0:y1, x - half:x + half + 1] = 235
    if right:
        x = 460 + shift
        frame[y0:y1, x - half:x + half + 1] = 235
    return frame


def test_centered_track_is_symmetric():
    obs = _scanner().scan(_track_image())
    assert abs(obs.center_x - 320.0) <= 5.0, f"对称赛道中心应≈320，实际 {obs.center_x:.1f}"
    assert obs.confidence > 0.6, f"双侧齐全且宽度一致，置信度应偏高，实际 {obs.confidence:.2f}"
    assert obs.valid_rows >= 20, f"有效行太少：{obs.valid_rows}"
    assert obs.left_x is not None and obs.right_x is not None
    assert obs.left_x < 320 < obs.right_x


def test_shifted_track_moves_center():
    obs = _scanner().scan(_track_image(shift=40))
    assert 350.0 <= obs.center_x <= 370.0, f"整体右移 40px 后中心应≈360，实际 {obs.center_x:.1f}"


def test_single_side_line_gives_zero_confidence():
    """只有单侧线时**不给中心**（不用 0/w-1 兜底）→ 置信度必须为 0，让仲裁层去降级。

    2026-09-16 实车教训：旧实现用画面边界兜底，单侧丢线时行中点被拉飞，
    而置信度还有 0.5~0.6 → 被当成有效 → 车左右猛打冲出赛道。
    """
    obs = _scanner().scan(_track_image(left=False))
    assert obs.confidence == 0.0, f"单侧线不该给出可信中心，实际置信度 {obs.confidence:.2f}"
    assert obs.valid_rows >= 5, "行是看到了的，只是不足以给出中心"


def test_wide_white_region_is_not_a_lane_line():
    """大片白色区域（实验室地面/纸边）不应被当成车道线。"""
    frame = _track_image()
    frame[int(H * 0.45):H - 16, :] = 235        # 下半幅全白
    obs = _scanner().scan(frame)
    assert obs.confidence == 0.0, "整片白不是两条线，应判无有效车道"


def test_glare_blob_is_rejected():
    """**打印跑道表面的反光**：低饱和亮区，颜色上和白线一样，但形状是块状的。

    2026-09-16 实车：打印图有镜面高光，掩膜把整片中路的反光当白线 → 扫线锁到反光上，
    中心乱跳、车歪歪扭扭冲出赛道。修法 = 连通域形状过滤（只留"细长的"）。
    """
    frame = _track_image()
    # 中间偏右一坨反光（90×70 的块），不碰左右两条真线
    frame[300:370, 350:440] = 235
    obs = _scanner().scan(frame)
    assert abs(obs.center_x - 320.0) <= 8.0, f"反光块不应把中心拽走，实际 {obs.center_x:.1f}"
    assert obs.confidence > 0.5, f"两条真线还在，置信度不该掉太多，实际 {obs.confidence:.2f}"


def test_thin_line_survives_shape_filter():
    """形状过滤不能把真线也滤掉：细长（厚度小）的必须是"线"。"""
    from vision.lane_scan import white_mask
    mask, _ = white_mask(_track_image(thickness=5))
    assert mask.sum() > 0, "厚度 5px 的细线被形状过滤误杀"


def test_blank_track_has_zero_confidence():
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    frame[:, :, 2] = 130
    frame[:, :, 1] = 40
    frame[:, :, 0] = 30
    scanner = _scanner()
    obs = scanner.scan(frame)
    assert obs.confidence == 0.0, "没有白线时置信度必须为 0（不能假装可信）"
    assert abs(obs.center_x - scanner.target_x) < 1e-6, "无信息时返回目标值占位"


def test_noisy_white_blobs_do_not_dominate():
    """零散白噪声（地面反光）不应把中心拉跑。"""
    frame = _track_image()
    rng = np.random.default_rng(0)
    ys = rng.integers(int(H * 0.45), H - 16, size=40)
    xs = rng.integers(300, 340, size=40)
    for y, x in zip(ys, xs):
        frame[y, x:x + 2] = 235
    obs = _scanner().scan(frame)
    assert abs(obs.center_x - 320.0) <= 8.0, f"白噪声不应显著改变中心，实际 {obs.center_x:.1f}"


if __name__ == "__main__":
    test_centered_track_is_symmetric()
    test_shifted_track_moves_center()
    test_single_side_line_gives_zero_confidence()
    test_wide_white_region_is_not_a_lane_line()
    test_glare_blob_is_rejected()
    test_thin_line_survives_shape_filter()
    test_blank_track_has_zero_confidence()
    test_noisy_white_blobs_do_not_dominate()
    print("test_lane_scan: all passed")
