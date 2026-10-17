"""`vision/zebra_detect.py` 的单测：**合成图**（不碰摄像头/模型/硬件）。

为什么单独一个文件：`doc/开发文档.md` §2.8 给每个元素定死了一套流程（阈值进 settings、带开关且
默认关、单测 + 离线误检率、现场先架空）。这个文件就是"斑马线检测通道"那一套的**检测侧**验收：
该检的检得出（远边细白条 / 近处粘连大块都要能出**近边行号**），不该检的一个都不许出
（车道线、路面颗粒反光、塑料膜/地面炫光、画面顶部"远景亮带"、单张纸片、车身反光）。

★ 反向验证（每条"不误检"的用例旁边都有一条"把闸门摘掉就会误检"的断言）：
  2026-10-16 实测就是这么定的档 —— 开运算摘掉 → 2.mp4 颗粒反光 900 帧误触发 3~18 次；
  阈值 180 → 塑料膜反光 48 帧里命中 95 次；ROI 上沿 ≤0.20 → 顶部远景亮带每帧命中。

跑法：cd dev && PYTHONPATH=. python tests/test_zebra_detect.py
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import settings  # noqa: E402
from vision.camera_guard import crop_frame  # noqa: E402
from vision.zebra_detect import (ZebraDetector, ZebraTracker, detect_zebra_band,  # noqa: E402
                                 merge_runs, row_stats, white_mask)
from tests.test_lane_final import _lane_frame  # noqa: E402

W, H = 640, 480


# ---------------------------------------------------------------- 合成图
def _ground(seed: int = 11, speckle: bool = True) -> np.ndarray:
    """红棕跑道底（与真实素材同色系；颗粒灰 130~175 = 低于阈值 200，不该干扰）。"""
    rng = np.random.default_rng(seed)
    frame = np.full((H, W, 3), 95, np.uint8)
    frame[:, :, 2], frame[:, :, 1] = 105, 80
    if speckle:
        ys = rng.integers(5, 400, 2600)
        xs = rng.integers(0, W, 2600)
        frame[ys, xs] = rng.integers(130, 175, (2600, 3))
    return frame


def _zebra_frame(near_row: int = 300, far_row: int = 120, strips: int = 6,
                 strip_w: int = 60, gap: int = 22, value: int = 245,
                 x0: int = 0, y0: int = 0, base: np.ndarray = None) -> np.ndarray:
    """在底图上加一条"横道"：`strips` 条竖白条（far_row~near_row），条间留 `gap` 缝。

    几何口径照现场：A4 白条**长边顺着行进方向**（画面里是竖条）、横向平铺、中间那条压中线。
    `base` 不给 = 纯路面；要"横道 + 车道线同帧"就传 `_lane_frame()`。
    """
    frame = _ground() if base is None else base.copy()
    x = x0
    n = 0
    while n < strips and x < W:
        frame[far_row:near_row, x:min(W, x + strip_w)] = value
        x += strip_w + gap
        n += 1
    return frame


def _speckle_frame(frac: float = 0.13, rows=(300, 370), seed: int = 5) -> np.ndarray:
    """路面**颗粒反光**（2.mp4 那个误检源）：散点状亮点，单行占比 ~13% 但都是 1~3px 孤立点。"""
    rng = np.random.default_rng(seed)
    frame = _ground()
    y0, y1 = rows
    n = int((y1 - y0) * W * frac)
    ys = rng.integers(y0, y1, n)
    xs = rng.integers(0, W, n)
    for dy, dx in ((0, 0), (0, 1), (1, 0)):            # 1~2px 的点（连不成 25px 的带）
        frame[np.clip(ys + dy, 0, H - 1), np.clip(xs + dx, 0, W - 1)] = 240
    return frame


def _glare_frame(value: int = 196, rows=(99, 176)) -> np.ndarray:
    """塑料膜 / 地面**炫光**（e844be 那个误检源）：一大片"亮"，但没有白纸那么白。"""
    frame = _ground()
    y0, y1 = rows
    frame[y0:y1, :] = value
    return frame


def _far_band_frame(value: int = 250, rows=(0, 78)) -> np.ndarray:
    """画面顶部那条**远景亮带**（赛道尽头/围栏/楼；每帧都在，与横道无关）。"""
    frame = _ground()
    y0, y1 = rows
    frame[y0:y1, :] = value
    return frame


def _det(**kw) -> ZebraDetector:
    return ZebraDetector(**kw)


def _c(fr) -> np.ndarray:
    """按入口口径裁剪（只喂 y<384 的画面）——检测档就是按裁剪后画面定的。"""
    return crop_frame(fr)


# ================================================================ 该检的检得出
def test_far_crosswalk_reports_near_row() -> None:
    """远边刚露头（细白条带）就要能出读数，且 `near_row` = 近边行（= 刹停主控信号）。"""
    det = _det()
    fr = _zebra_frame(near_row=112, far_row=104)
    hit = det.detect(_c(fr))
    assert hit is not None, "6 条白竖条（远边）应当检出"
    assert abs(hit.near_row - 112) <= 2, f"near_row 应≈112（近边行）：{hit}"
    assert abs(hit.far_row - 104) <= 2, f"far_row 应≈104：{hit}"
    assert hit.n_pixels > 1200 and hit.n_rows >= 3
    assert hit.width > 0.3 * W, f"横道横跨整条赛道：{hit.width}"


def test_near_crosswalk_merged_blob_still_has_near_row() -> None:
    """★ 车贴近后白条**粘连成一大块**（连通域数会掉到 1）——本通道照样给近边行号。

    这正是旧判据（官方"连通域数 ≥4"）在停车那一刻判否的退化点。
    """
    det = _det()
    hit = det.detect(_c(_zebra_frame(near_row=330, far_row=150, strips=1,
                                     strip_w=W, gap=0, value=250)))
    assert hit is not None, "粘连大块必须照样检出（否则停车那一刻没读数）"
    assert abs(hit.near_row - 330) <= 2, hit
    assert hit.n_pixels > 30000


def test_two_bands_returns_nearest() -> None:
    """多条带都过闸门时取**近边行最大**的那条（行号越大 = 离车越近）。"""
    fr = _zebra_frame(near_row=150, far_row=130)
    fr2 = fr.copy()
    fr2[200:260, :] = 250                       # 再加一条更近的带
    hit = detect_zebra_band(fr2, roi=(0.02, 0.98, 0.0, 1.0), thresh=200, row_frac=0.15,
                            min_rows=3, min_width_frac=0.30, min_pixels=1200, min_run_px=25)
    assert hit is not None and abs(hit.near_row - 260) <= 2, f"应取更近的那条：{hit}"


def test_threshold_is_stricter_than_official() -> None:
    """阈值 200 与 180 的行为差：白纸（245）两档都过；塑料膜炫光（196）只有 180 档过。"""
    det = _det()
    assert det.detect(_c(_zebra_frame(value=245))) is not None
    glare = _c(_glare_frame(value=196))
    assert det.detect(glare) is None, "炫光（~196）在阈值 200 下必须不检出"
    loose = _det(thresh=180)
    hit = loose.detect(glare)
    assert hit is not None, "反向验证：阈值放回 180，同一片炫光就会被当成横道（所以定档 200）"


# ================================================================ 不该检的一个都不许出
def test_lane_lines_not_detected() -> None:
    """两条细亮车道线（5px 宽的斜线）不是横道。"""
    assert _det().detect(_c(_lane_frame())) is None


def test_speckle_not_detected() -> None:
    """★ 路面颗粒反光（2.mp4 的误检源）——反向验证：关掉横向开运算它就会误检。"""
    fr = _c(_speckle_frame())
    assert _det().detect(fr) is None, "散点反光不该被当成横道（横向开运算负责清它）"
    loose = _det(min_run_px=1)
    assert loose.detect(fr) is not None, "反向验证：min_run_px=1（不开运算）时这片散点会过关"


def test_far_field_band_outside_roi_not_detected() -> None:
    """★ 画面顶部那条"远景亮带"（每帧都在）——ROI 上沿负责把它挡在门外。"""
    fr = _c(_far_band_frame(rows=(0, 78)))
    assert _det().detect(fr) is None, "远景亮带在 ROI 之上，不该检出"
    loose = _det(roi=(0.02, 0.98, 0.0, 1.0))          # 反向验证：上沿开到 0 就会每帧命中
    assert loose.detect(fr) is not None, "反向验证：ROI 上沿开到 0，这条亮带立刻过关"


def test_single_sheet_not_detected() -> None:
    """单张白纸（窄条）不是横道：跨度/像素数不够。"""
    fr = _c(_zebra_frame(near_row=300, far_row=150, strips=1, strip_w=60, gap=0))
    assert _det().detect(fr) is None, "单张 60px 宽的纸片不该判横道"


def test_body_glints_not_detected() -> None:
    """车身上的亮斑（实测 f226：三小块、**行段错开**，x55-99/521-566/269-344，单行占比 ≤0.14）
    不够"横道"的量级：单行占比过不了 0.15，横向开运算也把碎点清掉。"""
    fr = _ground()
    for x0, x1, y0, y1 in ((55, 99, 300, 320), (521, 566, 315, 335), (269, 344, 330, 350)):
        fr[y0:y1, x0:x1] = 235
    assert _det().detect(_c(fr)) is None, "车身亮斑不该判横道"


def test_sunlit_track_not_detected() -> None:
    """★ 阳光直晒的红跑道面（`2.mp4` 后段的误检源）：灰≈211 越过阈值、跨度也长，
    但它是**有色的**（S≈47）⇒ 被饱和度闸门压掉。反向验证：关掉 S 闸门就会过关。"""
    fr = _ground()
    fr[100:140, 0:400] = (188, 205, 230)          # BGR：灰≈211、S≈47（实测那片日照面 213/47）
    assert _det().detect(_c(fr)) is None, "日照跑道面不该判横道（白纸是中性色 S≈5）"
    loose = _det(s_max=0)
    assert loose.detect(_c(fr)) is not None,         "反向验证：s_max=0（不看饱和度）时同一片日照面会过关 —— 这就是 S≤35 存在的理由"


def test_blank_frame_not_detected() -> None:
    assert _det().detect(_c(_ground(speckle=False))) is None


# ================================================================ 逐行统计（定档工具的底座）
def test_row_stats_geometry() -> None:
    fr = _zebra_frame(near_row=200, far_row=180, strips=2, strip_w=100, gap=100, x0=0)
    st = row_stats(fr, roi=(0.0, 1.0, 0.0, 1.0), thresh=200, min_run_px=25)
    assert st["frac"][190] > 0.30, f"横道行的白占比应 >0.30：{st['frac'][190]}"
    assert st["frac"][100] < 0.05, "非横道行应当几乎没白"
    assert st["count"][190] == st["count"][181] > 0, "两行都在带里（宽度一致）"
    assert st["xmin"][190] == 0 and st["xmax"][190] >= 199, "横向范围要覆盖两条白带"
    assert st["xmin"][100] == -1, "空行 xmin 应为 -1"


def test_merge_runs_allows_small_gaps() -> None:
    assert merge_runs([True, True, False, False, True], max_gap=2) == [(0, 4)]
    assert merge_runs([True, False, False, False, True], max_gap=2) == [(0, 0), (4, 4)]
    assert merge_runs([False, False]) == []
    assert merge_runs([True, True]) == [(0, 1)]


def test_white_mask_shape_and_roi() -> None:
    fr = _zebra_frame(near_row=360, far_row=340)
    m = white_mask(fr, roi=(0.0, 1.0, 0.0, 1.0), thresh=200, )
    assert m.shape == (H, W) and m.dtype == np.uint8
    assert m[350].sum() > 0 and m[100].sum() == 0
    m2 = white_mask(fr, roi=(0.0, 1.0, 0.5, 1.0), thresh=200)   # ROI 只留 y>=240
    assert m2[100].sum() == 0 and m2[350].sum() == m[350].sum(), "ROI 之外应为 0"


# ================================================================ 防抖
def test_tracker_debounce_3_in_2_out() -> None:
    trk = ZebraTracker()
    hit = _det().detect(_c(_zebra_frame()))
    assert hit is not None
    assert trk.update(hit) is False            # 1
    assert trk.update(hit) is False            # 2
    assert trk.update(hit) is True             # 3 → 成立
    assert trk.last is not None and trk.last.near_row == hit.near_row
    assert trk.update(None) is True            # 丢 1 帧不算消失
    assert trk.update(None) is False           # 丢 2 帧 → 消失
    assert trk.last is not None, "消失后 last 仍留着（供日志复盘）"
    trk.reset()
    assert trk.present is False and trk.last is None


def test_tracker_uses_settings_defaults() -> None:
    trk = ZebraTracker()
    assert trk.enter_frames == int(settings.ZEBRA_HYSTERESIS_FRAMES)
    assert trk.exit_frames == max(2, int(settings.ZEBRA_HYSTERESIS_FRAMES) // 2)


def test_detector_defaults_come_from_settings() -> None:
    det = ZebraDetector()
    assert det.roi == tuple(settings.ZEBRA_GATE_ROI)
    assert det.thresh == int(settings.ZEBRA_GRAY_THRESH)
    assert det.s_max == int(settings.ZEBRA_S_MAX)
    assert det.row_frac == settings.ZEBRA_ROW_FRAC
    assert det.min_rows == int(settings.ZEBRA_MIN_ROWS)
    assert det.min_run_px == int(settings.ZEBRA_MIN_RUN_PX)
    assert det.min_pixels == int(settings.ZEBRA_MIN_PIXELS)
    assert det.min_width_frac == settings.ZEBRA_MIN_WIDTH_FRAC


def test_detector_never_raises_on_garbage() -> None:
    """异常只打日志、按"本帧无横道"返回 —— 主循环绝不能被它带崩。"""
    det = _det()
    assert det.detect(np.zeros((0, 0, 3), np.uint8)) is None
    assert det.detect(np.zeros((4, 4, 3), np.uint8)) is None


def main() -> int:
    funcs = [v for k, v in sorted(globals().items())
             if k.startswith("test_") and callable(v) and getattr(v, "__module__", "") == __name__]
    failed = 0
    for fn in funcs:
        try:
            fn()
            print(f"  ✅ {fn.__name__}")
        except Exception as exc:                      # noqa: BLE001
            failed += 1
            import traceback
            print(f"  ❌ {fn.__name__}: {exc}")
            traceback.print_exc()
    print(f"[test_zebra_detect] {len(funcs) - failed}/{len(funcs)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
