"""斑马线检测通道（2026-10-16，B 路径 `scripts/lane_final_test.py --zebra` 专用）。

**为什么另开一个文件**：A 路径（`vision_main` + `control/fsm.py` 的 ZEBRA_STOP）用的是
`vision/color_elements.py::detect_zebra`（官方 14 号 demo 配方：灰度>180 + 连通域面积>200 +
**连通域数 ≥4**）。那条判据在**远处**能用，但现场视频实测有个致命退化：
车靠近后 A4 白条会**粘连成大块**，连通域数掉到 1~3（停车那一刻实测 3 个）⇒ 恰恰在最需要
位置读数的时候"数不够、判否"。所以 B 路径改成**逐行白占比**（斑马线是"成排的横向白带"，
每一行的白像素占比高达 30~90%，而车道线只有 1~2%），并且**逐帧给出近边行号**给刹停用。

实测（2026-10-16 现场视频 `train/opencv-level1/video/true_track_sub_video/斑马线1016.mp4`，
手推 855 帧 @30fps，640x480、红跑道 + A4 白纸横道）：
· 白条灰度 236~241 / 地面 82~118（对比度 2~3 倍）⇒ `gray>=180` 分隔干净（官方口径沿用）；
· ★ **画面顶部有一条"永久亮带"**（赛道尽头 + 白色围栏/楼，行 0~75~95、单行白占比最高 0.9，
  **每一帧都在**）。ROI 上沿开在 ≤80 时**全片 855 帧每帧触发**（实测）——
  这跟锥桶的"顶部远景带"是同一类坑，所以 ROI 上沿是**机位相关档**，必须当天视频复量；
· 上沿 100（= 裁剪后 384 的 0.26）+ `行白占比≥0.10 且连续 ≥3 行`：整片 855 帧**零误触发**，
  首次触发在 f186（比手推停车点早 2.2s；那 3 行白就是 6 张纸的远边，逐帧核过是真横道）；
· 停车那一刻斑马线占 **行 146~328**（0.3m 的横道深度占了 182 行 ⇒ 该段约 1.6mm/行）
  ⇒ 行分辨率足够精确，但**行→距离必须现场用卷尺标定**（见 doc/开发文档.md §2.3）。

结构：`row_stats()`（每帧一次，逐行白像素统计）→ `detect_from_stats()`（闸门/选带）→
`detect_zebra_band()`（= 前两者串起来，入口用）。定档工具扫档时**预计算一次 stats 再反复调用
detect_from_stats**，闸门逻辑只有这一份实现（不再抄一遍）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from config import settings


@dataclass
class ZebraHit:
    """一帧的斑马线证据（近边行 `near_row` 是相位机刹停的主控信号）。"""

    near_row: float                 # 近边行号（离车最近那条白带的下沿；裁剪后画面坐标）
    far_row: float                  # 远边行号（同一条带的上沿）
    x0: float                       # 白带横向范围（像素）
    x1: float
    n_rows: int                     # 连续行段长度
    n_pixels: int                   # 段内白像素总数
    coverage: float                 # 段内白像素 / 段面积（0~1）
    conf: float                     # 0~1（只作日志/排序用）

    @property
    def height(self) -> float:
        return self.near_row - self.far_row

    @property
    def width(self) -> float:
        return self.x1 - self.x0


def _roi_slice(frame_bgr: np.ndarray,
               roi: Tuple[float, float, float, float]) -> Tuple[np.ndarray, int, int]:
    """比例 ROI (x0,x1,y0,y1) → (子图, x 偏移, y 偏移)。"""
    h, w = frame_bgr.shape[:2]
    x0, x1 = int(w * roi[0]), int(w * roi[1])
    y0, y1 = int(h * roi[2]), int(h * roi[3])
    return frame_bgr[y0:y1, x0:x1], x0, y0


def row_stats(frame_bgr: np.ndarray, roi: Optional[Tuple[float, float, float, float]] = None,
              thresh: Optional[int] = None,
              s_max: Optional[int] = None,
              min_run_px: Optional[int] = None) -> Dict[str, np.ndarray]:
    """ROI 内**每一行**的白像素统计（索引 = 原图行号；ROI 之外的行全 0/-1）。

    "白" = `灰 ≥ thresh` **且** `HSV 饱和度 ≤ s_max`（`s_max<=0` = 不看饱和度）。
    ★ 为什么还要饱和度（2026-10-16/17 实测，都是"误触发"级证据）：
      · 白纸被阳光打爆后是**中性色**（S p50=5、p90=26）；而 `2.mp4` 后段的误检源是
        **阳光直晒的红色跑道面**（灰 p50=213 → 越过 200 阈值，跨度还很长）——它的 S p50=47
      ⇒ 加 `S≤35` 后：正样本首帧/命中数**逐位不变**（f194、近边 107），而 2.mp4 3573 帧
        **从 194 帧命中直接掉到 0**（`gray≥225` 也能压掉它，但那把"纸不够亮"的余量吃光了）。
    ★ `min_run_px` = **横向开运算核长**（默认 `ZEBRA_MIN_RUN_PX=25`）：只有"横向连成一片、
    长度 ≥ 它"的白像素才算白。为什么必须有：`2.mp4` 画面下半部的赛道**颗粒反光**是散点
    （单行 12~14% 的像素 >180，但都是 1~3px 的孤立点）——不做开运算时它会让"行白占比"冲到
    0.12~0.14（越过阈值）⇒ 误触发；开运算把散点整行清空，而 A4 白条（远边实测 36~58px 宽）照样留下。

    返回 dict：`frac`（该行白像素占比 0~1）、`count`（白像素个数）、
    `xmin` / `xmax`（该行白像素的左右列号；该行没有白像素时都是 -1；xmax=-1 表示空行）。
    定档工具靠它"算一次、反复扫闸门"，也方便直接画逐行曲线。
    """
    roi = settings.ZEBRA_GATE_ROI if roi is None else roi
    thresh = settings.ZEBRA_GRAY_THRESH if thresh is None else thresh
    s_max = settings.ZEBRA_S_MAX if s_max is None else s_max
    min_run_px = int(settings.ZEBRA_MIN_RUN_PX if min_run_px is None else min_run_px)
    h, w = frame_bgr.shape[:2]
    stats = {
        "frac": np.zeros(h, dtype=np.float32),
        "count": np.zeros(h, dtype=np.int32),
        "xmin": np.full(h, -1, dtype=np.int32),
        "xmax": np.full(h, -1, dtype=np.int32),
    }
    patch, x0, y0 = _roi_slice(frame_bgr, roi)
    if patch.size == 0:
        return stats
    pw = patch.shape[1]
    on = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY) >= int(thresh)
    if int(s_max) > 0:
        on = on & (cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[..., 1] <= int(s_max))
    on = on.astype(np.uint8)
    if min_run_px > 1:
        L = min(int(min_run_px), pw)
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (L, 1))
        on = cv2.morphologyEx(on, cv2.MORPH_OPEN, kern)      # 只留"横向连成一片"的白
    on = on.astype(bool)
    cnt = on.sum(axis=1).astype(np.int32)
    cols = np.arange(pw, dtype=np.int32)
    xm = np.where(on, cols, pw).min(axis=1)
    xM = np.where(on, cols, -1).max(axis=1)
    rows = slice(y0, y0 + patch.shape[0])
    stats["frac"][rows] = cnt / float(max(1, pw))
    stats["count"][rows] = cnt
    stats["xmin"][rows] = np.where(cnt > 0, xm + x0, -1)
    stats["xmax"][rows] = np.where(cnt > 0, xM + x0, -1)
    return stats


def white_mask(frame_bgr: np.ndarray, roi: Optional[Tuple[float, float, float, float]] = None,
               thresh: Optional[int] = None, s_max: Optional[int] = None) -> np.ndarray:
    """入口同款"白"掩膜（**与原图同尺寸**，便于叠加图/调试）：灰 ≥ 阈值 且 饱和度 ≤ 上限。

    官方口径 `ZEBRA_THRESH=180`（白条灰度实测 236~241、地面 82~118）；本通道用 200 + S≤35
    （证据见 `row_stats` 的 docstring：阳光直晒的红跑道面会越过灰度阈值，但饱和度分得开）。
    """
    roi = settings.ZEBRA_GATE_ROI if roi is None else roi
    thresh = settings.ZEBRA_GRAY_THRESH if thresh is None else thresh
    s_max = settings.ZEBRA_S_MAX if s_max is None else s_max
    patch, x0, y0 = _roi_slice(frame_bgr, roi)
    mask = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
    if patch.size:
        on = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY) >= int(thresh)
        if int(s_max) > 0:
            on = on & (cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[..., 1] <= int(s_max))
        mask[y0:y0 + patch.shape[0], x0:x0 + patch.shape[1]] = on.astype(np.uint8) * 255
    return mask


def merge_runs(flags: Sequence[bool], max_gap: int = 2) -> List[Tuple[int, int]]:
    """把布尔行序列并成 [(起行, 止行)] 段（允许 ≤max_gap 行空洞：白条纹之间本来就有缝）。"""
    runs: List[Tuple[int, int]] = []
    start = None
    gap = 0
    for i, v in enumerate(flags):
        if v:
            if start is None:
                start = i
            gap = 0
        elif start is not None:
            gap += 1
            if gap > max_gap:
                runs.append((start, i - gap))
                start = None
                gap = 0
    if start is not None:
        runs.append((start, len(flags) - 1 - gap))
    return runs


def detect_from_stats(stats: Dict[str, np.ndarray], roi_w: float, row_lo: int,
                      roi: Optional[Tuple[float, float, float, float]] = None,
                      row_frac: Optional[float] = None,
                      min_rows: Optional[int] = None,
                      min_width_frac: Optional[float] = None,
                      min_pixels: Optional[int] = None,
                      gap_merge: int = 2) -> Optional[ZebraHit]:
    """从 `row_stats` 的逐行统计里按闸门挑出斑马线带（纯函数，供扫档反复调用）。

    闸门（默认全来自 settings，实测证据见模块 docstring）：
      ① 段长 ≥ `ZEBRA_MIN_ROWS`：斑马线是成排的，单行/两行白（车道虚线、画线）不算；
      ② 段内白像素横向跨度 ≥ `ZEBRA_MIN_WIDTH_FRAC`×ROI宽：横道横跨整条赛道，
         局部亮块（一张纸/车身反光/反光斑）横向很窄；
      ③ 段内白像素总数 ≥ `ZEBRA_MIN_PIXELS`：挡"一小段很亮但不宽"的东西
         （实测 f226 车身上有一片 12 行 × 占比 0.11 的蓝色反光 ≈850px，本闸门拦下）。

    多条带都过闸门时取 **near_row 最大**的一条（行号向下 = 离车更近）：顶部那条"永久亮带"
    （赛道尽头/围栏）正常应落在 ROI 上沿之外，万一漏进来也不会被当成横道。
    """
    row_frac_th = settings.ZEBRA_ROW_FRAC if row_frac is None else row_frac
    min_rows = settings.ZEBRA_MIN_ROWS if min_rows is None else min_rows
    min_w_frac = settings.ZEBRA_MIN_WIDTH_FRAC if min_width_frac is None else min_width_frac
    min_pix = settings.ZEBRA_MIN_PIXELS if min_pixels is None else min_pixels
    roi = settings.ZEBRA_GATE_ROI if roi is None else roi

    frac, cnt = stats["frac"], stats["count"]
    xmin, xmax = stats["xmin"], stats["xmax"]
    flags = [bool(v >= float(row_frac_th)) for v in frac[row_lo:]]
    best: Optional[ZebraHit] = None
    for r0, r1 in merge_runs(flags, gap_merge):
        a, b = row_lo + r0, row_lo + r1
        if b - a + 1 < int(min_rows):
            continue
        n_pix = int(cnt[a:b + 1].sum())
        if n_pix < int(min_pix):
            continue
        rows_on = cnt[a:b + 1] > 0
        if not rows_on.any():
            continue
        xs_lo = int(xmin[a:b + 1][rows_on].min())
        xs_hi = int(xmax[a:b + 1][rows_on].max())
        span = (xs_hi - xs_lo + 1) / float(max(1.0, roi_w))
        if span < float(min_w_frac):
            continue
        near = float(b)
        if best is None or near > best.near_row:
            rows = b - a + 1
            best = ZebraHit(
                near_row=near, far_row=float(a), x0=float(xs_lo), x1=float(xs_hi),
                n_rows=rows, n_pixels=n_pix,
                coverage=n_pix / max(1.0, rows * roi_w),
                conf=min(1.0, n_pix / max(1.0, 4.0 * min_pix)),
            )
    return best


def detect_zebra_band(frame_bgr: np.ndarray,
                      roi: Optional[Tuple[float, float, float, float]] = None,
                      thresh: Optional[int] = None,
                      s_max: Optional[int] = None,
                      row_frac: Optional[float] = None,
                      min_rows: Optional[int] = None,
                      min_width_frac: Optional[float] = None,
                      min_pixels: Optional[int] = None,
                      min_run_px: Optional[int] = None,
                      gap_merge: int = 2) -> Optional[ZebraHit]:
    """入口用的整链：逐行统计 → 闸门 → 返回**离车最近**那条带的读数（没有则 None）。"""
    roi = settings.ZEBRA_GATE_ROI if roi is None else roi
    thresh = settings.ZEBRA_GRAY_THRESH if thresh is None else thresh
    stats = row_stats(frame_bgr, roi=roi, thresh=thresh, s_max=s_max, min_run_px=min_run_px)
    roi_w = float(max(1, int(frame_bgr.shape[1] * roi[1]) - int(frame_bgr.shape[1] * roi[0])))
    row_lo = int(frame_bgr.shape[0] * roi[2])
    return detect_from_stats(stats, roi_w, row_lo, roi=roi, row_frac=row_frac,
                             min_rows=min_rows, min_width_frac=min_width_frac,
                             min_pixels=min_pixels, gap_merge=gap_merge)


class ZebraDetector:
    """斑马线检测器（薄封装：ROI/阈值可现场覆盖；纯函数本体是 `detect_zebra_band`）。"""

    def __init__(self, roi: Optional[Tuple[float, float, float, float]] = None,
                 thresh: Optional[int] = None,
                 s_max: Optional[int] = None,
                 row_frac: Optional[float] = None,
                 min_rows: Optional[int] = None,
                 min_width_frac: Optional[float] = None,
                 min_pixels: Optional[int] = None,
                 min_run_px: Optional[int] = None) -> None:
        self.roi = tuple(float(v) for v in (roi if roi is not None else settings.ZEBRA_GATE_ROI))
        self.thresh = int(settings.ZEBRA_GRAY_THRESH if thresh is None else thresh)
        self.s_max = int(settings.ZEBRA_S_MAX if s_max is None else s_max)
        self.row_frac = float(settings.ZEBRA_ROW_FRAC if row_frac is None else row_frac)
        self.min_rows = int(settings.ZEBRA_MIN_ROWS if min_rows is None else min_rows)
        self.min_width_frac = float(settings.ZEBRA_MIN_WIDTH_FRAC
                                   if min_width_frac is None else min_width_frac)
        self.min_pixels = int(settings.ZEBRA_MIN_PIXELS if min_pixels is None else min_pixels)
        self.min_run_px = int(settings.ZEBRA_MIN_RUN_PX if min_run_px is None else min_run_px)

    def detect(self, frame_bgr: np.ndarray) -> Optional[ZebraHit]:
        try:
            return detect_zebra_band(frame_bgr, roi=self.roi, thresh=self.thresh,
                                     s_max=self.s_max,
                                     row_frac=self.row_frac, min_rows=self.min_rows,
                                     min_width_frac=self.min_width_frac,
                                     min_pixels=self.min_pixels,
                                     min_run_px=self.min_run_px)
        except Exception as exc:                                    # noqa: BLE001
            print(f"[ZEBRA] ⚠️ 检测异常（本帧按无斑马线）：{exc}")
            return None

    def release(self) -> None:
        return None


class ZebraTracker:
    """进出防抖：连续 `enter_frames` 帧有证据才算"在横道前"，丢 `exit_frames` 帧才算消失。

    与 `cone_detect.ConeTracker` 同口径（`ZEBRA_HYSTERESIS_FRAMES=3` / 退出 = enter//2）：
    单帧误检不会点着 10s 停车，单帧丢失也不会把"正在刹停"判成误检。
    """

    def __init__(self, enter_frames: Optional[int] = None,
                 exit_frames: Optional[int] = None) -> None:
        self.enter_frames = int(settings.ZEBRA_HYSTERESIS_FRAMES
                                if enter_frames is None else enter_frames)
        self.exit_frames = int(max(2, self.enter_frames // 2)
                               if exit_frames is None else exit_frames)
        self.present = False
        self.last: Optional[ZebraHit] = None
        self._seen = 0
        self._missing = 0

    def update(self, hit: Optional[ZebraHit]) -> bool:
        if hit is not None:
            self._seen += 1
            self._missing = 0
            self.last = hit
            if self._seen >= self.enter_frames:
                self.present = True
        else:
            self._missing += 1
            if self.present:
                if self._missing >= self.exit_frames:
                    self.present = False
                    self._seen = 0
            else:
                self._seen = 0
        return self.present

    def reset(self) -> None:
        self.present = False
        self.last = None
        self._seen = 0
        self._missing = 0
