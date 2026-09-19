#!/usr/bin/env python3
"""按拍摄片段整组切分检测数据集（产出 YOLO 的 images/labels 结构）。

为什么必须按片段切分（doc/数据与模型方案.md §4）：
相邻帧几乎相同，按图片随机划分会让同一段视频的帧同时出现在 train 和 val，
验证指标虚高、实车失效。Roboflow 的默认划分就是随机的，所以不要用它划分，
导出时选 "no split"，回到本地用本脚本切。

分组规则：
- `frame_YYYYMMDD_HHMMSS_mmm_NNNN` 命名：按时间戳排序，相邻间隔 > --gap-sec 即视为新片段；
- 其它命名（Roboflow 的 `xxx_jpg.rf.<hash>.jpg` 等）：用 dHash 8x8 + 汉明距离聚类，
  画面几乎相同的归为同一组。注意 dHash 是 O(n^2) 比较，大目录请耐心或用 --gap-sec 路线。

整组只进 train/val/test 之一，比例默认 8:1:1。

用法：
    # 预览（不落盘），看切分是否合理
    python scripts/split_dataset.py --src samples/smartcar5g.v1-test1.yolov11/train \
                                    --src samples/smartcar5g.v1-test1.yolov11/valid \
                                    --dst datasets/smartcar2026 --dry-run

    # 真正切分
    python scripts/split_dataset.py --src .../train --src .../valid --dst datasets/smartcar2026

    # 加上 INT8 校准图目录（从 train 里随机抽，不参与训练）
    python scripts/split_dataset.py ... --calib 30
"""
from __future__ import annotations

import argparse
import random
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

# 权威类别表：doc/具体实施方案.md §3.1.3 / dev/config/model_profile.yaml 的 smartcar2026
DEFAULT_CLASSES = [
    "blue_board", "crosswalk", "traffic_light_off", "traffic_light_red",
    "traffic_light_green", "obstacle_cone_blue", "obstacle_cone_red", "parking_area",
]

# frame_YYYYMMDD_HHMMSS_<亚秒>_NNNN，亚秒是 3 位毫秒或 6 位微秒（实测车上采集写的是 6 位）。
# 用 search 而非 match：Roboflow 导出会在原名后追加 `_jpg.rf.<hash>`，时间戳仍在中间。
FRAME_RE = re.compile(r"frame_(\d{8})_(\d{6})_(\d{3,6})")


class Item:
    """一张图 + 它的标注路径。"""

    __slots__ = ("img", "lbl", "stem", "ts", "group")

    def __init__(self, img: Path, lbl: Optional[Path]):
        self.img = img
        self.lbl = lbl
        self.stem = img.stem
        self.ts: Optional[datetime] = _parse_ts(img.stem)
        self.group = -1


def _parse_ts(stem: str) -> Optional[datetime]:
    m = FRAME_RE.search(stem)
    if not m:
        return None
    try:
        return datetime.strptime(m.group(1) + m.group(2) + m.group(3), "%Y%m%d%H%M%S%f")
    except ValueError:
        # 容忍亚秒位异常，退回秒级（同一秒内多帧会被算作同一时刻，但有 gap_sec 兜底）
        try:
            return datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
        except ValueError:
            return None


# ------------------------------------------------------------------ 收集
def _resolve_pair(src: Path) -> Tuple[Optional[Path], Optional[Path]]:
    """给一个 --src 目录，判断 images / labels 两个目录各在哪。"""
    if (src / "images").is_dir():
        imgd = src / "images"
        labs = [src / "labels", src.parent / "labels" / src.name]
        lbld = next((p for p in labs if p.is_dir()), None)
        return imgd, lbld
    if src.is_dir():
        # 平铺目录：图片就在这一层
        lbld = src / "labels"
        return src, lbld if lbld.is_dir() else None
    return None, None


def collect(sources: Sequence[Path]) -> List[Item]:
    items: List[Item] = []
    for src in sources:
        imgd, lbld = _resolve_pair(src)
        if imgd is None:
            print(f"[warn] 跳过不存在的源目录: {src}")
            continue
        imgs = sorted(p for p in imgd.iterdir()
                      if p.is_file() and p.suffix.lower() in IMG_EXTS)
        for img in imgs:
            lbl = None
            if lbld is not None:
                cand = lbld / (img.stem + ".txt")
                lbl = cand if cand.is_file() else None
            items.append(Item(img, lbl))
    return items


# ------------------------------------------------------------------ 分组
def group_by_time(items: List[Item], gap_sec: float) -> List[List[Item]]:
    """按时间戳间隔切片段；没有时间戳的返回空列表。"""
    timed = sorted((it for it in items if it.ts is not None), key=lambda x: x.ts)
    groups: List[List[Item]] = []
    cur: List[Item] = []
    for it in timed:
        if cur and (it.ts - cur[-1].ts).total_seconds() > gap_sec:
            groups.append(cur)
            cur = []
        cur.append(it)
    if cur:
        groups.append(cur)
    return groups


def _parse_seq(stem: str) -> Optional[Tuple[str, int]]:
    """从 `stop_0017_png_png.rf.<hash>` 里取出 ('stop', 17)；取不到返回 None。"""
    s = re.sub(r"\.rf\.[0-9a-f]+$", "", stem, flags=re.I)
    s = re.sub(r"(_(png|jpg|jpeg|bmp|webp))+$", "", s, flags=re.I)
    m = re.match(r"^(.+?)_(\d+)$", s)
    if not m:
        return None
    return m.group(1), int(m.group(2))


def group_by_index(items: List[Item], gap_idx: int
                   ) -> Tuple[List[List[Item]], List[Item]]:
    """按 `<前缀>_<序号>` 的序号间隔分组（同前缀内间隔 > gap_idx 视为新片段）。

    这是没有时间戳时的退路。因为不知道原始帧率，gap_idx 取小（默认 5）偏保守：
    宁可多切几组（组越小越可能把同段拆开 → 更不容易泄漏），不要少切。
    返回 (分组, 没能解析出序号、需要走 dHash 的剩余项)。
    """
    buckets: Dict[str, List[Tuple[int, Item]]] = {}
    leftover: List[Item] = []
    for it in items:
        parsed = _parse_seq(it.stem)
        if parsed is None:
            leftover.append(it)
        else:
            buckets.setdefault(parsed[0], []).append((parsed[1], it))

    groups: List[List[Item]] = []
    for lst in buckets.values():
        lst.sort(key=lambda x: x[0])
        cur: List[Tuple[int, Item]] = []
        for idx, it in lst:
            if cur and idx - cur[-1][0] > gap_idx:
                groups.append([x[1] for x in cur])
                cur = []
            cur.append((idx, it))
        if cur:
            groups.append([x[1] for x in cur])
    return groups, leftover


def _dhash(path: Path, size: int = 8) -> Optional[int]:
    """dHash 8x8 -> 64bit 指纹（需要 cv2；没有就返回 None）。"""
    try:
        import cv2
        import numpy as np
    except ImportError:
        return None
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    small = cv2.resize(img, (size + 1, size), interpolation=cv2.INTER_AREA)
    diff = small[:, 1:] > small[:, :-1]
    bits = np.packbits(diff.flatten())
    return int.from_bytes(bits.tobytes(), "big")


def group_by_dhash(items: List[Item], threshold: int = 6) -> List[List[Item]]:
    """对没有时间戳的图做 dHash 聚类（并查集，汉明距离 <= threshold 归为一组）。"""
    hashes: List[Optional[int]] = []
    for it in items:
        h = _dhash(it.img)
        hashes.append(h)
    n = len(items)
    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for i in range(n):
        hi = hashes[i]
        if hi is None:
            continue
        for j in range(i + 1, n):
            hj = hashes[j]
            if hj is None:
                continue
            if bin(hi ^ hj).count("1") <= threshold:
                union(i, j)

    buckets: Dict[int, List[Item]] = {}
    for i, it in enumerate(items):
        buckets.setdefault(find(i), []).append(it)
    return list(buckets.values())


def assign(groups: List[List[Item]], ratios: Sequence[float], seed: int
           ) -> Dict[str, List[List[Item]]]:
    """贪心：每组分配给「配额缺口最大」的集合。

    判据必须是"还剩多少配额没填满"（target - filled），不能是"离目标比例多近"——
    后者会让第一个大组被判给 val/test（10% 的小目标看着更"接近"），train 反而饿死。

    组顺序只做带种子的打乱、**不按大小排序**：排序会让大组全进 train、小组碎片填 val/test，
    破坏 doc §4 要求的「验证/测试集保持真实类别分布」。
    """
    names = ["train", "val", "test"]
    ratios = list(ratios) + [0.0] * (3 - len(ratios))
    rng = random.Random(seed)
    order = list(groups)
    rng.shuffle(order)

    total = sum(len(g) for g in groups) or 1
    target = {n: ratios[i] * total for i, n in enumerate(names)}
    filled = {n: 0 for n in names}
    out: Dict[str, List[List[Item]]] = {n: [] for n in names}

    for g in order:
        cands = [n for n in names if target[n] - filled[n] > 0]
        if cands:
            fits = [n for n in cands if len(g) <= target[n] - filled[n]]
            if fits:
                # 组装得下：给缺口最大的集合
                best = max(fits, key=lambda n: target[n] - filled[n])
            else:
                # 组比所有剩余缺口都大：挑超出最少的，别把某个集合撑爆
                best = min(cands, key=lambda n: len(g) - (target[n] - filled[n]))
        else:
            best = min(names, key=lambda n: filled[n] - target[n])
        out[best].append(g)
        filled[best] += len(g)
    return out


# ------------------------------------------------------------------ 落盘
def class_hist(items: List[Item], nc: int) -> List[int]:
    hist = [0] * nc
    for it in items:
        if it.lbl is None:
            continue
        for line in it.lbl.read_text(encoding="utf-8", errors="ignore").split("\n"):
            line = line.strip()
            if not line:
                continue
            try:
                cid = int(float(line.split()[0]))
            except (ValueError, IndexError):
                continue
            if 0 <= cid < nc:
                hist[cid] += 1
    return hist


def write_split(dst: Path, name: str, items: List[Item], move: bool) -> None:
    imgd = dst / "images" / name
    lbld = dst / "labels" / name
    imgd.mkdir(parents=True, exist_ok=True)
    lbld.mkdir(parents=True, exist_ok=True)
    for it in items:
        target_img = imgd / it.img.name
        if move:
            shutil.move(str(it.img), str(target_img))
        else:
            shutil.copy2(str(it.img), str(target_img))
        if it.lbl is not None:
            target_lbl = lbld / it.lbl.name
            if move:
                shutil.move(str(it.lbl), str(target_lbl))
            else:
                shutil.copy2(str(it.lbl), str(target_lbl))


def write_yaml(dst: Path, classes: Sequence[str]) -> None:
    names = "\n".join(f"  {i}: {n}" for i, n in enumerate(classes))
    (dst / "data.yaml").write_text(
        f"# 由 scripts/split_dataset.py 生成（按拍摄片段整组切分）\n"
        f"# 类别表依据 doc/具体实施方案.md §3.1.3\n"
        f"path: {dst.as_posix()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"test: images/test\n"
        f"\n"
        f"nc: {len(classes)}\n"
        f"names:\n{names}\n",
        encoding="utf-8",
    )


# ------------------------------------------------------------------ 主流程
def main() -> int:
    ap = argparse.ArgumentParser(description="按拍摄片段整组切分检测数据集")
    ap.add_argument("--src", action="append", required=True, type=Path,
                    help="源目录（可多次指定）；支持 <dir>/images+labels 或平铺目录")
    ap.add_argument("--dst", type=Path, default=Path("datasets/smartcar2026"),
                    help="输出根目录（默认 datasets/smartcar2026）")
    ap.add_argument("--gap-sec", type=float, default=10.0,
                    help="时间戳间隔超过该值即视为新片段（默认 10s）")
    ap.add_argument("--ratios", type=float, nargs=3, default=[0.8, 0.1, 0.1],
                    help="train/val/test 比例（默认 0.8 0.1 0.1）")
    ap.add_argument("--gap-idx", type=int, default=5,
                    help="无时间戳时按序号分组的间隔阈值，越小越保守（默认 5）")
    ap.add_argument("--dhash-threshold", type=int, default=6,
                    help="dHash 汉明距离阈值，越小越严格（默认 6）")
    ap.add_argument("--classes", nargs="+", default=DEFAULT_CLASSES,
                    help="类别表（默认 8 类，顺序即 class id）")
    ap.add_argument("--calib", type=int, default=0,
                    help="额外从 train 抽 N 张作 INT8 校准图，放 dst/calib/（默认 0=不抽）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--move", action="store_true", help="移动而非复制源文件")
    ap.add_argument("--dry-run", action="store_true", help="只打印切分结果，不落盘")
    ap.add_argument("--force", action="store_true", help="目标目录非空时也继续（会混入旧文件）")
    args = ap.parse_args()

    nc = len(args.classes)
    items = collect(args.src)
    if not items:
        print("[error] 没找到任何图片，检查 --src")
        return 1

    no_lbl = [it for it in items if it.lbl is None]
    print(f"共收集 {len(items)} 张图，其中 {len(no_lbl)} 张没有对应标注（将作为负样本）")

    # 三级分组：时间戳 > 序号 > dHash（可靠性依次下降，逐级降级）
    groups = group_by_time(items, args.gap_sec)
    timed = [it for it in items if it.ts is not None]
    rest = [it for it in items if it.ts is None]
    print(f"① 时间戳分组: {len(timed)} 张 -> {len(groups)} 个片段")

    if rest:
        seq_groups, rest = group_by_index(rest, args.gap_idx)
        groups += seq_groups
        print(f"② 序号分组  : {sum(len(g) for g in seq_groups)} 张 -> "
              f"{len(seq_groups)} 个片段（gap_idx={args.gap_idx}）")

    if rest:
        dhash_groups = group_by_dhash(rest, args.dhash_threshold)
        biggest = max(len(g) for g in dhash_groups)
        print(f"③ dHash 分组: {len(rest)} 张 -> {len(dhash_groups)} 个片段")
        if biggest > 0.15 * len(rest):
            print(f"  [warn] dHash 把 {biggest}/{len(rest)} 张并成了一组，明显过合并。\n"
                  f"         赛道图纹理少，dHash 区分度差；调小 --dhash-threshold（如 2~3），\n"
                  f"         或让源文件名保留 frame_ 时间戳，走①更可靠。")
        groups += dhash_groups

    if not groups:
        print("[error] 分组结果为空")
        return 1

    sizes = sorted((len(g) for g in groups), reverse=True)
    print(f"共 {len(groups)} 组；最大组 {sizes[0]} 张，最小组 {sizes[-1]} 张，"
          f"中位数 {sizes[len(sizes)//2]} 张")

    buckets = assign(groups, args.ratios, args.seed)
    splits = {n: [it for g in buckets[n] for it in g] for n in buckets}

    # ---- 报表 ----
    print(f"\n{'集合':<6}{'图片':>7}{'组数':>6}   各类实例数")
    for n in ("train", "val", "test"):
        hist = class_hist(splits[n], nc)
        print(f"{n:<6}{len(splits[n]):>7}{len(buckets[n]):>6}   {hist}")

    for i, n in enumerate(("train", "val", "test")):
        if args.ratios[i] > 0 and not splits[n]:
            print(f"\n[warn] {n} 集为空：整组切分下每个集合至少要分到一整组，"
                  f"当前组太大或组数太少填不进去。\n"
                  f"       换成更多更小的片段，或调整 --ratios（例如不要 test）。")

    tot = [sum(class_hist(splits[n], nc)[i] for n in splits) for i in range(nc)]
    print(f"\n{'类别':<24}{'train':>7}{'val':>6}{'test':>6}{'合计':>7}")
    for i, cname in enumerate(args.classes):
        row = [class_hist(splits[n], nc)[i] for n in ("train", "val", "test")]
        print(f"{cname:<24}{row[0]:>7}{row[1]:>6}{row[2]:>6}{tot[i]:>7}")

    neg = {n: sum(1 for it in splits[n] if it.lbl is None) for n in splits}
    for n in ("train", "val", "test"):
        if splits[n]:
            pct = neg[n] / len(splits[n]) * 100
            print(f"\n负样本 {n}: {neg[n]} 张（{pct:.1f}%）")

    if args.dry_run:
        print("\n[dry-run] 未落盘。去掉 --dry-run 才会真正切分。")
        return 0

    if args.dst.exists() and any(args.dst.iterdir()) and not args.force:
        print(f"\n[error] 目标目录非空: {args.dst}\n"
              f"        先清空它，或加 --force 承担混入旧文件的风险")
        return 1

    for n in ("train", "val", "test"):
        write_split(args.dst, n, splits[n], args.move)
    write_yaml(args.dst, args.classes)

    if args.calib > 0 and splits["train"]:
        calib_dir = args.dst / "calib"
        calib_dir.mkdir(parents=True, exist_ok=True)
        rng = random.Random(args.seed)
        picked = rng.sample(splits["train"], min(args.calib, len(splits["train"])))
        for it in picked:
            shutil.copy2(str(it.img), str(calib_dir / it.img.name))
        print(f"已抽 {len(picked)} 张到 {calib_dir}")

    print(f"\n[OK] 切分完成 -> {args.dst}")
    print(f"     data.yaml: {args.dst / 'data.yaml'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
