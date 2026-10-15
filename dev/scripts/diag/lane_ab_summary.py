#!/usr/bin/env python3
"""循迹 A/B 会话汇总：把最近几趟的 run.csv 指标 + 该趟用的参数**一行一趟**打出来。

用法（车上）：
    python3 /root/dev/scripts/diag/lane_ab_summary.py            # 最近 4 趟（不含只读 probe）
    python3 /root/dev/scripts/diag/lane_ab_summary.py /root/dev/logs/lane_run_161233 ...
    python3 /root/dev/scripts/diag/lane_ab_summary.py --all      # 全部会话

为什么要有它：CSV 里**不记参数**（kp/kd/速度只在 stdout 里），所以本工具按包装脚本的
命名规则 `lane_run_<ts>/` ↔ `lane_<ts>.log` 把那一趟的参数也读出来 —— 现场跑完直接看
一行就知道"这趟是什么档、跑成什么样"，不用再记纸笔记。

指标口径（A/B 就看这几个）：
    guard > 0        → 触发过护栏（跑偏/画龙）＝ 这档发散
    |err| p90 小     → 跟线稳（<20 很稳；今天发散档是 85~194）
    单侧跳拒 N       → 检测器拒掉的"单侧垃圾读数"帧数（2026-10-15 新护栏）
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
import re
import sys
from collections import Counter

DEFAULT_GLOB = "/root/dev/logs/lane_*"


def _params_of(session_dir: str) -> str:
    """从同名的 stdout 日志里抠出这趟的参数（增益/速度/target）；没有日志就返回提示。"""
    log = session_dir.replace("lane_run_", "lane_") + ".log"
    if not os.path.isfile(log):
        return "（没找到 stdout 日志：/root/dev/logs/lane_<ts>.log）"
    own = f"落盘：{session_dir}/run.csv"          # 日志里必须有"这一趟"的落盘行
    parts = {}
    has_own = False
    with open(log, encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            if own in line:
                has_own = True
            m = re.search(r"kp=([\d.]+)\s+ki=([\d.]+)\s+kd=([\d.]+)", line)
            if m:
                parts["增益"] = f"kp={m.group(1)} ki={m.group(2)} kd={m.group(3)}"
            m = re.search(r"循迹\s+(\d+)us", line)
            if m:
                parts["循迹"] = f"{m.group(1)}us"
            m = re.search(r"target_x=([\d.]+)", line)
            if m:
                parts["target"] = m.group(1)
    out = " | ".join(f"{k} {v}" for k, v in parts.items())
    if not has_own:
        # 日志里没有这一趟（只有只读档那段）→ 参数**可能不是**这一趟实际用的
        return f"⚠️ 日志缺这一趟的记录（下面的参数来自只读档）：{out}"
    return out


def _report(csv_path: str) -> None:
    name = os.path.basename(os.path.dirname(csv_path))
    # 跑动被 kill 时 CSV 最后一行可能截断（字段缺失 → DictReader 给 None）——
    # 直接把缺字段的行剔掉，别让一行坏数据把整张表打挂（2026-10-15 现场踩到）。
    rows = [r for r in csv.DictReader(open(csv_path, newline="", encoding="utf-8"))
            if r.get("phase") and r.get("angle_deg")]
    if not rows:
        print(f"{name:24s} （空/截断）")
        return
    es = sorted(abs(float(r["error_px"])) for r in rows if r.get("error_px"))
    ph = Counter(r["phase"] for r in rows)
    tk = sum(1 for r in rows if r.get("both_sides") == "1")
    sj = sum(1 for r in rows if "1side-jump" in (r.get("tag") or ""))

    def pct(q: float) -> int:
        return int(es[min(len(es) - 1, int(len(es) * q))]) if es else 0

    print(f"{name:24s} 帧{len(rows):5d} 双侧{100 * tk / max(1, len(rows)):4.0f}% "
          f"|err| p50 {pct(.5):4d} p90 {pct(.9):4d} max {int(es[-1]) if es else 0:4d} "
          f"单侧跳拒 {sj:3d}")
    # 舵角抖动 / 饱和（2026-10-15 新增：往上加 kd 时最该盯的两个量）
    #   抖动 = 相邻帧舵角变化量；饱和 = 舵角贴着 ±15° 限幅的帧占比。
    #   基线参考（当天实测）：kd0.12 → |Δ|p90 0.7~0.9°；kd0.24 → 0.3~1.2°。
    #   若某档跳到 3°+ → D 项在放大检测噪声（"抽舵"），别再往上加。
    ang = [float(r["angle_deg"]) for r in rows
           if r["phase"] in ("track", "single", "single-follow")]
    if len(ang) >= 2:
        import statistics
        dlt = [abs(b - a) for a, b in zip(ang, ang[1:])]
        med = statistics.median(ang)
        hits = sum(1 for a in ang if abs(a - med) >= 14.5)
        d90 = sorted(dlt)[min(len(dlt) - 1, int(len(dlt) * 0.9))]
        print(f"{'':24s} 抖动 |Δ舵角|/帧 p90 {d90:.2f}° max {max(dlt):.1f}° "
              f"饱和(±14.5°) {100 * hits / len(ang):.1f}%  中位舵角 {med:.1f}°")
    print(f"{'':24s} 相位 {dict(sorted(ph.items()))}")
    print(f"{'':24s} 参数 {_params_of(os.path.dirname(csv_path))}")


def main() -> int:
    ap = argparse.ArgumentParser(description="循迹 A/B 会话汇总")
    ap.add_argument("dirs", nargs="*", help="会话目录（默认最近 4 趟非 probe 会话）")
    ap.add_argument("--all", action="store_true", help="列出全部会话")
    args = ap.parse_args()

    targets = []
    if args.dirs:
        targets = list(args.dirs)
    else:
        cand = [d for d in glob.glob(DEFAULT_GLOB)
                if os.path.isfile(os.path.join(d, "run.csv")) and "probe" not in d]
        cand.sort(key=os.path.getmtime, reverse=True)
        targets = cand if args.all else cand[:4]
    if not targets:
        print(f"[AB] 没找到会话目录（{DEFAULT_GLOB}）；跑完一趟再来")
        return 1
    for d in targets:
        csv_path = d if d.endswith(".csv") else os.path.join(d, "run.csv")
        if not os.path.isfile(csv_path):
            print(f"[AB] 跳过（没有 run.csv）：{d}")
            continue
        try:
            _report(csv_path)
        except Exception as exc:                      # noqa: BLE001
            print(f"[AB] ⚠️ {os.path.basename(os.path.dirname(csv_path))} 读不了：{exc}")
    print("\n[AB] 判定：guard>0 = 这档发散；guard 为 0/缺失且 p90 小（<20）＝ 这档能用。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
