#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实验室跑道（广告纸）设计图生成器 —— 输出 drawio（可编辑）+ PDF（1:1 打印）。

尺寸依据 `../比赛规则/规则概要.md` 与规则初稿 §2.2.1~2.2.3、图 2-2 / 图 9：
  - 红底（跑道色）+ 两侧白线（白线 = 赛道分界线，就是赛道边缘）；
  - 白线外侧再补一条红，把整幅补到 1.4m 宽（真实跑道的白线是车道线，外侧还有路面）；
  - 斑马线：白条 105mm × 300mm、条间隙 105mm、中间那条压在赛道中线上；
  - 停车区：长 1m × 赛道宽，黄胶带 100mm，沿赛道中线再分两格；
    挡板与付款码**用实物道具**（团队已有），图上不印（要印占位就把 DRAW_BOARD / DRAW_QR 改 True）。

所有尺寸单位 **mm**，1:1 输出。改参数后重跑：
    python gen_lab_track.py              # 出 drawio + PDF
    python gen_lab_track.py --preview     # 另外出预览图（需要 numpy + cv2）
"""
from __future__ import annotations

import base64
import math
import os
import random
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))

# ============================================================ 参数（mm）
LEN_MM = 7000            # 广告纸总长（沿行驶方向，起点在左）
LANE_MM = 1220           # 白线内的红底宽度（赛道有效宽度）
WHITE_MM = 40            # 两侧白线宽度（= 赛道分界线）
OUTER_RED_MM = 50        # 白线外侧再补的红条宽度 → 总宽 1400
WID_MM = OUTER_RED_MM * 2 + WHITE_MM * 2 + LANE_MM      # 1400
BLEED_MM = 0             # 出血：店家要求时改 5（四周红底各外扩 5mm）

C_RED = "#924C4A"        # 跑道红（取自规则插图 图2-2 / 图9）
C_WHITE = "#FFFFFF"
C_YELLOW = "#FEFF00"     # 黄色胶带
C_BLUE = "#105FC6"       # 蓝板
C_BLACK = "#000000"

# 斑马线（规则 §2.2.3）
XW_X0_MM = 1400          # 起点 → 斑马线前缘
XW_LEN_MM = 300          # 斑马线沿行驶方向长度
XW_BAR_MM = 105          # 单条白条宽度（10.5cm）
XW_GAP_MM = 105          # 白条间隙（10.5cm）
XW_BARS = 5              # 条数（奇数 → 中间那条压在赛道中线上）

# 停车区（规则 §2.2.1-4）
PK_LEN_MM = 1000         # 停车区长度
PK_TAPE_MM = 100         # 黄胶带宽度（10cm）
# 挡板与付款码**都用实物**（团队已有道具），所以图上不印，停车区只留黄胶带。
# 需要打印占位时把下面两个开关改 True。
DRAW_BOARD = False       # 是否在图上印蓝板占位
DRAW_QR = False          # 是否在图上印付款码占位
PK_BOARD_L_MM = 400      # 蓝板长度（实物 40cm × 50cm，竖直放）
PK_BOARD_T_MM = 100      # 蓝板在图上画出的厚度（实物约 2cm，画粗一点便于辨认/检出）
PK_BOARD_SLOT = "top"    # 蓝板挡哪一格："top" / "bottom"
PK_QR_MM = 100           # 付款码边长（10cm × 10cm）
PK_QR_ANGLE = 45         # 与行驶方向的倾角（规则 §2.2.2）
PK_QR_FROM_END_MM = 300  # 付款码中心距广告纸尾端的距离

# ============================================================ 画布
W_MM = WID_MM + 2 * BLEED_MM
L_MM = LEN_MM + 2 * BLEED_MM
CY = BLEED_MM + WID_MM / 2.0                    # 赛道中线（y）
LANE_Y0 = BLEED_MM + OUTER_RED_MM + WHITE_MM    # 白线内红底的上沿
LANE_Y1 = LANE_Y0 + LANE_MM                     # 白线内红底的下沿

PK_X0 = BLEED_MM + LEN_MM - PK_LEN_MM           # 停车区入口胶带外沿
PK_X1 = BLEED_MM + LEN_MM                       # 停车区尾端胶带外沿
SLOT_TOP_CY = (LANE_Y0 + (CY - PK_TAPE_MM / 2.0)) / 2.0     # 上格中心
SLOT_BOT_CY = ((CY + PK_TAPE_MM / 2.0) + LANE_Y1) / 2.0     # 下格中心


# ============================================================ 图形基元
def rect(x, y, w, h, color):
    return ("rect", float(x), float(y), float(w), float(h), color)


def poly(points, color):
    return ("poly", [(float(px), float(py)) for px, py in points], color)


def qr(cx, cy, size, angle_deg):
    """付款码占位：向量输出时摊成小方块，drawio 里换成嵌入图片。

    自带白色底（真实付款码是白底黑码的贴纸），所以先铺一层白方块再画码点。
    """
    return ("qr", float(cx), float(cy), float(size), float(angle_deg))


def rotate(px, py, cx, cy, angle_deg):
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    dx, dy = px - cx, py - cy
    return (cx + dx * ca - dy * sa, cy + dx * sa + dy * ca)


def qr_modules(n=21, seed=20260916):
    """生成一个"长得像二维码"的占位图案（不可扫描），返回 n×n 布尔矩阵。"""
    rnd = random.Random(seed)
    m = [[False] * n for _ in range(n)]

    def finder(r0, c0):
        for r in range(7):
            for c in range(7):
                m[r0 + r][c0 + c] = (r in (0, 6) or c in (0, 6)) or (2 <= r <= 4 and 2 <= c <= 4)

    finder(0, 0)
    finder(0, n - 7)
    finder(n - 7, 0)
    reserved = set()
    for r0, c0 in ((0, 0), (0, n - 7), (n - 7, 0)):
        for r in range(-1, 8):
            for c in range(-1, 8):
                if 0 <= r0 + r < n and 0 <= c0 + c < n:
                    reserved.add((r0 + r, c0 + c))
    for i in range(8, n - 8):                       # 定时图案
        m[6][i] = m[i][6] = (i % 2 == 0)
        reserved.add((6, i))
        reserved.add((i, 6))
    for r in range(n):
        for c in range(n):
            if (r, c) not in reserved:
                m[r][c] = rnd.random() < 0.5
    return m


def qr_shapes(cx, cy, size, angle_deg):
    """把二维码占位图案摊成一组已旋转的小方块（矢量输出用），含白色底。"""
    n = 21
    mod = size / n
    out = [poly([rotate(cx - size / 2.0, cy - size / 2.0, cx, cy, angle_deg),
                 rotate(cx + size / 2.0, cy - size / 2.0, cx, cy, angle_deg),
                 rotate(cx + size / 2.0, cy + size / 2.0, cx, cy, angle_deg),
                 rotate(cx - size / 2.0, cy + size / 2.0, cx, cy, angle_deg)], C_WHITE)]
    for r, row in enumerate(qr_modules(n)):
        for c, on in enumerate(row):
            if not on:
                continue
            x0 = cx - size / 2.0 + c * mod
            y0 = cy - size / 2.0 + r * mod
            out.append(poly([rotate(x0, y0, cx, cy, angle_deg),
                             rotate(x0 + mod, y0, cx, cy, angle_deg),
                             rotate(x0 + mod, y0 + mod, cx, cy, angle_deg),
                             rotate(x0, y0 + mod, cx, cy, angle_deg)], C_BLACK))
    return out


def png_from_matrix(m, scale=16, quiet=1):
    """把布尔矩阵写成 8 位灰度 PNG（纯 Python，无依赖），返回 bytes。"""
    n = len(m)
    px = (n + 2 * quiet) * scale
    rows = []
    for r in range(px):
        rr = r // scale - quiet
        line = bytearray(b"\x00")                     # filter type 0
        for c in range(px):
            cc = c // scale - quiet
            dark = 0 <= rr < n and 0 <= cc < n and m[rr][cc]
            line.append(0 if dark else 255)
        rows.append(bytes(line))
    raw = b"".join(rows)

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data +
                struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", px, px, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9))
            + chunk(b"IEND", b""))


def build_shapes():
    """按参数生成全部图形（顺序即绘制顺序）。"""
    s = []
    s.append(rect(0, 0, L_MM, W_MM, C_RED))                      # 红底（铺满整幅）
    s.append(rect(0, BLEED_MM + OUTER_RED_MM, L_MM, WHITE_MM, C_WHITE))     # 白线（上）
    s.append(rect(0, LANE_Y1, L_MM, WHITE_MM, C_WHITE))                     # 白线（下）

    # ---- 斑马线
    span = XW_BARS * XW_BAR_MM + (XW_BARS - 1) * XW_GAP_MM
    y = CY - span / 2.0
    for _ in range(XW_BARS):
        s.append(rect(BLEED_MM + XW_X0_MM, y, XW_LEN_MM, XW_BAR_MM, C_WHITE))
        y += XW_BAR_MM + XW_GAP_MM

    # ---- 停车区：入口胶带 / 尾端胶带 / 中线分隔胶带
    s.append(rect(PK_X0, LANE_Y0, PK_TAPE_MM, LANE_MM, C_YELLOW))
    s.append(rect(PK_X1 - PK_TAPE_MM, LANE_Y0, PK_TAPE_MM, LANE_MM, C_YELLOW))
    s.append(rect(PK_X0, CY - PK_TAPE_MM / 2.0, PK_LEN_MM, PK_TAPE_MM, C_YELLOW))

    # ---- 蓝板占位（贴在入口胶带内侧，挡住其中一格；默认不印，用实物板）
    if DRAW_BOARD:
        board_cy = SLOT_TOP_CY if PK_BOARD_SLOT == "top" else SLOT_BOT_CY
        s.append(rect(PK_X0 + PK_TAPE_MM, board_cy - PK_BOARD_L_MM / 2.0,
                      PK_BOARD_T_MM, PK_BOARD_L_MM, C_BLUE))

    # ---- 付款码占位（每格尾部一个，45°；默认不印，用实物付款码）
    if DRAW_QR:
        qx = PK_X1 - PK_QR_FROM_END_MM
        s.append(qr(qx, SLOT_TOP_CY, PK_QR_MM, PK_QR_ANGLE))
        s.append(qr(qx, SLOT_BOT_CY, PK_QR_MM, PK_QR_ANGLE))
    return s


def expand(shapes, kinds=("qr",)):
    """把特殊基元（付款码）摊成矢量小方块。"""
    out = []
    for sh in shapes:
        if sh[0] in kinds:
            out.extend(qr_shapes(sh[1], sh[2], sh[3], sh[4]))
        else:
            out.append(sh)
    return out


# ============================================================ PDF（矢量，1:1）
PT_PER_MM = 72.0 / 25.4


def write_pdf(shapes, path):
    w_pt, h_pt = L_MM * PT_PER_MM, W_MM * PT_PER_MM

    def y_of(y_mm):
        return h_pt - y_mm * PT_PER_MM

    body = []
    cur = None
    for sh in expand(shapes):
        col = sh[-1]
        if col != cur:
            body.append("{:.4f} {:.4f} {:.4f} rg".format(
                int(col[1:3], 16) / 255.0, int(col[3:5], 16) / 255.0, int(col[5:7], 16) / 255.0))
            cur = col
        if sh[0] == "rect":
            _, x, y, w, h, _c = sh
            body.append(f"{x * PT_PER_MM:.3f} {y_of(y + h):.3f} "
                        f"{w * PT_PER_MM:.3f} {h * PT_PER_MM:.3f} re f")
        else:
            _, pts, _c = sh
            body.append(" ".join(f"{px * PT_PER_MM:.3f} {y_of(py):.3f} m" for px, py in pts) + " h f")
    content = ("\n".join(body) + "\n").encode("ascii")

    objs = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {w_pt:.3f} {h_pt:.3f}] "
         f"/Resources << >> /Contents 4 0 R >>").encode("ascii"),
        (b"<< /Length " + str(len(content)).encode("ascii") + b" >>\nstream\n"
         + content + b"endstream"),
    ]
    buf = bytearray(b"%PDF-1.6\n%\xe2\xe3\xcf\xd3\n")
    offsets = []
    for i, o in enumerate(objs, start=1):
        offsets.append(len(buf))
        buf += f"{i} 0 obj\n".encode("ascii") + o + b"\nendobj\n"
    xref_pos = len(buf)
    buf += f"xref\n0 {len(objs) + 1}\n".encode("ascii") + b"0000000000 65535 f \n"
    for off in offsets:
        buf += f"{off:010d} 00000 n \n".encode("ascii")
    buf += (f"trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\n"
            f"startxref\n{xref_pos}\n%%EOF\n").encode("ascii")
    with open(path, "wb") as f:
        f.write(bytes(buf))


def verify_pdf(path):
    """自检：xref 偏移是否都指向对应对象（手写 PDF 最容易错的地方）。"""
    data = open(path, "rb").read()
    i = data.rfind(b"startxref")
    if i < 0:
        return "no startxref"
    xref_pos = int(data[i + 9:].split()[0])
    if data[xref_pos:xref_pos + 4] != b"xref":
        return f"startxref {xref_pos} does not point at 'xref'"
    lines = data[xref_pos:].split(b"\n")
    count = int(lines[1].split()[1])            # 第 1 行形如 "0 5"
    for n in range(1, count):
        off = int(lines[2 + n].split()[0])
        if not data[off:off + 20].split(b"\n")[0].startswith(f"{n} 0 obj".encode()):
            return f"obj {n} offset mismatch"
    return f"ok ({count - 1} objects, {len(data)} bytes)"


# ============================================================ drawio
PX_PER_MM = 100.0 / 25.4          # draw.io 页面单位：100 px = 1 inch


def esc(s):
    """drawio 的 value 是 XML 属性：HTML 片段（如 <br>）必须转义，否则文件不是合法 XML。"""
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
             .replace('"', "&quot;"))


def _cell(cid, value, style, x, y, w, h):
    return (f'        <mxCell id="{cid}" value="{esc(value)}" style="{style}" '
            f'vertex="1" parent="1">\n'
            f'          <mxGeometry x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" '
            f'height="{h:.2f}" as="geometry" />\n        </mxCell>\n')


def _edge(cid, value, style, x1, y1, x2, y2):
    return (f'        <mxCell id="{cid}" value="{esc(value)}" style="{style}" '
            f'edge="1" parent="1">\n'
            f'          <mxGeometry relative="1" as="geometry">\n'
            f'            <mxPoint x="{x1:.1f}" y="{y1:.1f}" as="sourcePoint" />\n'
            f'            <mxPoint x="{x2:.1f}" y="{y2:.1f}" as="targetPoint" />\n'
            f'          </mxGeometry>\n        </mxCell>\n')


def _dim(cid, x1, y1, x2, y2, label):
    style = ("endArrow=classic;startArrow=classic;html=1;rounded=0;strokeWidth=1;"
             "strokeColor=#C0392B;fontColor=#C0392B;fontSize=11;labelBackgroundColor=#FFFFFF;"
             "align=center;verticalAlign=middle;")
    return _edge(cid, label, style, x1, y1, x2, y2)


def _text(cid, value, x, y, w, h, size=11, color="#333333", align="left", bold=False):
    style = (f"text;html=1;strokeColor=none;fillColor=none;align={align};verticalAlign=middle;"
             f"whiteSpace=wrap;rounded=0;fontSize={size};fontColor={color};"
             f"{'fontStyle=1;' if bold else ''}")
    return _cell(cid, value, style, x, y, w, h)


def _qr_image_cell(cid, cx_mm, cy_mm, size_mm, angle_deg, k, ox, oy):
    """付款码：drawio 里用一张嵌入图片 + rotation，避免上千个小方块单元。"""
    b64 = base64.b64encode(png_from_matrix(qr_modules(21), scale=16)).decode("ascii")
    x = ox + (cx_mm - size_mm / 2.0) * k
    y = oy + (cy_mm - size_mm / 2.0) * k
    style = (f"shape=image;html=1;imageAspect=0;strokeColor=none;rotation={angle_deg:g};"
             f"image=data:image/png;base64,{b64}")
    return _cell(cid, "", style, x, y, size_mm * k, size_mm * k)


def _shape_cell(cid, sh, k, ox=0.0, oy=0.0):
    """drawio 只处理 rect 与 qr（旋转过的码点用嵌入图片，不用 polygon 单元）。"""
    if sh[0] == "qr":
        return _qr_image_cell(cid, sh[1], sh[2], sh[3], sh[4], k, ox, oy)
    _, x, y, w, h, col = sh
    style = f"rounded=0;whiteSpace=wrap;html=1;fillColor={col};strokeColor=none;"
    return _cell(cid, "", style, ox + x * k, oy + y * k, w * k, h * k)


def write_drawio(path, shapes):
    pages = []

    # ---------- 页 1：1:1 打印用（只有图形，没有标注） ----------
    k = PX_PER_MM
    cells = "".join(_shape_cell(f"s{i}", sh, k) for i, sh in enumerate(shapes))
    pages.append(("lab-track-1to1", "跑道 1:1（打印用）",
                  f"{L_MM * k:.0f}", f"{W_MM * k:.0f}", cells))

    # ---------- 页 2：1:10 + 尺寸标注 ----------
    k2 = PX_PER_MM * 0.1
    OX, OY = 340.0, 460.0
    art_w, art_h = L_MM * k2, W_MM * k2
    cells = "".join(_shape_cell(f"a{i}", sh, k2, OX, OY) for i, sh in enumerate(shapes))
    cid = 0

    def nid():
        nonlocal cid
        cid += 1
        return f"n{cid}"

    # 顶部：长度方向尺寸链
    ytop = OY - 46
    for a, b, lab in ((0, XW_X0_MM, "1400（起点→斑马线前缘）"),
                      (XW_X0_MM, XW_X0_MM + XW_LEN_MM, "300"),
                      (XW_X0_MM + XW_LEN_MM, LEN_MM - PK_LEN_MM, "4300"),
                      (LEN_MM - PK_LEN_MM, LEN_MM, "1000（停车区）")):
        cells += _dim(nid(), OX + a * k2, ytop, OX + b * k2, ytop, lab)
    cells += _text(nid(), "总长 7000", OX, ytop - 40, art_w, 26, 13, "#C0392B", "center", True)

    # 左侧：宽度方向尺寸链
    xleft = OX - 46
    for a, b, lab in ((0, OUTER_RED_MM, "50"),
                      (OUTER_RED_MM, OUTER_RED_MM + WHITE_MM, "40 白线"),
                      (OUTER_RED_MM + WHITE_MM, OUTER_RED_MM + WHITE_MM + LANE_MM, "1220 红底"),
                      (OUTER_RED_MM + WHITE_MM + LANE_MM, WID_MM - OUTER_RED_MM, "40 白线"),
                      (WID_MM - OUTER_RED_MM, WID_MM, "50")):
        cells += _dim(nid(), xleft, OY + a * k2, xleft, OY + b * k2, lab)
    cells += _text(nid(), "总宽 1400", xleft - 200, OY, 160, 26, 13, "#C0392B", "right", True)

    # 右侧：停车区细节
    xright = OX + art_w + 70
    cells += _dim(nid(), xright, OY + LANE_Y0 * k2, xright, OY + LANE_Y1 * k2, "赛道宽 1220")
    cells += _text(nid(), "黄胶带 100mm 宽", xright + 20, OY + (CY - 10) * k2, 200, 24, 11)

    notes = [
        f"斑马线：白条 {XW_BAR_MM}mm（沿行驶方向 {XW_LEN_MM}mm）× {XW_BARS} 条，条间隙 "
        f"{XW_GAP_MM}mm，中间那条压在赛道中线上（规则 §2.2.3：10.5×29.7cm 白纸平铺、间隙 10.5cm）",
        "停车区：黄胶带 100mm 宽（入口 / 尾端 / 沿中线各一条），1m（沿行驶方向）× 赛道宽，"
        "中线胶带把停车区分成上下两格",
        "挡板（40cm×50cm 蓝板）与付款码（10cm×10cm、与行驶方向成 45°）<b>都用实物道具</b>，"
        "所以图上不印 —— 裁判在某一格入口放板，车停未被遮挡的那一格",
        "颜色：红底 #924C4A / 白线 #FFFFFF / 黄胶带 #FEFF00（取自规则插图）",
        "打印：1:1 整幅输出 7000×1400mm，材料用哑光写真/背胶，不要拼接、不要覆亮膜（反光会干扰摄像头）",
    ]
    cells += _text(nid(), "<br>".join(notes), OX - 40, OY + art_h + 34, art_w + 80, 190, 12, "#444444")
    cells += _text(nid(), "单位 mm；本页 1:10，用于看图/改图 —— 打印请用第 1 页（1:1）或同目录的 PDF",
                   OX, OY - 130, art_w, 26, 12, "#888888")

    pages.append(("lab-track-annot", "尺寸标注（1:10）",
                  f"{art_w + 2 * OX + 400:.0f}", f"{OY + art_h + 300:.0f}", cells))

    xml = ['<mxfile host="app.diagrams.net" agent="ZCode gen_lab_track.py" version="24.7.17" '
           'type="device">']
    for pid, name, pw, ph, body in pages:
        xml.append(f'  <diagram id="{pid}" name="{name}">')
        xml.append(f'    <mxGraphModel dx="1400" dy="800" grid="0" gridSize="10" guides="1" '
                   f'tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" '
                   f'pageWidth="{pw}" pageHeight="{ph}" math="0" shadow="0">')
        xml.append('      <root>')
        xml.append('        <mxCell id="0" />')
        xml.append('        <mxCell id="1" parent="0" />')
        xml.append(body)
        xml.append('      </root>')
        xml.append('    </mxGraphModel>')
        xml.append('  </diagram>')
    xml.append('</mxfile>')
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(xml) + "\n")


# ============================================================ PNG 预览
def write_previews(shapes):
    try:
        import cv2
        import numpy as np
    except Exception as e:                                  # pragma: no cover
        print(f"  (skip previews: {e})")
        return

    flat = expand(shapes)

    def bgr(col):
        return (int(col[5:7], 16), int(col[3:5], 16), int(col[1:3], 16))

    def raster(ppm, x0_mm, y0_mm, w_mm, h_mm):
        img = np.full((int(round(h_mm * ppm)), int(round(w_mm * ppm)), 3), 255, np.uint8)
        for sh in flat:
            if sh[0] == "rect":
                _, x, y, w, h, col = sh
                cv2.rectangle(img,
                              (int(round((x - x0_mm) * ppm)), int(round((y - y0_mm) * ppm))),
                              (int(round((x + w - x0_mm) * ppm)), int(round((y + h - y0_mm) * ppm))),
                              bgr(col), -1)
            else:
                _, pts, col = sh
                arr = np.array([[int(round((px - x0_mm) * ppm)), int(round((py - y0_mm) * ppm))]
                                for px, py in pts], np.int32)
                cv2.fillPoly(img, [arr], bgr(col))
        return img

    def imwrite_u(p, img):
        ok, buf = cv2.imencode(os.path.splitext(p)[1], img)
        if ok:
            buf.tofile(p)

    imwrite_u(os.path.join(HERE, "preview_full.png"), raster(1.0, 0, 0, L_MM, W_MM))
    imwrite_u(os.path.join(HERE, "preview_crosswalk.png"), raster(2.0, 1000, 0, 1200, W_MM))
    imwrite_u(os.path.join(HERE, "preview_parking.png"), raster(2.0, 5700, 0, 1300, W_MM))
    print("  preview_full.png / preview_crosswalk.png / preview_parking.png")


# ============================================================ main
def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    shapes = build_shapes()
    print(f"lab track: {LEN_MM}x{WID_MM}mm  lane={LANE_MM} white={WHITE_MM} "
          f"outer_red={OUTER_RED_MM} bleed={BLEED_MM} "
          f"board={'draw' if DRAW_BOARD else 'physical'} qr={'draw' if DRAW_QR else 'physical'}")
    pdf = os.path.join(HERE, "lab_track_7m_1to1.pdf")
    write_pdf(shapes, pdf)
    print(f"  lab_track_7m_1to1.pdf  self-check: {verify_pdf(pdf)}")
    write_drawio(os.path.join(HERE, "lab_track_7m.drawio"), shapes)
    print("  lab_track_7m.drawio  (page1 = 1:1 print, page2 = 1:10 annotated)")
    if "--preview" in argv:                      # 只在需要肉眼检查时出预览图
        write_previews(shapes)


if __name__ == "__main__":
    main()
