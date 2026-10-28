"""
生成桌面打印用圆环靶标 —— 严格按「图 4 圆环」结构。

图四结构（由内向外，去掉尺寸标注后）：
  1. 实线圆   直径 = φ + 3 mm
  2. 虚线圆   与内实线圆心距约 2.5/2（两实线间距 2.5，虚线居中）
  3. 实线圆   与内实线圆心距 = 2.5 mm
  4. 虚线圆   与外实线圆心距 = 3.5 mm

φ 未知时默认 PHI_MM=70（可改）。外径 = φ+3+2* (2.5+3.5) = φ+15。

输出: printables/
  ring_1_a4_1to1.png / ring_2 / ring_3
  ring_triplet_a4.png
  ring_platform_a4_scaled.png
  ring_fig4_only_a4.png   — 单枚图四样式（无数字，纯环）
"""

from __future__ import annotations

import math
import os

from PIL import Image, ImageDraw, ImageFont

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "printables")
DPI = 300
MM_TO_PX = DPI / 25.4

# ---- 图二台面 ----
PLATFORM_W_MM = 580.0
PLATFORM_H_MM = 150.0
CENTER_SPACING_MM = 150.0

# ---- 图四几何（工作假设 φ，确认后改 PHI_MM 重跑）----
PHI_MM = 70.0                 # 物料特征尺寸 φ；未知时的假设
INNER_DIA_MM = PHI_MM + 3.0   # 图四：Ø+3mm
# 图四标注：两实线之间 2.5；外实线到最外虚线 3.5
GAP_SOLIDS_MM = 2.5
GAP_OUTER_MM = 3.5
STROKE_MM = 0.55              # 细线，接近图纸
DASH_ON_OFF = (2, 1)          # 虚线：2 段画、1 段空（更密、更像图四）
A4_W_MM, A4_H_MM = 210.0, 297.0


def mm(x: float) -> int:
    return max(1, int(round(x * MM_TO_PX)))


def a4_size(landscape: bool = False):
    if landscape:
        return mm(A4_H_MM), mm(A4_W_MM)
    return mm(A4_W_MM), mm(A4_H_MM)


def outer_dia_mm() -> float:
    # 最外虚线直径 = 内径 + 2*(2.5+3.5)
    return INNER_DIA_MM + 2.0 * (GAP_SOLIDS_MM + GAP_OUTER_MM)


def try_font(size_px: int):
    for name in (
        "C:/Windows/Fonts/msyhbd.ttc",
        "C:/Windows/Fonts/msyh.ttc",
        "C:/Windows/Fonts/arialbd.ttf",
        "C:/Windows/Fonts/arial.ttf",
    ):
        if os.path.exists(name):
            try:
                return ImageFont.truetype(name, size_px)
            except Exception:
                continue
    return ImageFont.load_default()


def _circle_radii_mm():
    """由内向外四个圆的半径 (mm)：实、虚、实、虚。"""
    r0 = INNER_DIA_MM / 2.0
    r1 = r0 + GAP_SOLIDS_MM / 2.0      # 虚线居中于两实线之间
    r2 = r0 + GAP_SOLIDS_MM            # 外实线
    r3 = r2 + GAP_OUTER_MM             # 最外虚线
    return (r0, r1, r2, r3)


def _draw_solid_circle(draw, cx, cy, r_px, stroke_px):
    # 用环形近似描边，避免 ellipse width 在不同 Pillow 版本不一致
    r_out = r_px + stroke_px / 2.0
    r_in = max(0.0, r_px - stroke_px / 2.0)
    draw.ellipse([cx - r_out, cy - r_out, cx + r_out, cy + r_out], fill=(0, 0, 0))
    if r_in > 1:
        draw.ellipse([cx - r_in, cy - r_in, cx + r_in, cy + r_in], fill=(255, 255, 255))


def _draw_dashed_circle(draw, cx, cy, r_px, stroke_px, n_seg=96):
    """虚线圆：按 DASH_ON_OFF 周期画（默认 2 开 1 关，更密）。"""
    half = stroke_px / 2.0
    on, off = DASH_ON_OFF
    period = on + off
    for i in range(n_seg):
        if (i % period) >= on:
            continue
        a0 = 2 * math.pi * i / n_seg
        a1 = 2 * math.pi * (i + 1) / n_seg
        steps = 4
        outer, inner = [], []
        for k in range(steps + 1):
            a = a0 + (a1 - a0) * k / steps
            ca, sa = math.cos(a), math.sin(a)
            outer.append((cx + (r_px + half) * ca, cy + (r_px + half) * sa))
            inner.append((cx + max(0.0, r_px - half) * ca, cy + max(0.0, r_px - half) * sa))
        draw.polygon(outer + list(reversed(inner)), fill=(0, 0, 0))

def draw_fig4_ring(draw: ImageDraw.ImageDraw, cx: float, cy: float, scale: float = 1.0, digit: str | None = None):
    """画图四样式圆环（无尺寸标线）。digit 可选，画在中心。"""
    r0, r1, r2, r3 = _circle_radii_mm()
    stroke = STROKE_MM * MM_TO_PX * scale

    def rp(rmm: float) -> float:
        return rmm * MM_TO_PX * scale

    # 由外向内画，避免覆盖；实际互不重叠
    _draw_dashed_circle(draw, cx, cy, rp(r3), stroke, n_seg=80)
    _draw_solid_circle(draw, cx, cy, rp(r2), stroke)
    _draw_dashed_circle(draw, cx, cy, rp(r1), stroke, n_seg=64)
    _draw_solid_circle(draw, cx, cy, rp(r0), stroke)

    if digit:
        # 字号约占内圆直径 45%
        inner_d_px = max(10, int(INNER_DIA_MM * MM_TO_PX * scale * 0.45))
        font = try_font(inner_d_px)
        try:
            draw.text((cx, cy), digit, fill=(0, 0, 0), font=font, anchor="mm")
        except TypeError:
            bbox = draw.textbbox((0, 0), digit, font=font)
            tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
            draw.text((cx - tw / 2, cy - th / 2), digit, fill=(0, 0, 0), font=font)


def _footer(draw, lines, y0):
    font = try_font(mm(3.2))
    y = y0
    for line in lines:
        draw.text((mm(10), y), line, fill=(0, 0, 0), font=font)
        y += mm(5)


def _crop_marks(draw, w, h):
    mark = mm(5)
    for x, y0 in ((mm(8), mm(8)), (w - mm(8), mm(8)), (mm(8), h - mm(8)), (w - mm(8), h - mm(8))):
        draw.line([(x - mark, y0), (x + mark, y0)], fill=(0, 0, 0), width=1)
        draw.line([(x, y0 - mark), (x, y0 + mark)], fill=(0, 0, 0), width=1)


def make_fig4_only():
    """纯图四，无数字。"""
    w, h = a4_size(False)
    img = Image.new("RGB", (w, h), (255, 255, 255))
    draw = ImageDraw.Draw(img)
    draw_fig4_ring(draw, w / 2, h / 2 - mm(8), scale=1.0, digit=None)
    _crop_marks(draw, w, h)
    _footer(draw, [
        "Fig.4 style  1:1  phi=%.0f  inner=Ø+3=%.0fmm  outer≈%.0fmm" % (PHI_MM, INNER_DIA_MM, outer_dia_mm()),
        "Print 100%%. Disable fit-to-page. Measure outer dia after print.",
    ], h - mm(18))
    path = os.path.join(OUT_DIR, "ring_fig4_only_a4.png")
    img.save(path, dpi=(DPI, DPI))
    return path


def make_single_1to1(digit: str):
    w, h = a4_size(False)
    img = Image.new("RGB", (w, h), (255, 255, 255))
    draw = ImageDraw.Draw(img)
    draw_fig4_ring(draw, w / 2, h / 2 - mm(10), scale=1.0, digit=digit)
    _crop_marks(draw, w, h)
    _footer(draw, [
        "1:1  digit=%s  Fig.4 rings  phi=%.0f  outer≈%.0fmm" % (digit, PHI_MM, outer_dia_mm()),
        "Print 100%%. Disable fit-to-page / shrink-to-fit.  DPI=%d" % DPI,
    ], h - mm(18))
    path = os.path.join(OUT_DIR, "ring_%s_a4_1to1.png" % digit)
    img.save(path, dpi=(DPI, DPI))
    return path


def make_triplet():
    w, h = a4_size(True)
    img = Image.new("RGB", (w, h), (255, 255, 255))
    draw = ImageDraw.Draw(img)
    need_w = 2 * CENTER_SPACING_MM + outer_dia_mm() + 16
    need_h = outer_dia_mm() + 24
    scale = min(1.0, (A4_H_MM - 16) / need_w, (A4_W_MM - 18) / need_h)
    cx0, cy = w / 2, h / 2 - mm(4)
    for dig, dx in (("3", -CENTER_SPACING_MM), ("2", 0.0), ("1", CENTER_SPACING_MM)):
        draw_fig4_ring(draw, cx0 + dx * MM_TO_PX * scale, cy, scale=scale, digit=dig)
    msg = ("1:1 triplet" if scale >= 0.999 else "SCALED %.1f%%" % (scale * 100))
    _footer(draw, ["%s  Fig.4  spacing=150mm  print 100%% of this file" % msg], h - mm(10))
    path = os.path.join(OUT_DIR, "ring_triplet_a4.png")
    img.save(path, dpi=(DPI, DPI))
    return path, scale


def make_platform_scaled():
    w, h = a4_size(True)
    img = Image.new("RGB", (w, h), (255, 255, 255))
    draw = ImageDraw.Draw(img)
    margin = mm(8)
    usable_w = w - 2 * margin
    usable_h = h - 2 * margin - mm(12)
    scale = min(usable_w / (PLATFORM_W_MM * MM_TO_PX), usable_h / (PLATFORM_H_MM * MM_TO_PX))
    pw, ph = PLATFORM_W_MM * MM_TO_PX * scale, PLATFORM_H_MM * MM_TO_PX * scale
    ox, oy = (w - pw) / 2, margin + (usable_h - ph) / 2
    draw.rectangle([ox, oy, ox + pw, oy + ph], outline=(0, 0, 0), width=max(1, mm(0.25)))
    cy = oy + ph / 2
    for dig, xmm in (("3", PLATFORM_W_MM / 2 - CENTER_SPACING_MM),
                     ("2", PLATFORM_W_MM / 2),
                     ("1", PLATFORM_W_MM / 2 + CENTER_SPACING_MM)):
        draw_fig4_ring(draw, ox + xmm * MM_TO_PX * scale, cy, scale=scale, digit=dig)
    _footer(draw, [
        "NOT 1:1  scale=%.1f%%  platform %dx%d mm  Fig.4 rings  print 100%% no fit-to-page"
        % (scale * 100, int(PLATFORM_W_MM), int(PLATFORM_H_MM))
    ], h - mm(8))
    path = os.path.join(OUT_DIR, "ring_platform_a4_scaled.png")
    img.save(path, dpi=(DPI, DPI))
    return path, scale * 100


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("PHI_MM=%.1f  INNER=%.1f  OUTER≈%.1f" % (PHI_MM, INNER_DIA_MM, outer_dia_mm()))
    print("wrote", make_fig4_only())
    for d in ("1", "2", "3"):
        print("wrote", make_single_1to1(d))
    p, s = make_triplet()
    print("wrote", p, "scale=%.3f" % s)
    p2, pct = make_platform_scaled()
    print("wrote", p2, "scale=%.1f%%" % pct)
    print("OUT_DIR=", OUT_DIR)


if __name__ == "__main__":
    main()
