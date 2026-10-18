# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""Procedural livery atlas for the GT3 body (original design, invented sponsors).

Atlas layout (all charts are planar world projections, so a feature painted at
the same world position in every chart lines up across chart seams):
  right side  v 0.00-0.25   u = 1 - (z - L0) / LEN  (text reads correctly from outside)
  left side   v 0.25-0.50   u = (z - L0) / LEN
  top         v 0.50-0.80   u = (z - L0) / LEN, v from +x (top) to -x
  front       v 0.80-1.00   u 0.0-0.5, viewer right = -x
  rear        v 0.80-1.00   u 0.5-1.0, viewer right = +x
"""
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

L0, L1 = -2.36, 2.38
LEN = L1 - L0
HMAX = 1.25
XMAX = 1.10
SIZE = 2048
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_OBL = "/usr/share/fonts/truetype/dejavu/DejaVuSans-BoldOblique.ttf"

CHARTS = {
    "right": (0.0, 0.0, 1.0, 0.25),
    "left": (0.0, 0.25, 1.0, 0.50),
    "top": (0.0, 0.50, 1.0, 0.80),
    "front": (0.0, 0.80, 0.5, 1.00),
    "rear": (0.5, 0.80, 1.0, 1.00),
}


def chart_uv(chart, p):
    """World points (N,3) -> uv (N,2) for a chart."""
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    u0, v0, u1, v1 = CHARTS[chart]
    if chart == "right":
        a, b = 1 - (z - L0) / LEN, 1 - y / HMAX
    elif chart == "left":
        a, b = (z - L0) / LEN, 1 - y / HMAX
    elif chart == "top":
        a, b = (z - L0) / LEN, 1 - (x + XMAX) / (2 * XMAX)
    elif chart == "front":
        a, b = (XMAX - x) / (2 * XMAX), 1 - y / HMAX
    else:
        a, b = (x + XMAX) / (2 * XMAX), 1 - y / HMAX
    return np.c_[u0 + (u1 - u0) * a, v0 + (v1 - v0) * b]


def px(chart, x=0.0, y=0.0, z=0.0):
    uv = chart_uv(chart, np.array([[x, y, z]], float))[0]
    return (uv[0] * SIZE, uv[1] * SIZE)


# palette
WHITE = (236, 238, 240)
NAVY = (18, 32, 72)
ORANGE = (246, 112, 22)
GREY = (60, 64, 70)
BLACK = (14, 14, 16)


def font(size, oblique=True):
    return ImageFont.truetype(FONT_OBL if oblique else FONT_BOLD, size)


def side_poly(draw, chart, pts_zy, fill):
    draw.polygon([px(chart, 0, y, z) for z, y in pts_zy], fill=fill)


def text_side(img, chart, z, y, txt, size, fill, oblique=True, height_m=None, outline=None):
    """Draw text centred at world (z, y) on a side chart, reading front-to-back correctly on both sides."""
    f = font(size, oblique)
    d = ImageDraw.Draw(img)
    bbox = d.textbbox((0, 0), txt, font=f, stroke_width=3 if outline else 0)
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    layer = Image.new("RGBA", (w + 12, h + 12), (0, 0, 0, 0))
    ImageDraw.Draw(layer).text((6 - bbox[0], 6 - bbox[1]), txt, font=f, fill=fill, stroke_width=3 if outline else 0, stroke_fill=outline)
    cx, cy = px(chart, 0, y, z)
    img.alpha_composite(layer, (int(cx - layer.width / 2), int(cy - layer.height / 2)))


def roundel(img, chart_pts, r_px, number):
    d = ImageDraw.Draw(img)
    for (cx, cy) in chart_pts:
        d.ellipse([cx - r_px, cy - r_px, cx + r_px, cy + r_px], fill=(240, 240, 242), outline=BLACK, width=6)
        f = font(int(r_px * 1.15), oblique=False)
        bb = d.textbbox((0, 0), number, font=f)
        d.text((cx - (bb[2] - bb[0]) / 2 - bb[0], cy - (bb[3] - bb[1]) / 2 - bb[1]), number, font=f, fill=BLACK)


SCHEMES = {
    "a": {"base": (236, 238, 240), "dark": (18, 32, 72), "accent": (246, 112, 22), "text": (236, 238, 240)},
    "b": {"base": (24, 24, 28), "dark": (170, 16, 24), "accent": (240, 196, 30), "text": (240, 240, 240)},
    "c": {"base": (20, 120, 92), "dark": (12, 40, 34), "accent": (236, 238, 240), "text": (236, 238, 240)},
}


def build_livery(number="27", scheme="a"):
    global WHITE, NAVY, ORANGE
    sc = SCHEMES[scheme]
    WHITE, NAVY, ORANGE = sc["base"], sc["dark"], sc["accent"]
    img = Image.new("RGBA", (SIZE, SIZE), WHITE + (255,))
    emis = Image.new("RGB", (SIZE, SIZE), (0, 0, 0))
    d = ImageDraw.Draw(img)
    de = ImageDraw.Draw(emis)

    for chart in ("right", "left"):
        # navy lower body rising to the rear, with an orange slash
        side_poly(d, chart, [(L0, 0.0), (L0, 0.40), (-1.0, 0.36), (0.1, 0.44), (0.9, 0.62), (L1, 0.74), (L1, 0.0)], NAVY)
        side_poly(d, chart, [(-0.35, 0.0), (-0.05, 0.0), (0.55, 0.95), (0.25, 0.95)], ORANGE)
        side_poly(d, chart, [(0.05, 0.0), (0.16, 0.0), (0.74, 0.92), (0.63, 0.92)], ORANGE)
        side_poly(d, chart, [(L0, 0.42), (-1.0, 0.385), (0.1, 0.465), (0.9, 0.645), (L1, 0.765), (L1, 0.75), (0.9, 0.632), (0.1, 0.452), (-1.0, 0.372), (L0, 0.408)], ORANGE)
        # door number
        roundel(img, [px(chart, 0, 0.60, -0.15)], 92, number)
        # sponsors (invented)
        text_side(img, chart, 1.55, 0.60, "HALVARD", 64, WHITE)
        text_side(img, chart, -1.80, 0.52, "NORVANTE LUBRICANTS", 30, NAVY if sum(WHITE) > 400 else (230, 230, 230))
        text_side(img, chart, 0.62, 0.29, "VELOXIA TELEMETRY", 34, WHITE)
        text_side(img, chart, -0.95, 0.68, "OPEN SIM RACING", 30, NAVY if sum(WHITE) > 400 else (230, 230, 230), oblique=False)
        # side vent behind the front wheel (black grille)
        for k in range(5):
            z0 = -0.88 + k * 0.035
            side_poly(d, chart, [(z0, 0.40), (z0 + 0.018, 0.40), (z0 + 0.05, 0.64), (z0 + 0.032, 0.64)], BLACK)

    # top: racing stripes, hood louvres, roof number, wing shadow area
    for xs in (-0.17, 0.17):
        pts = [px("top", xs - 0.09, 0, z) for z in (L0, L1)] + [px("top", xs + 0.09, 0, z) for z in (L1, L0)]
        d.polygon(pts, fill=NAVY)
        pts = [px("top", xs - 0.105, 0, L0), px("top", xs - 0.09, 0, L0), px("top", xs - 0.09, 0, L1), px("top", xs - 0.105, 0, L1)]
        d.polygon(pts, fill=ORANGE)
    for side in (1, -1):
        for k in range(9):
            z0 = -1.62 + k * 0.045
            pts = [px("top", side * 0.42, 0, z0), px("top", side * 0.78, 0, z0), px("top", side * 0.78, 0, z0 + 0.022), px("top", side * 0.42, 0, z0 + 0.022)]
            d.polygon(pts, fill=BLACK)
    # roof number (rotated for an overhead camera, reads from the left side)
    cx, cy = px("top", 0, 0, 0.30)
    roundel(img, [(cx, cy)], 80, number)
    # windscreen-base cowl vent
    d.polygon([px("top", -0.55, 0, -0.66), px("top", 0.55, 0, -0.66), px("top", 0.55, 0, -0.60), px("top", -0.55, 0, -0.60)], fill=BLACK)

    # headlights: dark housing + LED strip, painted in top and front charts at the same world spots
    for side in (1, -1):
        # top view housing (x, z)
        hz = [(-2.20, 0.84), (-2.14, 0.90), (-1.94, 0.86), (-1.90, 0.62), (-2.02, 0.50), (-2.18, 0.55)]
        d.polygon([px("top", side * x, 0, z) for z, x in hz], fill=(22, 24, 30))
        led = [(-2.15, 0.84), (-2.12, 0.87), (-1.97, 0.83), (-1.99, 0.80)]
        d.polygon([px("top", side * x, 0, z) for z, x in led], fill=(150, 152, 160))
        # front view housing (x, y)
        hf = [(0.86, 0.50), (0.90, 0.56), (0.62, 0.58), (0.50, 0.54), (0.56, 0.48)]
        d.polygon([px("front", side * x, y, 0) for x, y in hf], fill=(22, 24, 30))
        lf = [(0.82, 0.535), (0.86, 0.555), (0.66, 0.565), (0.62, 0.548)]
        d.polygon([px("front", side * x, y, 0) for x, y in lf], fill=(250, 250, 255))
        de.polygon([px("front", side * x, y, 0) for x, y in lf], fill=(255, 255, 255))

    # front: big grille + lower intakes + number plate area
    grille = [(-0.62, 0.20), (0.62, 0.20), (0.70, 0.42), (-0.70, 0.42)]
    d.polygon([px("front", x, y, 0) for x, y in grille], fill=(10, 10, 12))
    for gx in np.arange(-0.66, 0.66, 0.035):
        d.line([px("front", gx, 0.21, 0), px("front", gx + 0.02, 0.41, 0)], fill=(45, 45, 50), width=2)
    for side in (1, -1):
        d.polygon([px("front", side * x, y, 0) for x, y in [(0.72, 0.16), (0.92, 0.18), (0.92, 0.34), (0.76, 0.36)]], fill=(10, 10, 12))
    d.rectangle([*px("front", 0.30, 0.47, 0), *px("front", -0.30, 0.45, 0)], fill=ORANGE)

    # rear: black lower panel with mesh, light bar, tail lamps
    rp = [(-1.0, 0.0), (1.0, 0.0), (1.0, 0.52), (-1.0, 0.52)]
    d.polygon([px("rear", x, y, 0) for x, y in rp], fill=(12, 12, 14))
    for gx in np.arange(-0.95, 0.95, 0.03):
        d.line([px("rear", gx, 0.30, 0), px("rear", gx, 0.50, 0)], fill=(40, 40, 44), width=2)
    bar = [(-0.92, 0.70), (0.92, 0.70), (0.92, 0.735), (-0.92, 0.735)]
    d.polygon([px("rear", x, y, 0) for x, y in bar], fill=(120, 10, 12))
    de.polygon([px("rear", x, y, 0) for x, y in bar], fill=(170, 4, 4))
    for side in (1, -1):
        lamp = [(0.70, 0.64), (0.96, 0.66), (0.96, 0.80), (0.72, 0.79)]
        d.polygon([px("rear", side * x, y, 0) for x, y in lamp], fill=(110, 6, 8))
        de.polygon([px("rear", side * x, y, 0) for x, y in lamp], fill=(150, 4, 4))
    tz = [(2.25, 0.70), (L1, 0.70), (L1, 0.80), (2.22, 0.80)]
    for chart in ("right", "left"):
        side_poly(d, chart, [(z, y) for z, y in tz], (110, 6, 8))
        side_poly(de, chart, [(z, y) for z, y in tz], (150, 4, 4))
    text_side(img, "rear", 0, 0.0, "", 10, WHITE)
    cx, cy = px("rear", 0, 0.60, 0)
    f = font(46)
    bb = d.textbbox((0, 0), "OPEN SIM RACING", font=f)
    d.text((cx - (bb[2] - bb[0]) / 2, cy - 30), "OPEN SIM RACING", font=f, fill=NAVY)

    # subtle metallic flake noise for the white
    rng = np.random.default_rng(7)
    arr = np.asarray(img).astype(np.int16)
    arr[..., :3] += rng.integers(-3, 4, size=arr.shape[:2] + (1,), dtype=np.int16)
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8)).convert("RGB")
    return img, emis


if __name__ == "__main__":
    a, e = build_livery()
    a.save("/tmp/livery.png"); e.save("/tmp/livery_e.png")
