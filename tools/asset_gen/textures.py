# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""Procedural, tileable PBR textures for the circuit (all generated, CC0-compatible)."""
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_OBL = "/usr/share/fonts/truetype/dejavu/DejaVuSans-BoldOblique.ttf"


def tile_noise(n, octaves, seed, base=4, persistence=0.55):
    """Tileable value noise via FFT-filtered white noise (sum of bands)."""
    rng = np.random.default_rng(seed)
    out = np.zeros((n, n))
    amp = 1.0
    fy = np.fft.fftfreq(n)[:, None]
    fx = np.fft.fftfreq(n)[None, :]
    f = np.sqrt(fx ** 2 + fy ** 2) * n
    for o in range(octaves):
        freq = base * 2 ** o
        w = np.exp(-((f - freq) ** 2) / (2 * (freq * 0.5) ** 2))
        spec = np.fft.fft2(rng.standard_normal((n, n))) * w
        layer = np.real(np.fft.ifft2(spec))
        layer /= layer.std() + 1e-9
        out += amp * layer
        amp *= persistence
    out = (out - out.min()) / (out.max() - out.min())
    return out


def normal_from_height(h, strength=2.0):
    gy, gx = np.gradient(h)
    # wrap-aware gradient
    gx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * 0.5
    gy = (np.roll(h, -1, 0) - np.roll(h, 1, 0)) * 0.5
    nx, ny = -gx * strength, gy * strength
    nz = np.ones_like(h)
    l = np.sqrt(nx ** 2 + ny ** 2 + nz ** 2)
    n = np.stack([nx / l, ny / l, nz / l], -1)
    return Image.fromarray(((n * 0.5 + 0.5) * 255).astype(np.uint8))


def mr_image(rough, metal=0.0):
    """glTF metallicRoughness: G = roughness, B = metallic."""
    r = np.clip(rough, 0, 1)
    a = np.zeros(r.shape + (3,), np.uint8)
    a[..., 1] = (r * 255).astype(np.uint8)
    a[..., 2] = int(metal * 255)
    return Image.fromarray(a)


def colorize(t, c0, c1):
    t = t[..., None]
    return Image.fromarray((np.array(c0) * (1 - t) + np.array(c1) * t).clip(0, 255).astype(np.uint8))


def asphalt(n=1024, seed=1):
    big = tile_noise(n, 3, seed, 3)
    mid = tile_noise(n, 3, seed + 1, 24)
    rng = np.random.default_rng(seed + 2)
    grain = rng.random((n, n))
    stones = (grain > 0.985).astype(float) * rng.uniform(0.4, 1.0, (n, n))
    dark = (grain < 0.02).astype(float)
    v = 0.42 + 0.10 * big + 0.12 * mid + 0.35 * stones - 0.15 * dark
    v = np.clip(v, 0, 1)
    col = np.stack([v * 128, v * 128, v * 132], -1)
    base = Image.fromarray(col.clip(0, 255).astype(np.uint8))
    h = 0.6 * mid + 0.8 * stones + 0.2 * grain
    rough = 0.80 + 0.12 * (1 - big) - 0.25 * stones
    return base, normal_from_height(h, 3.0), mr_image(rough)


def grass(n=1024, seed=10):
    big = tile_noise(n, 3, seed, 2)
    mid = tile_noise(n, 4, seed + 1, 16)
    rng = np.random.default_rng(seed + 2)
    fine = rng.random((n, n))
    t = 0.22 * big + 0.38 * mid + 0.40 * fine
    c0 = np.array([52, 78, 30]); c1 = np.array([96, 128, 50])
    col = c0 * (1 - t[..., None]) + c1 * t[..., None]
    dry = np.clip(big - 0.6, 0, 1)[..., None] * np.array([40, 24, -10])
    base = Image.fromarray((col + dry).clip(0, 255).astype(np.uint8))
    return base, normal_from_height(0.6 * mid + 0.6 * fine, 2.5), mr_image(0.92 - 0.1 * fine)


def gravel(n=1024, seed=20):
    rng = np.random.default_rng(seed)
    h = np.zeros((n, n))
    col = np.zeros((n, n, 3))
    yy, xx = np.mgrid[0:n, 0:n]
    # scatter pebbles (tileable through wrap)
    img = Image.new("RGB", (n, n), (150, 132, 104))
    hi = Image.new("L", (n, n), 0)
    d, dh = ImageDraw.Draw(img), ImageDraw.Draw(hi)
    for _ in range(9000):
        x, y = rng.uniform(0, n, 2)
        r = rng.uniform(3, 9)
        c = tuple(int(v) for v in np.array([168, 150, 120]) * rng.uniform(0.7, 1.15) + rng.integers(-8, 9, 3))
        for ox in (-n, 0, n):
            for oy in (-n, 0, n):
                bb = [x - r + ox, y - r * 0.8 + oy, x + r + ox, y + r * 0.8 + oy]
                d.ellipse(bb, fill=c)
                dh.ellipse(bb, fill=int(rng.uniform(120, 255)))
    hh = np.asarray(hi).astype(float) / 255
    return img, normal_from_height(hh, 4.0), mr_image(0.85 + 0.0 * hh)


def concrete(n=512, seed=30, tone=(170, 168, 162)):
    big = tile_noise(n, 3, seed, 3)
    rng = np.random.default_rng(seed)
    fine = rng.random((n, n))
    v = 0.86 + 0.10 * big + 0.06 * fine
    col = np.stack([v * tone[0], v * tone[1], v * tone[2]], -1)
    img = Image.fromarray(col.clip(0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    d.line([(0, 0), (n, 0)], fill=(120, 118, 112), width=3)
    return img, normal_from_height(0.3 * big + 0.4 * fine, 1.5), mr_image(0.8 + 0.1 * fine)


def kerb(w=512, h=128):
    """Red/white kerb: u along the track (one texture = 2 m: red 1 m, white 1 m)."""
    img = Image.new("RGB", (w, h), (225, 225, 222))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, w // 2, h], fill=(170, 18, 22))
    arr = np.asarray(img).astype(float)
    rng = np.random.default_rng(5)
    dirt = tile_noise(w, 3, 7, 6)[:h, :w] if h <= w else None
    arr *= (0.86 + 0.14 * dirt[..., None])
    arr += rng.integers(-6, 7, (h, w, 1))
    # rubber marks near the track edge (v = 0 side)
    edge = np.linspace(1, 0, h)[:, None, None] ** 3
    arr = arr * (1 - 0.35 * edge * dirt[..., None])
    base = Image.fromarray(arr.clip(0, 255).astype(np.uint8))
    return base, mr_image(np.full((h, w), 0.78))


def white_paint(n=256):
    rng = np.random.default_rng(9)
    v = 0.88 + 0.08 * tile_noise(n, 3, 3, 8) + 0.04 * rng.random((n, n))
    col = np.stack([v * 245, v * 245, v * 240], -1)
    return Image.fromarray(col.clip(0, 255).astype(np.uint8))


def fence(n=256):
    """Chain-link diamond mesh, alpha in A."""
    a = np.zeros((n, n), float)
    yy, xx = np.mgrid[0:n, 0:n] / n * 8
    d1 = np.abs(((xx + yy) % 1) - 0.5)
    d2 = np.abs(((xx - yy) % 1) - 0.5)
    wire = np.minimum(d1, d2) > 0.40
    rgba = np.zeros((n, n, 4), np.uint8)
    rgba[..., :3] = 110
    rgba[..., 3] = (wire * 255).astype(np.uint8)
    return Image.fromarray(rgba, "RGBA")


def seats(w=512, h=512):
    """Grandstand: 16 rows of seats per texture (v), coloured blocks (u)."""
    img = Image.new("RGB", (w, h), (90, 90, 95))
    d = ImageDraw.Draw(img)
    rows, per = 16, 32
    rng = np.random.default_rng(4)
    for r in range(rows):
        y0 = r * h / rows
        for c in range(per):
            x0 = c * w / per
            col = (30, 70, 160) if (c // 8) % 2 == 0 else (240, 190, 30)
            if rng.random() < 0.18:
                col = tuple(int(v) for v in rng.integers(40, 230, 3))  # spectators
            d.rectangle([x0 + 2, y0 + h / rows * 0.25, x0 + w / per - 2, y0 + h / rows * 0.9], fill=col)
    return img


def garage_facade(n_doors=8, w=2048, h=512):
    img = Image.new("RGB", (w, h), (205, 207, 210))
    d = ImageDraw.Draw(img)
    f = ImageFont.truetype(FONT, 46)
    dw = w / n_doors
    for k in range(n_doors):
        x0 = k * dw + dw * 0.08
        x1 = (k + 1) * dw - dw * 0.08
        d.rectangle([x0, h * 0.30, x1, h], fill=(60, 64, 70))
        for y in np.arange(h * 0.32, h, 14):
            d.line([(x0, y), (x1, y)], fill=(80, 84, 90), width=3)
        d.rectangle([x0, h * 0.10, x1, h * 0.25], fill=(22, 32, 70))
        d.text((x0 + 18, h * 0.12), f"{k + 1:02d}", font=f, fill=(250, 250, 250))
    d.rectangle([0, 0, w, h * 0.06], fill=(160, 162, 166))
    return img


def glass_band(w=1024, h=128):
    img = Image.new("RGB", (w, h), (40, 55, 70))
    d = ImageDraw.Draw(img)
    for x in range(0, w, 64):
        d.line([(x, 0), (x, h)], fill=(120, 125, 130), width=4)
    arr = np.asarray(img).astype(float)
    arr[:, :, :] += np.linspace(30, -10, h)[:, None, None]
    return Image.fromarray(arr.clip(0, 255).astype(np.uint8))


BRANDS = [("HALVARD", (230, 230, 235), (200, 30, 40)), ("NORVANTE", (20, 40, 90), (255, 255, 255)),
          ("VELOXIA", (250, 190, 20), (20, 20, 20)), ("OPEN SIM RACING", (240, 120, 20), (255, 255, 255)),
          ("SIMGRIP TYRES", (15, 15, 15), (250, 220, 40)), ("SOLARA FUEL", (30, 140, 70), (255, 255, 255)),
          ("PITLANE CAFE", (240, 240, 240), (30, 30, 30)), ("TORVALE WATCHES", (90, 20, 30), (230, 200, 140))]


def ad_atlas(w=2048, h=1024):
    """8 banners, each 2048 x 128 (aspect 16:1)."""
    img = Image.new("RGB", (w, h), (0, 0, 0))
    d = ImageDraw.Draw(img)
    for i, (name, bg, fg) in enumerate(BRANDS):
        y0 = i * h // 8
        d.rectangle([0, y0, w, y0 + h // 8], fill=bg)
        f = ImageFont.truetype(FONT_OBL, 84)
        for k in range(2):
            cx = w * (0.25 + 0.5 * k)
            bb = d.textbbox((0, 0), name, font=f)
            d.text((cx - (bb[2] - bb[0]) / 2, y0 + (h // 8 - (bb[3] - bb[1])) / 2 - bb[1]), name, font=f, fill=fg)
    return img


def board_numbers():
    """Distance boards 300 / 200 / 100 (u in thirds)."""
    w, h = 768, 256
    img = Image.new("RGB", (w, h), (250, 250, 250))
    d = ImageDraw.Draw(img)
    f = ImageFont.truetype(FONT, 150)
    for i, t in enumerate(("300", "200", "100")):
        x0 = i * 256
        d.rectangle([x0 + 6, 6, x0 + 250, h - 6], outline=(20, 20, 20), width=10)
        bb = d.textbbox((0, 0), t, font=f)
        d.text((x0 + 128 - (bb[2] - bb[0]) / 2 - bb[0], 128 - (bb[3] - bb[1]) / 2 - bb[1]), t, font=f, fill=(20, 20, 20))
    return img


def bark(n=256):
    t = tile_noise(n, 3, 50, 4)
    stripes = (np.sin(np.linspace(0, 40 * np.pi, n))[None, :] * 0.5 + 0.5) * 0.3
    v = 0.5 * t + stripes
    return colorize(v, (52, 38, 26), (96, 74, 52))


def leaves(n=512, seed=60, c0=(26, 52, 20), c1=(84, 120, 46)):
    t = 0.5 * tile_noise(n, 4, seed, 8) + 0.5 * np.random.default_rng(seed).random((n, n))
    return colorize(t, c0, c1)


def sign_panel(text, w=1024, h=256, bg=(20, 30, 60), fg=(255, 255, 255)):
    img = Image.new("RGB", (w, h), bg)
    d = ImageDraw.Draw(img)
    f = ImageFont.truetype(FONT_OBL, int(h * 0.55))
    bb = d.textbbox((0, 0), text, font=f)
    d.text(((w - (bb[2] - bb[0])) / 2 - bb[0], (h - (bb[3] - bb[1])) / 2 - bb[1]), text, font=f, fill=fg)
    return img


def grid_marks(w=512, h=256):
    """Starting grid box (white L) on transparent asphalt, alpha mask."""
    a = np.zeros((h, w, 4), np.uint8)
    a[..., :3] = 245
    a[0:24, :, 3] = 255
    a[:, 0:24, 3] = 255
    return Image.fromarray(a, "RGBA")


def chequer(w=512, h=64, n=16):
    img = Image.new("RGB", (w, h), (245, 245, 245))
    d = ImageDraw.Draw(img)
    s = w / n
    for i in range(n):
        for j in range(2):
            if (i + j) % 2 == 0:
                d.rectangle([i * s, j * h / 2, (i + 1) * s, (j + 1) * h / 2], fill=(15, 15, 15))
    return img
