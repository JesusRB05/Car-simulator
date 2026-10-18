# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""Builds the GT3-style 'Halvard GT3' test car as content/base/cars/halvard_gt3/model/car.glb.

Original design; no real manufacturer, brand or trademarked shape.
Node layout (see README written next to the model):
  car_root
    body            painted shell (livery atlas), glass, trim, lights
    aero            splitter, canards, skirts, diffuser, rear wing
    interior        seat, dash, steering wheel, roll cage
    details         mirrors, exhausts, tow hooks, antenna, wipers
    wheel_FL/FR/RL/RR   hub pivot (steer here); children:
        wheel_XX_spin   tyre + rim + disc (rotate about local X)
        wheel_XX_caliper  does not spin
"""
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import car_body as cb
import livery
from gltfw import GLB, Mesh, box, cylinder, merge, revolve, sweep, face_normals, fix_outward

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/car.glb"
SCHEME = sys.argv[2] if len(sys.argv) > 2 else "a"
NUMBER = sys.argv[3] if len(sys.argv) > 3 else "27"


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def sphere(r=1.0, seg=24, rings=12):
    prof = [(r * math.sin(t), -r * math.cos(t)) for t in np.linspace(0, math.pi, rings + 1)]
    return revolve(prof, seg, "y")


def mirror_x(m):
    return m.transformed(s=(-1, 1, 1))


# ------------------------------------------------------------------ body
def build_body_parts():
    m, caps, info = cb.build_body()
    m.compute_normals()
    S, R = info["S"], info["R"]
    z = info["z"]; par = info["params"]; P = info["P"]
    nf = len(m.tris)
    f = np.arange(nf)
    row = f // (2 * R)
    col = (f // 2) % R
    col2 = (col + 1) % R
    fpar = 0.5 * (par[col] + par[col2])
    fz = 0.5 * (z[row] + z[np.minimum(row + 1, S - 1)])
    cab = P["cabin"][row]
    fn = face_normals(m.pos, m.tris)
    # glass regions
    side_glass = (fpar > 6.18) & (fpar < 6.86) & (fz > -0.40) & (fz < 0.98) & (cab > 0.92) & ~((fz > 0.16) & (fz < 0.26))
    wind = (fpar > 7.12) & (fz > -0.52) & (fz < -0.09)
    rear_w = (fpar > 7.20) & (fz > 0.62) & (fz < 1.22)
    glass = side_glass | wind | rear_w
    trim = (fpar < 2.95) | (fn[:, 1] < -0.6)
    paint = ~glass & ~trim

    def sub(mask, uv_mode):
        tris = m.tris[mask]
        p = m.pos[tris].reshape(-1, 3)
        n = m.nrm[tris].reshape(-1, 3)
        uv = np.zeros((len(p), 2))
        if uv_mode == "livery":
            fnn = fn[mask]
            ax = np.abs(fnn)
            chart = np.where(ax[:, 1] >= np.maximum(ax[:, 0], ax[:, 2]) * 0.85, "top",
                             np.where(ax[:, 0] >= ax[:, 2], np.where(fnn[:, 0] > 0, "right", "left"),
                                      np.where(fnn[:, 2] < 0, "front", "rear")))
            chart = np.repeat(chart, 3)
            for c in livery.CHARTS:
                sel = chart == c
                if sel.any():
                    uv[sel] = livery.chart_uv(c, p[sel])
        else:
            uv = np.c_[p[:, 2], p[:, 1]]
        return Mesh(p, np.arange(len(p)).reshape(-1, 3), uv, n)

    body_paint = sub(paint, "livery")
    body_glass = sub(glass, "plain")
    body_trim = sub(trim, "plain")
    cap_meshes = []
    for c in caps:
        c = c.copy().compute_normals(flat=True)
        fnn = face_normals(c.pos, c.tris)
        chart = "front" if fnn[0, 2] < 0 else "rear"
        c.uv = livery.chart_uv(chart, c.pos)
        cap_meshes.append(c)
    body_paint = merge([body_paint] + cap_meshes)
    return body_paint, body_glass, body_trim, info


def surface_y(info, x, zq):
    """Approximate top-surface height of the body at (x, z)."""
    rings, z = info["rings"], info["z"]
    i = int(np.clip(np.searchsorted(z, zq), 1, len(z) - 1))
    ring = rings[i]
    half = ring[ring[:, 0] >= -1e-6]
    best = None
    for k in range(len(ring) - 1):
        a, b = ring[k], ring[k + 1]
        if (a[0] - x) * (b[0] - x) <= 0 and a[0] != b[0]:
            t = (x - a[0]) / (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
            best = y if best is None else max(best, y)
    return best


# ------------------------------------------------------------------ aero
def airfoil(chord, thick=0.12, camber=0.06, n=40):
    """Inverted cambered airfoil in (z, y): leading edge at z=0 (front), trailing at z=chord."""
    t = (1 - np.cos(np.linspace(0, math.pi, n))) / 2
    p = 0.4
    yt = 5 * thick * (0.2969 * np.sqrt(t) - 0.1260 * t - 0.3516 * t ** 2 + 0.2843 * t ** 3 - 0.1036 * t ** 4)
    yc = np.where(t < p, camber / p ** 2 * (2 * p * t - t ** 2), camber / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * t - t ** 2))
    upper = np.c_[t, yc + yt]
    lower = np.c_[t, yc - yt]
    loop = np.vstack([upper[::-1], lower[1:]])  # TE -> LE (upper) -> TE (lower)
    loop[:, 1] *= -1  # inverted (downforce)
    return loop * chord


def extrude_profile(loop_zy, x0, x1, closed=True):
    """Extrude a 2D (z, y) loop along X from x0 to x1, with end caps."""
    n = len(loop_zy)
    pos = []
    for x in (x0, x1):
        for zz, yy in loop_zy:
            pos.append((x, yy, zz))
    tris = []
    for k in range(n if closed else n - 1):
        a, b = k, (k + 1) % n
        tris += [(a, b, n + b), (a, n + b, n + a)]
    side = Mesh(pos, tris, np.c_[np.r_[np.arange(n), np.arange(n)] / n, np.r_[np.zeros(n), np.ones(n)]])
    side = fix_outward(side, (0.5 * (x0 + x1), np.mean(loop_zy[:, 1]), np.mean(loop_zy[:, 0])))
    side.compute_normals(flat=True)
    caps = []
    for x, sgn in ((x0, -1), (x1, 1)):
        c = np.r_[x, np.mean(loop_zy[:, 1]), np.mean(loop_zy[:, 0])]
        p = np.vstack([c, [(x, yy, zz) for zz, yy in loop_zy]])
        t = [(0, 1 + k, 1 + (k + 1) % n) for k in range(n)]
        cm = Mesh(p, t)
        fnn = face_normals(cm.pos, cm.tris)
        if fnn[0, 0] * sgn < 0:
            cm.tris = cm.tris[:, ::-1]
        cm.compute_normals(flat=True)
        caps.append(cm)
    return merge([side] + caps)


def build_aero(info):
    carbon, paint_dark = [], []
    # front splitter + endplates
    carbon.append(box((1.96, 0.014, 0.46), (0, 0.082, -2.15)))
    for s in (1, -1):
        carbon.append(box((0.012, 0.09, 0.40), (s * 0.975, 0.12, -2.13)))
        # canards (dive planes) on the bumper corners
        for k, (yy, zz) in enumerate(((0.25, -2.19), (0.34, -2.15))):
            c = box((0.17, 0.006, 0.10)).transformed(m=rot_z(s * 0.18) @ rot_x(-0.12), t=(s * 0.93, yy, zz))
            carbon.append(c)
        # side skirts
        carbon.append(box((0.07, 0.10, 1.80), (s * 0.905, 0.13, -0.05)))
    # rear diffuser: angled floor + fins
    dif = box((1.62, 0.012, 0.70)).transformed(m=rot_x(-0.24), t=(0, 0.19, 2.03))
    carbon.append(dif)
    for xf in (-0.60, -0.30, 0.0, 0.30, 0.60):
        fin = box((0.008, 0.20, 0.62)).transformed(m=rot_x(-0.24), t=(xf, 0.22, 2.04))
        carbon.append(fin)
    # rear wing: main plane + flap, endplates, swan-neck pylons
    span = 0.93
    main = airfoil(0.30, 0.12, 0.07)
    rot = np.array([[math.cos(-0.10), -math.sin(-0.10)], [math.sin(-0.10), math.cos(-0.10)]])
    main = main @ rot.T + np.array([1.98, 1.205])
    flap = airfoil(0.13, 0.10, 0.06) @ np.array([[math.cos(-0.45), -math.sin(-0.45)], [math.sin(-0.45), math.cos(-0.45)]]).T + np.array([2.25, 1.255])
    carbon.append(extrude_profile(main, -span, span))
    carbon.append(extrude_profile(flap, -span, span))
    for s in (1, -1):
        ep = box((0.010, 0.24, 0.42), (s * (span + 0.005), 1.20, 2.12))
        carbon.append(ep)
        # swan neck: rectangular section along a hook curve from the deck to the wing's top
        ctrl = np.array([(1.78, 0.89), (1.86, 1.12), (1.97, 1.30), (2.06, 1.31), (2.09, 1.228)])
        t = np.linspace(0, 1, 24)
        u = np.linspace(0, 1, len(ctrl))
        zc = np.interp(t, u, ctrl[:, 0]); yc = np.interp(t, u, ctrl[:, 1])
        from scipy.ndimage import gaussian_filter1d
        zc = gaussian_filter1d(zc, 1.5, mode="nearest"); yc = gaussian_filter1d(yc, 1.5, mode="nearest")
        zc[0], yc[0], zc[-1], yc[-1] = ctrl[0, 0], ctrl[0, 1], ctrl[-1, 0], ctrl[-1, 1]
        path = np.c_[np.full_like(t, s * 0.36), yc, zc]
        prof = [(-0.01, -0.05), (0.01, -0.05), (0.01, 0.05), (-0.01, 0.05)]
        sn = sweep(prof, path, up=(1, 0, 0))
        sn = fix_outward(sn, path.mean(axis=0)); sn.compute_normals(flat=True)
        paint_dark.append(sn)
    return merge(carbon), merge(paint_dark)


# ------------------------------------------------------------------ details
def build_details(info):
    black, chrome, red, metal = [], [], [], []
    for s in (1, -1):
        # mirror: stalk + housing + glass
        stalk = box((0.17, 0.02, 0.05)).transformed(m=rot_z(s * 0.35), t=(s * 0.90, 0.905, -0.40))
        black.append(stalk)
        hous = sphere(1.0, 20, 10).transformed(s=(0.075, 0.052, 0.095), t=(s * 1.00, 0.955, -0.40))
        black.append(hous)
        glass = box((0.12, 0.075, 0.004), (s * 1.00, 0.955, -0.315))
        chrome.append(glass)
        # side exhaust exits ahead of the rear wheels
        ex = cylinder(0.042, 0.10, 20, "x", (s * 0.95, 0.20, 0.72))
        metal.append(ex)
        black.append(cylinder(0.034, 0.102, 20, "x", (s * 0.95, 0.20, 0.72)))
    # tow hooks
    for (x, y, zz) in ((0.55, 0.24, -2.31), (-0.55, 0.30, 2.33)):
        ring = revolve([(0.045, -0.006), (0.055, -0.006), (0.055, 0.006), (0.045, 0.006), (0.045, -0.006)], 20, "z")
        red.append(ring.transformed(t=(x, y, zz)))
    # roof air scoop and antenna
    black.append(box((0.26, 0.045, 0.30), (0, 1.18, -0.02)))
    black.append(cylinder(0.004, 0.22, 8, "y", (0.0, 1.27, 0.75)))
    # wipers
    black.append(box((0.60, 0.012, 0.02)).transformed(m=rot_y(0.12), t=(0.10, 0.79, -0.56)))
    # rain light
    rain = box((0.16, 0.05, 0.02), (0, 0.33, 2.335))
    return merge(black), merge(chrome), merge(red), merge(metal), rain


def build_interior():
    grey, dark, red = [], [], []
    xd = -0.37  # left-hand drive
    # seat (bucket): base + back + side bolsters
    dark.append(box((0.44, 0.10, 0.48), (xd, 0.28, 0.32)))
    dark.append(box((0.44, 0.62, 0.10)).transformed(m=rot_x(0.30), t=(xd, 0.62, 0.55)))
    for s in (1, -1):
        dark.append(box((0.06, 0.55, 0.12)).transformed(m=rot_x(0.30), t=(xd + s * 0.22, 0.60, 0.50)))
    # dashboard and centre console
    grey.append(box((1.40, 0.16, 0.25), (0, 0.74, -0.42)))
    grey.append(box((0.20, 0.30, 0.55), (0, 0.40, 0.05)))
    # steering wheel (rectangular-ish formula style) + column
    ring = revolve([(0.15, -0.012), (0.17, -0.012), (0.17, 0.012), (0.15, 0.012), (0.15, -0.012)], 28, "z")
    sw = ring.transformed(m=rot_x(-0.35), t=(xd, 0.80, -0.18))
    dark.append(sw)
    dark.append(box((0.24, 0.10, 0.03)).transformed(m=rot_x(-0.35), t=(xd, 0.80, -0.18)))
    grey.append(cylinder(0.025, 0.40, 10, "z", (0, 0, 0)).transformed(m=rot_x(-0.35), t=(xd, 0.74, -0.38)))
    # roll cage: main hoop, front hoop, roof bars, diagonals
    tube = [(0.02 * math.cos(a), 0.02 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 9)[:-1]]

    def bar(points):
        p = np.array(points, float)
        t = np.linspace(0, 1, 12)
        path = np.array([np.interp(t, np.linspace(0, 1, len(p)), p[:, k]) for k in range(3)]).T
        return sweep(tube, path, up=(0, 1, 0) if abs(p[-1, 1] - p[0, 1]) < 0.5 else (1, 0, 0), closed_profile=True)

    for zz, h in ((0.30, 1.07), (-0.06, 1.05)):
        red.append(bar([(-0.70, 0.20, zz), (-0.62, 0.80, zz), (-0.46, h - 0.03, zz), (0.0, h, zz), (0.46, h - 0.03, zz), (0.62, 0.80, zz), (0.70, 0.20, zz)]))
    for s in (1, -1):
        red.append(bar([(s * 0.44, 1.02, -0.06), (s * 0.44, 1.04, 0.30)]))
        red.append(bar([(s * 0.62, 0.80, -0.06), (s * 0.66, 0.62, -0.52)]))
        red.append(bar([(s * 0.68, 0.40, -0.06), (s * 0.70, 0.42, 0.30)]))
    red.append(bar([(-0.46, 1.03, 0.30), (0.66, 0.30, 0.30)]))
    for i, mm in enumerate(red):
        mm.compute_normals()
    return merge(grey), merge(dark), merge(red)


# ------------------------------------------------------------------ wheels
def tyre_texture():
    W, H = 1024, 256
    img = Image.new("RGB", (W, H), (22, 22, 24))
    d = ImageDraw.Draw(img)
    rng = np.random.default_rng(3)
    arr = np.asarray(img).astype(np.int16)
    arr += rng.integers(-4, 5, size=(H, W, 1), dtype=np.int16)
    img = Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))
    d = ImageDraw.Draw(img)
    f = ImageFont.truetype(livery.FONT_BOLD, 34)
    fs = ImageFont.truetype(livery.FONT_BOLD, 16)
    for band in (0.17, 0.83):  # outer and inner sidewall centre lines (v)
        y = int(band * H)
        for k in range(2):
            x0 = k * W // 2 + 60
            d.text((x0, y - 20), "SIMGRIP  SLICK", font=f, fill=(222, 196, 40))
            d.text((x0 + 330, y - 8), "30/68 R18  RACING", font=fs, fill=(190, 190, 190))
    # tread: slightly lighter, scrubbed
    tread = np.asarray(img).copy()
    t0, t1 = int(0.38 * H), int(0.62 * H)
    tread[t0:t1] = (tread[t0:t1] * 1.25).clip(0, 255)
    return Image.fromarray(tread)


def build_wheel(R, W, rim_r=0.229):
    """Right-hand wheel at the origin, axis X, outer face at +X."""
    h = W / 2
    sh = 0.035
    prof = [(rim_r + 0.008, h - 0.012), (R - sh, h), (R - 0.008, h - 0.012), (R, h - sh), (R, -h + sh), (R - 0.008, -h + 0.012), (R - sh, -h), (rim_r + 0.008, -h + 0.012)]
    # reverse so the tread normal points outward with revolve()'s orientation rule
    prof = prof[::-1]
    L = np.r_[0, np.cumsum(np.linalg.norm(np.diff(np.array(prof), axis=0), axis=1))]
    v = L / L[-1]
    tyre = revolve([(r, a) for r, a in prof], 72, "x", uv_v=list(v))
    tyre.uv[:, 0] = 1.0 - tyre.uv[:, 0]  # lettering reads correctly on the right-hand wheels
    # rim barrel (visible inside the spokes), outer lip
    barrel = revolve([(rim_r, h - 0.02), (rim_r, -h + 0.02)], 48, "x").flipped()
    lip = revolve([(rim_r + 0.012, h - 0.004), (rim_r - 0.004, h - 0.004)], 48, "x")
    # spokes: 5 double spokes
    spokes = []
    for k in range(10):
        a = 2 * math.pi * (k // 2) / 5 + (0.17 if k % 2 else -0.17)
        sp = box((0.022, 0.16, 0.026)).transformed(m=rot_x(a), t=(0, 0, 0))
        sp = sp.transformed(t=(0, 0, 0))
        r0 = 0.145
        sp = box((0.024, 0.165, 0.028), (0, 0, 0)).transformed(m=rot_z(0.0))
        sp.pos[:, 1] += r0
        # dish: outer end sits further out
        sp.pos[:, 0] += h - 0.05 + (sp.pos[:, 1] - r0) * 0.12
        spokes.append(sp.transformed(m=rot_x(a)))
    hub = cylinder(0.065, 0.05, 24, "x", (h - 0.045, 0, 0))
    nut = cylinder(0.042, 0.04, 6, "x", (h - 0.012, 0, 0))
    disc = cylinder(0.190, 0.032, 48, "x", (-0.02, 0, 0))
    bell = cylinder(0.11, 0.06, 24, "x", (0.02, 0, 0))
    caliper = box((0.07, 0.18, 0.09))
    caliper.pos[:, 1] += 0.165
    caliper = caliper.transformed(m=rot_x(-0.9), t=(-0.02, 0, 0))
    return {"tyre": tyre, "rim": merge([barrel, lip] + spokes + [hub]), "nut": nut, "disc": disc, "bell": bell, "caliper": caliper}


# ------------------------------------------------------------------ assemble
def main():
    g = GLB()
    liv, liv_e = livery.build_livery(NUMBER, SCHEME)
    t_liv = g.image(liv, "livery_base")
    t_live = g.image(liv_e, "livery_emissive")
    t_tyre = g.image(tyre_texture(), "tyre")
    M = {
        "paint": g.material("paint_livery", (1, 1, 1, 1), 0.15, 0.32, base_tex=t_liv, clearcoat=1.0, clearcoat_rough=0.04,
                             emissive=(1, 1, 1), emissive_tex=t_live, emissive_strength=4.0),
        "glass": g.material("glass", (0.05, 0.07, 0.09, 0.55), 0.0, 0.03, alpha="BLEND", double=True),
        "trim": g.material("trim_black", (0.03, 0.03, 0.035, 1), 0.0, 0.6),
        "carbon": g.material("carbon", (0.045, 0.045, 0.05, 1), 0.3, 0.28, clearcoat=0.8),
        "pylon": g.material("anodised_black", (0.02, 0.02, 0.02, 1), 0.6, 0.35),
        "chrome": g.material("mirror_glass", (0.9, 0.92, 0.95, 1), 1.0, 0.05),
        "red": g.material("cage_red", (0.70, 0.04, 0.03, 1), 0.0, 0.45),
        "metal": g.material("exhaust_metal", (0.55, 0.50, 0.45, 1), 1.0, 0.35),
        "rain": g.material("rain_light", (0.4, 0.02, 0.02, 1), 0.0, 0.3, emissive=(1, 0.05, 0.05), emissive_strength=3.0),
        "grey": g.material("interior_grey", (0.12, 0.12, 0.13, 1), 0.0, 0.7),
        "seat": g.material("seat_fabric", (0.05, 0.05, 0.06, 1), 0.0, 0.95),
        "tyre": g.material("tyre_rubber", (1, 1, 1, 1), 0.0, 0.88, base_tex=t_tyre),
        "rim": g.material("rim_gunmetal", (0.12, 0.12, 0.13, 1), 0.9, 0.32),
        "nut": g.material("wheel_nut", (0.85, 0.55, 0.05, 1), 1.0, 0.3),
        "disc": g.material("brake_disc", (0.32, 0.31, 0.30, 1), 1.0, 0.55),
        "caliper": g.material("caliper", (0.85, 0.08, 0.05, 1), 0.0, 0.35, clearcoat=0.6),
    }
    paint, glass, trim, info = build_body_parts()
    aero_c, aero_p = build_aero(info)
    blk, chrome, red, metal, rain = build_details(info)
    ig, idk, icage = build_interior()

    body = g.node("body", g.mesh("body", [(paint, M["paint"]), (trim, M["trim"]), (glass, M["glass"])]), root=False)
    aero = g.node("aero", g.mesh("aero", [(aero_c, M["carbon"]), (aero_p, M["pylon"])]), root=False)
    details = g.node("details", g.mesh("details", [(blk, M["trim"]), (chrome, M["chrome"]), (red, M["red"]), (metal, M["metal"]), (rain, M["rain"])]), root=False)
    interior = g.node("interior", g.mesh("interior", [(ig, M["grey"]), (idk, M["seat"]), (icage, M["red"])]), root=False)

    wheels = []
    for name, zz, R, W in (("FL", cb.Z_FA, cb.R_F, 0.30), ("FR", cb.Z_FA, cb.R_F, 0.30), ("RL", cb.Z_RA, cb.R_R, 0.31), ("RR", cb.Z_RA, cb.R_R, 0.31)):
        right = name[1] == "R"
        w = build_wheel(R, W)
        if not right:
            w = {k: mirror_x(v) for k, v in w.items()}
            w["tyre"].uv[:, 0] = 1.0 - w["tyre"].uv[:, 0]
        spin_mesh = g.mesh(f"wheel_{name}_spin", [(w["tyre"], M["tyre"]), (w["rim"], M["rim"]), (w["nut"], M["nut"]), (w["disc"], M["disc"]), (w["bell"], M["rim"])])
        cal_mesh = g.mesh(f"wheel_{name}_caliper", [(w["caliper"], M["caliper"])])
        spin = g.node(f"wheel_{name}_spin", spin_mesh, root=False, extras={"spin_axis": [1, 0, 0]})
        cal = g.node(f"wheel_{name}_caliper", cal_mesh, root=False)
        x = cb.X_WHEEL if right else -cb.X_WHEEL
        wheels.append(g.node(f"wheel_{name}", None, t=(x, R, zz), children=[spin, cal], root=False,
                             extras={"radius": R, "width": W, "steered": name[0] == "F"}))
    g.node("car_root", None, children=[body, aero, details, interior] + wheels,
           extras={"forward": [0, 0, -1], "up": [0, 1, 0], "units": "m", "ground_y": 0.0,
                   "wheelbase": cb.Z_RA - cb.Z_FA, "name": "Halvard GT3"})
    size = g.save(OUT)
    print("saved", OUT, size // 1024, "KB")


if __name__ == "__main__":
    main()
