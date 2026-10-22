# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""Builds the 'Sierra Verde Circuit' test track (original, fictional) as track.glb + track.json.

Coordinates: metres, +Y up. The start/finish line is at the origin and the cars
start facing -Z. The pit lane and the pit building are on the +X (inside) side.
"""
import json
import math
import sys

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree

import textures as T
from gltfw import GLB, Mesh, box, cylinder, merge, revolve, sweep, grid_tris, fix_outward, face_normals
from layout import centerline

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/track.glb"
OUT_JSON = sys.argv[2] if len(sys.argv) > 2 else "/tmp/track.json"
RNG = np.random.default_rng(2026)
HW = 6.5  # road half width


def smooth_periodic(v, sigma):
    return gaussian_filter1d(v, sigma, mode="wrap")


# ------------------------------------------------------------------ centreline
def build_line():
    c, L = centerline(2.0)
    # start at the point closest to the origin, heading -Z
    i0 = int(np.argmin(np.hypot(c[:, 0], c[:, 1])))
    c = np.roll(c, -i0, axis=0)
    if c[1, 1] > c[0, 1]:
        c = c[::-1]
        c = np.roll(c, 1, axis=0)
    c = c - c[0]  # exact origin
    n = len(c)
    s = np.arange(n) * (L / n)
    f = s / L
    kf = [(0.0, 0.0), (0.12, 0.0), (0.20, 1.2), (0.30, 3.8), (0.42, 6.8), (0.52, 6.0), (0.62, 3.0),
          (0.72, 4.2), (0.82, 2.0), (0.90, 0.3), (0.94, 0.0), (1.0, 0.0)]
    y = np.interp(f, *zip(*kf))
    y = smooth_periodic(y, 20)
    p = np.c_[c[:, 0], y, c[:, 1]]
    t = np.roll(p, -1, 0) - np.roll(p, 1, 0)
    t[:, 1] = 0
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    r = np.c_[-t[:, 2], np.zeros(n), t[:, 0]]
    dt = (np.roll(t, -1, 0) - np.roll(t, 1, 0)) / (2 * L / n)
    k = np.einsum("ij,ij->i", dt, r)  # >0: turning right (inside on the right)
    k = smooth_periodic(k, 3)
    return {"p": p, "t": t, "r": r, "k": k, "s": s, "L": L, "n": n}


def zones(mask, min_len=8):
    """Contiguous True runs on a periodic boolean array -> list of index arrays."""
    n = len(mask)
    if mask.all():
        return [np.arange(n)]
    start = int(np.argmin(mask))  # a False index
    out, cur = [], []
    for j in range(n):
        i = (start + j) % n
        if mask[i]:
            cur.append(i)
        elif cur:
            if len(cur) >= min_len:
                out.append(np.array(cur))
            cur = []
    if cur and len(cur) >= min_len:
        out.append(np.array(cur))
    return out


def ribbon(line, idx, o0, o1, dy0=0.0, dy1=0.0, across=2, side=1, uv_len=1.0, uv_w=None, closed=False, prof=None):
    """Strip between lateral offsets o0..o1 (signed by side) for stations idx."""
    p, r, s, k = line["p"], line["r"], line["s"], line["k"]
    if prof is None:
        prof = [(a, dy0 + (dy1 - dy0) * a) for a in np.linspace(0, 1, across + 1)]
    m = len(prof)
    pos, uv = [], []
    for i in idx:
        bank = -np.clip(k[i] * 90, -0.035, 0.035)
        for a, dy in prof:
            o = (o0 + (o1 - o0) * a) * side
            q = p[i] + r[i] * o
            q[1] += bank * o - 0.012 * min(abs(o), HW) + dy
            pos.append(q)
            uv.append((s[i] / uv_len, a if uv_w is None else abs(o) / uv_w))
    tris = grid_tris(len(idx), m, closed_rows=closed)
    mesh = Mesh(pos, tris, uv)
    mesh.compute_normals()
    if mesh.nrm[:, 1].mean() < 0:
        mesh.tris = mesh.tris[:, ::-1]
        mesh.compute_normals()
    return mesh


def road_height(line, i, o):
    p, r, k = line["p"], line["r"], line["k"]
    bank = -np.clip(k[i] * 90, -0.035, 0.035)
    return p[i][1] + bank * o - 0.012 * min(abs(o), HW)


# ------------------------------------------------------------------ build
def main():
    line = build_line()
    p, r, t, k, s, L, n = line["p"], line["r"], line["t"], line["k"], line["s"], line["L"], line["n"]
    allidx = np.arange(n)
    g = GLB("car-simulator track builder")

    # textures / materials
    a_b, a_n, a_mr = T.asphalt()
    gr_b, gr_n, gr_mr = T.grass()
    gv_b, gv_n, gv_mr = T.gravel()
    c_b, c_n, c_mr = T.concrete()
    kb_b, kb_mr = T.kerb()
    tex = lambda img, name, jpeg=True: g.image(img, name, jpeg)
    surf = lambda name, fr: {"surface": name, "friction": fr}
    M = {}
    M["asphalt"] = g.material("asphalt", (1, 1, 1, 1), 0, 1, tex(a_b, "asphalt_base"), tex(a_mr, "asphalt_mr"), tex(a_n, "asphalt_normal"), 0.8, extras=surf("asphalt", 1.0))
    M["pit_asphalt"] = g.material("pit_asphalt", (0.85, 0.85, 0.88, 1), 0, 1, tex(a_b, "asphalt_base"), tex(a_mr, "asphalt_mr"), tex(a_n, "asphalt_normal"), 0.6, extras=surf("asphalt", 0.95))
    M["runoff"] = g.material("runoff_asphalt", (0.80, 0.82, 0.90, 1), 0, 1, tex(a_b, "asphalt_base"), tex(a_mr, "asphalt_mr"), tex(a_n, "asphalt_normal"), 0.8, extras=surf("asphalt_runoff", 0.95))
    M["grass"] = g.material("grass", (1, 1, 1, 1), 0, 1, tex(gr_b, "grass_base"), tex(gr_mr, "grass_mr"), tex(gr_n, "grass_normal"), 1.0, extras=surf("grass", 0.55))
    M["gravel"] = g.material("gravel", (1, 1, 1, 1), 0, 1, tex(gv_b, "gravel_base"), tex(gv_mr, "gravel_mr"), tex(gv_n, "gravel_normal"), 1.2, extras=surf("gravel", 0.45))
    M["kerb"] = g.material("kerb", (0.92, 0.92, 0.92, 1), 0, 1, tex(kb_b, "kerb_base", False), tex(kb_mr, "kerb_mr", False), extras=surf("kerb", 0.9))
    M["line"] = g.material("white_paint", (1, 1, 1, 1), 0, 0.55, tex(T.white_paint(), "white_paint"), extras=surf("paint", 0.85))
    M["concrete"] = g.material("concrete", (1, 1, 1, 1), 0, 1, tex(c_b, "concrete_base"), tex(c_mr, "concrete_mr"), tex(c_n, "concrete_normal"), 1.0, extras=surf("concrete", 0.9))
    M["armco"] = g.material("armco_steel", (0.78, 0.80, 0.82, 1), 1.0, 0.38)
    M["post"] = g.material("galvanised_post", (0.55, 0.56, 0.58, 1), 1.0, 0.5)
    M["tyrewall"] = g.material("tyre_wall", (0.05, 0.05, 0.055, 1), 0, 0.9)
    M["belt"] = g.material("tyre_wall_belt", (0.08, 0.22, 0.62, 1), 0, 0.6)
    M["fence"] = g.material("catch_fence", (1, 1, 1, 1), 0.8, 0.5, g.image(T.fence(), "fence_rgba"), alpha="MASK", double=True)
    M["seats"] = g.material("grandstand_seats", (1, 1, 1, 1), 0, 0.7, tex(T.seats(), "seats"))
    M["steel_dark"] = g.material("steel_dark", (0.20, 0.21, 0.23, 1), 0.8, 0.45)
    M["roof"] = g.material("roof_white", (0.90, 0.91, 0.92, 1), 0.1, 0.5)
    M["garage"] = g.material("garage_facade", (1, 1, 1, 1), 0, 0.6, tex(T.garage_facade(), "garages"))
    M["glassband"] = g.material("building_glass", (1, 1, 1, 1), 0.6, 0.12, tex(T.glass_band(), "glass_band"))
    M["wall_white"] = g.material("building_wall", (0.86, 0.87, 0.88, 1), 0, 0.7)
    M["ads"] = g.material("ad_banners", (1, 1, 1, 1), 0, 0.5, tex(T.ad_atlas(), "ads"), double=True)
    M["boards"] = g.material("distance_boards", (1, 1, 1, 1), 0, 0.6, tex(T.board_numbers(), "boards"))
    M["chequer"] = g.material("chequer", (1, 1, 1, 1), 0, 0.5, tex(T.chequer(), "chequer", False), extras=surf("paint", 0.85))
    M["gridmark"] = g.material("grid_marks", (1, 1, 1, 1), 0, 0.55, g.image(T.grid_marks(), "grid_marks"), alpha="MASK", extras=surf("paint", 0.85))
    M["bark"] = g.material("bark", (1, 1, 1, 1), 0, 0.9, tex(T.bark(), "bark"))
    M["leaves"] = g.material("leaves", (1, 1, 1, 1), 0, 0.85, tex(T.leaves(), "leaves"))
    M["conifer"] = g.material("conifer", (1, 1, 1, 1), 0, 0.85, tex(T.leaves(seed=61, c0=(16, 40, 22), c1=(44, 82, 40)), "conifer"))
    M["orange"] = g.material("marshal_orange", (0.95, 0.42, 0.05, 1), 0, 0.5)
    M["lights_off"] = g.material("start_lights", (0.05, 0.02, 0.02, 1), 0, 0.3, emissive=(1, 0.05, 0.03), emissive_strength=0.2)
    M["sign"] = g.material("gantry_sign", (1, 1, 1, 1), 0, 0.5, tex(T.sign_panel("SIERRA VERDE"), "sign_sierra"))
    M["bridge_sign"] = g.material("bridge_sign", (1, 1, 1, 1), 0, 0.5, tex(T.sign_panel("OPEN SIM RACING", bg=(240, 120, 20)), "sign_osr"), double=True)

    nodes = []

    def add(name, prims, collision=False, extras=None):
        ex = {"collision": collision}
        if extras:
            ex.update(extras)
        nodes.append(g.node(name, g.mesh(name, prims), root=False, extras=ex))

    # ---------------------------------------------------- road + racing line rubber
    o_samples = np.linspace(-HW, HW, 9)
    race = np.clip(smooth_periodic(k, 25) * 700, -HW + 1.6, HW - 1.6)
    road = ribbon(line, allidx, -HW, HW, across=8, uv_len=8.0, uv_w=None, closed=True)
    road.uv = np.c_[road.uv[:, 0], road.uv[:, 1] * (2 * HW / 8.0)]
    cols = []
    for i in allidx:
        for o in o_samples:
            d = 1 - 0.20 * math.exp(-((o - race[i]) / 1.9) ** 2)
            cols.append((d, d, d, 1))
    road.col = np.array(cols)
    # edge lines (white) and start/finish chequer + grid boxes
    lines = [ribbon(line, allidx, HW - 0.35, HW - 0.12, 0.004, 0.004, 1, side=sd, uv_len=4, closed=True) for sd in (1, -1)]
    add("road", [(road, M["asphalt"]), (merge(lines), M["line"])], True)

    chq = box((2 * HW, 0.004, 0.6), (0, road_height(line, 0, 0) + 0.004, 0))
    chq.uv = np.c_[(chq.pos[:, 0] + HW) / (2 * HW), (chq.pos[:, 2] + 0.3) / 0.6]
    grid = []
    slots = []
    for q in range(12):
        dz = 10 + q * 8
        xx = 3.0 if q % 2 == 0 else -3.0
        dz += 4 if q % 2 else 0
        slots.append({"position": [xx, 0.0, dz], "heading_deg": 0.0})
        quad = Mesh([(xx - 1.2, 0.006, dz - 0.6), (xx + 1.2, 0.006, dz - 0.6), (xx + 1.2, 0.006, dz + 0.6), (xx - 1.2, 0.006, dz + 0.6)],
                    [(0, 2, 1), (0, 3, 2)], [(0, 0), (1, 0), (1, 1), (0, 1)])
        grid.append(quad)
    add("start_finish", [(chq, M["chequer"]), (merge(grid), M["gridmark"])], False)

    # ---------------------------------------------------- kerbs, gravel, runoff
    ks = smooth_periodic(k, 6)
    corner = np.abs(ks) > 1 / 170
    kerb_meshes, gravel_meshes, runoff_meshes = [], [], []
    kz = zones(corner, 6)
    kerb_prof = [(0.0, 0.0), (0.15, 0.025), (0.5, 0.045), (0.85, 0.035), (1.0, 0.0)]
    gravel_zones, kerb_zones = [], []
    straight_main = (np.abs(p[:, 0]) < 4) & (p[:, 2] > -385) & (p[:, 2] < 145)
    for zi, z in enumerate(kz):
        side_in = 1 if ks[z].mean() > 0 else -1
        ext = np.arange(z[0] - 6, z[-1] + 7) % n
        kerb_meshes.append(ribbon(line, ext, HW, HW + 1.3, side=side_in, uv_len=2.0, prof=kerb_prof))
        kerb_meshes.append(ribbon(line, ext, HW, HW + 1.6, side=-side_in, uv_len=2.0, prof=kerb_prof))
        kerb_zones.append({"from_s": float(s[ext[0]]), "to_s": float(s[ext[-1]]), "sides": "both"})
        tight = np.abs(ks[z]).max() > 1 / 90
        run = np.arange(z[0] - 10, z[-1] + 30) % n
        run = run[~straight_main[run]]
        if len(run) < 5:
            continue
        if tight or zi % 2 == 0:
            gravel_meshes.append(ribbon(line, run, HW + 4.0, HW + 24.0, -0.03, -0.09, 4, side=-side_in, uv_len=6.0, uv_w=6.0))
            gravel_zones.append({"from_s": float(s[run[0]]), "to_s": float(s[run[-1]]), "side": "left" if side_in > 0 else "right", "surface": "gravel"})
        else:
            runoff_meshes.append(ribbon(line, run, HW + 1.6, HW + 16.0, -0.01, -0.06, 3, side=-side_in, uv_len=8.0, uv_w=8.0))
            gravel_zones.append({"from_s": float(s[run[0]]), "to_s": float(s[run[-1]]), "side": "left" if side_in > 0 else "right", "surface": "asphalt_runoff"})
    add("kerbs", [(merge(kerb_meshes), M["kerb"])], True)
    add("gravel_traps", [(merge(gravel_meshes), M["gravel"])], True)
    add("runoff", [(merge(runoff_meshes), M["runoff"])], True)

    # ---------------------------------------------------- terrain
    tree = cKDTree(p[:, [0, 2]])
    xmin, xmax = p[:, 0].min() - 220, p[:, 0].max() + 220
    zmin, zmax = p[:, 2].min() - 220, p[:, 2].max() + 220
    step = 5.0
    gx = np.arange(xmin, xmax + step, step)
    gz = np.arange(zmin, zmax + step, step)
    X, Z = np.meshgrid(gx, gz)
    d, idx = tree.query(np.c_[X.ravel(), Z.ravel()])
    d = d.reshape(X.shape); idx = idx.reshape(X.shape)
    base = (3.5 * np.sin(X / 140 + 0.7) * np.cos(Z / 170) + 2.5 * np.sin((X + Z) / 90) + 1.5 * np.cos(X / 55 - Z / 75) + 2.0)
    far = np.clip((np.hypot(X - p[:, 0].mean(), Z - p[:, 2].mean()) - 450) / 300, 0, 1)
    base += far * 18 * (0.5 + 0.5 * np.sin(X / 120) * np.cos(Z / 110))
    # terrain sits below the road surface and the verges (no z-fighting at the edges)
    dxz = np.c_[X.ravel(), Z.ravel()] - p[idx.ravel()][:, [0, 2]]
    o_s = np.einsum("ij,ij->i", dxz, r[idx.ravel()][:, [0, 2]]).reshape(X.shape)
    bank = -np.clip(k[idx] * 90, -0.035, 0.035)
    road_y = p[idx, 1] + bank * np.clip(o_s, -HW - 3, HW + 3) - 0.012 * np.minimum(np.abs(o_s), HW)
    drop = np.interp(np.abs(o_s), [0, HW + 1.0, HW + 2.5, HW + 30], [0.40, 0.40, 0.14, 0.14])
    ynear = road_y - drop
    b = np.clip((d - (HW + 26)) / 70, 0, 1)
    b = b * b * (3 - 2 * b)
    # paddock / pit area flat
    pad = (X > 4) & (X < 95) & (Z > -330) & (Z < 150)
    b[pad] = np.minimum(b[pad], np.clip((np.maximum(X[pad] - 95, 0)) / 30, 0, 1))
    Y = (1 - b) * ynear + b * (ynear + base - 2.0)
    # flat paddock heights equal to the straight (y ~ 0)
    Y[pad] = np.where(d[pad] > HW + 2, -0.30, Y[pad])
    rows, cols_ = X.shape
    tpos = np.c_[X.ravel(), Y.ravel(), Z.ravel()]
    ttris = grid_tris(rows, cols_)
    terr = Mesh(tpos, ttris, np.c_[X.ravel() / 6.0, Z.ravel() / 6.0])
    terr.compute_normals()
    if terr.nrm[:, 1].mean() < 0:
        terr.tris = terr.tris[:, ::-1]; terr.compute_normals()
    # macro colour variation (mowed stripes near the track, drier further out)
    near = np.clip(1 - (d.ravel() - 20) / 120, 0, 1)
    mow = 1.0 - 0.035 * near * (np.sin((X.ravel() * 0.8 + Z.ravel() * 0.6) / 3.0) > 0)
    dry = np.clip((d.ravel() - 60) / 200, 0, 1)
    terr.col = np.c_[mow * (1 + 0.12 * dry), mow * (1 - 0.05 * dry), mow * (1 - 0.15 * dry), np.ones(len(tpos))]
    add("terrain", [(terr, M["grass"])], True)

    def ground(x, z):
        """Terrain height at (x, z) (bilinear on the grid)."""
        fx = (x - gx[0]) / step; fz = (z - gz[0]) / step
        i0 = int(np.clip(math.floor(fz), 0, rows - 2)); j0 = int(np.clip(math.floor(fx), 0, cols_ - 2))
        ax, az = fx - j0, fz - i0
        return (Y[i0, j0] * (1 - ax) * (1 - az) + Y[i0, j0 + 1] * ax * (1 - az) + Y[i0 + 1, j0] * (1 - ax) * az + Y[i0 + 1, j0 + 1] * ax * az)

    # ---------------------------------------------------- barriers
    off = np.full((n, 2), HW + 11.0)  # [right, left]
    tyre_mask = np.zeros((n, 2), bool)
    for gzn, zz in zip(gravel_zones, kz):
        pass
    for zi, z in enumerate(kz):
        side_in = 1 if ks[z].mean() > 0 else -1
        run = np.arange(z[0] - 12, z[-1] + 32) % n
        col_out = 0 if side_in < 0 else 1  # outside index: right if turning left
        tight = np.abs(ks[z]).max() > 1 / 90
        if tight or zi % 2 == 0:
            off[run, col_out] = HW + 26.0
            tyre_mask[run, col_out] = True
        else:
            off[run, col_out] = HW + 18.0
    for c in range(2):
        off[:, c] = smooth_periodic(off[:, c], 4)
    pit_side = straight_main.copy()
    wall_main = straight_main
    armco_prof = [(-0.0, -0.17), (0.06, -0.12), (0.03, -0.05), (0.06, 0.02), (0.03, 0.09), (0.06, 0.14), (0.0, 0.17)]
    armco_parts, post_parts, tyre_parts, belt_parts, wall_parts, fence_parts, fpost_parts, ad_parts = [], [], [], [], [], [], [], []
    for c, sd in ((0, 1), (1, -1)):
        mask_armco = ~tyre_mask[:, c] & ~wall_main
        for z in zones(mask_armco, 4):
            path = np.array([p[i] + r[i] * sd * off[i, c] for i in z])
            for j, i in enumerate(z):
                path[j, 1] = ground(path[j, 0], path[j, 2])
            for hgt in (0.42, 0.74):
                prof = [(sd * a, b_ * 1.0) for a, b_ in armco_prof]
                m = sweep(prof, path + np.array([0, hgt, 0]), closed_profile=False, uv_u_scale=0.25)
                m.compute_normals()
                armco_parts.append(m)
                armco_parts.append(m.flipped())
            for j in range(0, len(z), 1):
                q = path[j]
                post_parts.append(box((0.12, 0.92, 0.08), (q[0] + r[z[j]][0] * sd * 0.10, q[1] + 0.46, q[2] + r[z[j]][2] * sd * 0.10)))
            # ad banners on the armco every ~80 m, facing the track
            for j in range(10, len(z) - 8, 40):
                a_, b_ = path[j] - r[z[j]] * sd * 0.08, path[j + 6] - r[z[j + 6]] * sd * 0.08
                row = (j // 40 + c) % 8
                v0, v1 = row / 8, (row + 1) / 8
                lo, hi = 0.25, 0.95
                quad = Mesh([a_ + [0, lo, 0], b_ + [0, lo, 0], b_ + [0, hi, 0], a_ + [0, hi, 0]], [(0, 1, 2), (0, 2, 3)],
                            [(0, v1), (1, v1), (1, v0), (0, v0)])
                ad_parts.append(quad)
        for z in zones(tyre_mask[:, c], 4):
            path = np.array([p[i] + r[i] * sd * off[i, c] for i in z])
            seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
            for dd in np.arange(0, seg[-1], 0.66):
                q = np.array([np.interp(dd, seg, path[:, a]) for a in range(3)])
                gy = ground(q[0], q[2])
                tyre_parts.append(cylinder(0.31, 0.78, 9, "y", (q[0], gy + 0.39, q[2]), caps=False))
            # conveyor belt facing the track
            bpath = path - np.array([r[i] * sd * 0.36 for i in z])
            for j in range(len(z)):
                bpath[j, 1] = ground(bpath[j, 0], bpath[j, 2]) + 0.42
            belt = sweep([(0, -0.40), (0, 0.40)], bpath, closed_profile=False, uv_u_scale=0.2)
            belt.compute_normals()
            belt_parts.append(belt)
            belt_parts.append(belt.flipped())
    # main straight: concrete walls both sides + catch fence, ads on the outside wall
    zst = zones(wall_main, 4)
    for z in zst:
        for sd, o in ((-1, HW + 3.2), (1, HW + 1.6)):
            path = np.array([p[i] + r[i] * sd * o for i in z])
            prof = [(0, 0.0), (0, 1.05), (sd * 0.35, 1.05), (sd * 0.35, 0.0)]
            w = sweep(prof, path, closed_profile=False, uv_u_scale=0.25)
            w = fix_outward(w, path.mean(axis=0) + np.array([0, 0.5, 0]))
            w.compute_normals(flat=True)
            wall_parts.append(w)
            fp = path + np.array([0, 1.05, 0])
            fence = sweep([(sd * 0.1, 0), (sd * 0.1, 3.0)], fp, closed_profile=False, uv_u_scale=0.5)
            fence.uv = np.c_[fence.uv[:, 0], fence.uv[:, 1] / 0.5 * 0.5]
            fence.compute_normals()
            fence_parts.append(fence)
            for j in range(0, len(z), 2):
                q = fp[j]
                fpost_parts.append(box((0.08, 3.1, 0.08), (q[0] + r[z[j]][0] * sd * 0.1, q[1] + 1.5, q[2] + r[z[j]][2] * sd * 0.1)))
            # ad banners on the track-facing side of the outside wall
            if sd == -1:
                for j in range(0, len(z) - 12, 14):
                    a_, b_ = path[j] + r[z[j]] * 0.02, path[j + 12] + r[z[j + 12]] * 0.02
                    row = (j // 14) % 8
                    v0, v1 = row / 8, (row + 1) / 8
                    quad = Mesh([a_ + [0, 0.1, 0], b_ + [0, 0.1, 0], b_ + [0, 0.95, 0], a_ + [0, 0.95, 0]], [(0, 1, 2), (0, 2, 3)],
                                [(0, v1), (1, v1), (1, v0), (0, v0)])
                    ad_parts.append(quad)
    add("barriers_armco", [(merge(armco_parts), M["armco"]), (merge(post_parts), M["post"])], True)
    add("barriers_tyres", [(merge(tyre_parts), M["tyrewall"]), (merge(belt_parts), M["belt"])], True)
    add("barriers_walls", [(merge(wall_parts), M["concrete"]), (merge(fence_parts), M["fence"]), (merge(fpost_parts), M["post"])], True)
    add("ad_banners", [(merge(ad_parts), M["ads"])], False)

    # ---------------------------------------------------- pit lane, pit wall, pit building, paddock
    zs = p[straight_main, 2]
    pit = []
    pz0, pz1 = -300.0, 110.0
    pit.append(box((15.0, 0.02, pz1 - pz0), (HW + 10.0, -0.10, 0.5 * (pz0 + pz1)), uv_scale=1 / 8))
    # entry and exit lanes (tapers back to the track)
    for zz0, zz1 in ((pz1, 140.0), (-345.0, pz0)):
        pit.append(box((8.0, 0.02, zz1 - zz0), (HW + 6.0, -0.10, 0.5 * (zz0 + zz1)), uv_scale=1 / 8))
    paddock = box((36.0, 0.02, 260.0), (HW + 52.0, -0.11, -80.0), uv_scale=1 / 8)
    pw = box((0.5, 1.1, pz1 - pz0), (HW + 1.85, 0.45, 0.5 * (pz0 + pz1)), uv_scale=0.25)
    pit_line = box((0.15, 0.01, pz1 - pz0), (HW + 4.5, -0.085, 0.5 * (pz0 + pz1)))
    add("pit_lane", [(merge(pit + [paddock]), M["pit_asphalt"]), (pw, M["concrete"]), (pit_line, M["line"])], True)
    # building: garages at ground level, glass hospitality level, roof
    bx0, bx1, bz0, bz1 = HW + 17.5, HW + 33.0, -190.0, 30.0
    blen = bz1 - bz0
    gar = box((bx1 - bx0, 4.5, blen), (0.5 * (bx0 + bx1), 2.15, 0.5 * (bz0 + bz1)))
    face = []
    nf = 0
    for kseg in range(int(blen // 40)):
        za, zb = bz0 + kseg * 40, bz0 + (kseg + 1) * 40
        quad = Mesh([(bx0 - 0.02, -0.1, zb), (bx0 - 0.02, -0.1, za), (bx0 - 0.02, 4.4, za), (bx0 - 0.02, 4.4, zb)], [(0, 2, 1), (0, 3, 2)],
                    [(0, 1), (1, 1), (1, 0), (0, 0)])
        face.append(quad)
    upper = box((bx1 - bx0 - 1.0, 3.5, blen - 1.0), (0.5 * (bx0 + bx1) + 0.5, 6.15, 0.5 * (bz0 + bz1)))
    band = []
    for kseg in range(int(blen // 20)):
        za, zb = bz0 + 0.5 + kseg * 20, bz0 + 0.5 + (kseg + 1) * 20
        x = bx0 + 0.48
        band.append(Mesh([(x, 4.5, zb), (x, 4.5, za), (x, 7.8, za), (x, 7.8, zb)], [(0, 2, 1), (0, 3, 2)], [(0, 1), (1, 1), (1, 0), (0, 0)]))
    roof = box((bx1 - bx0 + 5.0, 0.35, blen + 4.0), (0.5 * (bx0 + bx1) - 1.5, 8.1, 0.5 * (bz0 + bz1)))
    sign = Mesh([(bx0 - 4.0, 8.3, 10.0), (bx0 - 4.0, 8.3, -50.0), (bx0 - 4.0, 10.8, -50.0), (bx0 - 4.0, 10.8, 10.0)], [(0, 2, 1), (0, 3, 2)],
                [(1, 1), (0, 1), (0, 0), (1, 0)])
    add("pit_building", [(gar, M["wall_white"]), (merge(face), M["garage"]), (upper, M["wall_white"]), (merge(band), M["glassband"]),
                         (roof, M["roof"]), (sign, M["sign"])], True)
    # paddock trucks
    trucks = []
    for q in range(8):
        zz = -170 + q * 24
        trucks.append(box((2.5, 3.8, 13.5), (HW + 48 + (q % 2) * 6, 1.8, zz)))
    add("paddock_trucks", [(merge(trucks), M["wall_white"])], True)

    # ---------------------------------------------------- grandstands (outside of the straight)
    stands = []
    st_roof, st_steel = [], []
    for (za, zb) in ((-150.0, -70.0), (-55.0, 25.0)):
        x_front = -(HW + 6.0)
        for row in range(14):
            depth, rise = 0.85, 0.42
            x0 = x_front - row * depth
            step_box = box((depth, 0.42 + row * rise, zb - za), (x0 - depth / 2, (0.42 + row * rise) / 2 + 0.6, 0.5 * (za + zb)))
            # seat texture on the top and riser faces
            step_box.uv = np.c_[(step_box.pos[:, 2] - za) / 20.0, (row + (step_box.pos[:, 1] > 0.6 + row * rise)) / 16.0]
            stands.append(step_box)
        back_x = x_front - 14 * 0.85
        st_steel.append(box((0.4, 9.0, zb - za), (back_x - 0.2, 5.1, 0.5 * (za + zb))))
        for zc in np.arange(za, zb + 0.1, 16.0):
            st_steel.append(box((0.35, 10.5, 0.35), (back_x + 0.3, 5.25, zc)))
            st_steel.append(box((0.30, 0.30, 13.0), (back_x + 6.0, 10.4, zc)).transformed(m=np.eye(3)))
        st_roof.append(box((13.5, 0.25, zb - za + 2), (back_x + 6.2, 10.7, 0.5 * (za + zb))))
    add("grandstands", [(merge(stands), M["seats"]), (merge(st_steel), M["steel_dark"]), (merge(st_roof), M["roof"])], True)

    # ---------------------------------------------------- start gantry
    gy0 = 0.0
    gan = [box((0.6, 7.5, 0.6), (-(HW + 2.4), 3.75, -6.0)), box((0.6, 7.5, 0.6), (HW + 2.4, 3.75, -6.0)),
           box((2 * HW + 5.4, 0.8, 0.6), (0, 7.2, -6.0))]
    lights = [box((0.5, 0.5, 0.12), (x, 6.35, -5.66)) for x in np.linspace(-1.6, 1.6, 5)]
    lights_housing = box((4.4, 0.8, 0.3), (0, 6.35, -5.85))
    gsign = Mesh([(-(HW + 2.7), 7.6, -5.68), (HW + 2.7, 7.6, -5.68), (HW + 2.7, 9.2, -5.68), (-(HW + 2.7), 9.2, -5.68)], [(0, 1, 2), (0, 2, 3)],
                 [(0, 1), (1, 1), (1, 0), (0, 0)])
    gsign_back = gsign.flipped()
    gsign_back.uv = np.c_[1 - gsign_back.uv[:, 0], gsign_back.uv[:, 1]]
    gsign_back.pos[:, 2] -= 0.6
    add("start_gantry", [(merge(gan + [lights_housing]), M["steel_dark"]), (merge(lights), M["lights_off"]), (merge([gsign, gsign_back]), M["sign"])], True,
        {"start_lights": [[float(x), 6.35, -5.6] for x in np.linspace(-1.6, 1.6, 5)]})

    # ---------------------------------------------------- footbridge over the back straight
    ib = int(np.argmin(np.hypot(p[:, 0] - 664, p[:, 2] + 200)))
    q, rr, tt = p[ib], r[ib], t[ib]
    span = HW + 9
    deck_c = q + np.array([0, 6.4, 0])
    ang = math.atan2(rr[0], rr[2])
    rotm = np.array([[math.cos(ang), 0, math.sin(ang)], [0, 1, 0], [-math.sin(ang), 0, math.cos(ang)]])
    deck = box((3.0, 0.5, 2 * span), (0, 0, 0)).transformed(m=rotm, t=deck_c)
    br = [deck]
    for sd in (1, -1):
        tower = box((3.6, 7.5, 3.6), (0, 0, 0)).transformed(m=rotm, t=q + rr * sd * (span + 1.0) + np.array([0, 3.0, 0]))
        br.append(tower)
    banners = []
    for sd in (1, -1):
        c0 = deck_c + tt * sd * 1.55
        a_ = c0 - rr * span; b_ = c0 + rr * span
        banners.append(Mesh([a_ + [0, -0.9, 0], b_ + [0, -0.9, 0], b_ + [0, 1.1, 0], a_ + [0, 1.1, 0]], [(0, 1, 2), (0, 2, 3)],
                            [(0, 1), (1, 1), (1, 0), (0, 0)]))
    add("footbridge", [(merge(br), M["concrete"]), (merge(banners), M["bridge_sign"])], True)

    # ---------------------------------------------------- distance boards before the two heaviest brakings
    boards = []
    brakes = [z for z in kz if np.abs(ks[z]).max() > 1 / 90][:2]
    for z in brakes:
        i_turn = z[0]
        side_out = -1 if ks[z].mean() > 0 else 1
        for bi, dist in enumerate((300, 200, 100)):
            i = (i_turn - int(dist / 2.0)) % n
            base_q = p[i] + r[i] * side_out * (HW + 3.0)
            gy = ground(base_q[0], base_q[2])
            post = box((0.12, 1.4, 0.12), (base_q[0], gy + 0.7, base_q[2]))
            boards.append((post, M["post"]))
            u0, u1 = bi / 3, (bi + 1) / 3
            rr_ = r[i]
            # quad faces -t (towards the approaching car); viewer's right = +r
            a_ = base_q - rr_ * 0.6 - t[i] * 0.05
            b_ = base_q + rr_ * 0.6 - t[i] * 0.05
            y0_, y1_ = gy + 1.3, gy + 2.2
            panel = Mesh([a_ + [0, y0_ - a_[1], 0], b_ + [0, y0_ - b_[1], 0], b_ + [0, y1_ - b_[1], 0], a_ + [0, y1_ - a_[1], 0]],
                         [(0, 1, 2), (0, 2, 3)], [(u0, 1), (u1, 1), (u1, 0), (u0, 0)])
            panel.compute_normals()
            if panel.nrm[0] @ (-t[i]) < 0:
                panel.tris = panel.tris[:, ::-1]; panel.compute_normals()
            back = panel.flipped(); back.uv = np.c_[np.full(4, 0.999), np.full(4, 0.5)]
            boards.append((panel, M["boards"])); boards.append((back, M["boards"]))
    add("distance_boards", [(merge([b for b, mm in boards if mm == M["post"]]), M["post"]), (merge([b for b, mm in boards if mm == M["boards"]]), M["boards"])], False)

    # ---------------------------------------------------- marshal posts
    huts = [], []
    for i in range(40, n, 130):
        if straight_main[i]:
            continue
        sd = 1 if ks[i] < 0 else -1  # inside of the bend side is fine for a post
        c = 0 if sd > 0 else 1
        q = p[i] + r[i] * sd * (off[i, c] + 2.5)
        gy = ground(q[0], q[2])
        huts[0].append(box((2.0, 2.3, 2.0), (q[0], gy + 1.15, q[2])))
        huts[1].append(box((2.4, 0.15, 2.4), (q[0], gy + 2.38, q[2])))
    add("marshal_posts", [(merge(huts[0]), M["wall_white"]), (merge(huts[1]), M["orange"])], True)

    # ---------------------------------------------------- trees
    trunks, crowns, cones = [], [], []
    tree_pts = []
    tries = 0
    while len(tree_pts) < 650 and tries < 20000:
        tries += 1
        x = RNG.uniform(xmin + 20, xmax - 20); z = RNG.uniform(zmin + 20, zmax - 20)
        dd, _ = tree.query((x, z))
        if dd < HW + 40:
            continue
        if -60 < x < 110 and -340 < z < 170:
            continue
        tree_pts.append((x, z))
    ico = revolve([(0.0001, -1.0), (0.7, -0.7), (1.0, 0.0), (0.7, 0.7), (0.0001, 1.0)], 8, "y")
    cone = revolve([(1.0, 0.0), (0.0001, 1.0)], 8, "y")
    for (x, z) in tree_pts:
        gy = ground(x, z)
        hgt = RNG.uniform(6, 13)
        trunks.append(cylinder(0.18 * hgt / 9, hgt * 0.45, 6, "y", (x, gy + hgt * 0.22, z), caps=False))
        if RNG.random() < 0.45:
            for kk in range(2):
                cr = cone.transformed(s=(hgt * 0.22 * (1 - 0.3 * kk), hgt * 0.55, hgt * 0.22 * (1 - 0.3 * kk)), t=(x, gy + hgt * (0.25 + 0.3 * kk), z))
                cr.uv = cr.pos[:, [0, 1]] / 3.0
                cones.append(cr)
        else:
            rad = hgt * RNG.uniform(0.28, 0.36)
            cr = ico.transformed(s=(rad, rad * 0.85, rad), t=(x, gy + hgt * 0.62, z))
            cr.uv = cr.pos[:, [0, 1]] / 3.0
            crowns.append(cr)
    add("trees", [(merge(trunks), M["bark"]), (merge(crowns), M["leaves"]), (merge(cones), M["conifer"])], False)

    g.node("track_root", None, children=nodes, extras={"name": "Sierra Verde Circuit", "length_m": round(L, 1), "forward_at_start": [0, 0, -1],
                                                       "units": "m"})
    size = g.save(OUT)

    # ---------------------------------------------------- data for the engine
    cl = [[round(float(a), 3) for a in q] for q in p[::2]]
    third = n // 3
    data = {
        "name": "Sierra Verde Circuit", "length_m": round(L, 2), "direction": "clockwise (seen from above, +X right, -Z up)",
        "units": "m", "up": [0, 1, 0],
        "centerline_step_m": round(2 * L / n, 3), "centerline": cl,
        "half_width_m": HW,
        "start_finish": {"position": [0, 0, 0], "forward": [0, 0, -1], "width_m": 2 * HW},
        "sectors_m": [0.0, round(float(s[third]), 1), round(float(s[2 * third]), 1)],
        "grid_slots": slots,
        "pit_lane": {"speed_limit_kmh": 60, "polyline": [[HW + 6, -0.1, 140], [HW + 10, -0.1, pz1], [HW + 10, -0.1, pz0], [HW + 6, -0.1, -345]],
                     "garages_x": bx0, "garages_z": [bz0, bz1]},
        "kerb_zones": kerb_zones, "runoff_zones": gravel_zones,
        "surfaces": {"asphalt": 1.0, "asphalt_runoff": 0.95, "kerb": 0.9, "paint": 0.85, "concrete": 0.9, "grass": 0.55, "gravel": 0.45},
        "notes": "Surface ids are also in each glTF material's extras.surface; nodes carry extras.collision.",
    }
    with open(OUT_JSON, "w") as f:
        json.dump(data, f, indent=1)
    print("saved", OUT, size // 1024, "KB; length", round(L), "m")


if __name__ == "__main__":
    main()
