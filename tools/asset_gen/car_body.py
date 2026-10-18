# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""GT3-style body shell built as a loft of cross sections (original design)."""
import numpy as np
from scipy.ndimage import gaussian_filter1d

from gltfw import Mesh, grid_tris

# Car layout (metres). The car faces -Z; +X is the car's right; ground at y = 0.
L_FRONT, L_REAR = -2.30, 2.32
Z_FA, Z_RA = -1.38, 1.32          # axles (wheelbase 2.70)
R_F, R_R = 0.340, 0.355           # tyre radii
X_WHEEL = 0.835                   # wheel centre lateral offset
TYRE_HALF_W = 0.160
ARCH_CLEAR = 0.045


def kf(z, pts, smooth=0.0, n=None):
    zs, vs = zip(*pts)
    v = np.interp(z, zs, vs)
    if smooth:
        dz = z[1] - z[0]
        v = gaussian_filter1d(v, smooth / dz, mode="nearest")
    return v


def profiles(z):
    s = 0.06
    P = {}
    P["bottom"] = kf(z, [(-2.30, 0.13), (-2.22, 0.105), (1.70, 0.10), (2.32, 0.28)], s)
    # Centre-line top: nose, hood valley, windscreen, roof, fastback, deck, ducktail
    P["top"] = kf(z, [(-2.30, 0.40), (-2.27, 0.49), (-2.18, 0.545), (-1.90, 0.59), (-1.20, 0.635), (-0.62, 0.705),
                      (-0.40, 0.92), (-0.12, 1.135), (0.15, 1.175), (0.55, 1.15), (0.95, 1.05),
                      (1.45, 0.925), (1.95, 0.905), (2.22, 0.93), (2.32, 0.90)], 0.05)
    # Shoulder (fender tops / beltline at the side)
    P["shoulder_h"] = kf(z, [(-2.30, 0.42), (-2.24, 0.53), (-2.05, 0.66), (-1.75, 0.745), (-1.38, 0.80), (-1.00, 0.775),
                             (-0.55, 0.80), (0.40, 0.84), (0.95, 0.875), (1.32, 0.895), (1.80, 0.88),
                             (2.32, 0.86)], s)
    # Widths
    P["w_max"] = kf(z, [(-2.30, 0.80), (-2.24, 0.90), (-2.08, 0.97), (-1.70, 1.015), (-1.05, 1.015),
                        (-0.75, 0.955), (-0.20, 0.935), (0.55, 0.955), (0.85, 1.02), (1.80, 1.02),
                        (2.15, 0.985), (2.32, 0.90)], s)
    P["w_low"] = P["w_max"] - kf(z, [(-2.3, 0.05), (-0.8, 0.03), (0.4, 0.06), (2.3, 0.04)])
    P["w_sh"] = P["w_max"] - kf(z, [(-2.3, 0.10), (-1.4, 0.06), (-0.6, 0.12), (0.5, 0.15), (1.3, 0.07), (2.3, 0.08)], s)
    # Cabin / greenhouse: roof half width and the belt (glass base) inset
    cabin = kf(z, [(-0.72, 0.0), (-0.45, 1.0), (1.00, 1.0), (1.55, 0.0)], 0.06)
    P["cabin"] = np.clip(cabin, 0, 1)
    P["w_belt"] = kf(z, [(-0.6, 0.80), (0.4, 0.80), (1.2, 0.78)])
    P["w_roof"] = kf(z, [(-0.55, 0.70), (-0.15, 0.585), (0.6, 0.57), (1.2, 0.62)])
    P["roof_edge"] = P["top"] - kf(z, [(-0.6, 0.05), (0.0, 0.06), (1.2, 0.03)])
    return P


def arch_top(z):
    out = np.full_like(z, -1.0)
    for za, r in ((Z_FA, R_F), (Z_RA, R_R)):
        ra = r + ARCH_CLEAR
        d = np.abs(z - za)
        inside = d < ra
        out[inside] = np.maximum(out[inside], r + np.sqrt(ra ** 2 - d[inside] ** 2))
    return out


def catmull(pts, n):
    """Centripetal-free uniform Catmull-Rom through pts, n samples per segment. Returns (points, param)."""
    pts = np.asarray(pts)
    ext = np.vstack([2 * pts[0] - pts[1], pts, 2 * pts[-1] - pts[-2]])
    out, par = [], []
    for i in range(len(pts) - 1):
        p0, p1, p2, p3 = ext[i], ext[i + 1], ext[i + 2], ext[i + 3]
        for k in range(n):
            t = k / n
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
            par.append(i + t)
    out.append(pts[-1]); par.append(len(pts) - 1)
    return np.array(out), np.array(par)


SEG_PER = 6


def half_section(zi, P, i):
    """Right half of the section at station i: list of (x, y) from bottom centre to top centre and param."""
    zb = P["bottom"][i]
    wl, wm, ws = P["w_low"][i], P["w_max"][i], P["w_sh"][i]
    sh = P["shoulder_h"][i]
    top = P["top"][i]
    cab = P["cabin"][i]
    at = arch_top(np.array([zi]))[0]
    y_in = X_WHEEL - TYRE_HALF_W - 0.04
    side_low = zb + 0.10
    # lower part (straight, crisp): bottom centre -> floor edge -> arch -> side
    if at > side_low - 0.02:
        lower = [(0.0, zb), (min(y_in, wl - 0.02), zb), (min(y_in, wl - 0.02), at), (wl, at)]
        side_start_y = at
    else:
        lower = [(0.0, zb), (wl - 0.03, zb), (wl - 0.03, zb + 0.004), (wl, zb + 0.03)]
        side_start_y = zb + 0.03
    # upper part (smooth): side -> max width -> shoulder -> belt/hood -> roof edge -> centre
    y_mid = max(side_start_y + 0.02, 0.38 + 0.3 * (sh - 0.6))
    hood_in_x = ws * 0.62
    hood_in_y = top + 0.035 * (1 - cab) + (sh - top) * 0.22 * (1 - cab)
    belt = (P["w_belt"][i], sh + 0.035)
    roof = (P["w_roof"][i], P["roof_edge"][i])
    p7 = (hood_in_x * (1 - cab) + belt[0] * cab, hood_in_y * (1 - cab) + belt[1] * cab)
    p8 = ((ws * 0.30) * (1 - cab) + roof[0] * cab, (top + 0.006) * (1 - cab) + roof[1] * cab)
    upper = [(wl, side_start_y), (wm, max(y_mid, side_start_y + 0.03)), (ws, sh), p7, p8, (0.0, top)]
    up_pts, up_par = catmull(upper, SEG_PER)
    pts = list(lower[:-1]) + [tuple(p) for p in up_pts]
    par = list(range(len(lower) - 1)) + list(3 + up_par)  # params: 0..2 lower, 3.. upper (3=side start, 4=max, 5=shoulder, 6=p7, 7=p8, 8=centre)
    return pts, par


def build_body(n_stations=150):
    # denser stations near the ends and the arches
    u = np.linspace(0, 1, n_stations)
    z = L_FRONT + (L_REAR - L_FRONT) * (0.5 - 0.5 * np.cos(np.pi * u)) * 0.35 + (L_FRONT + (L_REAR - L_FRONT) * u) * 0.65 - L_FRONT * 0.35 + L_FRONT * 0.35
    z = L_FRONT + (L_REAR - L_FRONT) * (0.35 * (0.5 - 0.5 * np.cos(np.pi * u)) + 0.65 * u)
    P = profiles(z)
    rings, params = [], None
    for i, zi in enumerate(z):
        pts, par = half_section(zi, P, i)
        half = np.array(pts)
        # full ring: right half bottom->top, then left half top->bottom (mirrored), skipping duplicate centre points
        left = half[::-1][1:-1] * np.array([-1, 1])
        ring = np.vstack([half, left])
        rings.append(np.c_[ring[:, 0], ring[:, 1], np.full(len(ring), zi)])
        if params is None:
            params = np.array(par + par[::-1][1:-1])
            side = np.r_[np.ones(len(half)), -np.ones(len(left))]
    rings = np.array(rings)  # (S, R, 3)
    S, R, _ = rings.shape
    pos = rings.reshape(-1, 3)
    tris = grid_tris(S, R, closed_cols=True)
    m = Mesh(pos, tris)
    # orientation: the face at the top centre of the middle station must point up
    mid = S // 2
    j = int(np.argmax(rings[mid, :, 1]))
    tri_idx = 2 * (mid * R + j)
    t = m.tris[tri_idx]
    fn = np.cross(pos[t[1]] - pos[t[0]], pos[t[2]] - pos[t[0]])
    if fn[1] < 0:
        m.tris = m.tris[:, ::-1]
    # end caps (fans), flat
    caps = []
    for k, sgn in ((0, -1), (S - 1, 1)):
        ring = rings[k]
        c = ring.mean(axis=0)
        p = np.vstack([c, ring])
        ct = np.array([(0, 1 + a, 1 + (a + 1) % R) for a in range(R)])
        cm = Mesh(p, ct)
        fn = np.cross(p[ct[0, 1]] - p[ct[0, 0]], p[ct[0, 2]] - p[ct[0, 0]])
        if fn[2] * sgn < 0:
            cm.tris = cm.tris[:, ::-1]
        caps.append(cm)
    info = {"z": z, "params": params, "side": side, "S": S, "R": R, "rings": rings, "P": P}
    return m, caps, info
