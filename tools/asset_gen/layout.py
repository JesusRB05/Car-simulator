# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
import numpy as np
from scipy.interpolate import splprep, splev
CTRL = [(0, 150), (0, -100), (0, -380), (5, -448), (32, -488), (90, -503), (230, -503), (290, -485),
        (325, -440), (370, -412), (430, -420), (480, -455), (540, -478), (600, -462), (640, -410),
        (660, -320), (665, -120), (662, 40), (655, 98), (638, 140), (606, 157), (572, 150), (550, 120), (528, 62),
        (488, 10), (430, -18), (372, -10), (330, 40), (285, 120), (230, 178), (160, 210), (90, 228),
        (40, 222), (10, 196)]
def centerline(step=2.0):
    p = np.array(CTRL, float)
    tck, _ = splprep([p[:, 0], p[:, 1]], s=20.0, per=True, k=3)
    u = np.linspace(0, 1, 20000, endpoint=False)
    x, z = splev(u, tck)
    d = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(z)))]
    L = d[-1] + np.hypot(x[0] - x[-1], z[0] - z[-1])
    n = int(round(L / step))
    s = np.linspace(0, L, n, endpoint=False)
    xs = np.interp(s, d, x); zs = np.interp(s, d, z)
    # make the main straight exactly straight (x = 0) between z = 135 and z = -370
    def smoothstep(e0, e1, v):
        t = np.clip((v - e0) / (e1 - e0), 0, 1); return t * t * (3 - 2 * t)
    w = smoothstep(150, 120, zs) * smoothstep(-395, -360, zs) * (np.abs(xs) < 25)
    xs = xs * (1 - w)
    # resample at constant arc length again
    d = np.r_[0, np.cumsum(np.hypot(np.diff(xs), np.diff(zs)))]
    L = d[-1] + np.hypot(xs[0] - xs[-1], zs[0] - zs[-1])
    n = int(round(L / step))
    s = np.linspace(0, L, n, endpoint=False)
    xs, zs = np.interp(s, d, xs), np.interp(s, d, zs)
    return np.c_[xs, zs], L
if __name__ == "__main__":
    from PIL import Image, ImageDraw
    c, L = centerline()
    print("length", L)
    img = Image.new("RGB", (900, 900), "white"); d = ImageDraw.Draw(img)
    m = lambda x, z: (120 + x * 1.05, 600 + z * 1.05)
    d.line([m(*p) for p in c] + [m(*c[0])], fill="black", width=6)
    for p in CTRL: d.ellipse([m(*p)[0]-3, m(*p)[1]-3, m(*p)[0]+3, m(*p)[1]+3], fill="red")
    d.ellipse([m(0,0)[0]-6, m(0,0)[1]-6, m(0,0)[0]+6, m(0,0)[1]+6], fill="green")
    img.save("/tmp/layout.png")
