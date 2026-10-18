# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""Paint template: the livery atlas with the body's UV wireframe and chart labels."""
import sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import car
import livery

paint, glass, trim, info = car.build_body_parts()
S = livery.SIZE
img = Image.new("RGB", (S, S), (255, 255, 255))
d = ImageDraw.Draw(img)
uv = paint.uv * S
for tri in paint.tris:
    pts = [tuple(uv[i]) for i in tri]
    d.polygon(pts, outline=(205, 210, 220))
f = ImageFont.truetype(livery.FONT_BOLD, 40)
labels = {"right": "RIGHT SIDE (front at right)", "left": "LEFT SIDE (front at left)", "top": "TOP (front at left, right side up)",
          "front": "FRONT", "rear": "REAR"}
for name, (u0, v0, u1, v1) in livery.CHARTS.items():
    d.rectangle([u0 * S, v0 * S, u1 * S - 1, v1 * S - 1], outline=(220, 40, 40), width=4)
    d.text((u0 * S + 16, v0 * S + 12), labels[name], font=f, fill=(220, 40, 40))
img.save(sys.argv[1] if len(sys.argv) > 1 else "/tmp/livery_template.png")
print("ok")
