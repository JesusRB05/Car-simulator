<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# Asset generators

Procedural generators for the base content. Everything (geometry and textures)
is created by code, so the assets carry no third-party licence issues and anyone
can tweak and rebuild them.

Requirements: Python 3.10+, `numpy`, `scipy`, `Pillow`. The preview renderer also
needs `playwright` with Chromium.

```bash
# car (scheme a/b/c, race number)
python3 tools/asset_gen/car.py content/base/cars/halvard_gt3/model/car.glb a 27
# paint template
python3 tools/asset_gen/template.py content/base/cars/halvard_gt3/model/livery_template.png
# circuit
python3 tools/asset_gen/track.py content/base/tracks/sierra_verde/model/track.glb content/base/tracks/sierra_verde/track.json
# preview images (three.js in headless Chromium)
python3 tools/asset_gen/render.py car.glb out/preview "front:floor=1&cam=5,1.4,-4.6&target=0,0.55,-0.2"
```

| File | Purpose |
|---|---|
| `gltfw.py` | minimal glTF 2.0 (.glb) writer and mesh helpers |
| `car_body.py` | body shell as a loft of cross sections (wheel arches, cabin, fenders) |
| `car.py` | full car: body regions, aero, wheels, interior, details |
| `livery.py` | livery atlas painter (3 schemes, invented sponsors) |
| `template.py` | paint template for skin makers |
| `textures.py` | tileable PBR textures (asphalt, grass, gravel, kerb, concrete...) |
| `layout.py` | circuit centreline |
| `track.py` | circuit geometry, scenery and `track.json` |
| `render.py`, `view/` | preview renderer (three.js r170, MIT, in `view/lib`) |

Fonts used for textures: DejaVu Sans (Bitstream Vera licence, free to embed in
rendered images).
