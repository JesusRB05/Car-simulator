<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
# Extra liveries for the Halvard GT3

| Skin | Number | Colours |
|---|---|---|
| `red_black_8` | 8 | black base, red lower body, yellow accents |
| `green_44` | 44 | green base, dark green lower body, white accents |

Each folder has `livery.png` (base colour atlas) and `livery_emissive.png`
(lights mask), with the atlas layout described in
`content/base/cars/halvard_gt3/README.md`.

`livery.toml` follows `docs/MOD_FORMAT.md` section 5.3 and replaces two maps of
the car's `paint_livery` material:

```toml
[[textures]]
slot = "paint_livery.base_color"
file = "livery.png"

[[textures]]
slot = "paint_livery.emissive"
file = "livery_emissive.png"
```

The game lists the car's skins in the garage; a new livery is one folder with a
descriptor and its atlases, and it works with any mod's copy of the car.
