# Content

## Where content goes

```
content/
  base/                 <- official content (CC BY-SA 4.0)
    mod.toml
    README.txt
    LICENSE.txt
    scripts/main.lua
    tracks/<id>/        <- track descriptor (track.toml) + model/
    cars/<id>/          <- car descriptor (car.toml) + model/
mods/                   <- user mods (scanned at start-up)
  my_mod/
    mod.toml
    ...
```

The engine mounts `content/base` at the **virtual root** (`/`) and the active
mods on top of it. That is why paths are always relative to the VFS root, no
matter where they physically come from:

```lua
sim.read("/README.txt")   -- reads content/base/README.txt
```

## Base content

As far as the system is concerned, `content/base` is the `base` mod: always
present and with the lowest priority. Any mod can **override** its files by
being mounted on top of it.

Current content:

| File | Description |
|---|---|
| `mod.toml` | Base content manifest (id `base`, version `0.1.0`) |
| `README.txt` | Description of the content tree |
| `LICENSE.txt` | CC BY-SA 4.0 notice and reference to the root license |
| `scripts/main.lua` | Entry script; only logs a load message |
| `tracks/sunset_circuit/` | Track descriptor (`track.toml`) and a placeholder model |
| `cars/voyager/` | Car descriptor (`car.toml`) and a placeholder model |

A **track** lives in `tracks/<id>/` with a `track.toml` (control points,
surface widths, terrain) and a `model/` folder; a **car** lives in `cars/<id>/`
with a `car.toml` (chassis, suspension, wheels, engine, brakes, aero) and a
`model/` folder. The demo currently builds both procedurally in
`apps/sim/src/drive/`, and these descriptors record the same parameters so a
future data-driven loader can reproduce them from content; nothing reads them
yet.

Compiled *shaders* (`shaders/*.spv`) are **not** here: the build generates the
SPIR-V next to the executable and the application mounts that directory after
loading the mods, so that mods cannot shadow it.

## Content license

Everything in `content/base` is distributed under **Creative Commons
Attribution-ShareAlike 4.0** (`CC BY-SA 4.0`), while the engine **code** is
under GPL-3.0-or-later with a linking exception. See `LICENSE.txt` in
`content/base` for details. Each mod chooses its own license and declares it in
its `mod.toml`.

## No real brands

Real brands, cars and tracks are deliberately **not** used anywhere in the
content. Any contribution with real names or models will be rejected.

## Adding content

### As a mod (recommended)

Create `mods/my_mod/mod.toml` (see [MODS.md](MODS.md)) and put your files
inside. They are mounted over the base content and can replace its files.

### To the base content

To contribute to the official content, add the files under `content/base/`
respecting the CC BY-SA 4.0 license and update its `mod.toml` if needed. See
[CONTRIBUTING.md](CONTRIBUTING.md).

There is no fixed mesh or texture format yet: while the renderer grows, content
can use an intermediate format (`cgltf` and `ktx` are already available for
glTF and KTX2).
