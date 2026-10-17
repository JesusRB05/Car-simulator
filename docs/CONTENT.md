# Content

## Where content goes

```
content/
  base/                 <- official content (CC BY-SA 4.0), the mod `base.core`
    mod.toml
    README.txt
    LICENSE.txt
    apps/main.lua
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

As far as the system is concerned, `content/base` is the mod `base.core`:
always present and with the lowest priority. Any mod can **override** its files
by being mounted on top of it.

Current content:

| File | Description |
|---|---|
| `mod.toml` | Base content manifest (format 2, id `base.core`, version `0.1.0`) |
| `README.txt` | Description of the content tree |
| `LICENSE.txt` | CC BY-SA 4.0 notice and reference to the root license |
| `apps/main.lua` | Entry application; only logs a load message |
| `tracks/sunset_circuit/` | Track descriptor (`track.toml`) and a placeholder model |
| `cars/voyager/` | Car descriptor (`car.toml`) and a placeholder model |

A **track** lives in `tracks/<id>/` with a `track.toml` (control points,
surface widths, terrain) and a `model/` folder; a **car** lives in `cars/<id>/`
with a `car.toml` (chassis, suspension, wheels, engine, brakes, aero) and a
`model/` folder.

Both descriptors are **real**: at start-up the game scans the VFS for
`cars/<id>/car.toml` and `tracks/<id>/track.toml`, validates every value and
builds the physics from them (see [Data-driven cars and tracks](#data-driven-cars-and-tracks)).
Mods are mounted on top of the base content, so a mod can add a car or a track
with nothing but a descriptor, or override an official one. There is **no
compiled-in car or circuit**: official content is data (`base.core/voyager`,
`base.core/sunset_circuit`) like any other.

Compiled *shaders* (`shaders/*.spv`) are **not** here: the build generates the
SPIR-V next to the executable and the application mounts that directory after
loading the mods, so that mods cannot shadow it.

## Data-driven cars and tracks

The loaders (`apps/sim/src/session/CarLoader.cpp` and `TrackLoader.cpp`) read
every descriptor through the VFS, so official content and enabled mods are
treated the same. A descriptor that fails validation is **skipped with a clear
error** (naming the file, the line, the key, the value and the allowed range);
the other cars and tracks still load. Each car and circuit is identified by its
namespaced content reference `<mod-id>/<content-id>` (for example
`base.core/voyager`), never by a bare folder name, so two mods may ship a car
called `kart` without colliding.

All values are **SI** (metres, kilograms, radians, seconds); rotational speeds
are in **rad/s**, not rpm. Every number is range-checked and cross-checked
(e.g. the front axle must be ahead of the rear axle, the gear ratios must
decrease, `rev_limit` must exceed `idle_speed`).

### `car.toml`

Metadata keys: `id` (required, lower-case slug), `name`, `author`, `license`,
`category`, `description`, `color` (3–4 numbers in `[0, 1]`). The tables below
map onto `vehicle::VehicleSetup`; any key that is omitted keeps the default car's
value, so a descriptor only states what it changes.

| Table | Keys |
|---|---|
| `[chassis]` | `mass`, `inertia` (3), `half_extents` (3) |
| `[geometry]` | `front_axle_z`, `rear_axle_z`, `half_track`, `mount_y`, `rest_length` |
| `[suspension]` | `spring_stiffness_front`, `spring_stiffness_rear`, `damper_bump`, `damper_rebound`, `bump_stop_length`, `bump_stop_stiffness`, `bump_stop_damping`, `bump_stop_exponent` |
| `[wheels]` | `radius`, `inertia` |
| `[tyres]` | `peak_friction_longitudinal`, `peak_friction_lateral`, `nominal_load` |
| `[engine]` | `idle_speed`, `rev_limit`, `torque_curve_speed`, `torque_curve_torque`, `friction_constant`, `friction_linear` |
| `[drivetrain]` | `gear_ratios`, `reverse_ratio`, `final_drive`, `efficiency`, `upshift_speed`, `downshift_speed` |
| `[brakes]` | `max_brake_pressure`, `brake_bias`, `handbrake_pressure`, `pad_friction`, `piston_area`, `effective_radius` |
| `[steering]` | `max_steer_angle`, `min_steer_angle`, `steer_speed_scale`, `steer_input_rate`, `steer_return_rate` |
| `[aero]` | `drag_coefficient`, `lift_coefficient`, `frontal_area`, `centre_of_pressure` (3), `air_density` |

The torque curve is two equal-length arrays: `torque_curve_speed` (rad/s,
strictly increasing) and `torque_curve_torque` (N·m at wide-open throttle).
`content/base/cars/voyager/car.toml` reproduces `makeDefaultVehicleSetup()`
exactly, and `mods/example_mod/cars/hatchback/car.toml` is a second, data-only
car. (The vehicle model is rear-wheel drive only, so every car is RWD.)

### `track.toml`

Metadata keys: `id` (required), `name`, `author`, `license`, `location`,
`description`. Then:

| Table | Keys |
|---|---|
| `[geometry]` | `loop` (must be `true`), `sample_spacing`, `half_width`, `kerb_width`, `start_z`, `control_points` (array of `[x, z]`, at least three) |
| `[[terrain.hills]]` | `x`, `z`, `height`, `sigma` (one table per Gaussian hill) |

The centreline is a closed centripetal Catmull-Rom spline through the control
points; `content/base/tracks/sunset_circuit/track.toml` reproduces
`drive::defaultTrackDescription()` exactly.

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
inside. They are mounted over the base content and can replace its files. To add
a car or a track, drop a descriptor in `mods/my_mod/cars/<id>/car.toml` or
`mods/my_mod/tracks/<id>/track.toml`: it is discovered automatically and appears
in the garage / track list next to the official ones. A data-only car needs no
code and no mesh — see `mods/example_mod/cars/hatchback/car.toml`.

### To the base content

To contribute to the official content, add the files under `content/base/`
respecting the CC BY-SA 4.0 license and update its `mod.toml` if needed. See
[CONTRIBUTING.md](CONTRIBUTING.md).

There is no fixed mesh or texture format yet: while the renderer grows, content
can use an intermediate format (`cgltf` and `ktx` are already available for
glTF and KTX2).
