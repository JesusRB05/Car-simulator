# Mod format reference

Complete reference for the mod package format (v2). For the *why* and the
around-it design (installer, catalog, manager, security) see
[MODS.md](MODS.md). Everything here is normative for both the engine and the
creator tools (`sim mod ...`, M9).

All text is UTF-8. All numeric values are SI: metres, kilograms, seconds,
radians; engine and wheel speeds are rad/s, never rpm. See
[CONVENTIONS.md](CONVENTIONS.md).

## 1. Package

A mod is **one file** with the extension `.simmod` — a ZIP container whose
contents are the tree below. The same tree, as a plain folder, is a valid mod
while a creator is working on it (developer mode, M9).

```
<mod>/
  mod.toml                      required manifest (v2)
  cover.png                     optional cover image (referenced by `cover`)
  cars/<car>/car.toml           a car
  tracks/<track>/track.toml     a circuit
  tracks/<track>/layouts/<l>/   optional alternative layouts of that circuit
  skins/<car-mod>/<car>/<skin>/livery.toml   a skin for a car (any mod's car)
  parts/<car-mod>/<car>/<slot>/<part>.toml   a part for a slot of a car
  tyres/<tyre>.toml             a shared tyre compound
  sounds/                       audio files
  setups/<car-mod>/<car>/<setup>.toml        a saved setup preset
  weather/<preset>.toml         a weather preset
  apps/<app>.lua + apps/<app>/  a HUD or in-game app (Lua)
  modes/<mode>.toml + <mode>.lua  a game mode
  visuals/                      visual presets and shaders
  locale/<lang>.toml            translations
  input/<profile>.toml          wheel/pad input profiles
  plugins/                      native plugins (see M5)
  patches/*.toml                data patches against other content (M3)
```

The `.simmod` extension, the manifest name and the folder names live in one
place in the code (`src/modding/.../Package.hpp`) so the extension can be
renamed without touching anything else.

A mod may contain **several kinds of content at once** (a car *and* skins *and*
a HUD app). The set is described by `types` (section 3).

## 2. Identifiers

- **Mod id** — `<namespace>.<name>`, lowercase. Each part matches
  `[a-z][a-z0-9-]*`; there must be at least one `.`. Examples: `base.core`,
  `ana.karts`, `liga-2026.round1`.
- **Content id** — the folder name inside a package: `[a-z][a-z0-9_-]*`
  (for example `voyager`, `sunset_circuit`).
- **Content reference** — `<mod-id>/<content-id>`, for example
  `base.core/voyager`. **Everything the engine lists or references uses the
  full reference**, never a bare name, so two mods can never collide.

## 3. `mod.toml` (v2)

```toml
format = 2

id = "ana.karts"
name = "Ana's Karts"
version = "1.2.0"
authors = ["Ana Pérez <ana@example.org>"]
license = "CC-BY-SA-4.0"
description = "A light kart and two skins."
cover = "cover.png"
min_game_version = "0.1.0"

types = ["car", "skins"]

authors_note = "optional free text"

dependencies = [
    { id = "base.core", version = "^0.1.0" },
    { id = "tyres.race", version = ">=1.2.0 <2.0.0" },
]

conflicts = [
    { id = "other.karts", version = "*", reason = "overrides the kart slot" },
]

permissions = ["hud", "vehicle.tuning"]

overrides = ["base.core/tracks/sunset_circuit/model/track.gltf"]

[urls]
website = "https://example.org/karts"
source = "https://github.com/ana/karts"
```

### Fields

| Field | Type | Required | Notes |
|---|---|---|---|
| `format` | integer | yes for new mods | must be `2`; a missing value means a v1 manifest (section 4) |
| `id` | string | yes | mod id (section 2) |
| `name` | string | yes | display name |
| `version` | string | yes | semver 2.0.0 (`major.minor.patch[-pre][+build]`) |
| `authors` | array of strings | yes | free text, usually `Name <email>` |
| `license` | string | yes | an [SPDX](https://spdx.org/licenses/) identifier |
| `description` | string | no | one or two lines |
| `cover` | string | no | path inside the package to a PNG/JPEG |
| `min_game_version` | string | no | semver or `"*"` |
| `types` | array of strings | yes | subset of `car`, `track`, `skins`, `parts`, `tyres`, `sounds`, `setups`, `weather`, `apps`, `modes`, `visuals`, `locale`, `input`, `plugins`, `patches`, `collection` |
| `dependencies` | array of tables | no | `{ id, version }`; `version` is a range (section 6) |
| `conflicts` | array of tables | no | `{ id, version, reason }`; see M3 |
| `overrides` | array of strings | no | full-file overrides of another mod's content (M3); each entry is a path inside *this* package |
| `permissions` | array of strings | no | script permissions (M5); required when `types` contains `apps` or `modes` |
| `urls` | table | no | `website`, `source` |
| `entry_script` | string | no | **legacy (v1)**; new mods use `apps/` or `modes/` |
| `priority` | integer | no | **legacy (v1)**; new mods use the load order / profiles (M6) |

A collection (modpack/championship) uses `types = ["collection"]` and adds a
`collection` table (section 9).

### 4. v1 migration

A `mod.toml` without `format` is a v1 manifest. It is **migrated in memory**
with a warning in the log and the mod manager; nothing is written back to disk.
The mapping:

- `author` (string) → `authors = [author]`.
- `dependencies = ["lib >=1.2.0"]` (strings) → `[{ id = "lib", version = ">=1.2.0" }]`.
- `entry_script` → an `apps` entry (the file is run as before).
- `priority` → kept, but reported as deprecated; the effective order is the
  v2 load order.
- `id` — kept verbatim. If it has no `.` (a v1 id), the mod still loads but the
  manager warns that new mods must use `author.name`, and the id is treated as
  its own namespace for content references (`<old-id>/<content>`).
- `format` is set to `2` and `types` is inferred from the folders present.

The migration is pure and covered by tests; it never mutates the source text.

## 5. Content descriptors

### 5.1 `cars/<car>/car.toml`

Distances in metres, masses in kg, moments in kg·m², angles in radians, engine
and wheel speeds in rad/s. Ranges are validated (M2) and a value outside a
plausible range is an error naming the file, key, value and allowed range.

```toml
id = "voyager"                  # content id (folder name must match)
name = "Voyager"
authors = ["Car Simulator contributors"]
license = "CC-BY-SA-4.0"
category = "Sports car / RWD"
description = "..."
color = [0.10, 0.35, 0.80]      # linear RGB, 0..1
model = "model/body.gltf"       # optional

[chassis]
mass = 1250.0
inertia = [1900.0, 2100.0, 500.0]   # pitch (X), yaw (Y), roll (Z)
half_extents = [0.88, 0.35, 2.15]

[geometry]
front_axle_z = -1.25
rear_axle_z = 1.35
half_track = 0.775
mount_y = 0.0
rest_length = 0.25

[suspension]
spring_stiffness_front = 38000.0
spring_stiffness_rear = 34000.0
damper_bump = 2800.0
damper_rebound = 3600.0
bump_stop_length = 0.06
bump_stop_stiffness = 200000.0
bump_stop_damping = 800.0
bump_stop_exponent = 2.0

[wheels]
radius = 0.32
inertia = 1.2

[tyres]
compound = "base.core/street"   # optional shared tyre; inline values otherwise
peak_friction_longitudinal = 1.4
peak_friction_lateral = 1.4
nominal_load = 4000.0

[engine]
idle_speed = 90.0
rev_limit = 700.0
torque_curve_speed = [100.0, 200.0, 300.0, 450.0, 600.0, 700.0]
torque_curve_torque = [170.0, 215.0, 240.0, 250.0, 230.0, 190.0]
friction_constant = 10.0
friction_linear = 0.05

[drivetrain]
gear_ratios = [3.20, 2.15, 1.55, 1.20, 0.97, 0.80]
reverse_ratio = 3.30
final_drive = 3.60
efficiency = 0.92
upshift_speed = 650.0
downshift_speed = 250.0
drive = "rwd"                   # "rwd" | "fwd" | "awd"

[brakes]
max_brake_pressure = 1.40e7
brake_bias = 0.70
handbrake_pressure = 3.0e6
pad_friction = 0.40
piston_area = 0.002
effective_radius = 0.13

[steering]
max_steer_angle = 0.55
min_steer_angle = 0.10
steer_speed_scale = 14.0
steer_input_rate = 3.0
steer_return_rate = 5.0

[aero]
drag_coefficient = 0.30
lift_coefficient = 0.0
frontal_area = 2.1
centre_of_pressure = [0.0, 0.4, 0.0]
air_density = 1.225

# Slots and their default part (M3). A part may be inline or a reference to a
# part of another mod.
[slots.wheel]
name = "Wheels"
default = "base.core/voyager/wheel/stock"

[slots.rear_wing]
name = "Rear wing"
default = ""                    # empty means "none"
```

Validation: `id` equals the folder name; `gear_ratios` is non-empty and
strictly decreasing; `torque_curve_speed` is strictly increasing and the same
length as `torque_curve_torque`; `rev_limit > idle_speed`; `mass > 0`;
`half_track`, `half_extents`, wheel `radius`, `inertia` all positive;
`brake_bias` in `[0, 1]`; every value inside the physical ranges listed in
`docs/CONTENT.md` (extended there as the schema grows).

### 5.2 `tracks/<track>/track.toml`

```toml
id = "sunset_circuit"
name = "Sunset Circuit"
authors = ["Car Simulator contributors"]
license = "CC-BY-SA-4.0"
location = "Procedural test loop"
description = "..."

[geometry]
loop = true
sample_spacing = 1.0
half_width = 5.0
kerb_width = 1.0
start_z = 40.0
control_points = [ [0.0, 120.0], [0.0, 20.0], ... ]

[[terrain.hills]]
x = 90.0
z = -60.0
height = 5.0
sigma = 55.0

[model]
mesh = "model/track.gltf"       # optional

[start]
position = [0.0, 40.0]          # optional; defaults to the start line
heading = 0.0                   # radians, 0 faces -Z
```

Alternative layouts live in `tracks/<track>/layouts/<layout>/track.toml` with
the same schema. A layout is addressed as `<mod>/<track>:<layout>`
(for example `base.core/sunset_circuit:short`).

### 5.3 `skins/<car-mod>/<car>/<skin>/livery.toml`

A skin attaches itself to a car of **any** mod; it never copies the car.

```toml
id = "rojo"
name = "Rojo"
authors = ["Another Author"]
license = "CC-BY-SA-4.0"
car = "base.core/voyager"        # the car this skin is for
body_color = [0.75, 0.10, 0.10]
description = "Matte red with white stripes."

[[textures]]
slot = "body"                    # texture slot the car exposes
file = "body.png"
```

Skins are cosmetic and never take part in the physics fingerprint (M8). A skin
whose `car` cannot be resolved is reported as a non-fatal error in the manager
("needs car X, which is not installed").

### 5.4 `parts/<car-mod>/<car>/<slot>/<part>.toml`

A part fills a slot of a car of any mod. Parts are additive; adding one never
touches the car's own files.

```toml
id = "gt_wing"
name = "GT rear wing"
authors = ["Another Author"]
license = "CC-BY-SA-4.0"
slot = "rear_wing"
downforce_scale = 1.6
drag_delta = 0.05
mass_delta = 4.0
```

The keys a part may set are the keys of the slot's declared parameters, which
the car lists under `[slots.<slot>]`. Unknown keys are an error.

### 5.5 `setups/<car-mod>/<car>/<setup>.toml`

A named configuration: one part per slot plus numeric overrides.

```toml
id = "qualifying"
name = "Qualifying"
car = "base.core/voyager"

[parts]
rear_wing = "ana.aero/voyager/rear_wing/gt_wing"
tyre = "base.core/voyager/tyre/slick"

[values]
suspension.spring_stiffness_front = 42000.0
```

### 5.6 `patches/*.toml` (M3)

```toml
target = "base.core/voyager"     # required; a full content reference

[[patch]]
path = "suspension.spring_stiffness_front"
op = "set"                       # set | add | multiply | remove
value = 45000.0

[[patch]]
path = "aero.lift_coefficient"
op = "multiply"
value = 1.2
```

`path` is a dotted key *inside the target descriptor*. `set`/`add`/`multiply`
require `value`; `remove` deletes the key. A patch without `target`, or whose
target does not resolve, is an error. Two mods patching the same `(target,
path)` are a conflict (M3).

### 5.7 `tyres/<tyre>.toml`, `weather/<preset>.toml`, `locale/<lang>.toml`, `input/<profile>.toml`

```toml
# tyres/<id>.toml
id = "street"
name = "Street"
peak_friction_longitudinal = 1.4
peak_friction_lateral = 1.4
nominal_load = 4000.0
```

```toml
# weather/<id>.toml
id = "clear"
name = "Clear"
friction_scale = 1.0
wind_speed = 0.0
rain = 0.0
```

```toml
# locale/<lang>.toml — lang is an ISO-639-1 code or code_REGION
[ui]
"garage.title" = "Garage"
```

```toml
# input/<id>.toml — a wheel/pad mapping (see INPUT.md)
id = "g29"
name = "Logitech G29"
```

### 5.8 `apps/`, `modes/` (Lua)

An app is `apps/<app>.lua` (plus an optional `apps/<app>/` folder for assets);
a mode is `modes/<mode>.toml` (metadata) plus `modes/<mode>.lua`. Both run in
the mod sandbox and may use only the permissions declared in `mod.toml`
(section 7 of MODS.md).

## 6. Version ranges

Ranges use the syntax in `modding/Version.hpp` (all comparators ANDed,
`||` for alternatives): `*`, `1.2.3`, `=1.2.3`, `1.2`, `1`, `>=`, `>`, `<=`,
`<`, `^1.2.3` and `~1.2.3`. Build metadata is ignored in comparisons.

## 7. Load order and composition

The effective content is the base content plus the active mods, applied in
order:

```
base.core
  -> active mods in load-order position (0 = first, N = last)
```

For each file the **last** mount wins (as in the current VFS). On top of that:

- **Patches** are applied to the merged descriptor in load order (later patches
  see earlier ones).
- **Skins, parts, setups, tyres, weather, locales, apps, modes** are additive
  and merged by id; a mod never overwrites another mod's skin of a different
  name.
- A **full-file override** only happens when the overriding mod lists the path
  under `overrides`; it is flagged in the manager as a hard override.

## 8. Content index (M7)

A *source* is an HTTPS URL to a static index file (TOML or JSON). The index is
an open, plain list — no accounts, no queries, only GET.

```toml
# mod-index format (tools/mod-index/README.md)
format = 1

[[mod]]
id = "ana.karts"
name = "Ana's Karts"
version = "1.2.0"
authors = ["Ana Pérez"]
license = "CC-BY-SA-4.0"
types = ["car", "skins"]
description = "..."
url = "https://…/ana-karts-1.2.0.simmod"
sha256 = "…64 hex…"
size = 4823910
cover = "https://…/karts-cover.png"
min_game_version = "0.1.0"

[[mod.dependencies]]
id = "base.core"
version = "^0.1.0"
```

Every `url` must be HTTPS; every download is verified against `sha256` before
installation. Multiple sources may be configured; an entry is keyed by
`(id, version)`.

## 9. Collections (modpacks and championships) — M6

```toml
format = 2
id = "liga-2026.round1"
name = "Liga 2026 — Round 1"
version = "1.0.0"
authors = ["Liga 2026"]
license = "CC-BY-SA-4.0"
types = ["collection"]

[[collection.mods]]
id = "ana.karts"
version = "=1.2.0"

[[collection.mods]]
id = "base.core"
version = "^0.1.0"

[collection.session]
track = "ana.tracks/cart_circuit"
cars = ["ana.karts/kart"]
rules = "free-practice"
```

Installing a collection installs every listed mod at the requested version.

## 10. Profiles (M6)

A profile is a small file listing active mods and their order; it is exported
and imported to share a league setup.

```toml
format = 1
name = "League round 1"
order = ["base.core", "ana.karts", "liga-2026.round1"]
```

## 11. Required-content message (M8)

The host and the client exchange a `RequiredContent` list. Each entry is one
active mod that affects the simulation:

```
{ id = "ana.karts", version = "1.2.0", hash = "<sha256>", url = "https://…" }
```

- The `hash` is computed over the mod's contributing files (cars, tracks,
  physics-affecting patches, part definitions) in a deterministic order.
- **Skins are excluded**: they never require a match.
- Comparing the local content with the required list yields three states per
  entry: `missing`, `version-mismatch`, `hash-mismatch` (plus `ok`), with a
  one-click download+install for the first two.

## 12. Validation and error messages

`sim mod validate <path>` (M9) and the loaders share one validation core. A
message names the source and the exact location:

```
cars/voyager/car.toml: line 23, key 'chassis.mass': value -1250.0 is outside
the allowed range [1.0, 20000.0] (expected a positive mass in kg)
```

Hard errors (a malformed manifest, an out-of-range value, a missing
dependency) stop *that* mod from loading; the others keep loading. A mod is
never able to abort start-up. Warnings (a deprecated v1 field, a declared
override) are reported but do not stop loading.

## 13. JSON schemas

Editor autocompletion schemas live in `docs/schemas/`:
`mod.schema.json`, `car.schema.json`, `track.schema.json`, `livery.schema.json`,
`part.schema.json`, `setup.schema.json`, `patch.schema.json`,
`collection.schema.json`, `profile.schema.json`, `index.schema.json`.
They are generated/checked in CI and referenced from the format docs so Taplo
and *Even Better TOML* offer completion and inline validation.
