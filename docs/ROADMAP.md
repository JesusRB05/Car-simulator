<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
<!-- Copyright (C) 2026 The Car Simulator contributors -->

# Roadmap

This is the plan from the current prototype to a first public **alpha**, and the
direction after it. It is a living document: phases have *exit criteria*, not
dates. Effort sizes are relative (S = days, M = 1-2 weeks, L = several weeks,
XL = more than a month).

## Vision and priorities

A **serious driving simulator**. The priorities, in this order:

1. **Realistic physics.** Physics is the foundation, so it comes first and
   decides when each later phase may start. A pretty simulator with unreliable
   physics is not the goal.
2. **Visual quality** in the range of a modern AAA racing title.
3. **Modding** (content, tuning, scripting).
4. **Later:** career/story mode, online multiplayer and other game modes. They are
   not built in the alpha, but the architecture must not close the door on them
   (see [Designing for the future](#designing-for-the-future)).

Targets: **Windows first**, designed so that **Linux** works without a rewrite.

### Disciplines the physics must support

One engine, with the differences expressed as data and optional modules:
**circuit** (GT, touring), **rally**, **rallycross**, **karting** and **driving mode**
(road cars). The capability matrix, the equations of every model, the validation
manoeuvres and the numbered work packages are in the
[vehicle physics technical plan](PHYSICS_PLAN.md); the phases below summarise it.

## Ground rules (apply to every phase)

### Realism
- Every physical model needs a **validation test with a measurable tolerance**
  (analytic solution, published data or a reference model), and that tolerance is
  never relaxed to hide a regression. The test suite is the definition of "realistic".
- SI units, double precision, fixed step, deterministic stepping (see
  [CONVENTIONS](CONVENTIONS.md)). Model parameters come from data files, not
  from constants in code.
- No behaviour is tuned "until it feels right" without a reference. Feel is tuned
  on top of a physically correct base, through explicit, documented parameters.

### Licensing and content (free licences only)
- Code: **GPL-3.0-or-later** with the project's linking exception, SPDX header in
  every file, DCO sign-off on every commit (already in place).
- Content (models, textures, audio, fonts, tracks, vehicles) must be **original or
  under a free licence**: CC0 preferred, CC-BY acceptable with an attribution file.
  Every asset has a recorded source and licence. No "free for non-commercial" and
  no unclear-provenance assets.
- **Fictional vehicles and brands.** No real manufacturer names, logos or
  trademarked designs. Technical *facts* (masses, dimensions, public tyre data)
  may be used as references.
- Third-party data: terrain and maps only from sources whose licence is compatible
  (public-domain elevation data, open national LiDAR with attribution; OpenStreetMap
  derivatives carry ODbL share-alike obligations and need care).
- Every dependency is audited (a generated licence report from the vcpkg manifest)
  before each release. This roadmap is not legal advice: have the licensing
  reviewed before 1.0.

### Privacy (maximum possible)
- **Repository and releases:** commits use the noreply identity, UTC timestamps and
  no personal paths; CI and release artifacts contain no machine- or user-specific
  data; a hygiene check runs in CI (see Phase 0).
- **Runtime:** **no telemetry, analytics or phone-home**. Anything that leaves the
  machine is opt-in, off by default and documented. Crash dumps stay local. Logs
  never contain usernames or absolute home paths.
- **Future online play:** account-less by default (pseudonymous, locally
  generated identity), minimal data, no third-party SDKs, no voice/chat in the
  first versions, and a privacy statement written before the first network
  feature ships.
- **Releases are signed** and built reproducibly where possible.

### Windows first, Linux-ready
- All OS-specific code lives behind `src/platform/`; Vulkan through volk, windowing
  and input through SDL3, shaders in Slang to SPIR-V. No Win32 calls elsewhere, no
  MSVC-only extensions, `std::filesystem` for paths.
- The **Linux CI jobs stay green at all times** (they already exist). Linux packaging
  (tarball/AppImage) comes after the alpha.
- **Cross-platform determinism** needs explicit compiler settings: MSVC `/fp:precise`
  and, for GCC/Clang, `-ffp-contract=off` (FMA contraction would otherwise change
  results). Determinism hashes are compared between Windows and Linux in CI.
- Known issue: with a non-English MSVC install, Ninja loses header dependencies;
  install the English language pack (see [BUILDING](BUILDING.md)).

## Current state

| Area | State |
|---|---|
| Rigid-body core | Done: fixed step, deterministic, soft and rigid joints with an exact island block solve, 1:1000 mass-ratio tests, energy and angular-momentum tests |
| Collisions | In progress: exact-convex narrowphase for sphere/box/cylinder, a generic GJK/EPA convex path, a heightfield terrain surface, AABB-tree broadphase, contact manifolds solved as one-sided normal rows plus a Coulomb friction cone with restitution, warm started, speculative contacts against tunnelling, sleep-capable and deterministic. Still missing: triangle-mesh soup and a per-primitive (BVH) mid-phase. The raycast-wheel car still uses an analytic ground |
| Vehicle | Interfaces (`IVehicleModel`, tyres, suspension, drivetrain, aero, brakes, electronics), a Magic Formula `MagicFormulaTire` (pure slip, both combined-slip forms, self-aligning, overturning and rolling-resistance moments, camber thrust with its horizontal and vertical shifts, `.tir` scaling factors, `.tir` parser), a `TireVertical` spring-damper, a `TireRelaxation` transient filter, a `TireBristle` standstill model, an implicit `WheelSpin` step and a `DoubleWishbone` hardpoint closure solver (camber, toe, motion ratio), plus a temporary `PlaceholderVehicle` and a prototype `RaycastCar` |
| Rendering | Vulkan RHI, per-face lighting with hemispheric ambient, debug draw, headless capture and golden-image workflow. No shadows, no PBR |
| App | ImGui UI, scene browser, telemetry, orbit camera, keyboard-drivable test circuit |
| Mods | Sandboxed Lua (JIT disabled, instruction budget), mod VFS with priorities, native plugin API skeleton |
| CI | Defined (format, Linux, Windows, gpu-linux); runs on push once the repository has CI enabled |

## Milestones

```
Phase 0 hygiene ─► Phase 1 collisions ─► Phase 2 vehicle dynamics ─► Phase 3 validation + feel
                                                                         │
                                          ──────────── ALPHA 0.1 "Physics" ─┘
                                                                         ▼
                                          Phase 4 world physics ─► Phase 5 visuals ─► Phase 6 UX/immersion
                                                                         │
                                          ──────────── ALPHA 0.2 "Visuals" ─┘
                                                                         ▼
                                                           Phase 7 modding ─► BETA ─► 1.0
```

Only the phases of one row run in sequence; inside a phase, independent work packages
can run in parallel.

### Phase 0 - Foundation and hygiene (S)
- Push to GitHub; get all CI jobs green (Windows, Linux, gpu-linux).
- Format the codebase with the CI-pinned `clang-format` (113 of 116 files fail today).
- Review and approve the golden-image candidates; no skipped tests remain.
- Translate the remaining Spanish docs (`FISICA.md`, `RENDERIZADO.md`) and rename them.
- Compiler flags for cross-platform determinism; Windows-vs-Linux hash comparison in CI.
- CI hygiene checks: no personal data, no `config.toml`/`imgui.ini`, licence headers
  present, dependency licence report generated.
- Repository settings: private commit email, push protection, secret scanning,
  Dependabot, `SECURITY.md` (exists), issue templates, code of conduct.
- **Exit:** CI green on both platforms, zero skipped tests, determinism hashes equal
  across platforms.

### Phase 1 - Collisions and contacts (XL)
The base the vehicle needs: chassis, walls and props collide (tyres use a dedicated
ground interface, not rigid contacts). Work packages P1.1-P1.6 in the
[technical plan](PHYSICS_PLAN.md#3-contacts-and-ground-interface-phase-1).
- Broadphase (BVH/sweep-and-prune) and narrowphase for sphere, box, cylinder, convex
  hull (SAT or GJK/EPA), triangle mesh and heightfield for terrain. Done: sphere,
  box, cylinder (analytic, multi-point), convex hull via GJK/EPA (generic, single
  point) and a heightfield terrain surface (sphere/box contacts). Still brute-force
  broadphase and no triangle-mesh soup.
- Contact manifolds with persistent ids, friction (Coulomb cone), restitution,
  warm starting, speculative contacts or sub-stepping against tunnelling. Done:
  friction, restitution, warm starting and speculative contacts (a per-pair
  search margin grown by the relative speed, so a fast body cannot tunnel
  through thin geometry in one step); sub-stepping is not needed while the
  margin is active.
- Contacts integrated with the island solver (exact solve for bilateral rows,
  iterative for contacts, as documented in [PHYSICS](PHYSICS.md)); sleeping.
- Deterministic contact ordering (stable ids, no pointer-dependent order).
- **Tests:** box stack stays stable for 60 s; sphere bounce matches restitution;
  block-on-ramp matches `tan(theta) = mu` threshold; Newton's cradle conserves
  momentum; two runs and snapshot/restore stay bit-identical. Done: box stack
  (and sleeping), sphere bounce/restitution, friction (sliding sphere at 5/7),
  the block-on-ramp threshold, the Newton's cradle, the heightfield queries/
  contacts and the GJK/EPA convex path.
- **Exit:** the chassis no longer falls through the ground; the render-test scenes
  can be run as physics scenes.

### Phase 2 - Vehicle dynamics, realism core (XL)
The heart of the simulator. Everything implements `IVehicleModel` and its component
interfaces; the prototype `RaycastCar` and `PlaceholderVehicle` are removed. Equations
and work packages P2.1-P2.13 are in the [technical plan](PHYSICS_PLAN.md#4-tyre-model)
(tyre, suspension, chassis and aerodynamics, powertrain and brakes).
- **Tyres:** combined-slip model (Pacejka MF 6.x or equivalent), transient behaviour
  with relaxation length (needed for stability at low speed), camber and load
  dependence, self-aligning torque, rolling resistance, surface friction maps.
  Later in the phase: thermal model, pressure, wear.
- **Suspension:** hardpoint-based kinematics (double wishbone, MacPherson, multilink,
  solid axle), bump stops, anti-roll bars, bushing compliance, measured motion
  ratios, camber/toe/caster curves. Validated on a virtual K&C rig.
- **Chassis:** rigid body with correct inertia tensor, load transfer, aerodynamic
  loads (drag, downforce, ride-height dependence), collision geometry.
- **Powertrain:** engine torque map with rotational inertia and rev limiter, clutch,
  gearbox with realistic shift behaviour, open / limited-slip / locking differentials,
  driveline lash, engine braking; electric motor and hybrid variants later.
- **Brakes:** discs and pads with thermal fade, bias and handbrake; ABS, traction and
  stability control as plug-in `IVehicleElectronics`.
- **Steering:** rack, Ackermann, column torque for force feedback.
- Vehicles are described by **data files** (TOML/JSON) with units and validation.
- **Exit:** two fictional cars (for example a front-wheel-drive hatchback and a
  rear-wheel-drive coupe) defined only by data files behave measurably differently
  and pass the Phase 3 tests.
- **Started:** `MagicFormulaTire` (`vehicle/MagicFormulaTire.hpp`) implements
  `ITireModel` with the Magic Formula 6.x pure-slip curves, load-dependent peaks
  and stiffness, a friction-circle bound, both combined-slip forms (the MF
  weighting functions and the slip-vector ellipse) and the self-aligning torque
  with its residual, plus the overturning and rolling-resistance moments and a
  `.tir` coefficient parser (P2.2, P2.3). `TireRelaxation` filters the slip over a
  rolled distance and `TireBristle` (`vehicle/TireBristle.hpp`) adds the bristle
  standstill model (P2.4). `TireVertical` (`vehicle/TireVertical.hpp`) adds the
  one-sided vertical spring-damper and the effective-radius curve (P2.1 core).
  The wheel spin is integrated implicitly (`vehicle/WheelSpin.hpp`), camber thrust
  and its horizontal and vertical shifts are in, and the tyre carries the `.tir`
  scaling factors (`TireScaling`) that turn a base set into a variant.
  `DoubleWishbone` (`vehicle/DoubleWishbone.hpp`) starts the suspension group
  (P2.5): the upright closure solved by a damped Newton over the five links, with
  camber, toe, the spring motion ratio, the steering-axis alignment (caster,
  kingpin inclination, scrub radius, mechanical trail) and the front-view, side-
  view and roll centres derived from the pose. `SpringDamper`
  (`vehicle/SpringDamper.hpp`) adds the coil spring, the asymmetric damper (linear
  or piecewise-linear table) and the progressive bump and rebound stops,
  `AntiRollBar` (`vehicle/AntiRollBar.hpp`) the roll-stiffness couple and
  `Bushing` (`vehicle/Bushing.hpp`) the per-axis pivot compliance (P2.7), whose
  ride and damping are checked against the analytic quarter-car response, and
  `Steering` (`vehicle/Steering.hpp`) the Ackermann angles, the rack ratio and the
  column torque (P2.8), and `MassProperties` (`vehicle/MassProperties.hpp`) plus
  `Aerodynamics` (`vehicle/Aerodynamics.hpp`) the parallel-axis mass build-up and
  the constant-coefficient aero loads (P2.9). `Engine` (`vehicle/Engine.hpp`) opens
  the powertrain group (P2.10) with the bilinear torque map, the friction
  polynomial and the crankshaft dynamics (throttle lag, rev limiter, idle
  controller, the turbocharger boost model), followed by the `Clutch` (friction
  and centrifugal), the `Gearbox` ratios, torque converter and sequential shift
  controller, the `Differential` family (open, clutch-pack, Torsen, viscous,
  spool) and the `DrivelineCompliance` torsional spring-damper with lash (P2.10,
  P2.11). `Brake` (`vehicle/Brake.hpp`) closes the group with
  the disc torque, the front/rear bias and the thermal model (P2.12). For karts,
  `FrameTorsion` (`vehicle/FrameTorsion.hpp`) is the chassis torsion between the two
  halves (Section 5.6). P2.13 has started: `VehicleModel`
  (`vehicle/VehicleModel.hpp`) assembles every component on one rigid chassis in a
  `PhysicsWorld`, with raycast corners that read the tyre, the spring-damper and the
  `IGroundQuery` surface, and a rear-wheel-drive driveline (engine, clutch, gearbox,
  differential) plus brakes, Ackermann steering and aero; it implements
  `IVehicleModel` and a `makeDefaultVehicleSetup()` describes the first car. The
  `PlaceholderVehicle` and `RaycastCar` prototypes are still present and are removed
  once the app drives the assembled model.

### Phase 3 - Validation, tooling and force feedback (L) - closes ALPHA 0.1
The manoeuvre suite and its oracles (braking, steady-state circle, step steer, skidpad,
ISO 3888-2, jumps, kart and rally cases) are listed in
[PHYSICS_PLAN, Section 10](PHYSICS_PLAN.md#10-validation-plan); work packages P3.1-P3.6.
- **Validation suite in CI** with reference values: braking distance
  (`v^2 / (2 mu g)`), steady-state circular driving (understeer gradient against the
  linear bicycle model), step steer and sine-sweep response, constant-radius
  skidpad, ISO 3888 lane change, acceleration times from the torque curve and
  gearing, load-transfer checks, pitch/roll gradients.
- **Bench tools:** dynamometer, K&C rig, tyre test rig, cornering-stiffness plots.
- Telemetry panel with per-wheel slip, load, forces, temperatures; CSV export;
  deterministic replay from recorded inputs.
- **Force feedback** driven by the tyre's self-aligning torque and steering column
  model; wheel/pedal/gamepad input with calibration (SDL3).
- Performance budget (target): one car steps in under 50 us per 1 ms physics step
  on a laptop CPU, with several AI cars on separate islands.
- **Exit - Alpha 0.1 "Physics":** a person downloads a Windows build, drives a
  validated car on a flat test track with a wheel or gamepad, and the validation
  suite passes in CI. Visuals are still basic.

### Phase 4 - World physics (L)
Surface table, road roughness (ISO 8608), loose-surface layer, wet roads and
aquaplaning, jumps and weather: work packages P4.1-P4.7 in the
[technical plan](PHYSICS_PLAN.md#8-surfaces-and-environment).
- Track format: spline with per-segment surface/friction maps, banking and camber;
  glTF import with a collision mesh; kerbs, rumble strips and bumps as real geometry.
- Surfaces: asphalt, concrete, wet, gravel, grass, snow/ice parameters; dynamic
  grip evolution (rubbering-in, temperature).
- Weather and environment: air density and temperature, wind, rain state affecting
  grip. Damage and deformation are researched here, built later.
- AI drivers good enough to share a track (racing line, braking points).
- **Exit:** a real-scale circuit imported from open data, with surface changes
  that visibly alter lap times.

### Phase 5 - Visual fidelity, AAA-class (XL) - closes ALPHA 0.2
Runs in parallel with Phase 4 once Phase 3 is done.
- Physically based materials (metal-rough, clear coat, carbon, glass, tyre shaders),
  image-based lighting, HDR, tonemapping and bloom.
- Cascaded shadow maps, ambient occlusion, screen-space reflections, planar/probe
  reflections, atmosphere and time of day.
- TAA with optional upscaling (FSR), motion blur, depth of field, lens effects.
- Particles (smoke, dust, sparks), tyre marks, dirt, wet-road shading.
- Asset pipeline: KTX2 textures, meshoptimizer, LODs, instancing, streaming.
- Optional later: ray-traced reflections/shadows.
- Fix: ImGui colours under the sRGB swapchain.
- **Exit - Alpha 0.2 "Visuals":** the reference track and cars use PBR assets with
  shadows and a day/night cycle at 60 fps on a mid-range GPU.

### Phase 6 - Game experience and immersion (L)
- Main menu, car and track selection, settings, lap timing, ghosts, race starts,
  basic race rules.
- Audio: engine sound from RPM/load, tyre noise from slip, wind, collisions,
  3D positioned sound (miniaudio is already integrated).
- Cockpit and helmet cameras, mirrors, triple-screen and VR research.
- Save/replay system with a versioned format.

### Phase 7 - Modding (L) - closes BETA
- Stable, versioned Lua API v1 (events, tuning, scripted tracks), per-frame time and
  memory budgets, permission model.
- Mod formats for vehicles, tracks and skins, manifest validation, signing.
- Native plugin ABI versioning; example mods and documentation; mod browser UI.
- Fuzzing of manifest, glTF and mod loaders; sanitizer builds in CI.
- **Exit - Beta:** third parties can ship a car and a track as mods that load,
  validate and run without modifying the game.

### After 1.0 (not scheduled)
- **Career / story mode:** progression, team and sponsor systems, events,
  campaign scripting on top of the mod API.
- **Online multiplayer:** server-authoritative simulation with client prediction and
  snapshot interpolation (determinism is kept for replays and tests, not relied on
  for network sync), dedicated server for Linux, anti-cheat basics.
- Damage and deformation, endurance racing (fuel, tyre wear, day/night), driving
  schools, track editor, VR.

## Designing for the future

Constraints to respect from now on so that story and online modes stay possible:

- **No global state** in physics or vehicles: several simulation instances must be
  able to coexist (dedicated server, replays, AI).
- **Serializable state:** snapshot/restore exists; keep it complete and versioned.
- **Input as commands:** the simulation consumes recorded input frames, never reads
  devices directly.
- **Stable content ids** and versioned data formats from the first release.
- **Deterministic randomness** (seeded generators owned by the world).
- **Headless first:** every system must run without a window or GPU (already true for
  the physics core and the capture runner).
- The `net` module stays a stub until Phase 7+, but no code should assume a single
  local player.

## Risks

| Risk | Mitigation |
|---|---|
| Contacts break determinism or stability | Stable ids and ordering, determinism test with contacts in CI from day one |
| Tyre model is hard to validate | Reference datasets, analytic limits, validation rig before any tuning |
| Scope: realism, visuals and tools at once | Strict phase gates; visuals do not start before the alpha 0.1 exit criteria |
| Original content workload | Few vehicles and tracks at first, procedural and CC0 sources, every asset licensed |
| Windows-only testing hides Linux bugs | Linux CI green at all times, cross-platform hash check |
| Single-maintainer capacity | Small, independent work packages that can be developed in parallel |
