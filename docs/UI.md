<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
<!-- Copyright (C) 2026 The Car Simulator contributors -->

# Windowed user interface

The interactive app (`sim` without `--headless`) draws its interface with Dear
ImGui and ImPlot. All of it lives in `apps/sim/src/SimUi.{hpp,cpp}` and
`apps/sim/src/OrbitCamera.{hpp,cpp}`; `Application` only feeds it per-frame
state (`SimUiFrame`) and applies the actions it returns (`SimUiActions`). The
headless runner never constructs any of it.

## Main menu bar

| Menu   | Contents                                                              |
|--------|-----------------------------------------------------------------------|
| File   | Quit (`Ctrl+Q`).                                                      |
| View   | Toggles every panel (Scene browser, Simulation, Debug draw, Telemetry, Camera, Vehicle, Mods, Input). |
| Scenes | Driving, plus the *Physics* and *Render tests* submenus.             |
| Help   | Keyboard shortcuts (`F1`) and About (version from `core/Version.hpp`). |

The right end of the bar shows the active scene, whether it is paused, and the
frame rate.

## Panels

- **Scenes**: the scene registry (`sceneCatalog()` in `Scenes.hpp`) grouped in
  *Physics* (the `PhysicsWorld` demos of `PhysicsScenes.cpp`) and *Render tests*
  (the static scenes of `Scenes.cpp`, including the `orientation` harness).
  Hovering shows the description; clicking loads the scene. *Driving* returns to
  the vehicle scene.
- **Simulation** (physics scenes): Play/Pause, Step (exactly one fixed step),
  Reset (rebuild from the initial state, keeping the view), a time-scale slider
  (0.1x to 4x), simulated time, fixed step, measured steps per second, body and
  joint counts, and `PhysicsWorld::stateHash()` (copyable). The time scale only
  changes how much wall time is fed to `PhysicsWorld::advance()`; the step size
  is never changed, so the run stays deterministic and the hash for a given
  number of steps is reproducible.
- **Debug draw**: ground grid, world axes and, for physics scenes, body local
  axes, joints, springs, velocity vectors (0.25 s of motion, capped at 4 m) and
  the analytic reference (projectile parabola). Everything defaults to on. The
  headless captures keep their fixed overlay (`drawPhysicsSceneDebug` without
  options).
- **Telemetry**: ImPlot plots over the last 10 s of wall time: total mechanical
  energy (kinetic + gravitational + the scenes' spring energy), its relative
  drift `(E - E0) / |E0|` since the scene was loaded or reset, and CPU frame /
  GPU (`rhi::GpuTimer`) milliseconds.
- **Camera**: FOV slider (also used by the driving cameras), *Frame scene*,
  and the orbit target, yaw, pitch and distance. Inspection scenes start from
  their curated camera and then orbit around a target on its line of sight.
- **Vehicle**, **Mods**, **Input**: the previous driving, mod and input panels,
  hidden by default.

Panel visibility is stored in ImGui's own `imgui.ini` (relative to the working
directory, ignored by git) under a `[SimUi][Panels]` section; window positions
and sizes use ImGui's normal persistence.

## Keyboard and mouse

| Input            | Action                                                    |
|------------------|-----------------------------------------------------------|
| `Space`          | Play / pause the physics scene (handbrake when driving)   |
| `.`              | Advance one fixed step (pauses first; repeats when held)  |
| `R`              | Reset the physics scene                                   |
| `F`              | Frame the scene                                           |
| `G`              | Toggle the ground grid                                    |
| `C`              | Cycle chase / orbit / free camera (driving scene)         |
| Left drag        | Orbit (inspection scenes)                                 |
| Right/middle drag| Pan (inspection scenes); look around (driving scene)      |
| Mouse wheel      | Zoom                                                      |
| `W A S D`, arrows| Drive; fly with `Q`/`E` in the free camera                |
| `F1`             | Keyboard shortcuts window                                 |
| `Ctrl+Q`         | Quit                                                      |

Mouse input over a panel goes to ImGui, never to the camera. The orbit camera
is right-handed with `+Y` up; yaw 0 and pitch 0 look down `-Z`.
