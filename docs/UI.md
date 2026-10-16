<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
<!-- Copyright (C) 2026 The Car Simulator contributors -->

# Windowed user interface

The interactive app (`sim` without `--headless`) draws its interface with Dear
ImGui and ImPlot. All of it lives in `apps/sim/src/SimUi.{hpp,cpp}` and
`apps/sim/src/OrbitCamera.{hpp,cpp}`; `Application` only feeds it per-frame
state (`SimUiFrame`) and applies the actions it returns (`SimUiActions`). The
headless runner never constructs any of it.

## Session flow

Before driving, the windowed app shows a short pre-session flow over a 3D
showroom of the selected car. It is a small state machine (`GameScreen` in
`apps/sim/src/session/SessionState.hpp`) and the screens live in
`apps/sim/src/session/SessionUI.{hpp,cpp}`:

| Screen          | What it does                                                            |
|-----------------|-------------------------------------------------------------------------|
| Main menu       | Start session, choose car / track, configure controls, reduced-motion toggle and the driver-aid toggles (ABS/TCS/ESC), quit. Shows the saved car and track. |
| Choose your car | The car list with the derived specs (power, torque, mass, drivetrain, gearbox, wheelbase) and the 3D preview. |
| Choose your track | The circuit list with a 2D minimap drawn from the real centreline, its length and width, plus the game mode, weather and time-of-day. |
| Session         | The recap (car, track, mode, weather, time, controls) and the **Drive** button. |
| Control setup   | The six-step peripheral wizard (see below).                             |
| Driving         | The simulation runs; `Escape` returns to the main menu.                 |

While any menu screen is active the physics is paused and the **garage scene**
(`apps/sim/src/session/GarageScene.{hpp,cpp}`) fills the frame: the selected car,
assembled from the engine primitives on the real `VehicleSetup`, on a dark floor,
with an orbit camera the player drives with the mouse. There is no glTF pipeline
for the cars yet (the catalog bodies are placeholders), so the preview is the
same procedural car the simulation drives.

The catalogs (`CarCatalog`, `TrackCatalog`) are the single source of truth: the
car specs are derived from the physics `VehicleSetup` (never invented), and the
track length and minimap come from the real `Track`. Only one car (*Voyager*)
and one circuit (*Sunset Circuit*) exist today; the flow is built for more.
Game mode (only *Practice*), weather and time of day are listed but **disabled**
and labelled as not simulated, rather than faked. The car, track, mode, weather,
time and the "controls configured" flag persist in `config.toml` under
`session.*` (and `ui.reduced_motion`); the driver aids persist under
`assists.abs`, `assists.tcs` and `assists.esc`.

### Control setup wizard

`apps/sim/src/session/ControlSetupWizard.{hpp,cpp}` walks through the connected
hardware; it only drives `sim::input::WheelManager`, so it works with the same
mocks as the tests. Steps: **Devices** (list, hot-plug, pick the primary),
**Compatibility** (identified model, tier, force-feedback capabilities),
**Steering** (capture left/centre/right, deadzone, invert, lock-to-lock range),
**Pedals** (assign each pedal axis, combined-pedal mode, learned travel),
**Buttons** (assign shift, handbrake and look controls, with a conflict warning
before overwriting another role) and **Test** (live bars and a force-feedback
pulse). Finishing stores every profile and marks the session configured.

## Main menu bar

| Menu   | Contents                                                              |
|--------|-----------------------------------------------------------------------|
| File   | Quit (`Ctrl+Q`).                                                      |
| View   | Toggles every panel (Scene browser, Simulation, Debug draw, Telemetry, Camera, Vehicle, Mods, Input). |
| Scenes | Driving, plus the *Driving*, *Physics* and *Render tests* submenus.  |
| Help   | Keyboard shortcuts (`F1`) and About (version from `core/Version.hpp`). |

The right end of the bar shows the active scene, whether it is paused, and the
frame rate.

## Panels

- **Scenes**: the scene registry (`sceneCatalog()` in `Scenes.hpp`) grouped in
  *Driving* (the keyboard-drivable `drive` circuit, see `docs/DRIVE.md`),
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
- **Vehicle**, **Mods**: the previous driving and mod panels, hidden by default.
  The **Vehicle** panel carries the driver-aid checkboxes (ABS/TCS/ESC, the same
  persisted setting as the main menu) and shows which aid intervened on the last
  step plus the throttle the traction and stability control let through.
- **Input**: the wheel / pedal / gamepad stack (see `docs/INPUT.md`). Hidden by
  default. It lists the connected devices with their identified model and
  verification tier, lets you pick the primary device (or *Auto-select*), remaps
  every control role to an axis, button or hat direction, edits each axis'
  calibration (deadzone, saturation, invert, active), tunes force feedback
  (enable, gains, max force, direction), runs a bounded **Test** and
  **Save profiles**.

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
| `Escape`         | Leave the drive scene back to the main menu               |

In the `drive` scene the keys differ (they take priority, and play/pause/step
are disabled): `W`/Up throttle, `S`/Down brake and reverse, `A D`/Left Right
steer, `Space` handbrake, `R` reset the car, `C` cycle chase / cockpit / orbit
camera, `G` toggle the suspension and tyre force overlay. `Ctrl+Q` and `F1`
still work. Its camera overrides the orbit camera until you leave the scene.
Details in `docs/DRIVE.md`.

Mouse input over a panel goes to ImGui, never to the camera. The orbit camera
is right-handed with `+Y` up; yaw 0 and pitch 0 look down `-Z`.
