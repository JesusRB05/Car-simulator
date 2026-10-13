# Drive scene: keyboard car on a test circuit

The `drive` scene lets you drive a raycast-wheel car around a procedural test
circuit to try the physics engine. There is no general collision detection yet,
so the wheels query an analytic ground instead of colliding with meshes.

```bat
build\windows-msvc-release\bin\sim.exe --scene drive
```

It can also be picked at runtime from **Scenes > Driving > drive** (or the
*Driving* group of the scene browser). A headless
capture (chase camera after one second of settling) works too:

```bat
sim.exe --headless --frames 2 --scene drive --screenshot drive.png
sim.exe --headless --frames 2 --scene drive --debug-draw --screenshot drive_debug.png
```

## Controls

| Key | Action |
| --- | --- |
| W / Up | Throttle (brake while reversing) |
| S / Down | Brake; hold it while stopped to engage reverse, then it is the reverse throttle |
| A / D, Left / Right | Steer (smoothed, speed-dependent lock) |
| Space | Handbrake (rear wheels) |
| R | Reset the car to the start line (also resets the lap timer) |
| C | Cycle camera: chase, cockpit, orbit |
| G | Toggle the debug overlay: suspension rays, contacts, tyre forces (on by default; also the *Debug draw* panel checkbox) |
| Right drag / wheel | Orbit camera yaw/pitch and distance (orbit camera only) |

Holding throttle while stopped in reverse engages first gear again.

### Interaction with the windowed UI

While the drive scene is active the drive keys take priority over the generic
shortcuts: `Space` is the handbrake (not play/pause), `R` resets the car (not
the physics scene), `G` toggles the drive overlay (not the ground grid), and
`.` and `F` do nothing. The Simulation panel shows a note instead of the
play/pause/step/time-scale controls, because the scene runs in real time with
its fixed 1 ms step. `Ctrl+Q`, `F1` and the menus keep working, and keys are
ignored while an ImGui text field has focus. The scene's own camera replaces
the inspection camera until another scene is loaded, which restores it.

## What is simulated

All of it runs in its own `PhysicsWorld` on the main thread with the fixed
1 ms step, in double precision, deterministically (see
`tests/test_drive.cpp`). Code lives in `apps/sim/src/drive/`:

- `Track`: centripetal Catmull-Rom spline through 14 control points (main
  straight, right hairpin, back straight over a hill, fast sweeper, chicane,
  long left), resampled every ~1 m. The terrain is one smooth Gaussian hill
  (5 m high, max grade ~5.5 %), so `height(x, z)` and `normal(x, z)` are exact.
  `sample(x, z)` also returns the surface: asphalt (10 m wide, mu 1.0), kerbs
  (1 m, mu 0.9, only where the corner radius is under 160 m) and grass
  (mu 0.6, higher rolling resistance). Track projection uses a uniform grid.
- `RaycastCar`: a `ForceElement` attached to one chassis `RigidBody`. The world
  calls `apply()` once per step; it raycasts the four suspensions, adds spring,
  damper, bump-stop and anti-roll forces along the ground normal, runs the tyre
  model and integrates the wheel spin, engine and gearbox state with the fixed
  step it was created with.
- `DriveScene`: rendering (terrain grid, asphalt ribbon, kerbs, start line and
  gantry, the car), chase/cockpit/orbit cameras, the ImGui HUD and the debug
  overlay.

## Car parameters (`CarParams`)

| | |
| --- | --- |
| Mass, inertia (pitch/yaw/roll) | 1200 kg; 1900 / 2100 / 500 kg m^2 |
| Wheelbase, track | 2.60 m (CoM 1.25 m behind the front axle), 1.55 m |
| CoM height at rest | ~0.49 m |
| Springs | 35 kN/m, 0.25 m free length, bump stop beyond 0.19 m |
| Dampers | 2.8 kN s/m bump, 3.6 kN s/m rebound |
| Anti-roll bars | 12 kN/m front, 6 kN/m rear |
| Wheels | radius 0.32 m, inertia 1.2 kg m^2 |
| Tyres | combined-slip magic formula (C = 1.5), peak at 12 % slip ratio and 0.15 rad slip angle, friction circle in normalised slip |
| Engine | 250 N m peak at 4500 rpm, 900-7000 rpm, engine braking |
| Gearbox | automatic, 6 gears (3.20 ... 0.80), final drive 3.6, 0.25 s shifts at 6300 / 2600 rpm, reverse 3.3 |
| Drive | rear-wheel drive, open differential, simple traction control |
| Brakes | 3600 N m total, 70 % front; handbrake 2500 N m per rear wheel |
| Steering | 0.60 rad at standstill, limit / (1 + v / 14 m/s), floor 0.10 rad, Ackermann |
| Drag | 0.39 * \|v\| v |

The autopilot test laps the circuit in about 67 s with a 110 km/h top speed;
0-100 km/h takes roughly 6 s.

## HUD and debug overlay

The **Drive** window shows speed, gear, rpm, throttle/brake, steering angle,
surface under the car, lap timer (current, last, best) and a per-wheel table
(load, slip ratio, slip angle, Fx, Fy, compression, surface).

The overlay draws each suspension ray (green in contact, grey in the air), the
contact points (yellow crosses), the tyre force (orange, 1 m per 2.5 kN), the
normal load (blue) and the chassis velocity (white).

## Known limitations

- Only the wheels touch the ground. The chassis has no collision: on its roof
  or side it falls through the terrain (press R).
- The ground is a smooth analytic height function; kerbs are flat paint and
  there are no bumps, walls or other cars.
- Raycast wheels: the contact is a single point under the strut, there is no
  tyre radius or carcass on slopes, and no relaxation length (the slip
  denominators are clamped at 1 m/s instead, so a parked car on a slope creeps
  very slowly even with the brake held).
- Wheel spin is substepped 10x inside the 1 ms step; the chassis sees the mean
  tyre force of the substeps.
- Drivetrain: open differential with equal torque split, simplified clutch
  (slips up to 3000 rpm), no engine inertia, no ABS.
- Steering is a direct wheel angle, no self-aligning torque or force feedback.
- The car will be ported onto the `IVehicleModel` interface later; the
  existing vehicle on the physics thread is unaffected by this scene.
