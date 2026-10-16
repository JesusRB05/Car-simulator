# Drive scene: keyboard car on a test circuit

The `drive` scene drives the assembled [`VehicleModel`](PHYSICS_PLAN.md) (the
default *Voyager* setup) around a procedural test circuit to exercise the whole
vehicle stack. The wheels query the circuit's **collision triangle mesh**
(`TrackCollisionMesh`: the asphalt ribbon, the kerbs and a terrain grid, indexed
by a per-triangle BVH), so the tyres sit on the geometry that is actually drawn;
the chassis is a real rigid body, so it also collides with the barrier walls and
the run-off test yard.

```bat
build\windows-msvc-release\bin\sim.exe --scene drive
```

It can also be picked at runtime from **Scenes > Driving > drive** (or the
*Driving* group of the scene browser). A headless capture works too: by default
it frames the car on the start line with the chase camera after one second of
settling, and `--camera` / `--time` let a script take the other views.

```bat
sim.exe --headless --frames 2 --scene drive --screenshot drive.png
sim.exe --headless --frames 2 --scene drive --debug-draw --screenshot drive_debug.png

rem Other views: --camera chase|cockpit|orbit|side, --yaw/--pitch for orbit.
rem --time drives the car with the built-in autopilot for that many seconds
rem before the capture (0 = parked on the start/finish line).
sim.exe --headless --frames 4 --scene drive --camera cockpit --time 10 --screenshot cockpit.png
sim.exe --headless --frames 4 --scene drive --camera side --time 16 --screenshot side.png
```

`tools\capture_readme_shots.bat` regenerates the README screenshots (grid,
cornering, cockpit and side) with these flags.

## Autopilot

For captures, `--time <seconds>` runs a deterministic pure-pursuit driver: it
aims at a point a speed-dependent distance ahead on the centreline and picks a
target speed from the curvature it is about to meet. It needs no input device,
so a headless run can show the car mid-corner. It is a capture aid, not a racing
AI: it follows the centreline, so it is slower than a human through the corners.

## Controls

| Key | Action |
| --- | --- |
| W / Up | Throttle (brake while reversing) |
| S / Down | Brake; hold it while stopped to engage reverse, then it is the reverse throttle |
| A / D, Left / Right | Steer (smoothed, speed-dependent lock) |
| Space | Handbrake (rear wheels) |
| R | Reset the car to the start line (also resets the lap timer) |
| C | Cycle camera: chase, cockpit, orbit, side |
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
  long left), resampled every ~1 m. The terrain is a sum of four smooth Gaussian
  hills (2.2-5.0 m high, max grade ~6 %), so `height(x, z)` and `normal(x, z)`
  are exact. `sample(x, z)` also returns the surface: asphalt (nominal 10 m wide,
  mu 1.0, widened a little at the start/finish box and narrowed into the
  corners, with the outside edge banked up by up to ~3.5 deg), kerbs (1 m,
  mu 0.9, only where the corner radius is under 160 m) and grass (mu 0.6, higher
  rolling resistance). Track projection uses a uniform grid.
- `Obstacles`: barrier walls lining both sides of the circuit (thin red/white
  armco on the straights, taller concrete blocks in the corners) plus a run-off
  collision yard beside the start straight (concrete blocks with a gap to drive
  through, tyre stacks, a cone slalom, a ramp and a rotated deflector). Every
  obstacle is a static, collision-enabled body and doubles as its own render
  description.
- `VehicleModel`: the assembled car (engine, clutch, gearbox, differential,
  brakes, Ackermann steering, aero and the Magic Formula tyres on one rigid
  chassis in the world; see the [technical plan](PHYSICS_PLAN.md)). The scene
  attaches the driver aids (`ElectronicControlUnit`, ABS/TCS/ESC); the app
  enables them from the session setting every frame (all three on by default).
- `DriveScene`: rendering (terrain grid, asphalt ribbon, kerbs, start line and
  gantry, barrier walls, the car), chase/cockpit/orbit cameras, the ImGui HUD,
  the lap timer and the debug overlay.

## Car parameters

The demo car is the `VehicleSetup` defaults plus `makeDefaultVehicleSetup()`
(`src/vehicle/src/VehicleModel.cpp`): the same car the *driving* scene drives.

| | |
| --- | --- |
| Mass, inertia (pitch/yaw/roll) | 1250 kg; 1900 / 2100 / 500 kg m^2 |
| Wheelbase, track | 2.60 m (front axle z = -1.25 m, rear z = +1.35 m), 1.55 m |
| Springs | 38 kN/m front, 34 kN/m rear, 0.25 m rest length, progressive bump stops |
| Dampers | 2.8 kN s/m bump, 3.6 kN s/m rebound |
| Wheels | radius 0.32 m, inertia 1.2 kg m^2 |
| Tyres | Magic Formula 6.x combined slip (slip-vector ellipse) on the default coefficients |
| Engine | 250 N m peak at 450 rad/s, idle 90 rad/s, rev limit 700 rad/s |
| Gearbox | automatic, 6 gears (3.20 ... 0.80), final drive 3.60, reverse 3.30, ~0.25 s shifts at 650 / 250 rad/s |
| Drive | rear-wheel drive, open differential |
| Brakes | 5800 N m total at full pressure, 70 % front (sized to lock the wheels on dry asphalt); handbrake on the rear |
| Steering | 0.55 rad at standstill, limit / (1 + v / 14 m/s), floor 0.10 rad, Ackermann |
| Drag | 0.5 * rho * 0.30 * 2.1 * v^2 (about 0.39 v^2 at 1.225 kg/m^3) |

The autopilot test laps the circuit in about 69 s with a ~103 km/h top speed,
and a standing 0-100 km/h launch takes roughly 10 s.

## HUD and debug overlay

The **Drive** window shows speed, gear, rpm, throttle/brake, steering angle,
surface under the car, the driver-aid state (ABS/TCS/ESC shown as *off*, *on* or
*active* in green when the aid is intervening, plus the throttle the traction
and stability control let through), the lap timer (current, last, best) and a
per-wheel table (load, slip ratio, slip angle, Fx, Fy, compression, drive
torque, brake torque, the aids' brake-torque adjustment and surface).

The overlay draws each suspension ray (green in contact, grey in the air), the
contact points (yellow crosses), the tyre force (orange, 1 m per 2.5 kN), the
normal load (blue) and the chassis velocity (white).

## Known limitations

- Only the wheels touch the ground: they query the analytic surface, not a
  collider. The chassis box collides with the barrier walls and the test yard,
  but there are no other cars and no ground bumps.
- The wheel contact is a single point under the strut, with no tyre radius or
  carcass on slopes and no relaxation length (the slip denominators are clamped
  at 1 m/s instead).
- The driver aids are attached here for the *Voyager* setup only; the per-car
  aid configuration of [Section 7.8](PHYSICS_PLAN.md) is not read from data yet.
- Steering is a direct wheel angle and the circuit sends no force feedback; the
  app's `input` stack drives the wheel separately.
