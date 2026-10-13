# Physics

The physics engine is our own. It lives in `src/physics` (rigid bodies, force
elements and joints) and `src/vehicle` (the vehicle). It works in **SI units**
and **double precision** for world positions, with a fixed step and no
third-party dependencies ([`DECISIONS.md`](DECISIONS.md)).

> The engine solves rigid-body dynamics, springs and joints, and now
> **collisions and contacts** as well: shape-vs-shape detection plus one-sided
> normal rows with a Coulomb friction cone and restitution, integrated with the
> island solver (Phase 1 in [`ROADMAP.md`](ROADMAP.md)). Terrain shapes, tyres
> and the full vehicle model come later.

## Base types (`physics/PhysicsTypes.hpp`)

```cpp
using Real = double;      // physics engine scalar
using Vec3 = glm::dvec3;
using Quat = glm::dquat;
using Mat3 = glm::dmat3;
```

`double` is used because world coordinates can grow large and error in `float`
accumulates visibly within a simulation.

## Maths (`physics/Math.hpp`)

Pure, stateless header, double precision. It includes:

- `skew(v)` (antisymmetric matrix, `skew(v) * w == cross(v, w)`).
- Quaternions: `quaternionMultiply`, `quaternionInverse`,
  `quaternionNormalize`, `rotateVector`, `quaternionFromAxisAngle`.
- Exponential map `quaternionFromRotationVector(v)` and its inverse
  `quaternionRotationVector(q)`. The exponential is exact for constant angular
  velocity over the interval, which is what keeps fast wheels accurate.
- `perpendicularBasis`, `safeNormalize`, `isFinite`, `Transform`.

## Bodies (`physics/RigidBody.hpp`)

`BodyType` is `Static`, `Kinematic` or `Dynamic`. Only dynamic bodies respond to
forces; static bodies have zero inverse mass and inverse inertia.

Shapes (`ShapeType`): `Box` (`halfExtents`), `Sphere` (`radius`) and `Cylinder`
(`radius`, `halfHeight` about the local +Y axis). `shapeInertia()` returns the
principal moments of a uniform solid.

`RigidBody` stores:

- Mass properties: `mass`, `inverseMass`, `principalInertia`,
  `inertiaLocal` (full tensor) and `inverseInertiaLocal`.
- State: `position`, `orientation`, `linearVelocity`, `angularVelocity`.
- `forceAccumulator` / `torqueAccumulator` accumulators.
- Damping `linearDamping` / `angularDamping` (exponential decay per second).
- World caches `rotationCache` and `inverseInertiaWorldCache` (refreshed by
  `updateCache()`).

`validateInertia()` checks symmetry, positive definiteness and the triangle
inequality (`I1 + I2 ≥ I3`) with its own eigenvalue solver (cyclic Jacobi,
`symmetricEigenvalues`).

## Integrator (`physics/Integrator.hpp`)

**Semi-implicit** (symplectic) Euler: velocities first, then positions.

- Linear: `v += (g + F/m)·dt`.
- Angular: the gyroscopic term is solved with the **implicit midpoint rule**,

  $$
  \omega_{n+1} = \omega_n + \Delta t\, I^{-1}\!\left(\tau - \omega_{m}\times (I\,\omega_{m})\right),
  \qquad \omega_m = \tfrac{\omega_n + \omega_{n+1}}{2}
  $$

  solved with a few fixed-point iterations. The midpoint rule is symplectic and
  conserves **exactly** every quadratic invariant of the system frozen over the
  step, in particular the kinetic energy and the magnitude of the angular
  momentum, so a wheel at high revolutions neither gains energy nor loses spin.
- Positions: `p += v·dt` and the orientation with the exponential map,
  `q = exp(½ ω Δt) · q`, renormalised.

## Force elements (`physics/ForceElements.hpp`)

They only add to the accumulators; the integrator consumes them afterwards.

- `PointSpringDamper`: spring-damper between two points (or against a fixed
  world point when `bodyB` is `nullptr`). The force is
  `f = k·(L − L₀) + c·(v_rel · n̂)`; if a `Curve` is given it replaces `k`.
- `RotationalSpringDamper`: torsional torque about a local axis that drives the
  relative rotation of B with respect to A towards `restAngle`.

`Curve` is a function `y = f(x)` by linear interpolation between control points,
with clamped ends; empty is equivalent to `y = slope · x`.

## Constraint solver (`physics/Constraint.hpp`, `physics/Joints.hpp`)

**Sequential impulses (PGS)** solver over rows, with **warm starting** and
**soft constraints** in the Box2D style. Each row is a scalar:

$$
\dot C = J\,v = \lambda_{A}\!\cdot v_A + \alpha_A\!\cdot \omega_A
            + \lambda_{B}\!\cdot v_B + \alpha_B\!\cdot \omega_B .
$$

The applied impulse changes the velocities as:

$$
v_A \mathrel{+}= m_A^{-1}\lambda_A\,\lambda, \qquad
\omega_A \mathrel{+}= I_A^{-1}\alpha_A\,\lambda, \quad (\text{likewise } B).
$$

### Soft constraint (soft step)

With frequency `f` and damping `ζ`, the reduced mass `m = 1/K` (where
`K = J M⁻¹ Jᵀ`):

$$
\omega = 2\pi f,\quad d = 2m\zeta\omega,\quad k = m\omega^2,\quad
\text{den} = d + \Delta t\,k,
$$

$$
\gamma = \frac{1}{\Delta t\,\text{den}},\qquad
\beta = \frac{\Delta t\,k}{\text{den}},\qquad
b = \beta\,C,
$$

$$
m_{\text{eff}} = \frac{1}{K + \gamma},\qquad
\lambda = -\,m_{\text{eff}}\left(\dot C + b + \gamma\,\lambda_{\text{accum}}\right),
$$

with `λ` clamped to `[lower, upper]`. With `f > 0` the position error is removed
smoothly, without the energy an aggressive Baumgarte injects. Default joint
frequency: ~60 Hz.

### Compliance modes (`SoftParams`)

Each joint carries its own `SoftParams` (`soft`, and `limitSoft` for limits),
with three modes:

| Mode | How to select it | Velocity row | Position error |
|---|---|---|---|
| Rigid | `rigid = true` (or `SoftParams::makeRigid()`), or `stiffness >= 1e18` | hard (`γ = 0`, no bias) | removed by the projection pass (below) |
| Soft step | `stiffness > 0` (N/m or N·m/rad) or `frequency > 0` with `dampingRatio` | `γ`, `b` as above; an explicit `stiffness` overrides `m·ω²`, with `d = 2ζ√(k·m)` | removed gently by the bias |
| Velocity-only | `frequency = 0`, `stiffness = 0`, not rigid | hard | **none** (only for motor rows; a positional joint configured like this drifts) |

Soft joints sag by roughly `F/k`, and `k` scales with the row's own reduced
mass, so a soft joint whose light body carries a much heavier one stretches a
lot (0.17 m for a 1 kg link holding 1000 kg at 60 Hz). Use rigid mode there.

**Rigid mode.** Rigid rows are hard velocity rows, so the velocity solve never
injects energy, and after `integratePositions` a projection pass puts the
bodies back on the constraint manifold: up to `positionIterations` (default 4)
Newton steps that re-run `prepare()` at the current pose and move the bodies by
`Δx = M⁻¹Jᵀ·Δλ` with `K·Δλ = −C` (rotations through the exponential map). The
velocities are not touched. The pass stops once every rigid row error is below
`positionTolerance` (1e-10 m/rad) and a single pass never corrects a row by more
than `maxPositionCorrection` (0.2 m/rad). This was chosen over Baumgarte (which
needs a velocity bias that injects energy and still leaves a steady-state error
under load) and over substepping (which multiplies the cost and only shrinks
the drift). The remaining cost is the usual O(dt) numerical damping of velocity
projection: the swing energy of a rigid pendulum decays slowly (1–5 % in 60 s in
the tests) and is never gained beyond ~1e-3 transients.

### Island block solve

Plain PGS converges at a rate set by the mass ratio: with a 1 kg link between a
static anchor and a 1000 kg body, 8 iterations leave 3 mm of error and 100
iterations still 0.22 mm. `step()` therefore groups joints into **islands**
(joints connected through dynamic bodies; static and kinematic bodies do not
connect them). For each island whose bilateral rows fit in
`directSolverMaxRows` (default 128) the bilateral rows are solved **exactly**
with an LDLᵀ factorisation of the Delassus matrix `K + Γ` (the matrix form of
the soft-step update), done within the matrix envelope so a chain created in
order costs O(n). Bounded rows (motors, limits) are relaxed with block
Gauss-Seidel around that solve. Bigger islands fall back to PGS for velocities
and to a per-joint non-linear Gauss-Seidel projection for positions. Redundant
rows (zero pivots) receive no impulse instead of an unbounded one.
`directSolverMaxRows = 0` restores plain PGS everywhere.

### Joints

| Joint | Rows | Degrees of freedom |
|---|---|---|
| `DistanceConstraint` | 1 | keeps a fixed distance |
| `SphericalConstraint` | 3 | ball joint (the anchors coincide) |
| `FixedConstraint` | 6 | rigid (position + orientation) |
| `RevoluteConstraint` | 7 | hinge: 3 position + 2 angular + motor + limit |
| `PrismaticConstraint` | 7 | slider: 2 position + 3 angular + motor + limit |

Rows with a motor and with limits use impulse bounds: the motor limits the
impulse to `± max_torque·Δt` (or force), and the limits only allow the direction
that corrects the violation. Joints that fix orientation call
`captureReference()` once, after placing the bodies.

## World (`physics/PhysicsWorld.hpp`)

`PhysicsWorld` owns bodies (`std::deque<RigidBody>` so pointers stay stable),
constraints and force elements. It exposes `createBody()`,
`createConstraint<T>(a, b, ...)` and `createForceElement<T>(a, b, ...)`.

`step(dt)` runs, in this order:

```
1. updateCache() and clear accumulators
2. apply force elements
3. integrateVelocities(dt)          // gravity + implicit gyroscopic
4. prepare(dt) of each joint
5. buildContacts(dt)                 // detectCollisions(), wake sleeping bodies, warm-start seeding
6. warmStart() of every joint and contact
7. velocity solve per island          // exact block solve, or velocityIterations × PGS
8. integratePositions(dt)           // positions and orientations
9. projectPositions()               // rigid rows only: Newton projection onto C = 0
10. updateSleeping(dt)              // quiet bodies accrue sleep time and are frozen
```

`advance(elapsed)` consumes real time in `fixedStep` substeps. By default
`gravity = (0, -9.80665, 0)`, `velocityIterations = 8`, `fixedStep = 1/1000`,
`directSolverMaxRows = 128`, `positionIterations = 4`.

### Collisions and contacts (`physics/Collision.hpp`)

Shape-vs-shape detection is the "detection half" of Phase 1: a deterministic
brute-force broadphase (AABB overlap, pairs ordered by `(bodyA, bodyB)`) feeds
analytic narrowphase routines for **sphere–sphere, sphere–box, box–box (SAT with
face clipping and edge–edge) and sphere–cylinder**. Each `ContactManifold` carries
up to `kMaxContactPoints = 4` points; `normal` points from A to B and
`penetration` is the non-negative overlap depth.

A body only takes part when `RigidBody::collisionEnabled` is set (opt-in, so
existing scenes and the joint determinism probe are unchanged). Each contact
point becomes three transient rows in a `ContactConstraint`:

- One **one-sided normal row** (`lower = 0`, `upper = +∞`): the shapes may not
  approach. A Baumgarte term `-β·max(0, penetration − slop)/dt` removes
  penetration at the velocity level (`β = contactBaumgarte`, `slop =
  contactSlop`), without a position projection pass.
- Two **friction rows** along a deterministic orthonormal tangent basis (Duff et
  al.), clamped each iteration to the Coulomb cone
  `[−μ·λn, +μ·λn]` where `λn` is the current accumulated normal impulse.
- **Restitution** raises the normal row's velocity target by the inbound normal
  speed, `target = −e·vn` for approaching contacts, ignored below
  `kRestitutionVelocityThreshold` (1 cm/s) so resting stacks do not jitter.

Material comes from the bodies: `friction` combines as `sqrt(frictionA·frictionB)`
and `restitution` as `max(restitutionA, restitutionB)`. Accumulated normal and
tangential impulses are cached per body pair (`ContactImpulseCache`) and both
warm started and captured in the snapshot.

### Sleeping

A dynamic body whose linear speed stays below `sleepLinearThreshold` (1 cm/s) and
angular speed below `sleepAngularThreshold` (0.05 rad/s) for `sleepTime` (0.5 s)
is put to sleep: its velocities are zeroed, it is not integrated, and the solver
treats it as static (`constraintMass` adds nothing for it and `applyRowImpulse`
skips it), so a resting stack is free. Before the contacts are configured each
step, `wakeBodies()` wakes any sleeping body that a **moving** awake body is
touching, jointed to or coupled to through a force element, propagating along the
chain; a fully resting stack has no moving member, so it stays asleep. Applied
impulses and `RigidBody::wake()` also wake a body. The sleep flag and timer are
part of `stateHash` and the snapshot, so sleeping is reproducible and restorable.
`enableSleeping = false` disables it (and wakes everything).

### Determinism and snapshots

- Ids and creation order (bodies, constraints, elements) are fixed, so the
  solve order is reproducible.
- `stateHash()` is a 64-bit FNV-1a over every `double` of the state (position,
  orientation, velocities) plus time.
- `saveSnapshot()` / `restoreSnapshot()` store body state, each row's
  joint rows' accumulated impulses, the contact warm-start cache, the per-body
  sleep state and time, for interactive reset and the determinism test.

## Vehicle (`vehicle/`)

### Interfaces

The vehicle module is organised around header-only abstract interfaces in
`src/vehicle/include/vehicle/` (SI units, `physics::Real`/`Vec3`):

- `IVehicleModel`: top-level model (`setInput`, `reset`, fixed-step `step(dt)`,
  pose/velocity queries). `VehicleInput` holds throttle/brake/steer/handbrake.
- `ITireModel`: contact-patch forces from slip ratio, slip angle, normal load,
  camber and friction coefficient.
- `ISuspension`: hardpoint-based corner (`SuspensionHardpoints` in the chassis
  frame); wheel pose, spring/damper length and motion ratios.
- `IDrivetrain`: engine torque → clutch → gearbox → differential → wheel torques.
- `IAerodynamics`: drag/downforce/moments from the chassis-relative air velocity.
- `IBrakeSystem`: pedal/handbrake → per-wheel brake torque, front bias.
- `IVehicleElectronics`: ABS/TCS/ESC hooks that filter inputs and modulate brakes.

### `PlaceholderVehicle` (TEMPORARY)

**TEMPORARY**: `vehicle/PlaceholderVehicle.hpp` implements `IVehicleModel` with a
**deterministic kinematic bicycle model**, only so that the app, renderer and
telemetry have something to drive. It will be replaced by a real model built on
the interfaces above; do not extend it.

- 4 wheels in order **FL, FR, RL, RR**; the front ones steer (`steered`), the
  rear ones drive (`driven`).
- Steering is rate-limited (`steerRate`) towards the requested angle.
- Longitudinal forces: traction from `maxEngineTorque / radius`, braking from
  `maxBrakeTorque / radius`, aerodynamic drag (`dragCoefficient · v · |v|`) and
  rolling resistance.
- The handbrake adds a locking force and does not allow reversing direction.
- Kinematic yaw rate: `ω = v / wheelBase · tan(steerAngle)`.
- The engine is an approximation of rpm from the speed.

### Vehicle orientation convention

The vehicle follows the project-wide frame with no exceptions: heading `h` is
a yaw about `+Y`, heading 0 faces **-Z**, `forward = (-sin h, 0, -cos h)` and
`right = (cos h, 0, -sin h)`. Positive steer, yaw rate and heading turn **left**
(counter-clockwise seen from `+Y`; a car starting at heading 0 drifts towards
`-X`). The car's model matrix is `translate(p) * rotate(h, +Y)` with no axis
conversion. `tests/test_vehicle.cpp` pins both facts.

## Determinism

Physics is the heart of the simulator's determinism:

- Fixed step (default `1/1000 s`), never a variable `dt`.
- A single thread runs the step: the order of constraints and impulses is always
  the same.
- `-ffast-math` forbidden; FMA contraction disabled; MSVC in `/fp:precise`.

With this, the same input sequence produces exactly the same trajectory on any
machine.

## Tests (`tests/test_physics.cpp`, `tests/test_collision.cpp`)

Analytic suite covering: free fall and projectile motion (against the exact
solution), mass-spring oscillator period (< 0.5 %), energy drift bounded over
60 s, angular momentum of a free top (< 0.1 % over 10 s), gyroscope at 3000 rpm
(no energy gain and precession < 2 %), 20-link chain (error bounded over 60 s),
rigid 20-link chain and rigid weld (< 0.1 mm), 1:1000 mass ratio as a loaded
double pendulum (anchor → 1 kg → 1000 kg and the reverse order, joint error and
energy drift over 60 s), fixed joint under perturbation, linear damping decay at
the analytic rate, revolute motor reaching its target speed, bit-for-bit
determinism (20000 steps + snapshot/restore) and a benchmark of the 20-link
chain. `test_collision.cpp` covers the detection primitives (AABB, broadphase,
SAT/clipping, cylinder), a resting box, the opt-in behaviour, friction (a sliding
sphere rolling to 5/7 of its speed), restitution (dropped-sphere rebound),
sleeping (a stack that sleeps, stays frozen and is woken by a projectile), a
5-box stack stable for 60 s, the block-on-ramp friction threshold and contact
determinism across repeated runs and snapshot/restore.

## Results

Full suite: **62 cases / 7232 assertions, all green** (`sim_tests`, MSVC Release,
fixed step `dt = 1/1000 s`). Errors refer to the measurement against the
analytic value (relative error when both quantities are of the same kind,
absolute error in metres/radians/J for physical magnitudes).

| Case | Quantity | Analytic value | Obtained value | Error | Tolerance | Justification |
|---|---|---|---|---|---|---|
| Free fall | final v (1 s) | 9.80665 m/s | 9.80665 m/s | 1.52e-14 | < 1e-12 | Semi-implicit Euler gives an exact v under constant g; ~65× margin for cross-platform rounding |
| Free fall | final y (1 s) | 4.90333 m | 4.89843 m | 1.00e-3 | < 2e-3 | dt/t = 1e-3 delay inherent to the semi-implicit scheme; the measured error sets the bound |
| Projectile | x (1 s) | 10.0 m | 10.0 m | 1.69e-14 | < 1e-12 | Unforced horizontal motion: exact except for rounding |
| Projectile | y (1 s) | 5.09668 m | 5.09167 m | 9.62e-4 | < 2e-3 | Same dt/t delay as free fall |
| Mass-spring oscillator | Period | 0.628319 s | 0.628316 s | 4.17e-6 | < 2e-5 | O((ω·dt)²) deviation of the semi-implicit/symplectic integrator; measured 4.17e-6, ~5× margin |
| Simple pendulum | Period | 2.00641 s | 2.00679 s | 3.43e-5 | < 1e-4 | Soft 60 Hz springs + dt; the measured error is ~3× below the bound |
| Oscillator energy (60 s) | Max drift | 0 | 0.00503337 (rel) | 5.03e-3 | < 8e-3 | Integrator energy drift at ω·dt ~ 0.1; bound ~1.6× the measurement |
| Free angular momentum (10 s) | \|I·ω\| rel | 1.0 | 1.0 | 3.20e-15 | < 1e-12 | Conservation of the quadratic invariant; the error is O(dt³) rounding (bound tightened from 1e-9) |
| Dzhanibekov | intermediate-axis ω | inversion | −6.0 rad/s | — | < −0.5 | The sign flips from +6 to −6: unambiguous flip |
| Dzhanibekov | Angular momentum | const | const | 6.86e-12 | < 1e-10 | A free top conserves L (bound tightened from 1e-8) |
| Gyroscope 3000 rpm | Precession (20 s) | 0.312155 rad | 0.313539 rad | 4.43e-3 | < 1e-2 | Precession \|Ω·d/(I₃·ω)\|; soft ball-joint compliance + dt (~2.3× margin) |
| Gyroscope 3000 rpm | Energy growth | 0 | 0 | 0 | < 1e-4 | The integrator conserves the flywheel energy; the bound is only rounding |
| 20-link chain (60 s) | Max joint error | 0 | 0.00447127 m | 4.47e-3 m | < 7e-3 | ~2.8 mm sag from its own weight + swing; bound 1.6× the measurement |
| Rigid 20-link chain, bottom link kicked (60 s) | Max joint error | 0 | 3.34e-12 m | 3.34e-12 m | < 1e-6 | Exact island block solve + position projection; requirement 0.1 mm, bound keeps >10^5 headroom over round-off |
| 1:1000, anchor → 1 kg → 1000 kg, rigid ball joints (60 s) | Max joint error | 0 | 4.22e-12 m | 4.22e-12 m | < 1e-6 | Hard ordering: 8 PGS iterations leave 3.0 mm, 100 leave 0.22 mm, soft 60 Hz joints stretch 0.17 m; the block solve is exact |
| 1:1000, anchor → 1 kg → 1000 kg (60 s) | Swing-energy drift max \|E−E₀\|/(E₀−E_rest) | 0 | 1.46e-2 (−1.34e-2 at 60 s) | 1.46e-2 | < 3e-2 | O(dt) damping of velocity projection: steady decay, ~2× margin |
| 1:1000, anchor → 1 kg → 1000 kg (60 s) | Max energy gain (E−E₀)/(E₀−E_rest) | 0 | 6.48e-4 | 6.48e-4 | < 2e-3 | No energy injection: only a transient within one swing |
| 1:1000 reference, anchor → 1000 kg → 1 kg (60 s) | Max joint error | 0 | 2.55e-12 m | 2.55e-12 m | < 1e-6 | Easy ordering (plain PGS also holds it to ~1e-13 m) |
| 1:1000 reference, anchor → 1000 kg → 1 kg (60 s) | Swing-energy drift max \|E−E₀\|/(E₀−E_rest) | 0 | 4.67e-2 (−4.41e-2 at 60 s) | 4.67e-2 | < 8e-2 | The light tip whips faster, so the projection damping is larger; ~1.7× margin |
| 1:1000 reference, anchor → 1000 kg → 1 kg (60 s) | Max energy gain | 0 | 1.29e-3 | 1.29e-3 | < 3e-3 | No energy injection beyond a transient |
| Fixed joint (5 s) | Position / orientation | 0 | 5.75e-4 m / 1.96e-7 | 5.75e-4 / 1.96e-7 | < 5e-3 | Static sag ~7e-5 m + transient from the initial perturbation |
| Rigid weld, arm cantilevered 1 m, disturbed (5 s) | Anchor / angle error | 0 | 4.65e-19 m / 4.58e-19 rad | 4.65e-19 / 4.58e-19 | < 1e-6 | The block solve removes the disturbance before positions move; requirement 0.1 mm |
| Linear damping (1 s) | v with d = 1 | 3.67879 m/s | 3.68063 m/s | 5.00e-4 | < 1e-2 | Per-step decay 1/(1+d·dt); continuum limit e^(−dt); discrete error ~3e-4 |
| Revolute motor (3 s) | Angular velocity | 3.0 rad/s | 3.0 rad/s | 0 | < 5 % | No opposing torque (gravity aligned with the axis): the motor reaches and holds the speed |
| Determinism (2×20000) | stateHash | equal | 5269089099883223357 | 0 | bit-for-bit | Same thread, fixed order, /fp:precise, no ffast-math |
| Snapshot/restore | stateHash | equal | 5269089099883223357 | 0 | bit-for-bit | Stores body state, accumulated joint impulses, the contact warm-start cache and the sleep timers |
| Contact determinism (2×20000, box + sphere on ground) | contact_hash | equal | 9837289854647932968 | 0 | bit-for-bit | `sim_determinism` also steps an all-`collisionEnabled` scene (box + sphere onto a ground box) and compares `contact_hash` between Windows and Linux |
| Box stack (5 boxes, 60 s) | Max horizontal drift / height error / residual speed | 0 | 9.15e-5 m / 1.47e-4 m / 0 | < 0.05 m / 0.05 m / 0.05 m/s | Warm-started PGS holds the stack essentially rigid; it sleeps, so the residual speed is exactly zero |
| Resting stack wakes on impact | Woken bodies | 3 | 3 | 0 | exact | A projectile dropped on the sleeping stack wakes every box through the contact graph |
| Block on ramp (μ = 0.5, 2 s) | Displacement from rest | 0 m | 9.8e-6 m (tan θ = 0.4) / 1.18 m (tan θ = 0.7) | < 0.01 m / > 0.1 m | Static friction holds below the cone (tan θ ≤ μ), slides above it |
| Sliding sphere, friction μ = 0.6 | Rolled speed | 5/7·v₀ = 1.42857 m/s | 1.42857 m/s | < 1e-5 | < 1e-2 | Starts sliding, rolls without slipping once the contact point stops: v → 5/7 v₀ for a solid sphere (I = 2/5 m r²); the discrete friction impulses land on the analytic value |
| Sliding sphere, friction μ = 0 | Rolled speed | v₀ = 2.0 m/s | 2.0 m/s | 0 | < 2e-2 | No friction rows carry impulse, so the horizontal speed is preserved |
| Dropped sphere, restitution e = 0.8 | Rebound apex (centre y) | 1.46 m | 1.5718 m | 0.112 | < 0.22 (15 %) | Rebound height = e²·drop; the ~8 % excess is the discrete contact detection giving a slightly faster approach than the ideal free-fall at the sampled step |
| Dropped sphere, restitution e = 0 | Rebound apex (centre y) | ≤ 0.5 m | 0.546 m | — | < 0.6 | No restitution: the sphere settles at its resting height (plus contact slop), it does not bounce |
| Benchmark (20-link chain) | Performance | — | 0.0207 ms/step (48271 steps/s) | — | informative | 10000 steps in 207.2 ms, soft chain through the island block solve; measured while other builds were running on the machine, so only indicative |

Note on the 1:1000 cases: an earlier version of this table reported a 1:1000
"mass ratio" joint error of 1.8e-12 m. That test joined two free-falling
bodies with a distance joint, so the joint carried no load and the number
proved nothing about mass-ratio conditioning; it has been replaced by the
loaded double-pendulum cases above.

The rest of the suite (`test_core`: 3 cases, `test_modding`: 5 cases) stays
green: fixed-step scheduling, JSON configuration resolution, semantic-version
parsing, mod manifest loading and dependency resolution.
