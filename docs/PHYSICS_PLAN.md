<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
<!-- Copyright (C) 2026 The Car Simulator contributors -->

# Vehicle physics technical plan

Detailed plan for Phases 1-4 of the [roadmap](ROADMAP.md): which models are built,
which equations they implement, in what order, and how each one is validated. It
is the specification the physics work is measured against.

> **Status: plan only. Nothing in this document is implemented yet** except what
> [PHYSICS](PHYSICS.md) already describes (rigid bodies, joints, solver).
>
> **About numbers.** Equations are standard vehicle-dynamics results (references
> in the last section). Every *numeric* parameter quoted here is an order-of-magnitude
> placeholder; before a value goes into a vehicle file or a test it must be replaced
> by a sourced figure (regulation, datasheet, measurement) recorded next to it.

## Contents

1. [Scope: the five disciplines](#1-scope-the-five-disciplines)
2. [Conventions and simulation loop](#2-conventions-and-simulation-loop)
3. [Contacts and ground interface (Phase 1)](#3-contacts-and-ground-interface-phase-1)
4. [Tyre model](#4-tyre-model)
5. [Suspension and steering](#5-suspension-and-steering)
6. [Chassis, loads and aerodynamics](#6-chassis-loads-and-aerodynamics)
7. [Powertrain and brakes](#7-powertrain-and-brakes)
8. [Surfaces and environment](#8-surfaces-and-environment)
9. [Discipline profiles](#9-discipline-profiles)
10. [Validation plan](#10-validation-plan)
11. [Data formats and tooling](#11-data-formats-and-tooling)
12. [Work packages](#12-work-packages)
13. [References](#13-references)

---

## 1. Scope: the five disciplines

The physics must serve all of these from one engine, with differences expressed as
**data and optional modules**, never as separate code paths.

| Discipline | What dominates the physics |
|---|---|
| **Circuit** (GT, touring, formula-style) | Tyre peak grip and thermal window, aerodynamic downforce and ride-height sensitivity, brake thermals, load transfer, kerbs, high-speed stability |
| **Rally** | Loose and mixed surfaces (gravel, dirt, snow, tarmac), long-travel suspension, bumps and ruts, jumps and landings, 4WD with three differentials, handbrake and weight-transfer techniques, turbo anti-lag |
| **Rallycross** | Mixed asphalt/gravel in one lap, jumps, AWD launch with traction limit, turbo dynamics, close contact between cars |
| **Karting** | No suspension and no differential (rigid rear axle), chassis torsional flex acting as the suspension, small high-grip tyres, centrifugal or dry clutch, very low mass and inertia |
| **Driving mode** (road, free roam) | Road cars with ABS/ESC/TCS, wet and damp roads, aquaplaning, fuel and passenger mass changes, road roughness, traffic collisions |

### Required capabilities per discipline

`R` = required, `o` = optional / later, `-` = not needed.

| Capability | Circuit | Rally | Rallycross | Karting | Driving |
|---|:-:|:-:|:-:|:-:|:-:|
| Transient combined-slip tyre (Magic Formula family) | R | R | R | R | R |
| Tyre thermal and pressure model | R | o | o | R | o |
| Tyre wear | R | o | o | R | o |
| Loose-surface tyre behaviour (gravel, dirt, snow) | - | R | R | - | o |
| Deformable terrain (ruts, sinkage) | - | R | o | - | - |
| Hardpoint suspension kinematics | R | R | R | - | R |
| Compliance (bushings, steering) | R | R | R | - | R |
| Long travel, bump/rebound stops, bottoming | o | R | R | - | o |
| Chassis torsional flex | o | o | o | R | - |
| Solid / live axle | - | o | - | R (rigid axle) | o |
| Aerodynamics with ride-height maps | R | o | o | - | o |
| Turbo with spool dynamics, anti-lag | o | R | R | - | o |
| Centrifugal clutch / direct drive | - | - | - | R | - |
| Friction clutch, sequential/dog gearbox | R | R | R | o (shifter) | o |
| Automatic / DCT / torque converter | o | - | - | - | R |
| Open, LSD, Torsen, locked differentials | R | R | R | spool | R |
| AWD with centre differential / active coupling | o | R | R | - | o |
| Brake thermal and fade | R | o | R | o | o |
| ABS / TCS / ESC | o | - | - | - | R |
| Airborne dynamics (jumps, landings) | - | R | R | - | - |
| Hydroplaning and wet grip | o | o | o | o | R |
| Road roughness (ISO 8608) | - | R | R | - | R |
| Vehicle-to-vehicle and vehicle-to-world collisions | R | R | R | R | R |
| Variable mass (fuel, ballast, driver) | R | o | o | R | R |
| Force feedback quantities (column torque, vibration) | R | R | R | R | R |

---

## 2. Conventions and simulation loop

### 2.1 Frames

Equations in this document use the **ISO 8855** vehicle and tyre axes:
`x` forward, `y` left, `z` up (right-handed). The engine uses `+X` right, `+Y` up
and `-Z` forward. The conversion is fixed and lives in one place in
`src/vehicle`:

$$
x_{\mathrm{ISO}} = -Z, \qquad y_{\mathrm{ISO}} = -X, \qquad z_{\mathrm{ISO}} = Y .
$$

(They are both right-handed: $(-Z)\times(-X)=Z\times X=Y$.) Sign conventions below
are fixed once and verified by tests, not by inspection:

- Longitudinal slip $\kappa>0$ when the wheel drives the road forward (traction).
- Slip angle $\alpha>0$ when the contact point slides towards $-y$ (to the right);
  the tyre force then points to $+y$, so $F_y>0$ for $\alpha>0$.
- Camber $\gamma>0$ when the top of the wheel leans towards $+y$; camber thrust
  then also points to $+y$.

### 2.2 Time stepping

Fixed step $\Delta t = 1\,\mathrm{ms}$ for the whole simulation (already the engine
default). Components that are faster or stiffer are **sub-stepped or integrated
implicitly inside the step**; nothing uses a variable $\Delta t$.

Per physics step, in this order:

1. Update body caches. Query the ground under every tyre (Section 3.3).
2. Suspension kinematics: wheel carrier pose, steering angle, spring/damper
   lengths and rates (Section 5.1).
3. Tyre: vertical force, slip quantities, transient states, forces and moments,
   thermal update (Section 4).
4. Suspension and steering forces, anti-roll bars, stops (Section 5.2-5.5).
5. Powertrain: engine, clutch, gearbox, differentials, wheel torques; wheel spin
   integrated implicitly (Sections 4.8 and 7).
6. Aerodynamic and external forces (Section 6.3).
7. `PhysicsWorld::step` (rigid-body integration, joints, collisions).
8. Electronics (ABS, TCS, ESC) as discrete controllers sampled at a fixed rate,
   with zero-order hold (Section 7.8).

### 2.3 State and determinism

All state is double precision and serialisable (snapshot/restore). Transient tyre
states, temperatures, engine and driveline states, clutch/gear states, controller
memories and surface-layer state belong to the snapshot, so replays and tests are
bit-reproducible. No pointer-dependent ordering, no global state.

---

## 3. Contacts and ground interface (Phase 1)

### 3.1 Rigid-body contacts

Contacts are inequality constraints on the relative normal velocity $v_n$ of the
two bodies at each contact point, with complementarity:

$$
0 \le \lambda_n \;\perp\; v_n + e\,v_n^{-} \ge 0, \qquad
\|\boldsymbol{\lambda}_t\| \le \mu\,\lambda_n ,
$$

where $e$ is the restitution coefficient and $v_n^{-}$ the pre-impact normal speed.
Each row has the same form as the existing joint rows,
$K = J M^{-1} J^{\mathsf T}$, and the impulse is projected onto its bounds:

$$
\Delta\lambda = -\frac{J v + b}{K}, \qquad
\lambda \leftarrow \operatorname{clamp}(\lambda + \Delta\lambda,\; \lambda_{\min},\; \lambda_{\max}),
$$

with friction rows bounded by $\pm\mu\lambda_n$ (friction pyramid first, cone
projection later). Position error is removed with the same projection used for
rigid joints (no Baumgarte injection). Fast bodies use speculative margins
($d \le v\,\Delta t$) to avoid tunnelling: each pair's narrowphase search margin
grows by its relative speed (linear plus angular times the shape bounding radius)
over a step, and the solver resolves a contact as an impact with restitution once
the gap is about to close, otherwise it only limits the approach to the gap.

Collision detection: broadphase by dynamic AABB tree; narrowphase by closed forms
for sphere/capsule/box pairs, **SAT** for boxes and **GJK + EPA** for convex hulls;
triangle meshes and heightfields through BVH queries. Manifolds keep stable
contact ids so warm starting and ordering are deterministic.

### 3.2 Sleeping and islands

Islands are built from joints and contacts; an island sleeps when $\|v\|$ and
$\|\omega\|$ stay below thresholds for $T_{\mathrm{sleep}}$. Bilateral rows keep
using the exact block solve; contacts are iterative on top (Phase 1 documents the
iteration count needed).

### 3.3 Ground interface for tyres (not a rigid contact)

Tyres **do not use the rigid-body contact solver**: their forces come from the tyre
model. The ground is a service used by the tyre:

```
GroundSample sample(x, y, wheel_geometry) -> {
    effective_plane (height, normal), surface_id, layer_depth, wetness, ...
}
```

For smooth roads the effective plane is the surface under the contact point. For
rough ground (rally, kerbs) the **enveloping tyre** is used: the cylinder-torus tyre
profile is intersected with the terrain with several rays (5-9 along the contact
patch) and the effective height and slope are fitted:

$$
z_{\mathrm{eff}} = \frac{1}{n}\sum_i z_i, \qquad
\mathbf n_{\mathrm{eff}} \propto \sum_i w_i\,\mathbf n_i .
$$

The vertical force then acts along $\mathbf n_{\mathrm{eff}}$, and slip angles and
camber are measured against that plane. The chassis, bodywork and skid plates use
ordinary rigid contacts (Section 3.1) against the same ground.

---

## 4. Tyre model

The tyre is the most important model. The plan is a **Magic Formula** core with
documented extensions for transient behaviour, temperature, loose surfaces and low
speed. Parameters come from files in the TNO **`.tir`** format so published and
measured data can be used directly.

### 4.1 Kinematics

With the tyre axis system (Section 2.1), $V_{cx},V_{cy}$ the wheel-centre
velocity components, $\Omega$ the wheel spin and $R_e$ the effective rolling radius:

$$
\kappa = \frac{\Omega R_e - V_{cx}}{\max(|V_{cx}|,\,V_{\min})}, \qquad
\tan\alpha = \frac{-V_{cy}}{\max(|V_{cx}|,\,V_{\min})}, \qquad
\varphi = \frac{-\Omega_{\perp}}{\max(|V_{cx}|,\,V_{\min})}
$$

($\varphi$ is the spin slip, used for $M_z$ and camber effects; $\Omega_\perp$ is
the yaw-plus-camber spin component of the contact patch.) The floor $V_{\min}$ is
only a guard against division by zero; low-speed behaviour is handled by the
relaxation and standstill models (4.4-4.5), not by $V_{\min}$.

### 4.2 Vertical force and radius

The tyre is a nonlinear, one-sided spring-damper with deflection $\delta$ along the
effective normal:

$$
F_z = \max\!\big(0,\; k_z(\delta)\,\delta + c_z\,\dot\delta\big), \qquad
k_z(\delta) = k_{z0} + k_{z1}\,\delta .
$$

It never pulls (no negative load), and in airborne phases $F_z=0$ exactly.
The loaded radius is $R_l = R_0 - \delta$; the effective rolling radius $R_e$ is a
load- and speed-dependent function (growth with $\Omega^2$ is included for high speed):

$$
R_e = R_0 - \frac{F_{z0}}{k_{z0}}\Big[D\arctan\!\big(B\tfrac{F_z}{F_{z0}}\big) + F\tfrac{F_z}{F_{z0}}\Big] + c_\Omega\,\Omega^2 R_0 ,
$$

with $B,D,F,c_\Omega$ fitted per tyre. The contact patch half-length
$a \approx \sqrt{R_0^2-R_l^2}$ feeds the relaxation lengths.

### 4.3 Steady-state forces (Magic Formula 6.x family)

Every pure-slip force is the Magic Formula:

$$
y(x) = D\sin\!\Big(C\arctan\big(Bx - E\,(Bx - \arctan Bx)\big)\Big) + S_V ,
$$

with stiffness factor $B=K/(CD)$, shape $C$, peak $D$, curvature $E$ and shifts
$S_H,S_V$, so that $x = x_{\mathrm{slip}} + S_H$. The slope at the origin is
$BCD=K$.

**Load dependence** (with $\mathrm{d}f_z=(F_z-F_{z0})/F_{z0}$, scaled by the nominal
load and the scaling factors $\lambda$ of the `.tir` file):

$$
\mu_x = (p_{Dx1}+p_{Dx2}\,\mathrm{d}f_z)\,(1-p_{Dx3}\gamma^2)\,\lambda_{\mu x}, \qquad
D_x = \mu_x F_z, \qquad
K_{x\kappa} = F_z\,(p_{Kx1}+p_{Kx2}\,\mathrm{d}f_z)\,e^{p_{Kx3}\mathrm{d}f_z},
$$

$$
K_{y\alpha} = p_{Ky1}F_{z0}\sin\!\Big(2\arctan\frac{F_z}{p_{Ky2}F_{z0}}\Big)\,(1-p_{Ky3}|\gamma|), \qquad
D_y = \mu_y F_z .
$$

The peak coefficient decreases with load ($p_{Dx2},p_{Dy2}<0$): **load sensitivity**,
which is what makes load transfer cost grip and must be reproduced. Camber adds a
lateral force (camber thrust) and shifts $S_H,S_V$. The full coefficient set
follows Pacejka's MF 6.x; the implementation reads it from the `.tir` file rather
than hard-coding it.

**Combined slip.** Two options are implemented and selectable per tyre:

1. *Weighting functions* of MF 6.x:

$$
F_x = G_{x\alpha}\,F_{x0}(\kappa), \qquad
F_y = G_{y\kappa}\,F_{y0}(\alpha) + S_{Vy\kappa}, \qquad
G_{x\alpha}=\frac{\cos\!\big(C_{x\alpha}\arctan(B_{x\alpha}\alpha_s)\big)}{\cos\!\big(C_{x\alpha}\arctan(B_{x\alpha}S_{Hx\alpha})\big)} .
$$

2. *Slip-vector (friction-ellipse) form*, which keeps the force aligned with the
slip direction and is robust far outside the data (spins, drifts, loose surfaces):

$$
s = \sqrt{\Big(\tfrac{\kappa}{\kappa_m}\Big)^2 + \Big(\tfrac{\tan\alpha}{\tan\alpha_m}\Big)^2}, \qquad
F = F_{\mathrm{res}}(s), \qquad
F_x = F\,\frac{\kappa/\kappa_m}{s}, \quad F_y = F\,\frac{\tan\alpha/\tan\alpha_m}{s} .
$$

**Always enforced:** $\sqrt{F_x^2+F_y^2}\le \mu_{\mathrm{peak}}F_z$ (friction-circle
bound) and zero force at zero slip.

**Self-aligning and overturning moments.** The aligning moment is
$M_z = -t(\alpha_t)\,F_y + M_{zr}(\alpha_r)$ with pneumatic trail

$$
t(\alpha_t) = t_0\cos\!\Big(C_t\arctan\big(B_t\alpha_t - E_t(B_t\alpha_t-\arctan B_t\alpha_t)\big)\Big)\cos\alpha ,
$$

and residual torque $M_{zr}$. The overturning moment $M_x$ and rolling-resistance
moment $M_y$ follow the MF 6.x forms. $M_z$ is the main source of **force feedback**,
so it is validated against the pneumatic-trail curve, not only against $F_y$.

### 4.4 Transient behaviour (relaxation)

Real tyres respond to slip changes over a rolled **distance**, not a time. The
slip quantities fed to the steady-state formulas are filtered with first-order
relaxation:

$$
\dot\alpha' = \frac{|V_{cx}|}{\sigma_\alpha}\,(\alpha-\alpha'), \qquad
\dot\kappa' = \frac{|V_{cx}|}{\sigma_\kappa}\,(\kappa-\kappa'), \qquad
F_y = F_y^{\mathrm{ss}}(\alpha',\dots).
$$

Typical relaxation lengths are of the order of the tyre radius
($\sigma\sim 0.2$-$0.8\,\mathrm{m}$, measured per tyre and load). The filter is
integrated **exactly**, which is stable for any $\Delta t$ and any speed:

$$
\alpha'_{n+1} = \alpha + (\alpha'_n-\alpha)\,e^{-|V_{cx}|\,\Delta t/\sigma_\alpha} .
$$

Step-steer response of the tyre then has the time constant $\sigma/|V_{cx}|$, which
is a validation test (Section 10.1).

### 4.5 Standstill and very low speed

Below a speed threshold the slip definitions are singular and the relaxation form
loses meaning; a **bristle (LuGre) model** blends in. State $z$ is the mean bristle
deflection:

$$
\dot z = v - \frac{\sigma_0\,|v|}{g(v)}\,z, \qquad
g(v)=F_c+(F_s-F_c)\,e^{-(v/v_s)^2}, \qquad
F = \sigma_0 z + \sigma_1\dot z + \sigma_2 v,
$$

with $F_c=\mu_k F_z$, $F_s=\mu_s F_z$. The blend weight goes smoothly from 1 at
standstill to 0 above the threshold. This is what keeps a parked car parked on a
slope and removes the creep seen in the prototype.

### 4.6 Thermal, pressure and wear

Three thermal nodes per tyre: surface $T_s$, carcass $T_c$, inflation gas $T_g$:

$$
C_s\dot T_s = P_{\mathrm{fric}} - h_{\mathrm{air}}(v)A_s(T_s-T_{\mathrm{air}}) - k_{sc}(T_s-T_c) - h_{\mathrm{road}}A_p(T_s-T_{\mathrm{road}}),
$$

$$
C_c\dot T_c = k_{sc}(T_s-T_c) + P_{\mathrm{flex}} - k_{cg}(T_c-T_g), \qquad
C_g\dot T_g = k_{cg}(T_c-T_g),
$$

$$
P_{\mathrm{fric}} = |F_xV_{sx}| + |F_yV_{sy}|, \qquad
P_{\mathrm{flex}} \propto F_z\,v\,C_{rr}\ \text{(hysteresis)} .
$$

Grip depends on surface temperature through a window:

$$
\lambda_\mu(T_s) = 1 - c_T\Big(\frac{T_s-T_{\mathrm{opt}}}{\Delta T}\Big)^2,
$$

and cold-pressure change through the gas law, which shifts $k_z$, the contact
patch and the cornering stiffness:

$$
p = p_0\,\frac{T_g}{T_{g0}} .
$$

Wear removes tread proportional to friction work, $\dot w = c_w P_{\mathrm{fric}}$,
reducing grip and eventually exposing the carcass.

### 4.7 Rolling resistance

$$
F_{rr} = C_{rr}(v,p,T,\text{surface})\,F_z, \qquad
C_{rr} = C_0 + C_1 v + C_2 v^2 ,
$$

applied as the moment $M_y=-F_{rr}R_l$ (and a force on loose surfaces, Section 8.3).

### 4.8 Wheel rotation, integrated implicitly

The wheel obeys $J_w\dot\Omega = T_{\mathrm{drive}} - T_{\mathrm{brake}} - R_eF_x - M_y$.
Because $F_x$ depends steeply on $\Omega$ through $\kappa$ (stiff near lock-up),
explicit integration is unstable at 1 kHz. The force is linearised and the
equation solved **implicitly** each step:

$$
k = \frac{\partial F_x}{\partial\Omega}\bigg|_n, \qquad
\Omega_{n+1} = \frac{J_w\Omega_n + \Delta t\,\big(T - M_y - R_e(F_{x,n} - k\,\Omega_n)\big)}{J_w + \Delta t\,R_e\,k} .
$$

If the wheel would reverse through zero under braking it is clamped to $0$ and the
brake is handled as static friction (the unilateral behaviour of a locked wheel).

---

## 5. Suspension and steering

### 5.1 Kinematics from hardpoints

A vehicle is defined by **hardpoints** (chassis-side pivots, upright ball joints,
tie-rod ends, spring and damper mounts). Two implementations are built:

- **Reference (multibody).** Links are bodies and joints in the existing solver
  (the exact island block solve is designed for exactly this kind of short chain;
  bushings are soft joints with per-axis stiffness). Complete and accurate; used
  for validation, K&C simulation and unusual layouts.
- **Production (tabulated).** For each corner the closure problem is solved offline
  and stored as smooth tables of wheel pose against travel $z$ and rack
  displacement $s$; runtime cost is a table lookup. A test compares tables against
  the reference multibody over the full travel.

**Closure problem.** For a double wishbone the upright (6 DOF) is constrained by
five link-length equations, one per link $i$ with chassis point $\mathbf p_i$,
upright point $\mathbf q_i$ (in the upright frame) and length $L_i$:

$$
\big\|\,R_u\mathbf q_i + \mathbf x_u - \mathbf p_i(s)\,\big\| - L_i = 0, \qquad i=1\dots5,
$$

leaving one degree of freedom (wheel travel $z$). It is solved by Newton iteration
on the 6x6 system with the analytic Jacobian, warm-started from the previous
solution; residuals are driven below $10^{-10}\,\mathrm{m}$. MacPherson, multilink
and trailing-arm layouts only change the constraint list; solid and live axles
(with Panhard rod or links) go through the multibody path.

**Derived quantities (tested against hand geometry):** camber, toe, caster, kingpin
inclination, scrub radius, mechanical trail, roll-centre height, side-view instant
centre (anti-dive / anti-squat), and the **motion ratio**

$$
\mathrm{MR} = \frac{\mathrm d\ell_{\mathrm{spring}}}{\mathrm dz_{\mathrm{wheel}}}, \qquad
k_{\mathrm{wheel}} = k_{\mathrm{spring}}\,\mathrm{MR}^2, \qquad
f_{\mathrm{ride}} = \frac{1}{2\pi}\sqrt{\frac{k_{\mathrm{wheel}}}{m_{\mathrm{corner}}}} .
$$

### 5.2 Springs, dampers and stops

$$
F_{\mathrm{spring}} = k(\ell_0-\ell) + F_{\mathrm{preload}}, \qquad
F_{\mathrm{damper}} = c(\dot\ell)\,\dot\ell,
$$

with $c(\dot\ell)$ a piecewise-linear **force-velocity table**, asymmetric
(bump/rebound) and with blow-off. Bump and rebound stops are progressive:

$$
F_{\mathrm{stop}} = k_b\,\big(\ell_b-\ell\big)_{+}^{\,n}\;+\;c_b\,\dot\ell\,[\ell<\ell_b], \qquad n\ge 1 ,
$$

so landings after jumps dissipate energy instead of rebounding unrealistically.
Strut friction and gas-pressure effects are optional (important in rally).

### 5.3 Anti-roll bars

A bar of torsional stiffness $k_\theta$ at lateral spacing $t_a$ gives a force pair

$$
F_{\mathrm{arb}} = \frac{k_\theta}{t_a^{2}}\,(z_L - z_R)\ \ \text{(opposite sign at each wheel)},
$$

with the bar's motion ratio folded into $t_a$.

### 5.4 Compliance

Bushings are linear (or tabulated) 6-DOF stiffness at each pivot. Their effect is
observable and tested as compliance coefficients, for example

$$
\Delta\delta_{\mathrm{toe}} = c_{ts}\,F_y, \qquad \Delta\gamma = c_{\gamma s}\,F_y,
$$

where $c_{ts},c_{\gamma s}$ come out of the multibody and are compared with the K&C
values of the vehicle file.

### 5.5 Steering

Rack ratio, Ackermann and column torque. With track $t$ and wheelbase $L$, full
Ackermann satisfies

$$
\cot\delta_o - \cot\delta_i = \frac{t}{L},
$$

(outer wheel $\delta_o$, inner $\delta_i$); actual systems are a percentage of it,
produced by the tie-rod geometry rather than imposed. The steering-column torque is

$$
T_{\mathrm{col}} = \frac{1}{\eta_s}\Big(\sum_{w}\big(M_{z,w} + F_{y,w}\,(t_{m,w}+t_{p,w})\big)\Big) + T_{\mathrm{assist}}(T_{\mathrm{col}},v) + T_{\mathrm{friction}},
$$

with mechanical trail $t_m$, pneumatic trail $t_p$, ratio efficiency $\eta_s$. This
is the force-feedback signal, so it must be physically derived (never an
artificial "spring to centre").

### 5.6 Karting: no suspension, flexible frame

A kart has no springs or dampers. Its compliance is the **chassis torsion** between
front and rear halves, modelled as a torsion spring-damper about the longitudinal
axis between two rigid bodies:

$$
M = k_t\,(\theta_f-\theta_r) + c_t\,(\dot\theta_f-\dot\theta_r),
$$

which, together with steering geometry (large caster, toe-out, Ackermann), produces
the typical diagonal load transfer and inner-rear-wheel lift. Front and rear
frames are separate bodies joined by a stiff hinge and the torsion element.

---

## 6. Chassis, loads and aerodynamics

### 6.1 Mass properties

The sprung body, each unsprung mass, fuel, driver, passengers and ballast are
separate point/box contributions. The total inertia tensor about the combined
centre of mass uses the parallel-axis theorem:

$$
I = \sum_i\Big(I_i + m_i\big(\|\mathbf r_i\|^2\mathbb 1 - \mathbf r_i\mathbf r_i^{\mathsf T}\big)\Big) .
$$

Mass and inertia are **recomputed** when fuel burns or ballast moves.

### 6.2 Closed-form loads (also the test oracles)

Rigid-body load transfer for longitudinal acceleration $a_x$ (CG height $h$,
wheelbase $L$):

$$
\Delta F_z = \frac{m\,a_x\,h}{L} .
$$

Lateral transfer at the front axle, sprung mass $m_s$, front/rear roll-centre height
$h_{rf},h_{rr}$, roll stiffness $K_{\varphi f},K_{\varphi r}$ (N m/rad), distance
$b$ from CG to rear axle, roll angle $\varphi$:

$$
\Delta F_{zf} = \frac{1}{t_f}\Big(m_s a_y\,\tfrac{b}{L}\,h_{rf} + K_{\varphi f}\,\varphi + m_{uf}\,a_y\,r_f\Big), \qquad
\varphi = \frac{m_s\,a_y\,h_s}{K_{\varphi f}+K_{\varphi r}-m_s\,g\,h_s},
$$

with $h_s$ the CG height above the roll axis. The rear axle uses $a/L$ and its own
terms. The traction-limited acceleration of a rear-wheel-drive car on a surface of
friction $\mu$ ($a$ = CG to front axle) is

$$
a_{x,\max} = \frac{\mu\,g\,a}{L-\mu\,h} .
$$

### 6.3 Aerodynamics

$$
F_D = \tfrac12\rho\,C_D(\cdot)\,A\,v_{\mathrm{air}}^{2}, \qquad
F_L = \tfrac12\rho\,C_L(\cdot)\,A\,v_{\mathrm{air}}^{2}, \qquad
\mathbf v_{\mathrm{air}} = \mathbf v - \mathbf v_{\mathrm{wind}} ,
$$

with air density from the ideal gas law, $\rho = p/(R_{\mathrm{sp}}T)$ (altitude,
temperature, humidity correction), and the coefficients $C_D,C_L$ and the
**centre of pressure** read from **maps** of front/rear ride height, yaw angle and
(optionally) wing flap angle. Floor ground effect is captured by the ride-height
dependence, including the stall at very low height. Slipstream behind another car
reduces $C_D$ and $C_L$ as a function of gap and lateral offset. Forces and
moments act at the aerodynamic centre of the body, not at the CG.

---

## 7. Powertrain and brakes

The driveline is a **one-dimensional rotational system with unilateral elements**
(clutches that stick or slip, lash, one-way devices). It is integrated in reduced
coordinates with implicit steps and an explicit **state machine** for locked/slipping
configurations; every state change is energy-consistent and tested.

### 7.1 Engine

$$
J_e\,\dot\omega_e = T_e(\omega_e,\theta_{\mathrm{eff}}) - T_f(\omega_e) - T_{\mathrm{load}}, \qquad
T_f = a_f + b_f\omega_e + c_f\omega_e^2 ,
$$

with the torque map $T_e(\omega,\theta)$ bilinearly interpolated (negative at closed
throttle: engine braking), throttle dynamics $\dot\theta_{\mathrm{eff}} = (\theta-\theta_{\mathrm{eff}})/\tau_\theta$,
rev limiter by fuel cut with hysteresis and a PI idle controller. Electric motors
use $T = \min(T_{\max}, P_{\max}/\omega)$ with regeneration.

**Turbo (rally, rallycross):** boost pressure follows a first-order lag whose time
constant depends on exhaust energy,

$$
\dot p_b = \frac{p_{b}^{\ast}(\omega_e,\theta) - p_b}{\tau_t(\omega_e)}, \qquad
T_e = T_{\mathrm{na}}(\omega_e)\Big(1 + k_b\big(\tfrac{p_b}{p_{\mathrm{amb}}}-1\big)\Big),
$$

with an **anti-lag** mode that keeps $p_b^\ast$ high at closed throttle (retarded
ignition, small negative torque) and launch control.

### 7.2 Clutches

Friction clutch with clamp force $F_n$, $n_f$ friction faces, effective radius
(uniform pressure) $R_{\mathrm{eff}}$:

$$
T_{c} = \mu_c\,F_n\,R_{\mathrm{eff}}\,n_f, \qquad
R_{\mathrm{eff}}=\frac23\,\frac{R_o^3-R_i^3}{R_o^2-R_i^2} .
$$

Slipping: $T=T_c\,\mathrm{sign}(\Delta\omega)$ (smoothed through zero). Locked: the
transmitted torque is whatever the driveline needs, **clamped** to $\pm T_c$; if the
clamp is hit the clutch begins to slip. The sticking condition is a bilateral row
with impulse limits, exactly like the existing motor rows.

**Centrifugal clutch (karting):** capacity grows with engine speed once the shoes
engage,

$$
T_c = k_{cf}\,\big(\omega_e^2-\omega_{\mathrm{eng}}^2\big)_{+} ,
$$

which gives the characteristic engagement at a given rpm, no stall control and the
absence of engine braking below engagement. Shifter karts use a hand-operated dry
clutch instead.

### 7.3 Gearbox

Ratios $g_k$, efficiency $\eta_g$, torque $T_{\mathrm{out}} = g_k\,\eta_g\,T_{\mathrm{in}}$,
and reflected inertias $J_{\mathrm{eff}}=J_{\mathrm{out}}+g_k^2J_{\mathrm{in}}$.
Shift models: H-pattern and sequential with synchroniser or dog engagement (shift
time, torque interruption, rev-match), dual-clutch and automatic with torque
converter ($T_p = K_{\mathrm{cap}}(\mathrm{SR})\,\omega_p^2$, $T_t = \mathrm{TR}(\mathrm{SR})\,T_p$)
for the driving mode.

### 7.4 Differentials

With ring-gear speed $\omega_{\mathrm{in}}$ and output speeds $\omega_L,\omega_R$:

| Type | Kinematic law | Torque law |
|---|---|---|
| Open | $2\,\omega_{\mathrm{in}} = \omega_L+\omega_R$ | $T_L = T_R = T_{\mathrm{in}}/2$ |
| Locked / spool (kart axle, some rally centres) | $\omega_L=\omega_R$ | whatever is needed |
| Clutch-pack LSD | open law | $T_L-T_R = \Delta T,\ \ \Delta T = \big(T_{\mathrm{pre}} + k_{p/c}\,|T_{\mathrm{in}}|\big)\tanh(\Delta\omega/\omega_\varepsilon)$ |
| Torsen (torque-bias) | open law | $T_{\mathrm{high}}/T_{\mathrm{low}}\le \mathrm{TBR}$ |
| Viscous | open law | $\Delta T = c_v\,\Delta\omega$ |

The ramp coefficient $k_{p/c}$ differs for power and coast. AWD cars add a centre
differential (fixed or torque-split, e.g. 40/60) or an active coupling (torque limited
by a controller), and front and rear differentials; each is an instance of the
same element type, wired from data.

**Rigid axle without differential (karting):** both rear wheels share one shaft
($\omega_L=\omega_R$). The inner and outer tyres must slip against each other, which
is where the characteristic kart push/understeer on entry and the lift of the
inner wheel come from; there is no special case in the code, only the spool element.

### 7.5 Driveline compliance

Half-shafts and propshafts are torsional spring-dampers with lash:

$$
T = k_s\,\Delta\theta_{\mathrm{eff}} + c_s\,\Delta\omega, \qquad
\Delta\theta_{\mathrm{eff}} = \Delta\theta - \operatorname{clamp}(\Delta\theta,\,-\tfrac{\beta}{2},\,\tfrac{\beta}{2}),
$$

($\beta$ = backlash), which produces wind-up, shunt and the "clunk" on throttle
reversal.

### 7.6 Brakes

Disc brake torque from line pressure $p$ (pad coefficient $\mu_p$, piston area
$A_p$, effective radius $R_{\mathrm{eff}}$, two pads):

$$
T_b = 2\,\mu_p(T_d)\,A_p\,(p-p_{\mathrm{thr}})_{+}\,R_{\mathrm{eff}} .
$$

Front/rear bias comes from the master cylinders, and the handbrake acts on the rear
axle only (rally and rallycross use it as a technique). **Thermal model:**

$$
m_d c_{p}\dot T_d = P_b - h_{\mathrm{conv}}(v)A_d\,(T_d-T_{\mathrm{air}}) - \varepsilon\sigma A_d\big(T_d^4-T_{\mathrm{air}}^4\big), \qquad
P_b = T_b\,\omega_w ,
$$

and the pad coefficient follows a fade curve $\mu_p(T_d)$ (cold, optimum window,
fade).

### 7.7 Locked wheels and low speed

A wheel that locks under braking switches to the sliding branch of the tyre curve
(Section 4.3) with $\mu_k<\mu_s$ on loose surfaces, and to the standstill model
(Section 4.5) when the car stops. Never a hard velocity clamp.

### 7.8 Electronics

Controllers run as discrete-time systems at a **fixed rate** (for example 100 Hz)
with zero-order hold, so their behaviour is deterministic and testable:

- **ABS:** slip-threshold controller with hold/dump/increase pressure phases based
  on wheel deceleration and estimated slip against a reference speed.
- **Traction control:** reduces engine torque (and brakes the spinning wheel) when
  $\kappa$ exceeds a target.
- **ESC:** compares the measured yaw rate with the bicycle-model target
  $r^\ast = V\delta/(L+KV^2/g)$ and applies selective braking and torque cut.

They implement `IVehicleElectronics`, are optional per vehicle, and must be
**disabled** in the disciplines where they do not exist.

---

## 8. Surfaces and environment

### 8.1 Surface table

Every surface has a record in a data file:

| Parameter | Meaning |
|---|---|
| $\lambda_{\mu}$ | Scale on peak friction |
| $\lambda_{K}$ | Scale on slip stiffness |
| $\mu_k/\mu_s$ | Sliding-to-peak friction ratio (matters on loose surfaces) |
| $C_{rr}$ | Rolling-resistance coefficient |
| layer parameters | Loose-layer depth, shear and sinkage parameters (8.3) |
| roughness | Power-spectral-density class (8.2) |
| wetness model | Water film behaviour (8.4) |

Asphalt (dry, damp, wet), concrete, painted lines, kerbs, rumble strips, gravel,
packed dirt, mud, snow, ice, sand and grass are the initial set.

### 8.2 Road roughness

Road profiles follow ISO 8608: displacement PSD $G_d(n)=G_d(n_0)(n/n_0)^{-w}$ with
$w\approx 2$ and the class (A-H) setting $G_d(n_0)$. A profile is generated by
spectral synthesis with a seeded random generator, so it is deterministic:

$$
z(x) = \sum_{i=1}^{N}\sqrt{2\,G_d(n_i)\,\Delta n}\;\cos\!\big(2\pi n_i x + \phi_i\big), \qquad
\phi_i\sim U(0,2\pi) .
$$

Each side of the car receives a correlated profile (left/right coherence) so roll,
pitch and heave are excited realistically. Washboard on gravel is a narrow-band
version of the same method.

### 8.3 Loose surfaces and deformable terrain (rally, rallycross)

Loose gravel and snow are modelled as a **surface layer** of thickness $d$ over a
hard base. Effective friction depends on layer depth:

$$
\mu_{\mathrm{eff}} = \mu_{\mathrm{hard}} + (\mu_{\mathrm{loose}}-\mu_{\mathrm{hard}})\big(1-e^{-d/d_0}\big),
$$

the peak occurs at larger slip, sliding friction is clearly lower than peak, and
a sliding tyre ploughs material sideways into a lateral berm. The layer evolves: tyres
displace material ($\dot d\propto$ slip work), producing racing lines and ruts that
the following cars feel. Longer-term, a **terramechanics** model replaces the
empirical layer where required (Bekker-Wong):

$$
p = \Big(\frac{k_c}{b}+k_\phi\Big)z^{\,n}, \qquad
\tau = (c+p\tan\phi)\big(1-e^{-j/K}\big), \qquad
R_c = b\Big(\frac{k_c}{b}+k_\phi\Big)\frac{z^{\,n+1}}{n+1},
$$

(pressure-sinkage, Janosi-Hanamoto shear, compaction resistance). The first version
uses the empirical layer; Bekker-Wong is the validated upgrade path.

### 8.4 Wet roads and aquaplaning

Water film depth $h_w$ reduces grip through a smooth factor of speed and depth, and
above a critical speed the tyre loses contact. An indicative critical speed
(NASA/Horne) for an inflated tyre is

$$
v_{\mathrm{hp}} \approx 63.5\,\sqrt{p\ [\mathrm{bar}]}\ \ \mathrm{km/h},
$$

(about 90 km/h at 2 bar; order-of-magnitude value, refined per tyre and tread
depth). The grip factor is $\lambda_{w}=1/\big(1+(v/v_{\mathrm{hp}})^{k}\big)$ scaled
by the wetness; drying lines, puddles and standing water are surface-map layers.

### 8.5 Airborne dynamics, jumps and landings

In flight the tyre loads are zero and the body rotates under the gyroscopic term of
the spinning wheels (the integrator already conserves angular momentum for the
rigid body; wheel angular momentum is included explicitly):

$$
\mathbf L = I\boldsymbol\omega + \sum_w J_w\,\Omega_w\,\hat{\mathbf n}_w .
$$

Landing is handled by progressive bump stops (Section 5.2) plus **underbody and
bodywork contacts** (Section 3.1) so that bottoming out on the skid plate produces
a force and friction, not a fall through the ground.

### 8.6 Weather and track state

Air temperature, pressure and density; wind (steady plus turbulent gusts from a
seeded filter); track temperature and its coupling to the tyre thermal model;
rubber deposition (racing line gains grip); rain state driving the surface wetness.

---

## 9. Discipline profiles

A "car" is a data file; a discipline is a set of surfaces, vehicle classes and
rules. The profiles below are the **acceptance targets** each vehicle class must
meet; all quantities are to be filled with sourced values (regulations are public
facts) before use. All vehicles are fictional.

| Class | Key configuration | Why it stresses the model |
|---|---|---|
| Hatchback, front-wheel drive | Torsion beam or MacPherson, open or limited-slip differential, ABS/ESC | Understeer gradient, lift-off oversteer, driving mode baseline |
| Coupe, rear-wheel drive | Double wishbone or multilink, LSD, optional aero | Traction limit $a_{x,\max}$, oversteer, drift |
| GT / touring (circuit) | Double wishbone, sequential gearbox, aero maps, brake fade | Load sensitivity, downforce, tyre thermals |
| Rally car (4WD) | Long travel, three differentials, turbo with anti-lag, skid plate | Loose surfaces, jumps, handbrake, rough roads |
| Rallycross supercar | AWD, very high power-to-weight, launch control | Launch traction, mixed surface, jumps, contact |
| Kart (shifter or direct-drive) | Rigid axle, flexible frame, centrifugal or dry clutch | Spool axle, torsion, tiny inertia, high grip |
| Road car, automatic | Torque converter, ABS/ESC/TCS, wet tyres | ESC and ABS behaviour, wet grip, aquaplaning |

---

## 10. Validation plan

**Principle.** Every model is checked against (1) an analytic or closed-form
result, (2) published or measured data, or (3) a physical invariant. Tolerances are
fixed in the test and justified in a comment; they are never loosened to hide a
regression. All run in CI headless and deterministically.

### 10.1 Tyre

| Test | Oracle |
|---|---|
| Pure-slip curves reproduce the `.tir` data | Reference curves $F_x(\kappa)$, $F_y(\alpha)$, $M_z(\alpha)$ within a fixed relative error |
| Friction bound | $\sqrt{F_x^2+F_y^2}\le\mu_{\mathrm{peak}}F_z$ over random slip/load sweeps |
| Load sensitivity | $\mu(F_z)$ decreases monotonically; peak force concave in load |
| Symmetry and zero | $F(-\alpha)=-F(\alpha)$, zero at zero slip |
| Transient time constant | Step in $\alpha$ gives $\sigma/|V|$ to within a fixed percentage |
| Passivity | Power of tyre forces on slip velocities never positive (no energy creation) |
| Standstill | Parked car on a slope stays within a displacement bound for 60 s |
| Thermal steady state | Constant slip converges to $T = T_{\mathrm{air}}+P/h$ analytic value |
| Wheel implicit integration | Run-up with constant torque follows the analytic exponential; no oscillation at 1 kHz at lock-up |

### 10.2 Suspension and steering

| Test | Oracle |
|---|---|
| Closure residuals | $<10^{-10}$ m everywhere in travel and steer range |
| Tabulated vs multibody | Wheel pose difference below a fixed bound (for example 0.1 mm, 0.01 deg) |
| Motion ratio / ride frequency | $f_{\mathrm{ride}}=\frac1{2\pi}\sqrt{k_w/m}$ from a free-decay test, within 1 % |
| Damping ratio | Log-decrement from decay equals $c/(2\sqrt{km})$ |
| Roll gradient | Moment test gives $\varphi/a_y=m_sh_s/(K_\varphi-m_sgh_s)$ within 1 % |
| Static loads | Weight distribution equals the closed-form value |
| Ackermann | $\cot\delta_o-\cot\delta_i$ matches geometry |
| Kingpin / caster / trail | Match the hand-calculated geometry of the vehicle file |
| Compliance | Computed $c_{ts}$ and $c_{\gamma s}$ match the K&C values of the vehicle file |

### 10.3 Driveline and brakes

| Test | Oracle |
|---|---|
| Overall ratio and efficiency | $T_{\mathrm{out}}=g\,\eta\,T_{\mathrm{in}}$ exactly |
| Differential laws | Open split exact; LSD bias bounded by $\Delta T$; spool equal speeds |
| Engine run-up | With constant torque, $\omega(t)$ follows $J\dot\omega=T-T_f$ |
| Clutch energy | $\int T_c\Delta\omega\,\mathrm dt$ equals the heat generated plus kinetic energy lost |
| Shift | Torque interruption time and speed continuity within a bound; energy-consistent |
| Brake thermal | Constant power gives the analytic first-order response; fade follows $\mu_p(T_d)$ |
| Centrifugal clutch | Engagement rpm equals $\omega_{\mathrm{eng}}$; no torque below it |

### 10.4 Vehicle level, by manoeuvre

| Manoeuvre | Oracle |
|---|---|
| Straight braking (ISO) | $d = v^2/(2\mu g)$ plus the aerodynamic and rolling-resistance corrections |
| Acceleration | Power-limited: $P\,t=\tfrac12mv^2$ plus drag; traction-limited: $a=\mu g\,a/(L-\mu h)$ |
| Steady-state circle (ISO 4138) | $\delta = L/R + K\,a_y/g$, $K = W_f/C_{\alpha f}-W_r/C_{\alpha r}$ |
| Step steer / yaw gain (ISO 7401) | $r/\delta = V/(L+KV^2/g)$; characteristic speed $\sqrt{gL/K}$; critical speed $\sqrt{-gL/K}$ for oversteer |
| Skidpad | Peak lateral acceleration against load-transfer-corrected $\mu_{\mathrm{eff}}g$ |
| Roll and pitch gradients | Closed-form values of Section 6.2 |
| Lane change (ISO 3888-2) | Pass/fail speed envelope; no energy creation |
| Constant-speed sweep | Understeer gradient changes with aero and load as predicted |
| Jump | Ballistic trajectory; angular momentum conserved in flight; landing energy absorbed by stops |
| Loose surface braking | Stopping distance with $\mu_{\mathrm{loose}}$ and the layer model |
| Handbrake turn | Rear lock initiates yaw; yaw rate within a plausible envelope |
| Launch (rallycross) | 0-100 km/h limited by traction early and by power later |
| Kart | Spool axle gives equal rear wheel speeds; inner-rear load falls to zero at a corner acceleration that depends on torsional stiffness; no suspension travel |
| Road ESC / ABS | Stopping distance with ABS $\le$ without on wet; ESC keeps yaw within the bicycle target |
| Determinism | Identical `stateHash` on repeated runs, snapshots, and between Windows and Linux |
| Energy | Total energy never increases in conservative conditions (no tyre, brake or engine input) |

### 10.5 Cross-checking

Where possible results are compared against an **independent implementation** of
the same equations (a small reference script or a published worked example) and
against the linear bicycle and two-track models in their valid range.

---

## 11. Data formats and tooling

### 11.1 Vehicle description

A vehicle is a directory of text files: units are in key names (`mass_kg`,
`inertia_kg_m2`, `stiffness_N_per_m`) and every file is validated at load against
a schema with ranges and cross-checks (for example, the sum of corner masses must
equal the total). A sketch:

```toml
[chassis]
sprung_mass_kg = 1050.0
cg_m = [0.0, 0.48, -0.05]            # project axes: x right, y up, -z forward
inertia_kg_m2 = [520.0, 1650.0, 1750.0]

[[axle]]
name = "front"
suspension = "double_wishbone"       # hardpoints file
tyre = "tyres/fictional_road_205.tir"
spring_N_per_m = 32000.0
damper_table = "dampers/front_road.csv"
arb_Nm_per_rad = 900.0

[powertrain]
engine = "engines/fictional_2l_turbo.toml"
clutch = { type = "friction", capacity_Nm = 450.0 }
gearbox = "gearboxes/fictional_6_manual.toml"
differentials = ["front_open", "centre_torque_split", "rear_lsd"]
```

### 11.2 Tools (built early, before tuning)

- **Test rigs:** dynamometer (engine and chassis), K&C rig (bump, roll, steer,
  lateral and longitudinal force compliance), tyre test rig (slip sweeps), shock
  dyno, brake dyno, wind-tunnel map generator.
- **Telemetry:** per-wheel slip, load, forces, temperatures, deflections, and
  differential and clutch states; CSV export; plots in the UI.
- **Replay:** deterministic playback from recorded inputs; difference viewer between
  two runs.
- **Fitting:** least-squares fit of Magic Formula and damper parameters to data,
  with residual plots.
- **Regression numbers:** reference results of the validation suite stored in
  `tests/vehicle/` and compared on every run.

---

## 12. Work packages

Order follows the dependency chain. Sizes: S days, M 1-2 weeks, L several weeks,
XL more than a month. "Exit" is the objective condition for closing the package.

### Phase 1 - Collisions and contacts

| ID | Work | Equations | Size | Exit |
|---|---|---|---|---|
| P1.1 | Shapes, AABB tree broadphase, stable contact ids | 3.1 | M | Pairs deterministic and order-independent |
| P1.2 | Narrowphase: sphere, capsule, box, SAT, GJK+EPA | 3.1 | L | Penetration depth and normals match analytic cases |
| P1.3 | Triangle mesh and heightfield queries | 3.1, 3.3 | M | Queries match brute force |
| P1.4 | Contact rows, friction, restitution, warm start | 3.1 | L | Stack, bounce, ramp tests pass |
| P1.5 | Islands, sleeping, speculative margin | 3.2 | M | No tunnelling, no jitter, determinism with contacts |
| P1.6 | Ground interface and enveloping tyre samples | 3.3 | M | Effective plane matches analytic bumps |

### Phase 2 - Vehicle dynamics

| ID | Work | Equations | Size | Exit |
|---|---|---|---|---|
| P2.1 | Tyre: kinematics, vertical model, wheel implicit step | 4.1, 4.2, 4.8 | M | Run-up and lock-up tests |
| P2.2 | Magic Formula pure slip + `.tir` loader | 4.3 | M | Curves match data |
| P2.3 | Combined slip (both forms), $M_z$, $M_x$ | 4.3 | M | Friction bound and symmetry |
| P2.4 | Transient relaxation and standstill model | 4.4, 4.5 | M | Time constant and parked-on-slope tests |
| P2.5 | Hardpoint kinematics (multibody reference) | 5.1 | L | K&C curves from hardpoints |
| P2.6 | Tabulated kinematics + equivalence test | 5.1 | M | Tables match multibody |
| P2.7 | Springs, dampers, stops, ARB, bushings | 5.2-5.4 | M | Ride, damping and roll tests |
| P2.8 | Steering, Ackermann, column torque | 5.5 | M | Geometry and torque tests |
| P2.9 | Mass properties, variable mass, aero maps | 6.1, 6.3 | M | Load-transfer oracles |
| P2.10 | Engine, clutches, gearbox, driveline compliance | 7.1-7.3, 7.5 | L | Run-up, shift and energy tests |
| P2.11 | Differential family and AWD couplings | 7.4 | M | Differential laws exact |
| P2.12 | Brakes and thermal model; handbrake | 7.6 | M | Thermal and bias tests |
| P2.13 | `IVehicleModel` integration; remove prototypes | all | M | Data-defined cars run in the app |

### Phase 3 - Validation, tooling, force feedback

| ID | Work | Equations | Size | Exit |
|---|---|---|---|---|
| P3.1 | Rigs: dyno, K&C, tyre rig, shock dyno | 10.1-10.3 | M | Rigs produce the reference plots |
| P3.2 | Manoeuvre suite (Section 10.4) in CI | 10.4 | L | All manoeuvres pass with fixed tolerances |
| P3.3 | Telemetry, CSV, replay, fitting tools | 11.2 | M | Run difference viewer works |
| P3.4 | Force-feedback signal and device output | 5.5 | M | Torque derived from $M_z$ and trail only |
| P3.5 | ABS / TCS / ESC (road car) | 7.8 | M | Stopping distance and yaw tests |
| P3.6 | Performance pass | 2.2 | M | Per-car step budget met |

**Exit of Phase 3 (Alpha 0.1):** the five reference vehicle classes of Section 9 run
and pass their discipline manoeuvres.

### Phase 4 - World physics

| ID | Work | Equations | Size | Exit |
|---|---|---|---|---|
| P4.1 | Surface table, friction and rolling maps | 8.1 | M | Surface changes alter lap time predictably |
| P4.2 | ISO 8608 roughness and washboard | 8.2 | M | Spectral and class tests |
| P4.3 | Loose-surface layer, ruts | 8.3 | L | Layer-dependent braking and racing-line ruts |
| P4.4 | Wet roads, aquaplaning | 8.4 | M | Wet braking and $v_{\mathrm{hp}}$ tests |
| P4.5 | Jumps, landings, underbody contacts | 8.5 | M | Ballistic and landing energy tests |
| P4.6 | Weather, wind, tyre/track temperatures, rubbering | 8.6 | M | Thermal coupling test |
| P4.7 | Bekker-Wong deformable terrain (upgrade) | 8.3 | L | Sinkage and drawbar-pull references |

---

## 13. References

Equations and structure follow standard references; each package cites the exact
chapter it uses in its pull request.

- H. B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed. (Magic Formula 6.x, relaxation
  lengths, combined slip, transients).
- I. Besselink, A. Schmeitz, H. Pacejka, "An improved Magic Formula/Swift tyre model
  that can handle inflation pressure changes", *Vehicle System Dynamics* 48 (2010).
- G. Rill, *Road Vehicle Dynamics: Fundamentals and Modeling* (TMeasy, multibody
  chassis, driveline).
- W. F. Milliken, D. L. Milliken, *Race Car Vehicle Dynamics* (load transfer, roll
  stiffness, steady-state cornering).
- T. D. Gillespie, *Fundamentals of Vehicle Dynamics*.
- J. Y. Wong, *Theory of Ground Vehicles* (Bekker-Wong terramechanics).
- J. Reimpell, H. Stoll, J. Betzler, *The Automotive Chassis*; H. Heissing,
  M. Ersoy, *Chassis Handbook* (kinematics, compliance, K&C).
- R. Jazar, *Vehicle Dynamics: Theory and Application*.
- C. Canudas de Wit et al., "A new model for control of systems with friction", *IEEE
  TAC* 40 (1995) (LuGre).
- Standards: ISO 8855 (vehicle dynamics vocabulary), ISO 4138 (steady-state circular),
  ISO 7401 (lateral transient), ISO 3888-2 (obstacle avoidance), ISO 8608 (road
  roughness).
- TNO **MF-Tyre / `.tir`** file format documentation; the open `.tir` datasets used
  must be licence-checked like any other asset.
