// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <array>
#include <vector>

#include "drive/Track.hpp"
#include "physics/ForceElements.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/RigidBody.hpp"

namespace sim::app::drive {

// Parameters of the raycast-wheel test car. Body frame: origin at the centre of
// mass, +Y up, -Z forward, +X to the driver's right. Units are SI.
struct CarParams {
    // --- Chassis ----------------------------------------------------------
    Real mass = 1200.0;
    // Principal moments about body X (pitch), Y (yaw) and Z (roll).
    Vec3 inertia{1900.0, 2100.0, 500.0};
    Vec3 bodyHalfExtents{0.88, 0.35, 2.15}; // visual / shape only

    // --- Suspension geometry ----------------------------------------------
    Real frontAxleZ = -1.25; // CoM 52 % of the wheelbase from the rear axle
    Real rearAxleZ = 1.35;
    Real halfTrack = 0.775;
    Real mountY = 0.0; // suspension top mounts at CoM height

    // --- Suspension -------------------------------------------------------
    Real restLength = 0.25; // spring free length (along body -Y)
    Real springStiffness = 35000.0;
    Real damperBump = 2800.0;    // N s/m, compression
    Real damperRebound = 3600.0; // N s/m, extension
    Real maxCompression = 0.19;  // bump stop engages beyond this
    Real bumpStopStiffness = 200000.0;
    Real antiRollFront = 12000.0; // N/m of left-right compression difference
    Real antiRollRear = 6000.0;

    // --- Wheels and tyres -------------------------------------------------
    Real wheelRadius = 0.32;
    Real wheelHalfWidth = 0.11;
    Real wheelInertia = 1.2; // kg m^2, wheel + tyre + brake disc
    Real peakSlipRatio = 0.12;
    Real peakSlipAngle = 0.15;   // rad (tan of)
    Real tyreShape = 1.5;        // magic-formula C; sliding grip = sin(C pi/2) of peak
    Real lowSpeedVelocity = 1.0; // m/s, slip denominators are clamped to this
    int wheelSubsteps = 10;      // wheel spin substeps per physics step

    // --- Engine and drivetrain (rear-wheel drive, open differential) -------
    Real idleRpm = 900.0;
    Real maxRpm = 7000.0;
    // Full-throttle torque curve (rpm, N m), linearly interpolated.
    std::vector<Real> torqueCurveRpm{1000.0, 2000.0, 3000.0, 4500.0, 6000.0, 7000.0};
    std::vector<Real> torqueCurveNm{170.0, 215.0, 240.0, 250.0, 230.0, 190.0};
    Real engineBrakeTorque = 45.0; // N m at maxRpm with the throttle closed
    std::vector<Real> gearRatios{3.20, 2.15, 1.55, 1.20, 0.97, 0.80};
    Real reverseRatio = 3.30;
    Real finalDrive = 3.60;
    Real drivetrainEfficiency = 0.90;
    Real upshiftRpm = 6300.0;
    Real downshiftRpm = 2600.0;
    Real shiftTime = 0.25;       // s without drive torque while shifting
    bool tractionControl = true; // cuts engine torque when the rear tyres spin

    // --- Brakes -----------------------------------------------------------
    Real maxBrakeTorque = 3600.0;  // N m, all four wheels together
    Real brakeBias = 0.70;         // front share
    Real handbrakeTorque = 2500.0; // N m per rear wheel

    // --- Steering ---------------------------------------------------------
    Real maxSteerAngle = 0.60;   // rad at standstill
    Real minSteerAngle = 0.10;   // rad, floor of the speed-dependent limit
    Real steerSpeedScale = 14.0; // m/s; limit = max / (1 + v / scale)
    Real steerRate = 3.0;        // input units per second towards full lock
    Real steerReturnRate = 5.0;  // input units per second back to centre

    // --- Pedal smoothing (keyboard friendly) -------------------------------
    Real throttleRate = 4.0;
    Real brakeRate = 6.0;

    // --- Aerodynamics -----------------------------------------------------
    Real dragFactor = 0.39; // 0.5 rho Cd A, F = -k |v| v

    Real wheelbase() const { return rearAxleZ - frontAxleZ; }
};

// Driver commands (keyboard or pads). Pedals in [0, 1], steer in [-1, 1] with
// positive steering to the left.
struct DriverInput {
    Real throttle = 0.0;
    Real brake = 0.0;
    Real steer = 0.0;
    bool handbrake = false;
};

enum WheelIndex : std::size_t {
    kFrontLeft = 0,
    kFrontRight = 1,
    kRearLeft = 2,
    kRearRight = 3,
};

// Per-wheel state and the latest diagnostics (for the HUD and debug draw).
struct WheelState {
    Vec3 localMount{0.0}; // suspension top mount in the body frame
    bool front = false;
    bool left = false;

    // Integrated state.
    Real omega = 0.0;     // spin rate, rad/s (> 0 rolls forward)
    Real spinAngle = 0.0; // visual spin angle, wrapped to [0, 2 pi)

    // Diagnostics of the last step.
    bool contact = false;
    Real compression = 0.0;
    Real springForce = 0.0; // stiffness + bump-stop part, N
    Real load = 0.0;        // total normal load (spring + damper + anti-roll), N
    Real steerAngle = 0.0;
    Real slipRatio = 0.0;
    Real slipAngle = 0.0;
    Real longitudinalForce = 0.0;
    Real lateralForce = 0.0;
    Real driveTorque = 0.0;
    Real brakeTorque = 0.0;
    Surface surface = Surface::Asphalt;
    Vec3 mountWorld{0.0};
    Vec3 contactPoint{0.0};
    Vec3 contactNormal{0.0, 1.0, 0.0};
    Vec3 wheelCentre{0.0};
    Vec3 tyreForce{0.0}; // world-space tyre force (longitudinal + lateral)
};

// A raycast-wheel car: one rigid chassis plus four virtual wheels whose
// suspension and tyre forces are applied each step. It is a ForceElement, so the
// PhysicsWorld calls apply() exactly once per step, right after the force
// accumulators are cleared; the element advances its own wheel, engine and
// driver-filter state there with the fixed step it was created with, so the
// world must be stepped with that same fixed dt (determinism requires it
// anyway).
class RaycastCar final : public physics::ForceElement {
public:
    RaycastCar(const CarParams& params, const GroundQuery& ground, Real fixedStep);

    // Creates the chassis body and the car element inside `world`.
    static RaycastCar& spawn(physics::PhysicsWorld& world, const CarParams& params, const GroundQuery& ground);

    void apply() override;
    const char* typeName() const override { return "RaycastCar"; }

    // Puts the car at rest at `position` (ground point under the CoM) with the
    // given heading (yaw about +Y, 0 faces -Z), suspension near its static
    // deflection, first gear engaged.
    void reset(const Vec3& groundPosition, Real heading);

    void setInput(const DriverInput& input) { input_ = input; }
    const DriverInput& input() const { return input_; }

    physics::RigidBody& chassis() { return *bodyA; }
    const physics::RigidBody& chassis() const { return *bodyA; }
    const CarParams& params() const { return params_; }
    const std::array<WheelState, 4>& wheels() const { return wheels_; }

    // Signed speed along the chassis forward axis (m/s).
    Real forwardSpeed() const;
    Real speed() const;
    Real engineRpm() const { return engineRpm_; }
    int gear() const { return gear_; } // -1 reverse, 1..N forward
    bool shifting() const { return shiftTimer_ > 0.0; }
    Real throttle() const { return throttle_; }
    Real brake() const { return brake_; }
    Real steerAngle() const { return steerAngle_; }
    // Fraction of engine torque removed by traction control in the last step.
    Real tractionCut() const { return tractionCut_; }

    // Static ride height of the CoM above flat ground predicted by the linear
    // springs (used by reset() and the tests).
    Real staticRideHeight(Real gravity) const;

private:
    void updateDriver(Real dt);
    void updateGearbox(Real dt);
    Real engineTorque(Real rpm, Real throttle) const;

    CarParams params_;
    const GroundQuery* ground_;
    Real dt_;
    Real gravity_ = 9.80665;

    DriverInput input_{};
    std::array<WheelState, 4> wheels_{};

    Real throttle_ = 0.0;
    Real brake_ = 0.0;
    Real steerInput_ = 0.0;
    Real steerAngle_ = 0.0;
    int gear_ = 1;
    Real shiftTimer_ = 0.0;
    Real directionTimer_ = 0.0;
    Real engineRpm_ = 0.0;
    Real tractionCut_ = 0.0;
};

} // namespace sim::app::drive
