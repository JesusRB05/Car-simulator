// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <array>
#include <cstddef>

#include "physics/ForceElements.hpp"
#include "physics/PhysicsWorld.hpp"
#include "vehicle/Aerodynamics.hpp"
#include "vehicle/Brake.hpp"
#include "vehicle/Clutch.hpp"
#include "vehicle/Differential.hpp"
#include "vehicle/Engine.hpp"
#include "vehicle/Gearbox.hpp"
#include "vehicle/GroundQuery.hpp"
#include "vehicle/ITireModel.hpp"
#include "vehicle/IVehicleElectronics.hpp"
#include "vehicle/IVehicleModel.hpp"
#include "vehicle/MagicFormulaTire.hpp"
#include "vehicle/SpringDamper.hpp"
#include "vehicle/Steering.hpp"
#include "vehicle/WheelSpin.hpp"

namespace sim::vehicle {

using physics::Quat;
using physics::Real;
using physics::Vec3;

// Wheel order used everywhere in the model: front-left, front-right, rear-left,
// rear-right, matching IBrakeSystem.
enum WheelCorner : std::size_t {
    kCornerFrontLeft = 0,
    kCornerFrontRight = 1,
    kCornerRearLeft = 2,
    kCornerRearRight = 3,
};

// One suspension corner: the geometry the raycast strut reads plus the latest
// state. The strut hangs from `localMount` along the body's down axis; the wheel
// centre sits `wheelRadius` above the contact point.
struct VehicleCorner {
    Vec3 localMount{0.0}; // suspension top mount in the body frame (m)
    bool front = false;
    bool left = false;
    bool steered = false;
    bool driven = false;
    Real wheelRadius = 0.32; // m
    Real wheelInertia = 1.2; // J_w (kg*m^2)

    // Elastic element (spring, damper and progressive stops) along the strut.
    SpringDamperCoefficients spring{};

    // State integrated by the model.
    WheelSpinState spinWheel{}; // Omega and lock flag
    Real spinAngle = 0.0;       // visual, wrapped to [0, 2 pi)

    // Diagnostics of the last step.
    bool contact = false;
    Real compression = 0.0; // m, + = bump
    Real load = 0.0;        // total vertical force carried (N)
    Real steerAngle = 0.0;  // rad, + = left
    Real slipRatio = 0.0;
    Real slipAngle = 0.0;
    Real longitudinalForce = 0.0; // N
    Real lateralForce = 0.0;      // N
    Real driveTorque = 0.0;       // N*m at the wheel
    Real brakeTorque = 0.0;       // N*m at the wheel
    Vec3 mountWorld{0.0};         // strut top mount in world space (m)
    Vec3 contactPoint{0.0};
    Vec3 contactNormal{0.0, 1.0, 0.0};
    Vec3 wheelCentre{0.0};
    Vec3 tyreForce{0.0}; // world-space longitudinal + lateral tyre force (N)
};

// Everything needed to build a car. The defaults describe an ~1250 kg rear-wheel
// drive sports car; only the tyres, the elastic element and the powertrain are
// expected to change from one car to another.
struct VehicleSetup {
    // --- Chassis ----------------------------------------------------------
    Real mass = 1250.0;                           // kg
    Vec3 principalInertia{1900.0, 2100.0, 500.0}; // pitch, yaw, roll (kg*m^2)
    Vec3 bodyHalfExtents{0.88, 0.35, 2.15};       // collision/visual box (m)

    // --- Geometry ---------------------------------------------------------
    Real frontAxleZ = -1.25; // m, body -Z is forward
    Real rearAxleZ = 1.35;
    Real halfTrack = 0.775;
    Real mountY = 0.0;      // strut top mount height above the CoM (m)
    Real restLength = 0.25; // strut length at the design ride height (m)

    // --- Suspension -------------------------------------------------------
    SpringDamperCoefficients frontSpring{};
    SpringDamperCoefficients rearSpring{};
    Real wheelRadius = 0.32;
    Real wheelInertia = 1.2; // wheel + tyre + brake disc (kg*m^2)

    // --- Tyres ------------------------------------------------------------
    MagicFormulaCoefficients tyre{};
    CombinedSlipModel combinedSlip = CombinedSlipModel::SlipVector;
    Real lowSpeedSpeed = 1.0; // slip denominators are clamped to this (m/s)

    // --- Steering ---------------------------------------------------------
    AckermannGeometry ackermann{}; // front track and wheelbase
    Real maxSteerAngle = 0.55;     // rad at a standstill
    Real minSteerAngle = 0.10;     // rad, floor of the speed-limited lock
    Real steerSpeedScale = 14.0;   // m/s
    Real steerInputRate = 3.0;     // input units/s towards full lock
    Real steerReturnRate = 5.0;    // input units/s back to centre

    // --- Powertrain (rear-wheel drive) ------------------------------------
    Engine engine{};
    FrictionClutch clutch{};
    Gearbox gearbox{};
    Real finalDrive = 3.6;
    DifferentialCoefficients differential{};
    bool automaticTransmission = true;
    Real upshiftSpeed = 650.0;   // engine rad/s
    Real downshiftSpeed = 250.0; // rad/s

    // --- Brakes -----------------------------------------------------------
    DiscBrake brake{};
    BrakeBias brakeBias{};
    Real maxBrakePressure = 4.0e6;  // Pa at full pedal
    Real handbrakePressure = 2.0e6; // Pa added to the rear line

    // --- Aerodynamics -----------------------------------------------------
    Real dragCoefficient = 0.30;
    Real liftCoefficient = 0.0; // + = downforce
    Real frontalArea = 2.0;
    Vec3 centreOfPressure{0.0, 0.0, 0.0};
    Real airDensity = 1.225;

    // --- Pedal shaping ----------------------------------------------------
    Real throttleRate = 4.0; // input units/s
    Real brakeRate = 6.0;
};

// The production vehicle model (PHYSICS_PLAN Phase 2, P2.13). It composes the
// tyre, suspension, steering, powertrain, brake, aerodynamic and mass components
// built in P2.1..P2.12 onto a single rigid chassis in a PhysicsWorld.
//
// Suspension kinematics are a raycast strut for now: each corner casts along the
// body's down axis, the strut length drives the spring/damper and the tyre sits
// on the surface normal. The double-wishbone kinematics (P2.5) plug in here
// later without changing this interface.
//
// The model borrows a PhysicsWorld and registers itself as a ForceElement; its
// apply() runs the whole vehicle once per fixed step. step() advances that world
// (the model drives the world it was spawned into), so the application must not
// also step the same world. Determinism requires the world's fixed step.
class VehicleModel final : public physics::ForceElement, public IVehicleModel {
public:
    // Creates the chassis body and the model element inside `world`, then places
    // the car at rest at the origin. The ground query must outlive the model.
    // Prefer spawn(); the constructor is public only so PhysicsWorld can build
    // the element and leaves the body unbound until spawn() wires it up.
    static VehicleModel& spawn(physics::PhysicsWorld& world, const VehicleSetup& setup, const IGroundQuery& ground);

    VehicleModel(physics::PhysicsWorld& world, const VehicleSetup& setup, const IGroundQuery& ground);

    // --- IVehicleModel ----------------------------------------------------
    void setInput(const VehicleInput& input) override { input_ = input; }
    const VehicleInput& input() const override { return input_; }

    void reset(const Vec3& position, Real headingRadians) override;
    bool step(Real deltaSeconds) override;

    Vec3 position() const override { return chassis().position; }
    Quat orientation() const override { return chassis().orientation; }
    Real heading() const override;
    Vec3 linearVelocity() const override { return chassis().linearVelocity; }
    Vec3 angularVelocity() const override { return chassis().angularVelocity; }
    Real forwardSpeed() const override;
    Vec3 forward() const override;
    Vec3 right() const override;

    // --- physics::ForceElement -------------------------------------------
    void apply() override;
    const char* typeName() const override { return "VehicleModel"; }

    // --- Telemetry --------------------------------------------------------
    physics::RigidBody& chassis() { return *bodyA; }
    const physics::RigidBody& chassis() const { return *bodyA; }

    const VehicleSetup& setup() const { return setup_; }
    const ITireModel& tyre() const { return tyre_; }

    const std::array<VehicleCorner, 4>& corners() const { return corners_; }
    const VehicleCorner& corner(std::size_t index) const { return corners_[index]; }

    Real engineSpeed() const { return engineSpeed_; }   // rad/s
    Real engineTorque() const { return engineTorque_; } // N*m at the crankshaft
    int gear() const { return gear_; }
    bool shifting() const { return shift_.shifting(); }
    Real steerAngle() const { return steerAngle_; } // rad, + = left
    Real throttle() const { return throttle_; }
    Real brake() const { return brake_; }
    Real clutchEngagement() const { return clutchEngagement_; }

    // Optional driver aids. The model samples them once per step and lets them
    // trim the throttle and add per-wheel brake torque; passing nullptr (the
    // default) leaves the driver's request untouched. The unit must outlive the
    // model and its own state is not owned here.
    void setElectronics(IVehicleElectronics* electronics) { electronics_ = electronics; }
    IVehicleElectronics* electronics() const { return electronics_; }

private:
    void buildCorners();
    void updateDriver(Real dt);
    void updateTransmission(Real dt);
    Real totalRatio() const;
    Real drivenWheelSpeed() const; // mean spin of the driven wheels (rad/s)

    physics::PhysicsWorld* world_ = nullptr;
    const IGroundQuery* ground_ = nullptr;
    IVehicleElectronics* electronics_ = nullptr;
    Real dt_ = 1.0 / 1000.0;
    Real gravity_ = 9.80665;

    VehicleSetup setup_;
    MagicFormulaTire tyre_;
    std::array<VehicleCorner, 4> corners_{};

    Engine engine_;
    FrictionClutch clutch_;
    SequentialShift shift_;

    VehicleInput input_{};
    Real throttle_ = 0.0;
    Real brake_ = 0.0;
    Real steerInput_ = 0.0;
    Real steerAngle_ = 0.0;
    int gear_ = 1;
    Real engineSpeed_ = 0.0;
    Real engineTorque_ = 0.0;
    Real clutchEngagement_ = 1.0;
    Real loadTorque_ = 0.0; // clutch torque fed back to the engine next step
    Real shiftTimer_ = 0.0;
    Real directionTimer_ = 0.0;
    BrakeModulation brakeModulation_{}; // per-wheel brake torque from the aids
};

// A well-tuned default car (six-speed, rear-wheel drive) built from the library
// defaults, so a test or a scene can spawn one with a single call.
VehicleSetup makeDefaultVehicleSetup();

} // namespace sim::vehicle
