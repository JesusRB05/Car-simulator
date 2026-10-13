// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <vector>

#include "physics/PhysicsTypes.hpp"
#include "vehicle/IVehicleModel.hpp"

namespace sim::vehicle {

using physics::Quat;
using physics::Real;
using physics::Vec3;

struct Wheel
{
    // Wheel centre in the body frame (+X right, +Y up, -Z forward).
    Vec3 localPosition{0.0};
    Real radius = 0.34;
    Real steerAngle = 0.0; // radians, front wheels only, + = left
    Real spinAngle = 0.0;  // radians, for visual rolling (+ = rolling forward, i.e. about body -X)
    Real angularVelocity = 0.0;
    bool driven = false;
    bool steered = false;
};

struct VehicleConfig
{
    Real mass = 1250.0;           // kg
    Real wheelBase = 2.65;        // m
    Real trackWidth = 1.55;       // m
    Real wheelRadius = 0.34;      // m
    Real maxEngineTorque = 320.0; // N*m at the wheels
    Real maxBrakeTorque = 1600.0; // N*m
    Real maxSteerAngle = 0.61;    // radians (~35 degrees)
    Real steerRate = 3.0;         // rad/s
    Real dragCoefficient = 0.42;  // 0.5 * rho * Cd * A, divided by mass later
    Real rollingResistance = 0.015;
    Real engineInertia = 0.25; // rotational inertia of the drivetrain
};

// TEMPORARY: deterministic kinematic bicycle model kept only so that the app,
// renderer and telemetry have something plausible to drive while the real
// vehicle model (tyres, suspension, drivetrain, ... behind the I* interfaces
// in this directory) is built. It will be replaced; do not extend it.
//
// Uses the project-wide frame: heading 0 faces -Z, positive heading/steer
// turns left (counter-clockwise seen from +Y).
class PlaceholderVehicle final : public IVehicleModel
{
public:
    explicit PlaceholderVehicle(const VehicleConfig& config = {});

    void setInput(const VehicleInput& input) override { input_ = input; }
    const VehicleInput& input() const override { return input_; }

    void reset(const Vec3& position, Real headingRadians = 0.0) override;

    // Advances the vehicle by a fixed timestep. Returns false when dt <= 0.
    bool step(Real deltaSeconds) override;

    Vec3 position() const override { return position_; }
    Quat orientation() const override;
    Real heading() const override { return heading_; }
    Vec3 linearVelocity() const override { return forward() * speed_; }
    Vec3 angularVelocity() const override { return Vec3(0.0, yawRate_, 0.0); }
    Real forwardSpeed() const override { return speed_; }
    Vec3 forward() const override;
    Vec3 right() const override;

    Real speed() const { return speed_; }
    Real engineRpm() const { return engineRpm_; }
    Real steerAngle() const { return steerAngle_; }

    const VehicleConfig& config() const { return config_; }
    const std::vector<Wheel>& wheels() const { return wheels_; }
    const Wheel& wheel(int index) const { return wheels_[static_cast<std::size_t>(index)]; }

private:
    void updateWheels(Real deltaSeconds);

    VehicleConfig config_;
    VehicleInput input_;
    std::vector<Wheel> wheels_;

    Vec3 position_{0.0};
    Real heading_ = 0.0;
    Real speed_ = 0.0;
    Real yawRate_ = 0.0;
    Real steerAngle_ = 0.0;
    Real engineRpm_ = 800.0;
};

} // namespace sim::vehicle
