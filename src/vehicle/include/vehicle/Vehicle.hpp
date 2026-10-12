// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <array>
#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;
using physics::Vec3;

struct Wheel
{
    Vec3 localPosition{0.0};
    Real radius = 0.34;
    Real steerAngle = 0.0;     // radians, front wheels only
    Real spinAngle = 0.0;      // radians, for visual rolling
    Real angularVelocity = 0.0;
    bool driven = false;
    bool steered = false;
};

struct VehicleConfig
{
    Real mass = 1250.0;               // kg
    Real wheelBase = 2.65;            // m
    Real trackWidth = 1.55;           // m
    Real wheelRadius = 0.34;          // m
    Real maxEngineTorque = 320.0;     // N*m at the wheels
    Real maxBrakeTorque = 1600.0;     // N*m
    Real maxSteerAngle = 0.61;        // radians (~35 degrees)
    Real steerRate = 3.0;             // rad/s
    Real dragCoefficient = 0.42;      // 0.5 * rho * Cd * A, divided by mass later
    Real rollingResistance = 0.015;
    Real engineInertia = 0.25;        // rotational inertia of the drivetrain
};

struct VehicleInput
{
    Real throttle = 0.0;  // [0, 1]
    Real brake = 0.0;     // [0, 1]
    Real steer = 0.0;     // [-1, 1], positive = left
    bool handbrake = false;
};

// A deterministic, kinematic bicycle model. It is intentionally simple: the
// goal of this milestone is a plausible, reproducible vehicle that the renderer
// and telemetry can consume, not a full tire model yet.
class Vehicle
{
public:
    explicit Vehicle(const VehicleConfig& config = {});

    void setInput(const VehicleInput& input) { input_ = input; }
    const VehicleInput& input() const { return input_; }

    void reset(const Vec3& position, Real headingRadians = 0.0);

    // Advances the vehicle by a fixed timestep. Returns false when dt <= 0.
    bool step(Real deltaSeconds);

    const Vec3& position() const { return position_; }
    Real heading() const { return heading_; }
    Real speed() const { return speed_; }
    Real engineRpm() const { return engineRpm_; }
    Real steerAngle() const { return steerAngle_; }

    const VehicleConfig& config() const { return config_; }
    const std::vector<Wheel>& wheels() const { return wheels_; }
    const Wheel& wheel(int index) const { return wheels_[static_cast<std::size_t>(index)]; }

    // Interpolated pose, for the renderer.
    Vec3 forward() const;
    Vec3 right() const;

private:
    void updateWheels(Real deltaSeconds);

    VehicleConfig config_;
    VehicleInput input_;
    std::vector<Wheel> wheels_;

    Vec3 position_{0.0};
    Real heading_ = 0.0;
    Real speed_ = 0.0;
    Real steerAngle_ = 0.0;
    Real engineRpm_ = 800.0;
};

} // namespace sim::vehicle
