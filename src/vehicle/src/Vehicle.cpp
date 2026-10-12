// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/Vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace sim::vehicle {
namespace {

constexpr Real kIdleRpm = 800.0;
constexpr Real kRedlineRpm = 7000.0;
constexpr Real kPi = 3.14159265358979323846;

} // namespace

Vehicle::Vehicle(const VehicleConfig& config) : config_(config)
{
    const Real halfTrack = config_.trackWidth * 0.5;
    const Real halfWheelBase = config_.wheelBase * 0.5;

    Wheel frontLeft;
    frontLeft.localPosition = {-halfTrack, config_.wheelRadius, -halfWheelBase};
    frontLeft.radius = config_.wheelRadius;
    frontLeft.steered = true;

    Wheel frontRight = frontLeft;
    frontRight.localPosition = {halfTrack, config_.wheelRadius, -halfWheelBase};

    Wheel rearLeft;
    rearLeft.localPosition = {-halfTrack, config_.wheelRadius, halfWheelBase};
    rearLeft.radius = config_.wheelRadius;
    rearLeft.driven = true;

    Wheel rearRight = rearLeft;
    rearRight.localPosition = {halfTrack, config_.wheelRadius, halfWheelBase};

    wheels_ = {frontLeft, frontRight, rearLeft, rearRight};
}

void Vehicle::reset(const Vec3& position, Real headingRadians)
{
    position_ = position;
    heading_ = headingRadians;
    speed_ = 0.0;
    steerAngle_ = 0.0;
    engineRpm_ = kIdleRpm;
    for (Wheel& wheel : wheels_)
    {
        wheel.steerAngle = 0.0;
        wheel.spinAngle = 0.0;
        wheel.angularVelocity = 0.0;
    }
}

bool Vehicle::step(Real deltaSeconds)
{
    if (deltaSeconds <= 0.0)
    {
        return false;
    }

    // Steering is rate-limited towards the requested angle.
    const Real targetSteer = std::clamp(input_.steer, -1.0, 1.0) * config_.maxSteerAngle;
    const Real maxDelta = config_.steerRate * deltaSeconds;
    steerAngle_ += std::clamp(targetSteer - steerAngle_, -maxDelta, maxDelta);

    // Longitudinal forces.
    const Real driveForce = std::clamp(input_.throttle, 0.0, 1.0) * config_.maxEngineTorque / config_.wheelRadius;
    const Real brakeForce = std::clamp(input_.brake, 0.0, 1.0) * config_.maxBrakeTorque / config_.wheelRadius;
    const Real dragForce = config_.dragCoefficient * speed_ * std::abs(speed_);
    const Real rollingForce = config_.rollingResistance * config_.mass * 9.81 * (speed_ > 0.0 ? 1.0 : (speed_ < 0.0 ? -1.0 : 0.0));

    Real netForce = driveForce - dragForce - rollingForce;

    if (input_.handbrake)
    {
        netForce -= std::clamp(speed_ * config_.mass / deltaSeconds, -2.0 * brakeForce, 2.0 * brakeForce);
    }

    const Real acceleration = netForce / config_.mass;
    speed_ += acceleration * deltaSeconds;

    // Braking cannot reverse the direction of travel.
    if (brakeForce > 0.0)
    {
        const Real brakeDeceleration = brakeForce / config_.mass * deltaSeconds;
        if (std::abs(speed_) <= brakeDeceleration)
        {
            speed_ = 0.0;
        }
        else
        {
            speed_ -= std::copysign(brakeDeceleration, speed_);
        }
    }

    // Kinematic bicycle yaw rate.
    const Real yawRate = speed_ / config_.wheelBase * std::tan(steerAngle_);
    heading_ += yawRate * deltaSeconds;

    const Vec3 forwardVec(std::sin(heading_), 0.0, std::cos(heading_));
    position_ += forwardVec * speed_ * deltaSeconds;

    engineRpm_ = std::clamp(kIdleRpm + std::abs(speed_) * 180.0, kIdleRpm, kRedlineRpm);

    updateWheels(deltaSeconds);
    return true;
}

void Vehicle::updateWheels(Real deltaSeconds)
{
    for (Wheel& wheel : wheels_)
    {
        wheel.steerAngle = wheel.steered ? steerAngle_ : 0.0;
        wheel.angularVelocity = speed_ / wheel.radius;
        wheel.spinAngle = std::fmod(wheel.spinAngle + wheel.angularVelocity * deltaSeconds, 2.0 * kPi);
    }
}

Vec3 Vehicle::forward() const
{
    return Vec3(std::sin(heading_), 0.0, std::cos(heading_));
}

Vec3 Vehicle::right() const
{
    return Vec3(std::cos(heading_), 0.0, -std::sin(heading_));
}

} // namespace sim::vehicle
