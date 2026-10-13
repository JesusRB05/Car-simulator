// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/RaycastCar.hpp"

#include <algorithm>
#include <cmath>

#include "physics/Math.hpp"

namespace sim::app::drive {
namespace {

constexpr Real kRadPerSecToRpm = 60.0 / physics::kTwoPiPhysics;
constexpr Real kLaunchRpm = 3000.0; // simplified clutch: slip up to this with full throttle

// Exact ray/heightfield intersection: Newton iterations on
// f(t) = origin.y + t dir.y - h(origin.xz + t dir.xz). The ground is smooth and
// the ray is close to vertical, so a fixed number of iterations converges to
// machine precision (and keeps the step deterministic).
bool raycastGround(const GroundQuery& ground, const Vec3& origin, const Vec3& direction, Real maxLength,
                   Real& distance) {
    if (direction.y > -0.3) {
        return false; // car on its side or upside down: no wheel contact
    }
    Real t = (origin.y - ground.height(origin.x, origin.z)) / (-direction.y);
    for (int iteration = 0; iteration < 4; ++iteration) {
        const Vec3 p = origin + direction * t;
        const Real f = p.y - ground.height(p.x, p.z);
        const Vec3 n = ground.normal(p.x, p.z);
        const Real gradientX = -n.x / n.y;
        const Real gradientZ = -n.z / n.y;
        const Real derivative = direction.y - (gradientX * direction.x + gradientZ * direction.z);
        if (std::abs(derivative) < 1e-9) {
            break;
        }
        t -= f / derivative;
    }
    if (!(t <= maxLength)) {
        return false;
    }
    distance = std::max(t, 0.0);
    return true;
}

Real approach(Real current, Real target, Real maxDelta) {
    if (current < target) {
        return std::min(current + maxDelta, target);
    }
    return std::max(current - maxDelta, target);
}

// Coulomb-like brake: reduces |omega| by at most `delta` without crossing zero,
// so a brake can hold a wheel locked instead of chattering around zero.
Real applyBrake(Real omega, Real delta) {
    if (omega > 0.0) {
        return std::max(0.0, omega - delta);
    }
    return std::min(0.0, omega + delta);
}

Real wrapSpin(Real angle) {
    angle = std::fmod(angle, physics::kTwoPiPhysics);
    return angle < 0.0 ? angle + physics::kTwoPiPhysics : angle;
}

} // namespace

RaycastCar::RaycastCar(const CarParams& params, const GroundQuery& ground, Real fixedStep)
    : params_(params), ground_(&ground), dt_(fixedStep) {
    name = "raycast_car";
    const Real zs[4] = {params_.frontAxleZ, params_.frontAxleZ, params_.rearAxleZ, params_.rearAxleZ};
    const Real xs[4] = {-params_.halfTrack, params_.halfTrack, -params_.halfTrack, params_.halfTrack};
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
        wheels_[i].localMount = Vec3(xs[i], params_.mountY, zs[i]);
        wheels_[i].front = i < 2;
        wheels_[i].left = (i % 2) == 0;
    }
}

RaycastCar& RaycastCar::spawn(physics::PhysicsWorld& world, const CarParams& params, const GroundQuery& ground) {
    physics::RigidBody& chassis = world.createBody();
    chassis.name = "car";
    chassis.setType(physics::BodyType::Dynamic);
    chassis.shape = physics::Shape::box(params.bodyHalfExtents);
    chassis.setMass(params.mass);
    chassis.setInertiaPrincipal(params.inertia);
    chassis.updateCache();

    RaycastCar& car = world.createForceElement<RaycastCar>(&chassis, nullptr, params, ground, world.fixedStep);
    car.gravity_ = -world.gravity.y;
    car.reset(Vec3(0.0), 0.0);
    return car;
}

Real RaycastCar::staticRideHeight(Real gravity) const {
    const Real weight = params_.mass * gravity;
    const Real wheelbase = params_.wheelbase();
    const Real frontWheelLoad = 0.5 * weight * params_.rearAxleZ / wheelbase;
    const Real rearWheelLoad = 0.5 * weight * (-params_.frontAxleZ) / wheelbase;
    const Real full = params_.restLength + params_.wheelRadius - params_.mountY;
    const Real frontHeight = full - frontWheelLoad / params_.springStiffness;
    const Real rearHeight = full - rearWheelLoad / params_.springStiffness;
    // The CoM (z = 0) sits between the axles at -frontAxleZ / wheelbase from the front.
    return frontHeight + (rearHeight - frontHeight) * (-params_.frontAxleZ / wheelbase);
}

void RaycastCar::reset(const Vec3& groundPosition, Real heading) {
    physics::RigidBody& body = *bodyA;
    body.position = groundPosition + Vec3(0.0, staticRideHeight(gravity_), 0.0);
    body.orientation = physics::quaternionFromAxisAngle(Vec3(0.0, 1.0, 0.0), heading);
    body.linearVelocity = Vec3(0.0);
    body.angularVelocity = Vec3(0.0);
    body.clearAccumulators();
    body.updateCache();

    for (WheelState& wheel : wheels_) {
        wheel.omega = 0.0;
        wheel.spinAngle = 0.0;
        wheel.contact = false;
        wheel.load = 0.0;
        wheel.springForce = 0.0;
        wheel.slipRatio = 0.0;
        wheel.slipAngle = 0.0;
        wheel.longitudinalForce = 0.0;
        wheel.lateralForce = 0.0;
        wheel.tyreForce = Vec3(0.0);
        wheel.mountWorld = body.localToWorldPoint(wheel.localMount);
        wheel.wheelCentre = wheel.mountWorld - body.localToWorldDirection(Vec3(0.0, 1.0, 0.0)) * params_.restLength;
        wheel.contactPoint = wheel.wheelCentre;
    }
    throttle_ = 0.0;
    brake_ = 0.0;
    steerInput_ = 0.0;
    steerAngle_ = 0.0;
    gear_ = 1;
    shiftTimer_ = 0.0;
    directionTimer_ = 0.0;
    engineRpm_ = params_.idleRpm;
    tractionCut_ = 0.0;
}

Real RaycastCar::forwardSpeed() const {
    const physics::RigidBody& body = *bodyA;
    return glm::dot(body.linearVelocity, body.localToWorldDirection(Vec3(0.0, 0.0, -1.0)));
}

Real RaycastCar::speed() const {
    return glm::length(bodyA->linearVelocity);
}

Real RaycastCar::engineTorque(Real rpm, Real throttle) const {
    const std::vector<Real>& xs = params_.torqueCurveRpm;
    const std::vector<Real>& ys = params_.torqueCurveNm;
    Real full = 0.0;
    if (!xs.empty() && xs.size() == ys.size()) {
        if (rpm <= xs.front()) {
            full = ys.front();
        } else if (rpm >= xs.back()) {
            full = ys.back();
        } else {
            for (std::size_t i = 1; i < xs.size(); ++i) {
                if (rpm <= xs[i]) {
                    const Real t = (rpm - xs[i - 1]) / (xs[i] - xs[i - 1]);
                    full = ys[i - 1] + (ys[i] - ys[i - 1]) * t;
                    break;
                }
            }
        }
    }
    if (rpm >= params_.maxRpm) {
        full = 0.0; // rev limiter: fuel cut
    }
    const Real drag = params_.engineBrakeTorque * std::clamp(rpm / params_.maxRpm, 0.0, 1.2);
    return throttle * full - (1.0 - throttle) * drag;
}

void RaycastCar::updateDriver(Real dt) {
    const Real v = forwardSpeed();

    // Arcade direction logic: holding brake while (almost) stopped engages
    // reverse; holding throttle while (almost) stopped in reverse engages first.
    const bool reverse = gear_ < 0;
    if (!reverse && input_.brake > 0.5 && input_.throttle < 0.1 && v < 0.5) {
        directionTimer_ += dt;
        if (directionTimer_ > 0.3) {
            gear_ = -1;
            directionTimer_ = 0.0;
            shiftTimer_ = 0.0;
        }
    } else if (reverse && input_.throttle > 0.5 && input_.brake < 0.1 && v > -0.5) {
        directionTimer_ += dt;
        if (directionTimer_ > 0.3) {
            gear_ = 1;
            directionTimer_ = 0.0;
            shiftTimer_ = 0.0;
        }
    } else {
        directionTimer_ = 0.0;
    }

    const bool reversing = gear_ < 0;
    const Real throttleTarget = std::clamp(reversing ? input_.brake : input_.throttle, 0.0, 1.0);
    const Real brakeTarget = std::clamp(reversing ? input_.throttle : input_.brake, 0.0, 1.0);
    throttle_ =
        approach(throttle_, throttleTarget, (throttleTarget > throttle_ ? 1.0 : 2.0) * params_.throttleRate * dt);
    brake_ = approach(brake_, brakeTarget, (brakeTarget > brake_ ? 1.0 : 2.0) * params_.brakeRate * dt);

    const Real steerTarget = std::clamp(input_.steer, -1.0, 1.0);
    const bool returning = std::abs(steerTarget) < std::abs(steerInput_) || steerTarget * steerInput_ < 0.0;
    steerInput_ = approach(steerInput_, steerTarget, (returning ? params_.steerReturnRate : params_.steerRate) * dt);
    const Real limit =
        std::max(params_.minSteerAngle, params_.maxSteerAngle / (1.0 + std::abs(v) / params_.steerSpeedScale));
    steerAngle_ = steerInput_ * limit;
}

void RaycastCar::updateGearbox(Real dt) {
    shiftTimer_ = std::max(0.0, shiftTimer_ - dt);
    if (gear_ < 1 || shiftTimer_ > 0.0) {
        return;
    }
    // Shift points use the road speed, not the driven wheels, so wheelspin
    // does not trigger an upshift.
    const Real ratio = params_.gearRatios[static_cast<std::size_t>(gear_ - 1)] * params_.finalDrive;
    const Real wheelRpm = forwardSpeed() / params_.wheelRadius * ratio * kRadPerSecToRpm;
    const int topGear = static_cast<int>(params_.gearRatios.size());
    if (wheelRpm > params_.upshiftRpm && gear_ < topGear) {
        ++gear_;
        shiftTimer_ = params_.shiftTime;
    } else if (wheelRpm < params_.downshiftRpm && gear_ > 1) {
        --gear_;
        shiftTimer_ = 0.6 * params_.shiftTime;
    }
}

void RaycastCar::apply() {
    physics::RigidBody& body = *bodyA;
    const Real dt = dt_;
    const CarParams& p = params_;

    updateDriver(dt);
    updateGearbox(dt);

    const Vec3 up = body.localToWorldDirection(Vec3(0.0, 1.0, 0.0));
    const Vec3 forward = body.localToWorldDirection(Vec3(0.0, 0.0, -1.0));
    const Vec3 down = -up;
    const Real suspensionLength = p.restLength + p.wheelRadius;

    // --- 1. Suspension raycasts and spring/damper forces -------------------
    std::array<Real, 4> suspensionForce{};
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
        WheelState& wheel = wheels_[i];
        wheel.mountWorld = body.localToWorldPoint(wheel.localMount);
        Real distance = 0.0;
        wheel.contact = raycastGround(*ground_, wheel.mountWorld, down, suspensionLength, distance);
        if (!wheel.contact) {
            wheel.compression = 0.0;
            wheel.springForce = 0.0;
            wheel.wheelCentre = wheel.mountWorld + down * p.restLength;
            wheel.contactPoint = wheel.wheelCentre + down * p.wheelRadius;
            continue;
        }
        wheel.compression = suspensionLength - distance;
        wheel.contactPoint = wheel.mountWorld + down * distance;
        wheel.contactNormal = ground_->normal(wheel.contactPoint.x, wheel.contactPoint.z);
        wheel.wheelCentre = wheel.contactPoint + up * p.wheelRadius;

        // The ground is static, so the compression rate is the mount velocity
        // along the suspension axis.
        const Real compressionRate = glm::dot(body.velocityAtWorldPoint(wheel.mountWorld), down);
        Real spring = p.springStiffness * wheel.compression;
        if (wheel.compression > p.maxCompression) {
            spring += p.bumpStopStiffness * (wheel.compression - p.maxCompression);
        }
        wheel.springForce = spring;
        const Real damper = compressionRate > 0.0 ? p.damperBump * compressionRate : p.damperRebound * compressionRate;
        suspensionForce[i] = spring + damper;
    }

    // Anti-roll bars transfer load from the extended to the compressed side.
    const Real frontRoll = p.antiRollFront * (wheels_[kFrontLeft].compression - wheels_[kFrontRight].compression);
    const Real rearRoll = p.antiRollRear * (wheels_[kRearLeft].compression - wheels_[kRearRight].compression);
    suspensionForce[kFrontLeft] += frontRoll;
    suspensionForce[kFrontRight] -= frontRoll;
    suspensionForce[kRearLeft] += rearRoll;
    suspensionForce[kRearRight] -= rearRoll;

    for (std::size_t i = 0; i < wheels_.size(); ++i) {
        WheelState& wheel = wheels_[i];
        wheel.load = wheel.contact ? std::max(0.0, suspensionForce[i]) : 0.0;
        if (wheel.load > 0.0) {
            // The ground reacts along its normal; the strut itself is internal
            // (applying the load along the tilted body axis would make a
            // pitched car creep on flat ground).
            body.applyForceAtWorldPoint(wheel.contactNormal * wheel.load, wheel.contactPoint);
        }
    }

    // --- 2. Drivetrain: engine torque through the gearbox to the rear axle --
    const Real gearRatio = gear_ > 0 ? p.gearRatios[static_cast<std::size_t>(gear_ - 1)] : -p.reverseRatio;
    const Real totalRatio = gearRatio * p.finalDrive;
    const Real rearOmega = 0.5 * (wheels_[kRearLeft].omega + wheels_[kRearRight].omega);
    const Real wheelRpm = rearOmega * totalRatio * kRadPerSecToRpm;
    // Simplified clutch: below the launch speed the engine slips the clutch at
    // a throttle-dependent rpm instead of stalling.
    const Real clutchRpm = p.idleRpm + throttle_ * (kLaunchRpm - p.idleRpm);
    engineRpm_ = std::clamp(std::max(wheelRpm, clutchRpm), p.idleRpm, 1.05 * p.maxRpm);
    Real engine = engineTorque(engineRpm_, throttle_);
    if (wheelRpm < 1.1 * p.idleRpm && engine < 0.0) {
        engine = 0.0; // clutch open: no engine braking at crawling speed
    }
    if (shifting()) {
        engine = 0.0;
    }
    if (p.tractionControl && engine > 0.0) {
        // Proportional torque cut above the peak slip ratio of the driven wheels.
        const Real slip =
            std::max(wheels_[kRearLeft].slipRatio, wheels_[kRearRight].slipRatio) * (gear_ < 0 ? -1.0 : 1.0);
        const Real excess = (slip - p.peakSlipRatio) / p.peakSlipRatio;
        tractionCut_ = std::clamp(excess, 0.0, 1.0);
        engine *= 1.0 - tractionCut_;
    } else {
        tractionCut_ = 0.0;
    }
    const Real axleTorque = engine * totalRatio * p.drivetrainEfficiency;

    // --- 3. Brakes ---------------------------------------------------------
    const Real brakeTotal = brake_ * p.maxBrakeTorque;
    const Real frontBrake = 0.5 * brakeTotal * p.brakeBias;
    const Real rearBrake = 0.5 * brakeTotal * (1.0 - p.brakeBias) + (input_.handbrake ? p.handbrakeTorque : 0.0);

    // --- 4. Steering with Ackermann geometry --------------------------------
    std::array<Real, 4> steer{};
    if (std::abs(steerAngle_) > 1e-6) {
        const Real wheelbase = p.wheelbase();
        const Real radius = wheelbase / std::tan(std::abs(steerAngle_));
        const Real inner = std::atan(wheelbase / std::max(radius - p.halfTrack, 1e-3));
        const Real outer = std::atan(wheelbase / (radius + p.halfTrack));
        const Real sign = steerAngle_ > 0.0 ? 1.0 : -1.0; // > 0: left turn, left wheel inside
        steer[kFrontLeft] = sign * (steerAngle_ > 0.0 ? inner : outer);
        steer[kFrontRight] = sign * (steerAngle_ > 0.0 ? outer : inner);
    }

    // --- 5. Tyres (combined slip, friction ellipse) and wheel spin ----------
    const Real shape = p.tyreShape;
    const Real stiffnessB = std::tan(physics::kPiPhysics / (2.0 * shape)); // peak at normalised slip 1
    const int substeps = std::max(1, p.wheelSubsteps);
    const Real h = dt / static_cast<Real>(substeps);

    for (std::size_t i = 0; i < wheels_.size(); ++i) {
        WheelState& wheel = wheels_[i];
        wheel.steerAngle = steer[i];
        wheel.driveTorque = wheel.front ? 0.0 : 0.5 * axleTorque;
        wheel.brakeTorque = wheel.front ? frontBrake : rearBrake;

        if (!wheel.contact || wheel.load <= 0.0) {
            wheel.omega += dt * wheel.driveTorque / p.wheelInertia;
            wheel.omega = applyBrake(wheel.omega, dt * (wheel.brakeTorque + 2.0) / p.wheelInertia);
            wheel.slipRatio = 0.0;
            wheel.slipAngle = 0.0;
            wheel.longitudinalForce = 0.0;
            wheel.lateralForce = 0.0;
            wheel.tyreForce = Vec3(0.0);
            wheel.spinAngle = wrapSpin(wheel.spinAngle + wheel.omega * dt);
            continue;
        }

        // Wheel heading: chassis forward rotated by the steer angle about the
        // suspension axis, then projected on the ground plane.
        const Real c = std::cos(wheel.steerAngle);
        const Real s = std::sin(wheel.steerAngle);
        const Vec3 wheelForward = forward * c + glm::cross(up, forward) * s;
        const Vec3& n = wheel.contactNormal;
        const Vec3 tangent = physics::safeNormalize(wheelForward - n * glm::dot(wheelForward, n), forward);
        const Vec3 lateral = glm::cross(tangent, n); // to the wheel's right

        const Vec3 contactVelocity = body.velocityAtWorldPoint(wheel.contactPoint);
        const Real vx = glm::dot(contactVelocity, tangent);
        const Real vy = glm::dot(contactVelocity, lateral);
        const Real denominator = std::max(std::abs(vx), p.lowSpeedVelocity);
        const Real tanAlpha = vy / denominator;

        const GroundSample surface = ground_->sample(wheel.contactPoint.x, wheel.contactPoint.z);
        wheel.surface = surface.surface;
        const Real peakForce = surface.friction * wheel.load;

        // The spin degree of freedom is stiff at low speed (the slip ratio
        // divides by the clamped speed), so it is substepped with the chassis
        // velocity frozen; the chassis receives the mean tyre force.
        Real sumFx = 0.0;
        Real sumFy = 0.0;
        Real kappa = 0.0;
        for (int k = 0; k < substeps; ++k) {
            kappa = (wheel.omega * p.wheelRadius - vx) / denominator;
            const Real sx = kappa / p.peakSlipRatio;
            const Real sy = tanAlpha / p.peakSlipAngle;
            const Real combined = std::sqrt(sx * sx + sy * sy);
            Real fx = 0.0;
            Real fy = 0.0;
            if (combined > 1e-12) {
                const Real force = peakForce * std::sin(shape * std::atan(stiffnessB * combined));
                fx = force * sx / combined;
                fy = -force * sy / combined;
            }
            wheel.omega += h * (wheel.driveTorque - fx * p.wheelRadius) / p.wheelInertia;
            wheel.omega = applyBrake(wheel.omega, h * wheel.brakeTorque / p.wheelInertia);
            sumFx += fx;
            sumFy += fy;
        }
        const Real fx = sumFx / static_cast<Real>(substeps);
        const Real fy = sumFy / static_cast<Real>(substeps);
        const Real rolling = -surface.rollingResistance * wheel.load * std::clamp(vx / 0.5, -1.0, 1.0);

        wheel.slipRatio = kappa;
        wheel.slipAngle = std::atan(tanAlpha);
        wheel.longitudinalForce = fx + rolling;
        wheel.lateralForce = fy;
        wheel.tyreForce = tangent * wheel.longitudinalForce + lateral * fy;
        body.applyForceAtWorldPoint(wheel.tyreForce, wheel.contactPoint);
        wheel.spinAngle = wrapSpin(wheel.spinAngle + wheel.omega * dt);
    }

    // --- 6. Aerodynamic drag ------------------------------------------------
    const Real airspeed = glm::length(body.linearVelocity);
    body.applyCentralForce(-p.dragFactor * airspeed * body.linearVelocity);
}

} // namespace sim::app::drive
