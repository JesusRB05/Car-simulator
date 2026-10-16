// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/VehicleModel.hpp"

#include <algorithm>
#include <cmath>

#include "physics/Math.hpp"

namespace sim::vehicle {
namespace {

// Newton iteration for the ray against the smooth height field, identical in
// spirit to the app's raycast wheels.
bool raycastGround(const IGroundQuery& ground, const Vec3& origin, const Vec3& down, Real maxDistance, Real& distance,
                   GroundSample& sample) {
    if (down.y > -0.3) {
        return false; // the car is on its side or upside down
    }
    GroundSample s = ground.sample(origin.x, origin.z);
    Real t = (origin.y - s.height) / (-down.y);
    for (int iteration = 0; iteration < 4; ++iteration) {
        const Vec3 p = origin + down * t;
        s = ground.sample(p.x, p.z);
        const Vec3& n = s.normal;
        const Real gradientX = -n.x / n.y;
        const Real gradientZ = -n.z / n.y;
        const Real derivative = down.y - (gradientX * down.x + gradientZ * down.z);
        if (std::abs(derivative) < 1e-9) {
            break;
        }
        t -= (p.y - s.height) / derivative;
    }
    if (!(t <= maxDistance) || t < 0.0) {
        return false;
    }
    distance = t;
    const Vec3 hit = origin + down * t;
    sample = ground.sample(hit.x, hit.z);
    return true;
}

Real approach(Real current, Real target, Real maxDelta) {
    if (current < target) {
        return std::min(current + maxDelta, target);
    }
    return std::max(current - maxDelta, target);
}

Real wrapSpin(Real angle) {
    angle = std::fmod(angle, physics::kTwoPiPhysics);
    return angle < 0.0 ? angle + physics::kTwoPiPhysics : angle;
}

} // namespace

VehicleModel::VehicleModel(physics::PhysicsWorld& world, const VehicleSetup& setup, const IGroundQuery& ground)
    : world_(&world), ground_(&ground), dt_(world.fixedStep), setup_(setup), tyre_(setup.tyre) {
    name = "vehicle";
    tyre_.setCombinedSlipModel(setup_.combinedSlip);
    engine_ = setup_.engine;
    clutch_ = setup_.clutch;
    buildCorners();
}

VehicleModel& VehicleModel::spawn(physics::PhysicsWorld& world, const VehicleSetup& setup, const IGroundQuery& ground) {
    physics::RigidBody& body = world.createBody();
    body.name = "chassis";
    body.setType(physics::BodyType::Dynamic);
    body.shape = physics::Shape::box(setup.bodyHalfExtents);
    body.collisionEnabled = true;
    body.setMass(setup.mass);
    body.setInertiaPrincipal(setup.principalInertia);
    body.updateCache();

    VehicleModel& model = world.createForceElement<VehicleModel>(&body, nullptr, world, setup, ground);
    model.gravity_ = -world.gravity.y;
    model.reset(Vec3(0.0), 0.0);
    return model;
}

void VehicleModel::buildCorners() {
    const Real zs[4] = {setup_.frontAxleZ, setup_.frontAxleZ, setup_.rearAxleZ, setup_.rearAxleZ};
    const Real xs[4] = {-setup_.halfTrack, setup_.halfTrack, -setup_.halfTrack, setup_.halfTrack};
    for (std::size_t i = 0; i < corners_.size(); ++i) {
        VehicleCorner& corner = corners_[i];
        corner.localMount = Vec3(xs[i], setup_.mountY, zs[i]);
        corner.front = i < 2;
        corner.left = (i % 2) == 0;
        corner.steered = corner.front;
        corner.driven = !corner.front; // rear-wheel drive
        corner.wheelRadius = setup_.wheelRadius;
        corner.wheelInertia = setup_.wheelInertia;
        corner.spring = corner.front ? setup_.frontSpring : setup_.rearSpring;
    }
}

void VehicleModel::reset(const Vec3& position, Real headingRadians) {
    physics::RigidBody& body = *bodyA;
    const Real gravity = gravity_;
    const Real wheelbase = setup_.rearAxleZ - setup_.frontAxleZ;
    const Real frontShare = setup_.rearAxleZ / wheelbase;
    const Real frontLoad = 0.5 * setup_.mass * gravity * frontShare;
    const Real rearLoad = 0.5 * setup_.mass * gravity * (1.0 - frontShare);

    // Static equilibrium height: the strut length at which the spring carries the
    // corner load, so the car settles without a bounce.
    const SpringDamperCoefficients& front = setup_.frontSpring;
    const SpringDamperCoefficients& rear = setup_.rearSpring;
    const Real frontStrut = front.restLength + (front.preload - frontLoad) / std::max(front.stiffness, 1e-6);
    const Real rearStrut = rear.restLength + (rear.preload - rearLoad) / std::max(rear.stiffness, 1e-6);
    const Real frontDistance = frontStrut + setup_.wheelRadius;
    const Real rearDistance = rearStrut + setup_.wheelRadius;

    const GroundSample ground = ground_->sample(position.x, position.z);
    const Real bodyHeight = ground.height + 0.5 * (frontDistance + rearDistance) - setup_.mountY;

    body.position = Vec3(position.x, bodyHeight, position.z);
    body.orientation = physics::quaternionFromAxisAngle(Vec3(0.0, 1.0, 0.0), headingRadians);
    body.linearVelocity = Vec3(0.0);
    body.angularVelocity = Vec3(0.0);
    body.clearAccumulators();
    body.updateCache();

    const Vec3 up = body.localToWorldDirection(Vec3(0.0, 1.0, 0.0));
    for (VehicleCorner& corner : corners_) {
        corner.spinWheel.spin = 0.0;
        corner.spinWheel.locked = false;
        corner.spinAngle = 0.0;
        corner.contact = false;
        corner.compression = 0.0;
        corner.load = 0.0;
        corner.steerAngle = 0.0;
        corner.slipRatio = 0.0;
        corner.slipAngle = 0.0;
        corner.longitudinalForce = 0.0;
        corner.lateralForce = 0.0;
        corner.driveTorque = 0.0;
        corner.brakeTorque = 0.0;
        corner.mountWorld = body.localToWorldPoint(corner.localMount);
        corner.contactPoint = body.localToWorldPoint(corner.localMount) - up * setup_.restLength;
        corner.contactNormal = ground.normal;
        corner.wheelCentre = corner.contactPoint;
        corner.tyreForce = Vec3(0.0);
    }

    engine_.reset();
    engineSpeed_ = engine_.angularVelocity();
    engineTorque_ = 0.0;
    loadTorque_ = 0.0;
    clutchEngagement_ = 1.0;
    drivenInertiaReflect_ = 0.0;
    shift_ = SequentialShift{};
    gear_ = 1;
    shiftTimer_ = 0.0;
    directionTimer_ = 0.0;
    throttle_ = 0.0;
    brake_ = 0.0;
    steerInput_ = 0.0;
    steerAngle_ = 0.0;
}

bool VehicleModel::step(Real deltaSeconds) {
    if (deltaSeconds <= 0.0) {
        return false;
    }
    // The model owns the timeline of the world it was spawned into; the
    // application must not step that world as well.
    world_->step(deltaSeconds);
    return true;
}

Real VehicleModel::heading() const {
    const Vec3 f = forward();
    return std::atan2(-f.x, -f.z);
}

Real VehicleModel::forwardSpeed() const {
    return glm::dot(chassis().linearVelocity, forward());
}

Vec3 VehicleModel::forward() const {
    return chassis().localToWorldDirection(Vec3(0.0, 0.0, -1.0));
}

Vec3 VehicleModel::right() const {
    return chassis().localToWorldDirection(Vec3(1.0, 0.0, 0.0));
}

Real VehicleModel::totalRatio() const {
    return setup_.gearbox.ratio(gear_) * setup_.finalDrive;
}

Real VehicleModel::drivenWheelSpeed() const {
    Real sum = 0.0;
    int count = 0;
    for (const VehicleCorner& corner : corners_) {
        if (corner.driven) {
            sum += corner.spinWheel.spin;
            ++count;
        }
    }
    return count > 0 ? sum / static_cast<Real>(count) : 0.0;
}

void VehicleModel::updateDriver(Real dt) {
    const Real v = forwardSpeed();

    // Direction logic: holding the brake while stopped selects reverse, holding
    // the throttle while stopped in reverse selects first, like the app.
    const bool reverse = gear_ < 0;
    if (!reverse && input_.brake > 0.5 && input_.throttle < 0.1 && v < 0.5) {
        directionTimer_ += dt;
        if (directionTimer_ > 0.3) {
            gear_ = -1;
            directionTimer_ = 0.0;
        }
    } else if (reverse && input_.throttle > 0.5 && input_.brake < 0.1 && v > -0.5) {
        directionTimer_ += dt;
        if (directionTimer_ > 0.3) {
            gear_ = 1;
            directionTimer_ = 0.0;
        }
    } else {
        directionTimer_ = 0.0;
    }

    pedalThrottle_ = approach(pedalThrottle_, std::clamp(input_.throttle, 0.0, 1.0), setup_.throttleRate * dt);
    pedalBrake_ = approach(pedalBrake_, std::clamp(input_.brake, 0.0, 1.0), setup_.brakeRate * dt);
    throttle_ = pedalThrottle_;
    brake_ = pedalBrake_;

    const bool backToCentre = std::abs(input_.steer) < std::abs(steerInput_);
    steerInput_ = approach(steerInput_, std::clamp(input_.steer, -1.0, 1.0),
                           (backToCentre ? setup_.steerReturnRate : setup_.steerInputRate) * dt);
    const Real limit =
        std::max(setup_.minSteerAngle, setup_.maxSteerAngle / (1.0 + std::abs(v) / setup_.steerSpeedScale));
    steerAngle_ = steerInput_ * limit;
}

void VehicleModel::updateTransmission(Real dt) {
    if (setup_.automaticTransmission && !shift_.shifting() && gear_ > 0) {
        const int top = static_cast<int>(setup_.gearbox.forwardRatios.size());
        if (engineSpeed_ > setup_.upshiftSpeed && gear_ < top) {
            ++gear_;
            shift_.request(gear_);
        } else if (engineSpeed_ < setup_.downshiftSpeed && gear_ > 1) {
            --gear_;
            shift_.request(gear_);
        }
    }
    shift_.step(dt);

    const Real ratio = totalRatio();
    const Real inputOmega = drivenWheelSpeed() * ratio;
    const Real idle = engine_.config.idleSpeed;

    // Clutch engagement closes as the input shaft catches up; a throttle launch
    // map lets the pedal feed torque into a stationary driveline so the engine
    // slips instead of stalling.
    Real engagement = std::clamp(std::abs(inputOmega) / idle, 0.0, 1.0);
    if (throttle_ > 0.0) {
        engagement = std::max(engagement, std::min(throttle_ * 0.5, 0.5));
    }
    clutchEngagement_ = (ratio == 0.0) ? 0.0 : engagement;

    Real inputTorque = 0.0;
    drivenInertiaReflect_ = 0.0;
    if (ratio == 0.0) {
        // Neutral: the engine turns freely against the clutch as no load.
        engineTorque_ = engine_.step(dt, throttle_, 0.0);
        engineSpeed_ = engine_.angularVelocity();
        loadTorque_ = 0.0;
    } else {
        // Couple the engine to the input shaft implicitly (PHYSICS_PLAN 7.6). The
        // engine first responds freely to its map torque and friction; the clutch
        // then carries exactly the torque that brings its speed onto the driveline
        // this step, clamped to what the plates can hold. Below the capacity the
        // clutch is locked and the engine follows the wheels (delivering its net
        // torque, so it brakes the car when coasting); above it the clutch slips at
        // that limit during a launch.
        const Real engineInertia = engine_.config.inertia;
        engineTorque_ = engine_.step(dt, throttle_, 0.0);
        const Real freeSpeed = engine_.angularVelocity();

        FrictionClutch clutch = clutch_;
        clutch.engagement = clutchEngagement_;
        const Real capacity = clutch.capacity();

        const Real required = engineInertia * (freeSpeed - inputOmega) / dt;
        Real transmitted = std::clamp(required, -capacity, capacity);
        engine_.setAngularVelocity(freeSpeed - transmitted * dt / engineInertia);
        engineSpeed_ = engine_.angularVelocity();

        inputTorque = transmitted * shift_.torqueFactor();
        loadTorque_ = transmitted;

        // When the clutch holds and the driver is off the power (braking, coasting or
        // overrun) the engine is rigidly tied to the driven wheels, so fold its
        // inertia into theirs for the wheel-spin solve. Without this the stiff
        // engine-to-shaft link amplifies each step of wheel-speed jitter into a
        // capacity-limited bang-bang against the brakes, which is the chatter. Under
        // power the reflection is left off so a standing launch (and the traction
        // control that tames it) is untouched.
        if (std::abs(required) <= capacity && (throttle_ <= 0.05 || brake_ > 0.05)) {
            std::size_t driven = 0;
            for (const VehicleCorner& corner : corners_) {
                if (corner.driven) {
                    ++driven;
                }
            }
            if (driven > 0) {
                drivenInertiaReflect_ = engineInertia * ratio * ratio / static_cast<Real>(driven);
            }
        }
    }

    const Real axleTorque = setup_.gearbox.outputTorque(gear_, inputTorque) * setup_.finalDrive;

    DifferentialInput differentialInput;
    differentialInput.inputTorque = axleTorque;
    bool haveLeft = false;
    bool haveRight = false;
    for (const VehicleCorner& corner : corners_) {
        if (corner.driven && corner.left) {
            differentialInput.leftAngularVelocity = corner.spinWheel.spin;
            haveLeft = true;
        }
        if (corner.driven && !corner.left) {
            differentialInput.rightAngularVelocity = corner.spinWheel.spin;
            haveRight = true;
        }
    }
    const DifferentialOutput differential = computeDifferentialTorques(setup_.differential, differentialInput);

    for (VehicleCorner& corner : corners_) {
        corner.driveTorque = 0.0;
        if (corner.driven && corner.left && haveLeft) {
            corner.driveTorque = differential.leftTorque;
        } else if (corner.driven && !corner.left && haveRight) {
            corner.driveTorque = differential.rightTorque;
        }
    }
}

void VehicleModel::apply() {
    physics::RigidBody& body = *bodyA;
    const Real dt = dt_;
    const VehicleSetup& setup = setup_;

    // A car left standing for a moment is put to sleep by the solver, and while
    // asleep its forces are dropped. Any driver command must therefore wake the
    // chassis so the throttle, brake and steering below actually take effect;
    // otherwise the car is stranded until something else touches it.
    if (input_.throttle > 0.0 || input_.brake > 0.0 || input_.steer != 0.0 || input_.handbrake) {
        body.wake();
    }

    updateDriver(dt);

    // Optional driver aids: sampled from the last integrated step, they may trim
    // the throttle and add per-wheel brake torque before the drivetrain and the
    // brakes see the driver's request.
    brakeModulation_ = BrakeModulation{};
    if (electronics_ != nullptr) {
        ElectronicsSensors sensors;
        for (std::size_t i = 0; i < corners_.size(); ++i) {
            sensors.wheelSlipRatio[i] = corners_[i].slipRatio;
            sensors.wheelAngularVelocity[i] = corners_[i].spinWheel.spin;
        }
        sensors.yawRate = body.angularVelocity.y;
        sensors.forwardSpeed = forwardSpeed();
        sensors.steerAngle = steerAngle_;
        sensors.lateralAcceleration = glm::dot(body.linearVelocity, right());

        electronics_->update(sensors, dt);
        // The aids scale the pedal, not the pedal state: a cut lasts exactly as
        // long as the controller asks for it, with no re-ramp to recover from.
        const VehicleInput filtered =
            electronics_->filterInput(VehicleInput{pedalThrottle_, pedalBrake_, steerInput_, input_.handbrake});
        throttle_ = filtered.throttle;
        brake_ = filtered.brake;
        brakeModulation_ = electronics_->brakeModulation();
    }

    updateTransmission(dt);

    const Vec3 up = body.localToWorldDirection(Vec3(0.0, 1.0, 0.0));
    const Vec3 forward = body.localToWorldDirection(Vec3(0.0, 0.0, -1.0));
    const Vec3 down = -up;
    const Real suspensionLength = setup.restLength + setup.wheelRadius;

    // --- 1. Raycast suspension, spring and damper --------------------------
    for (VehicleCorner& corner : corners_) {
        const Vec3 mountWorld = body.localToWorldPoint(corner.localMount);
        corner.mountWorld = mountWorld;
        Real distance = 0.0;
        GroundSample sample;
        corner.contact = raycastGround(*ground_, mountWorld, down, suspensionLength, distance, sample);
        if (!corner.contact) {
            corner.compression = 0.0;
            corner.load = 0.0;
            corner.contactPoint = mountWorld + down * setup.restLength;
            corner.contactNormal = up;
            corner.wheelCentre = corner.contactPoint;
            continue;
        }
        const Real strutLength = distance - setup.wheelRadius;
        const Real compressionRate = glm::dot(body.velocityAtWorldPoint(mountWorld), down);
        SpringDamperState state;
        state.length = strutLength;
        state.velocity = compressionRate;
        const SpringDamperForces forces = computeSpringDamperForces(corner.spring, state);

        corner.compression = suspensionLength - distance;
        corner.contactPoint = mountWorld + down * distance;
        corner.contactNormal = sample.normal;
        corner.wheelCentre = corner.contactPoint + sample.normal * setup.wheelRadius;
        corner.load = std::max(0.0, forces.total());
        body.applyForceAtWorldPoint(sample.normal * corner.load, corner.contactPoint);
    }

    // --- 2. Brakes ---------------------------------------------------------
    const Real masterPressure = brake_ * setup.maxBrakePressure;
    BrakeBias bias = setup.brakeBias;
    bias.handbrakePressure = input_.handbrake ? setup.brakeBias.handbrakePressure : 0.0;
    const Real frontWheelBrake = setup.brake.torque(frontLinePressure(masterPressure, bias));
    const Real rearWheelBrake = setup.brake.torque(rearLinePressure(masterPressure, bias));

    // --- 3. Steering (Ackermann) ------------------------------------------
    for (VehicleCorner& corner : corners_) {
        corner.steerAngle = 0.0;
    }
    if (std::abs(steerAngle_) > 1e-6) {
        const SteerAngles angles = steerAngles(setup.ackermann, std::abs(steerAngle_), 1.0);
        const Real sign = steerAngle_ > 0.0 ? 1.0 : -1.0; // + = left: the left wheel is inside
        for (VehicleCorner& corner : corners_) {
            if (!corner.steered) {
                continue;
            }
            const bool inside = (steerAngle_ > 0.0) == corner.left;
            corner.steerAngle = sign * (inside ? angles.inner : angles.outer);
        }
    }

    // --- 4. Tyres and wheel spin ------------------------------------------
    for (std::size_t i = 0; i < corners_.size(); ++i) {
        VehicleCorner& corner = corners_[i];
        const Real baseBrake = corner.front ? frontWheelBrake : rearWheelBrake;
        corner.brakeTorque = std::max(0.0, baseBrake + brakeModulation_.torqueDelta[i]);

        const Real c = std::cos(corner.steerAngle);
        const Real s = std::sin(corner.steerAngle);
        const Vec3 wheelForward = forward * c + glm::cross(up, forward) * s;
        const Vec3 normal = corner.contactNormal;
        const Vec3 tangent = physics::safeNormalize(wheelForward - normal * glm::dot(wheelForward, normal), forward);
        const Vec3 lateral = glm::cross(tangent, normal); // to the wheel's right

        const Vec3 contactVelocity = body.velocityAtWorldPoint(corner.contactPoint);
        const Real vx = glm::dot(contactVelocity, tangent);
        const Real vy = glm::dot(contactVelocity, lateral);
        const Real denominator = std::max(std::abs(vx), setup.lowSpeedSpeed);

        const GroundSample surface = ground_->sample(corner.contactPoint.x, corner.contactPoint.z);

        TireState context;
        context.slipAngle = std::atan(vy / denominator);
        context.normalLoad = corner.load;
        context.camber = 0.0;
        context.frictionCoefficient = surface.friction;
        context.contactSpeed = vx;

        WheelSpinCoefficients spinCoefficients;
        spinCoefficients.inertia = corner.wheelInertia + (corner.driven ? drivenInertiaReflect_ : 0.0);
        spinCoefficients.minContactSpeed = setup.lowSpeedSpeed;

        WheelSpinInput spinInput;
        spinInput.driveTorque = corner.driveTorque;
        spinInput.brakeTorque = corner.brakeTorque;
        spinInput.contactSpeed = vx;
        spinInput.effectiveRadius = setup.wheelRadius;
        if (corner.contact) {
            spinInput.rollingResistanceMoment = surface.rollingResistance * corner.load * setup.wheelRadius;
        }

        const WheelSpinResult spin = stepWheelSpin(spinCoefficients, tyre_, context, spinInput, dt, corner.spinWheel);
        corner.spinAngle = wrapSpin(corner.spinAngle + corner.spinWheel.spin * dt);

        TireState state = context;
        state.slipRatio = spin.slipRatio;
        const TireForces forces = tyre_.computeForces(state);

        corner.slipRatio = spin.slipRatio;
        corner.slipAngle = context.slipAngle;
        corner.longitudinalForce = forces.longitudinal;
        corner.lateralForce = forces.lateral;
        corner.tyreForce = tangent * forces.longitudinal + lateral * forces.lateral;
        if (corner.contact && corner.load > 0.0) {
            body.applyForceAtWorldPoint(corner.tyreForce, corner.contactPoint);
        }
    }

    // --- 5. Aerodynamics ---------------------------------------------------
    const Vec3 airVelocityBody =
        physics::rotateVector(physics::quaternionConjugate(body.orientation), -body.linearVelocity);
    const AeroLoads aero =
        computeAerodynamicLoads(airVelocityBody, setup.airDensity, setup.dragCoefficient, setup.liftCoefficient,
                                setup.frontalArea, setup.centreOfPressure, Vec3(0.0));
    const Vec3 aeroForceWorld = physics::rotateVector(body.orientation, aero.force);
    body.applyForceAtWorldPoint(aeroForceWorld, body.localToWorldPoint(setup.centreOfPressure));
}

VehicleSetup makeDefaultVehicleSetup() {
    VehicleSetup setup;

    setup.frontSpring.stiffness = 38000.0;
    setup.frontSpring.restLength = 0.25;
    setup.frontSpring.dampingCompression = 2800.0;
    setup.frontSpring.dampingRebound = 3600.0;
    setup.frontSpring.hasBumpStop = true;
    setup.frontSpring.bumpStopLength = 0.06;
    setup.frontSpring.bumpStopStiffness = 200000.0;
    setup.frontSpring.bumpStopDamping = 800.0;
    setup.frontSpring.bumpStopExponent = 2.0;

    setup.rearSpring = setup.frontSpring;
    setup.rearSpring.stiffness = 34000.0;

    setup.frontAxleZ = -1.25;
    setup.rearAxleZ = 1.35;
    setup.halfTrack = 0.775;
    setup.restLength = 0.25;
    setup.wheelRadius = 0.32;
    setup.wheelInertia = 1.2;

    setup.ackermann.track = 2.0 * setup.halfTrack;
    setup.ackermann.wheelbase = setup.rearAxleZ - setup.frontAxleZ;

    setup.engine.config.inertia = 0.5;
    setup.engine.config.revLimit = 700.0;
    setup.engine.config.idleSpeed = 90.0;
    setup.engine.friction.constant = 10.0;
    setup.engine.friction.linear = 0.05;
    setup.engine.map.speed = {100.0, 200.0, 300.0, 450.0, 600.0, 700.0};
    setup.engine.map.throttle = {0.0, 1.0};
    const std::vector<Real> wideOpen{170.0, 215.0, 240.0, 250.0, 230.0, 190.0};
    setup.engine.map.torque.resize(setup.engine.map.speed.size() * setup.engine.map.throttle.size());
    for (std::size_t i = 0; i < setup.engine.map.speed.size(); ++i) {
        setup.engine.map.torque[i * 2 + 0] = 0.0;         // closed throttle
        setup.engine.map.torque[i * 2 + 1] = wideOpen[i]; // wide-open throttle
    }

    setup.gearbox.forwardRatios = {3.20, 2.15, 1.55, 1.20, 0.97, 0.80};
    setup.gearbox.reverseRatio = 3.30;
    setup.gearbox.efficiency = 0.92;
    setup.finalDrive = 3.60;
    setup.differential.type = DifferentialType::Open;

    setup.brakeBias.frontFraction = 0.62;
    setup.brakeBias.handbrakePressure = 3.0e6;
    // Master pressure that yields the documented 3600 N*m of total brake torque
    // (DRIVE.md): T = 2 mu_p A_p R_eff p, so p = 3600 / (2 * 0.40 * 0.002 * 0.13).
    setup.maxBrakePressure = 8.65e6;

    setup.centreOfPressure = Vec3(0.0, 0.4, 0.0);
    setup.dragCoefficient = 0.30;
    setup.frontalArea = 2.1;

    return setup;
}

} // namespace sim::vehicle
