// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include "physics/Constraint.hpp"
#include "physics/ForceElements.hpp"
#include "physics/Joints.hpp"
#include "physics/Math.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/RigidBody.hpp"

using namespace sim::physics;

namespace {

constexpr Real kDt = 1.0 / 1000.0;

RigidBody& makeDynamic(PhysicsWorld& world, Real mass, const Vec3& inertia, const Vec3& position)
{
    RigidBody& body = world.createBody();
    body.setMass(mass);
    body.setInertiaPrincipal(inertia);
    body.position = position;
    body.updateCache();
    return body;
}

RigidBody& makeStatic(PhysicsWorld& world, const Vec3& position)
{
    RigidBody& body = world.createBody();
    body.setType(BodyType::Static);
    body.setMass(0.0);
    body.position = position;
    body.updateCache();
    return body;
}

Real pointError(const RigidBody& bodyA, const Vec3& localA, const RigidBody& bodyB, const Vec3& localB)
{
    return glm::length(bodyA.localToWorldPoint(localA) - bodyB.localToWorldPoint(localB));
}

Real relativeError(Real measured, Real reference)
{
    return std::abs(measured - reference) / std::abs(reference);
}

// Mean period from the times at which a scalar signal crosses zero (successive
// crossings are half a period apart).
Real periodFromZeroCrossings(const std::vector<Real>& crossings)
{
    Real sum = 0.0;
    for (std::size_t i = 1; i < crossings.size(); ++i)
    {
        sum += crossings[i] - crossings[i - 1];
    }
    return 2.0 * sum / static_cast<Real>(crossings.size() - 1);
}

} // namespace

TEST_CASE("free fall matches the analytic solution")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);
    RigidBody& body = makeDynamic(world, 2.0, Vec3(1.0), Vec3(0.0));

    for (int i = 0; i < 1000; ++i)
    {
        world.step(kDt);
    }

    // Semi-implicit Euler is exact for the velocity (measured ~1e-14); the
    // position lags the exact solution by dt/t = 1e-3 at t = 1 s, so 2e-3 bounds
    // the known discretisation while still catching a wrong gravity or sign.
    const Real velocityError = relativeError(body.linearVelocity.y, -9.80665);
    const Real positionErrorValue = relativeError(body.position.y, -0.5 * 9.80665);
    MESSAGE("fall velocity rel.error = " << velocityError << ", position rel.error = " << positionErrorValue);
    // Constant-gravity velocity is exact under semi-implicit Euler; 1e-12 leaves
    // ~100x headroom over the measured 1.5e-14 for cross-platform round-off.
    CHECK(velocityError < 1e-12);
    CHECK(positionErrorValue < 2e-3);
}

TEST_CASE("projectile follows a parabola")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);
    RigidBody& body = makeDynamic(world, 1.0, Vec3(1.0), Vec3(0.0));
    body.linearVelocity = Vec3(10.0, 10.0, 0.0);

    for (int i = 0; i < 1000; ++i)
    {
        world.step(kDt);
    }

    // Horizontal motion has no force so it is exact (measured ~1.7e-14; 1e-12
    // leaves ~100x headroom for cross-platform round-off); vertical carries the
    // same dt/t = 1e-3 semi-implicit lag as the free-fall case.
    const Real xError = relativeError(body.position.x, 10.0);
    const Real yError = relativeError(body.position.y, 10.0 - 0.5 * 9.80665);
    MESSAGE("projectile x rel.error = " << xError << ", y rel.error = " << yError);
    CHECK(xError < 1e-12);
    CHECK(yError < 2e-3);
}

TEST_CASE("mass-spring oscillator period matches the analytic value")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0);

    RigidBody& body = makeDynamic(world, 1.0, Vec3(1.0), Vec3(1.1, 0.0, 0.0));
    PointSpringDamper& spring = world.createForceElement<PointSpringDamper>(&body, nullptr);
    spring.localAnchorA = Vec3(0.0);
    spring.fixedAnchorB = Vec3(0.0);
    spring.restLength = 1.0;
    spring.stiffness = 100.0;
    spring.damping = 0.0;

    const Real analyticPeriod = kTwoPiPhysics * std::sqrt(1.0 / 100.0);

    std::vector<Real> crossings;
    Real previous = body.position.x - 1.0;
    for (int i = 0; i < 10000; ++i)
    {
        const Real timeBefore = static_cast<Real>(world.time());
        world.step(kDt);
        const Real current = body.position.x - 1.0;
        if (previous < 0.0 && current >= 0.0)
        {
            const Real fraction = -previous / (current - previous);
            crossings.push_back(timeBefore + fraction * kDt);
        }
        previous = current;
    }

    REQUIRE(crossings.size() >= 5);
    const Real measuredPeriod = (crossings.back() - crossings.front()) / static_cast<Real>(crossings.size() - 1);
    const Real error = relativeError(measuredPeriod, analyticPeriod);
    MESSAGE("oscillator period measured = " << measuredPeriod << " s, analytic = " << analyticPeriod
                                            << " s, rel.error = " << error);
    // Symplectic Euler shifts the discrete frequency by O((omega*dt)^2) = 1e-4
    // here; the measured shift is 4e-6, so 2e-5 flags any wrong k/m or dt.
    CHECK(error < 2e-5);
}

TEST_CASE("simple pendulum period matches the analytic value")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    const Real length = 1.0;
    const Real angle0 = 0.05;  // small so the sin(theta) ~ theta approximation holds

    RigidBody& pivot = makeStatic(world, Vec3(0.0));
    RigidBody& bob = makeDynamic(world, 1.0, Vec3(1e-3), Vec3(length * std::sin(angle0), -length * std::cos(angle0), 0.0));

    DistanceConstraint& rod = world.createConstraint<DistanceConstraint>(pivot, bob);
    rod.localAnchorA = Vec3(0.0);
    rod.localAnchorB = Vec3(0.0);
    rod.distance = length;
    rod.soft = SoftParams{60.0, 1.0};

    const Real analyticPeriod = kTwoPiPhysics * std::sqrt(length / 9.80665);
    // Finite amplitude adds theta0^2/16 = 1.6e-4 relative, which is applied as a
    // correction; the soft joint adds a ~7e-5 m sag. The residual measured error
    // is 3.4e-5, so 1e-4 bounds it while still catching a wrong length/g.
    const Real finiteAmplitudeCorrection = 1.0 + angle0 * angle0 / 16.0;

    std::vector<Real> crossings;
    Real previous = bob.position.x;
    for (int i = 0; i < 8000; ++i)
    {
        const Real timeBefore = static_cast<Real>(world.time());
        world.step(kDt);
        const Real current = bob.position.x;
        if ((previous < 0.0) != (current < 0.0))
        {
            const Real fraction = -previous / (current - previous);
            crossings.push_back(timeBefore + fraction * kDt);
        }
        previous = current;
    }

    REQUIRE(crossings.size() >= 5);
    const Real measuredPeriod = periodFromZeroCrossings(crossings);
    const Real error = relativeError(measuredPeriod, analyticPeriod * finiteAmplitudeCorrection);
    MESSAGE("pendulum period measured = " << measuredPeriod << " s, analytic = " << analyticPeriod
                                          << " s, rel.error = " << error);
    CHECK(error < 1e-4);
}

TEST_CASE("oscillator energy stays bounded over 60 s")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0);

    RigidBody& body = makeDynamic(world, 1.0, Vec3(1.0), Vec3(1.1, 0.0, 0.0));
    PointSpringDamper& spring = world.createForceElement<PointSpringDamper>(&body, nullptr);
    spring.localAnchorA = Vec3(0.0);
    spring.fixedAnchorB = Vec3(0.0);
    spring.restLength = 1.0;
    spring.stiffness = 100.0;

    const auto energy = [&]() {
        const Real extension = body.position.x - spring.restLength;
        const Real kinetic = 0.5 * body.mass * glm::dot(body.linearVelocity, body.linearVelocity);
        const Real potential = 0.5 * spring.stiffness * extension * extension;
        return kinetic + potential;
    };

    const Real initial = energy();
    Real maxRelativeDrift = 0.0;
    for (int i = 0; i < 60000; ++i)
    {
        world.step(kDt);
        maxRelativeDrift = std::max(maxRelativeDrift, std::abs(energy() - initial) / initial);
    }
    MESSAGE("oscillator max energy drift over 60 s = " << maxRelativeDrift);
    // Symplectic Euler conserves a nearby energy, so the drift oscillates within
    // O(omega*dt)/2 ~ 5e-3 (measured 5.0e-3) and does not grow; 8e-3 catches any
    // secular leak without flaking on the known bounded oscillation.
    CHECK(maxRelativeDrift < 8e-3);
}

TEST_CASE("angular momentum of a torque-free top is conserved")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0);

    RigidBody& body = makeDynamic(world, 1.0, Vec3(1.0, 1.0, 2.0), Vec3(0.0));
    body.angularVelocity = Vec3(0.1, 0.0, 10.0);
    body.updateCache();

    const Real initial = glm::length(body.angularMomentumAbout(body.position));
    for (int i = 0; i < 10000; ++i)
    {
        world.step(kDt);
    }
    const Real final = glm::length(body.angularMomentumAbout(body.position));
    const Real error = relativeError(final, initial);
    MESSAGE("angular momentum rel.error over 10 s = " << error);
    // The implicit midpoint rule preserves |I*omega| exactly, so the only error
    // is O(dt^3) round-off (measured 3.2e-15); 1e-12 guards a broken torque with
    // ~100x headroom while still failing loudly on any discretization error.
    CHECK(error < 1e-12);
}

TEST_CASE("Dzhanibekov flip: an asymmetric body tumbling about its intermediate axis reverses")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0);

    // I1 < I2 < I3: rotation about the intermediate (y) axis is unstable.
    RigidBody& body = makeDynamic(world, 1.0, Vec3(1.0, 2.0, 3.0), Vec3(0.0));
    body.angularVelocity = Vec3(0.05, 6.0, 0.0);
    body.updateCache();

    const Real initialMomentum = glm::length(body.angularMomentumAbout(body.position));
    Real minIntermediate = body.angularVelocity.y;
    Real maxMomentumError = 0.0;
    for (int i = 0; i < 10000; ++i)
    {
        world.step(kDt);
        const Vec3 bodyAngular = rotateVector(quaternionConjugate(body.orientation), body.angularVelocity);
        minIntermediate = std::min(minIntermediate, bodyAngular.y);
        maxMomentumError =
            std::max(maxMomentumError, relativeError(glm::length(body.angularMomentumAbout(body.position)), initialMomentum));
    }
    MESSAGE("Dzhanibekov min intermediate-axis omega = " << minIntermediate
                                                         << ", max momentum rel.error = " << maxMomentumError);
    // The intermediate-axis component must cross zero and reverse (flip);
    // measured -6.0 out of an initial +6.0, so the sign flip is unambiguous.
    CHECK(minIntermediate < -0.5);
    // ...while the free body still conserves its angular momentum (measured
    // 6.9e-12); 1e-10 keeps ~15x headroom and flags a broken gyroscopic term.
    CHECK(maxMomentumError < 1e-10);
}

TEST_CASE("fast gyroscope keeps its energy and precesses at the analytic rate")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    const Real spinRate = 3000.0 * kTwoPiPhysics / 60.0;  // 3000 rpm
    const Real distance = 0.1;

    RigidBody& pivot = makeStatic(world, Vec3(0.0));
    RigidBody& wheel = makeDynamic(world, 1.0, Vec3(0.1, 0.2, 0.1), Vec3(0.0));
    wheel.orientation = quaternionFromAxisAngle(Vec3(0.0, 0.0, 1.0), 0.1);
    wheel.position = -rotateVector(wheel.orientation, Vec3(0.0, distance, 0.0));
    wheel.angularVelocity = rotateVector(wheel.orientation, Vec3(0.0, spinRate, 0.0));
    wheel.updateCache();

    SphericalConstraint& joint = world.createConstraint<SphericalConstraint>(pivot, wheel);
    joint.localAnchorA = Vec3(0.0);
    joint.localAnchorB = Vec3(0.0, distance, 0.0);

    const auto energy = [&]() {
        const Real kinetic = wheel.kineticEnergy();
        const Real potential = wheel.mass * 9.80665 * wheel.position.y;
        return kinetic + potential;
    };
    const auto azimuth = [&]() {
        const Vec3 axis = rotateVector(wheel.orientation, Vec3(0.0, 1.0, 0.0));
        return std::atan2(axis.x, axis.z);
    };

    const Real initialEnergy = energy();
    const Real initialAzimuth = azimuth();

    const Real duration = 20.0;
    const int steps = static_cast<int>(duration / kDt);
    Real maxEnergy = initialEnergy;
    for (int i = 0; i < steps; ++i)
    {
        world.step(kDt);
        maxEnergy = std::max(maxEnergy, energy());
    }

    Real delta = azimuth() - initialAzimuth;
    while (delta > kPiPhysics)
    {
        delta -= kTwoPiPhysics;
    }
    while (delta < -kPiPhysics)
    {
        delta += kTwoPiPhysics;
    }

    const Real analyticPrecession = 9.80665 * distance / (0.2 * spinRate) * duration;
    const Real precessionError = relativeError(std::abs(delta), analyticPrecession);
    const Real energyGrowth = (maxEnergy - initialEnergy) / initialEnergy;
    MESSAGE("gyroscope precession measured = " << std::abs(delta) << " rad, analytic = " << analyticPrecession
                                               << " rad, rel.error = " << precessionError
                                               << ", energy growth = " << energyGrowth);
    // No energy may be injected into the fast wheel (midpoint conserves it; the
    // measured growth is 0, so allow only 1e-4 of round-off)...
    CHECK(maxEnergy <= initialEnergy * (1.0 + 1e-4));
    // ...and the fast-top precession rate must match |m*g*d / (I3*omega)|; the
    // measured error is 4.4e-3 (soft-joint compliance + dt), so 1e-2 bounds it.
    CHECK(precessionError < 1e-2);
}

TEST_CASE("20-link chain keeps its joints bounded over 60 s")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    constexpr int kLinks = 20;
    constexpr Real kHalf = 0.25;
    const Vec3 localTop(0.0, kHalf, 0.0);
    const Vec3 localBottom(0.0, -kHalf, 0.0);

    RigidBody& anchor = makeStatic(world, Vec3(0.0));

    std::vector<RigidBody*> links;
    for (int i = 0; i < kLinks; ++i)
    {
        links.push_back(&makeDynamic(world, 1.0, Vec3(0.01), Vec3(0.0, -(i + 0.5) * 2.0 * kHalf, 0.0)));
    }

    world.createConstraint<SphericalConstraint>(anchor, *links[0]).localAnchorB = localTop;
    for (int i = 0; i < kLinks - 1; ++i)
    {
        SphericalConstraint& joint = world.createConstraint<SphericalConstraint>(*links[i], *links[i + 1]);
        joint.localAnchorA = localBottom;
        joint.localAnchorB = localTop;
    }

    Real maxError = 0.0;
    bool finite = true;
    for (int step = 0; step < 60000; ++step)
    {
        world.step(kDt);
        for (RigidBody* link : links)
        {
            finite = finite && link->isFiniteState();
        }
        maxError = std::max(maxError, pointError(anchor, Vec3(0.0), *links[0], localTop));
        for (int i = 0; i < kLinks - 1; ++i)
        {
            maxError = std::max(maxError, pointError(*links[i], localBottom, *links[i + 1], localTop));
        }
    }
    MESSAGE("20-link chain max joint error over 60 s = " << maxError << " m");
    // Soft 60 Hz joints sag ~F/k: the top joint carries ~196 N at k ~ 7.1e4 N/m,
    // i.e. ~2.8 mm static; the swing-inflated measured worst case is 4.5 mm, so
    // 7 mm bounds it while still flagging any drift escalation.
    CHECK(finite);
    CHECK(maxError < 7e-3);
}

TEST_CASE("a 1:1000 mass-ratio joint stays stable over 60 s")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    RigidBody& heavy = makeDynamic(world, 1000.0, Vec3(10.0), Vec3(0.0));
    RigidBody& light = makeDynamic(world, 1.0, Vec3(0.1), Vec3(0.0, -1.0, 0.0));

    DistanceConstraint& joint = world.createConstraint<DistanceConstraint>(heavy, light);
    joint.localAnchorA = Vec3(0.0);
    joint.localAnchorB = Vec3(0.0);
    joint.distance = 1.0;

    Real maxError = 0.0;
    bool finite = true;
    for (int step = 0; step < 60000; ++step)
    {
        world.step(kDt);
        finite = finite && heavy.isFiniteState() && light.isFiniteState();
        maxError = std::max(maxError, std::abs(pointError(heavy, Vec3(0.0), light, Vec3(0.0)) - joint.distance));
    }
    MESSAGE("1:1000 mass-ratio max joint error over 60 s = " << maxError << " m");
    // The effective mass of a 1:1000 pair is dominated by the light body
    // (m_eff ~ 1 kg), so the soft 60 Hz joint stays well-conditioned; the
    // measured error is 1.8e-12 m, so 1e-10 m bounds it ~55x while still
    // flagging any conditioning regression well before it becomes visible.
    CHECK(finite);
    CHECK(maxError < 1e-10);
}

TEST_CASE("fixed constraint keeps a welded pair rigid under disturbance")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    RigidBody& base = makeStatic(world, Vec3(0.0));
    RigidBody& arm = makeDynamic(world, 2.0, Vec3(0.4), Vec3(0.0));
    arm.linearVelocity = Vec3(0.5, 0.5, 0.0);
    arm.angularVelocity = Vec3(1.0, -1.0, 0.5);

    FixedConstraint& weld = world.createConstraint<FixedConstraint>(base, arm);
    weld.localAnchorA = Vec3(0.0);
    weld.localAnchorB = Vec3(0.0);
    weld.captureReference();

    const Quat reference = arm.orientation;
    const Vec3 referencePosition = arm.position;
    Real maxPointError = 0.0;
    Real maxOrientationError = 0.0;
    for (int i = 0; i < 5000; ++i)
    {
        world.step(kDt);
        maxPointError =
            std::max(maxPointError, glm::length(arm.position - referencePosition));
        maxOrientationError = std::max(
            maxOrientationError, 1.0 - std::abs(glm::dot(arm.orientation, reference))
                                      / (glm::length(arm.orientation) * glm::length(reference)));
    }
    MESSAGE("fixed joint max position error over 5 s = " << maxPointError
            << " m, max orientation error = " << maxOrientationError);
    // Six soft rows must hold a 2 kg arm 1 m from the base against gravity
    // (static sag ~7e-5 m) even after an initial linear+angular disturbance
    // while it still falls in free space (its own motion is unconstrained).
    CHECK(maxPointError < 5e-3);
    CHECK(maxOrientationError < 5e-3);
}

TEST_CASE("linear damping decays velocity at the analytic exponential rate")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0);

    RigidBody& body = makeDynamic(world, 1.0, Vec3(1.0), Vec3(0.0));
    body.linearVelocity = Vec3(10.0, 0.0, 0.0);
    body.linearDamping = 1.0;

    for (int i = 0; i < 1000; ++i)
    {
        world.step(kDt);
    }
    const Real time = world.time();
    const Real analytic = 10.0 * std::exp(-1.0 * time);
    const Real error = relativeError(body.linearVelocity.x, analytic);
    MESSAGE("damped velocity measured = " << body.linearVelocity.x << " m/s, analytic = " << analytic
                                          << " m/s over " << time << " s, rel.error = " << error);
    // Damping is applied as 1/(1 + d*dt) per step, whose continuous limit is
    // exp(-d*t); at d = 1 the discretization error is ~3e-4, so 1e-2 bounds it
    // while flagging a mis-scaled or missing decay term.
    CHECK(error < 1e-2);
}

TEST_CASE("revolute motor drives the hinge to its target angular speed")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    RigidBody& base = makeStatic(world, Vec3(0.0));
    RigidBody& arm = makeDynamic(world, 1.0, Vec3(0.5, 1.0, 0.5), Vec3(1.0, 0.0, 0.0));

    RevoluteConstraint& hinge = world.createConstraint<RevoluteConstraint>(base, arm);
    hinge.localAnchorA = Vec3(0.0);
    hinge.localAnchorB = Vec3(0.0);
    hinge.localAxisA = Vec3(0.0, 1.0, 0.0);
    hinge.motorEnabled = true;
    hinge.motorSpeed = 3.0;       // rad/s target
    hinge.maxMotorTorque = 100.0; // N*m
    hinge.captureReference();

    Real speed = 0.0;
    for (int i = 0; i < 3000; ++i)
    {
        world.step(kDt);
        const Vec3 axis = rotateVector(base.orientation, hinge.localAxisA);
        speed = glm::dot(arm.angularVelocity, axis);
    }
    MESSAGE("revolute motor speed after 3 s = " << speed << " rad/s (target " << hinge.motorSpeed << ")");
    // With gravity aligned with the hinge axis there is no opposing torque, so
    // the motor must reach and hold the target speed; gravity only bends the arm.
    CHECK(std::abs(speed - 3.0) < 0.05 * 3.0);
}

namespace {

void buildDeterministicScene(PhysicsWorld& world)
{
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    RigidBody& pivot = makeStatic(world, Vec3(0.0));
    RigidBody& bob = makeDynamic(world, 1.0, Vec3(0.1), Vec3(0.3, -1.0, 0.0));
    DistanceConstraint& joint = world.createConstraint<DistanceConstraint>(pivot, bob);
    joint.localAnchorA = Vec3(0.0);
    joint.localAnchorB = Vec3(0.0);
    joint.distance = 1.0;

    RigidBody& free = makeDynamic(world, 0.5, Vec3(1.0, 2.0, 3.0), Vec3(0.0, 2.0, 0.0));
    free.angularVelocity = Vec3(0.2, 0.4, 0.7);
    PointSpringDamper& spring = world.createForceElement<PointSpringDamper>(&bob, nullptr);
    spring.localAnchorA = Vec3(0.0);
    spring.fixedAnchorB = Vec3(1.0, 0.0, 0.0);
    spring.restLength = 0.5;
    spring.stiffness = 20.0;
    spring.damping = 0.5;
}

} // namespace

TEST_CASE("two independent runs of 20000 steps are bit-for-bit identical")
{
    PhysicsWorld first;
    buildDeterministicScene(first);
    for (int i = 0; i < 20000; ++i)
    {
        first.step(kDt);
    }

    PhysicsWorld second;
    buildDeterministicScene(second);
    for (int i = 0; i < 20000; ++i)
    {
        second.step(kDt);
    }

    MESSAGE("determinism hashes: run1 = " << first.stateHash() << ", run2 = " << second.stateHash());
    CHECK(first.stateHash() == second.stateHash());
}

TEST_CASE("snapshot/restore reproduces the exact continuation state")
{
    PhysicsWorld world;
    buildDeterministicScene(world);
    for (int i = 0; i < 10000; ++i)
    {
        world.step(kDt);
    }
    const WorldSnapshot snapshot = world.saveSnapshot();
    for (int i = 0; i < 10000; ++i)
    {
        world.step(kDt);
    }
    const std::uint64_t continuousHash = world.stateHash();

    world.restoreSnapshot(snapshot);
    for (int i = 0; i < 10000; ++i)
    {
        world.step(kDt);
    }

    MESSAGE("snapshot hashes: continuous = " << continuousHash << ", restored = " << world.stateHash());
    CHECK(world.stateHash() == continuousHash);
}

TEST_CASE("benchmark: 20-link chain throughput")
{
    PhysicsWorld world;
    world.gravity = Vec3(0.0, -9.80665, 0.0);

    constexpr int kLinks = 20;
    const Vec3 localTop(0.0, 0.25, 0.0);
    const Vec3 localBottom(0.0, -0.25, 0.0);
    RigidBody& anchor = makeStatic(world, Vec3(0.0));

    std::vector<RigidBody*> links;
    for (int i = 0; i < kLinks; ++i)
    {
        links.push_back(&makeDynamic(world, 1.0, Vec3(0.01), Vec3(0.0, -(i + 0.5) * 0.5, 0.0)));
    }
    world.createConstraint<SphericalConstraint>(anchor, *links[0]).localAnchorB = localTop;
    for (int i = 0; i < kLinks - 1; ++i)
    {
        SphericalConstraint& joint = world.createConstraint<SphericalConstraint>(*links[i], *links[i + 1]);
        joint.localAnchorA = localBottom;
        joint.localAnchorB = localTop;
    }

    const auto start = std::chrono::steady_clock::now();
    for (int step = 0; step < 10000; ++step)
    {
        world.step(kDt);
    }
    const double milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    MESSAGE("20-link chain: 10000 steps in " << milliseconds << " ms (" << (milliseconds / 10000.0)
                                             << " ms/step, " << (10000000.0 / milliseconds) << " steps/s)");
    CHECK(links.back()->isFiniteState());
}

TEST_CASE("math helpers behave as documented")
{
    const Vec3 v(1.0, 2.0, 3.0);
    const Vec3 w(-2.0, 0.5, 1.5);
    CHECK(skew(v) * w == glm::cross(v, w));

    const Vec3 axis = safeNormalize(Vec3(0.3, -0.7, 0.2));
    const Quat q = quaternionFromAxisAngle(axis, 1.234);
    const Vec3 recovered = quaternionRotationVector(q);
    CHECK(glm::length(recovered - axis * 1.234) < 1e-9);

    const Vec3 moments = symmetricEigenvalues(Mat3(3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0));
    CHECK(moments.x == doctest::Approx(1.0));
    CHECK(moments.y == doctest::Approx(2.0));
    CHECK(moments.z == doctest::Approx(3.0));

    const Vec3 box = shapeInertia(Shape::box(Vec3(0.5, 0.5, 0.5)), 1.0);
    CHECK(box.x == doctest::Approx(1.0 / 6.0));

    RigidBody invalid;
    invalid.setInertiaTensor(Mat3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 5.0));
    CHECK_FALSE(invalid.validateInertia());
}
