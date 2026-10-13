// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Quat;
using physics::Real;
using physics::Vec3;

// Hardpoints of one corner of a double-wishbone suspension, expressed in the
// chassis frame (metres; +X right, +Y up, -Z forward) at the design (static
// ride height) position. Simpler topologies (MacPherson strut, beam axle) may
// leave unused points at their defaults and must document which ones they read.
struct SuspensionHardpoints {
    // Chassis-side pivots of the upper wishbone (front and rear bushings).
    Vec3 upperWishboneFrontPivot{0.0};
    Vec3 upperWishboneRearPivot{0.0};
    // Chassis-side pivots of the lower wishbone (front and rear bushings).
    Vec3 lowerWishboneFrontPivot{0.0};
    Vec3 lowerWishboneRearPivot{0.0};
    // Upright (knuckle) ball joints.
    Vec3 upperBallJoint{0.0};
    Vec3 lowerBallJoint{0.0};
    // Steering tie rod: rack-side (inner) and upright-side (outer) points.
    Vec3 tieRodInner{0.0};
    Vec3 tieRodOuter{0.0};
    // Spring and damper mounts: chassis side and suspension (arm/upright) side.
    Vec3 springChassisMount{0.0};
    Vec3 springArmMount{0.0};
    Vec3 damperChassisMount{0.0};
    Vec3 damperArmMount{0.0};
    // Wheel centre at the design position.
    Vec3 wheelCenter{0.0};
};

// Kinematic description of one suspension corner. Given the corner travel and
// steering rack position, it reports where the wheel sits and how the elastic
// elements are deformed. Forces are applied by the vehicle model using the
// spring/damper lengths and motion ratios reported here.
class ISuspension {
public:
    virtual ~ISuspension() = default;

    virtual const SuspensionHardpoints& hardpoints() const = 0;

    // Sets the corner state: vertical wheel travel relative to the design
    // position (m, + = bump/compression) and rack displacement (m, + = steer left).
    virtual void setTravel(Real wheelTravel, Real rackDisplacement) = 0;

    // Wheel centre position and orientation in the chassis frame for the
    // current travel (includes camber, toe and caster changes).
    virtual Vec3 wheelPosition() const = 0;
    virtual Quat wheelOrientation() const = 0;

    // Current spring and damper lengths between their mounts (m).
    virtual Real springLength() const = 0;
    virtual Real damperLength() const = 0;

    // Motion ratios |d(element length) / d(wheel travel)| at the current
    // travel, dimensionless.
    virtual Real springMotionRatio() const = 0;
    virtual Real damperMotionRatio() const = 0;
};

} // namespace sim::vehicle
