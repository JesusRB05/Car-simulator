// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <array>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Mat3;
using physics::Real;
using physics::Vec3;

// A double-wishbone (SLA) corner described by hardpoints, following the
// reference kinematics of the technical plan (Section 5.1): the upright is a
// rigid body held by five fixed-length links, one per wishbone pivot pair plus
// the tie rod, so a pure bump leaves a single degree of freedom (wheel travel).
// Moving the tie-rod inner point adds steering as a second input later.
//
// All chassis points are in the vehicle/chassis frame at the design (static)
// pose. The upright points are given in the upright frame, which coincides with
// the chassis frame at the design pose, so at zero travel the upright pose is
// the identity.
struct DoubleWishboneCorner {
    // Chassis pivots (design pose).
    Vec3 lowerFrontPivot{0.0, 0.0, 0.0};
    Vec3 lowerRearPivot{0.0, 0.0, 0.0};
    Vec3 upperFrontPivot{0.0, 0.0, 0.0};
    Vec3 upperRearPivot{0.0, 0.0, 0.0};
    Vec3 tieRodInner{0.0, 0.0, 0.0}; // steering end, fixed for a pure bump

    // Upright points (design pose).
    Vec3 lowerBallJoint{0.0, 0.0, 0.0};
    Vec3 upperBallJoint{0.0, 0.0, 0.0};
    Vec3 tieRodOuter{0.0, 0.0, 0.0};
    Vec3 wheelCentre{0.0, 0.0, 0.0};
    Vec3 wheelAxle{1.0, 0.0, 0.0}; // spin axis; lateral (along +x) at zero camber

    // Optional coilover, for the motion ratio. The lower mount rides the upright
    // here; arm-mounted springs arrive with the kinematic tables (P2.6).
    bool hasSpring = false;
    Vec3 springUpper{0.0, 0.0, 0.0}; // chassis frame
    Vec3 springLower{0.0, 0.0, 0.0}; // upright frame
};

// Resulting pose and derived quantities for one corner.
struct CornerState {
    Vec3 translation{0.0, 0.0, 0.0}; // upright origin displacement from design (m)
    Mat3 rotation{1.0};              // upright orientation (design = identity)
    Vec3 wheelCentre{0.0, 0.0, 0.0}; // world (m)
    Vec3 wheelAxle{1.0, 0.0, 0.0};   // world, unit
    Real travel = 0.0;               // wheel-centre vertical travel (m)
    Real camber = 0.0;               // rad; a rotation about the longitudinal axis
    Real toe = 0.0;                  // rad; a rotation about the vertical axis
    Real springLength = 0.0;         // m (0 if there is no spring)

    // Steering-axis geometry (the line through the two ball joints), against the
    // ground plane y = 0. All are derived from the current pose.
    Vec3 steeringAxis{0.0, 1.0, 0.0}; // world, unit, from the lower ball joint up
    Real kingpinInclination = 0.0;    // rad; inboard lean of the axis, front view
    Real caster = 0.0;                // rad; rearward lean of the axis, side view
    Real scrubRadius = 0.0;           // m; axis ground point minus wheel centre (x)
    Real mechanicalTrail = 0.0;       // m; wheel centre minus axis ground point (z)

    // Front-view (x-y plane, z = 0) and side-view (y-z plane, x = 0) instant
    // centres of the upright, from the arm lines (ball joint to the chassis pivot
    // midpoint). A parallel arm pair puts the centre at infinity; it is stored as
    // zero in that case.
    Vec3 frontInstantCentre{0.0, 0.0, 0.0};
    Vec3 sideInstantCentre{0.0, 0.0, 0.0};

    bool converged = true;
};

// Roll centre: the meeting of the two lines from each corner's front-view
// instant centre through its contact patch (returned in the x-y plane).
Vec3 rollCentre(const CornerState& left, const CornerState& right);

// Anti-dive (front) or anti-squat (rear) fraction from a corner's side-view
// instant centre: the swing-arm angle times the wheelbase over the centre-of-
// gravity height. Returns 0 when the geometry gives no finite centre.
Real antiDiveFraction(const CornerState& corner, Real wheelbase, Real cgHeight);

class DoubleWishbone {
public:
    explicit DoubleWishbone(const DoubleWishboneCorner& corner);

    // Derived quantities for a supplied pose, without solving. The translation
    // and the rotation vector (exponential map) are the upright displacement
    // from the design pose.
    CornerState pose(const Vec3& translation, const Vec3& rotationVector) const;

    // Solves the closure so the wheel centre sits at design_z + travel, warm
    // started from the supplied guess (the design pose by default).
    CornerState solveForTravel(Real travel, const CornerState& guess = CornerState{}) const;

    // d(spring length)/d(wheel travel) at the given travel, by central difference
    // of two solves. Requires a spring (corner().hasSpring).
    Real motionRatio(Real travel) const;

    const DoubleWishboneCorner& corner() const { return corner_; }

    // The five link lengths at the design pose (m), in link order.
    const std::array<Real, 5>& linkLengths() const { return lengths_; }

private:
    CornerState finalize(const Vec3& translation, const Mat3& rotation) const;

    DoubleWishboneCorner corner_;
    std::array<Vec3, 5> chassisPoints_; // p_i
    std::array<Vec3, 5> uprightPoints_; // q_i
    std::array<Real, 5> lengths_;       // L_i
};

} // namespace sim::vehicle
