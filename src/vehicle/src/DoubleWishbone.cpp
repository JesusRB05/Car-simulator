// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "vehicle/DoubleWishbone.hpp"

#include <algorithm>
#include <cmath>

namespace sim::vehicle {
namespace {

// Rodrigues exponential map: a rotation vector to an active rotation matrix.
Mat3 expMap(const Vec3& r) {
    const Real theta = glm::length(r);
    if (theta < 1e-12) {
        return Mat3(1.0);
    }
    const Vec3 axis = r / theta;
    const Real c = std::cos(theta);
    const Real s = std::sin(theta);
    const Real t = 1.0 - c;
    const Real x = axis.x;
    const Real y = axis.y;
    const Real z = axis.z;

    Mat3 R;
    R[0] = Vec3(t * x * x + c, t * x * y + s * z, t * x * z - s * y);
    R[1] = Vec3(t * x * y - s * z, t * y * y + c, t * y * z + s * x);
    R[2] = Vec3(t * x * z + s * y, t * y * z - s * x, t * z * z + c);
    return R;
}

// Solves the dense 6x6 system J d = b by Gauss-Jordan with partial pivoting.
bool solve6x6(double j[6][6], double b[6], double d[6]) {
    for (int col = 0; col < 6; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 6; ++row) {
            if (std::abs(j[row][col]) > std::abs(j[pivot][col])) {
                pivot = row;
            }
        }
        if (std::abs(j[pivot][col]) < 1e-14) {
            return false; // singular
        }
        if (pivot != col) {
            for (int k = 0; k < 6; ++k) {
                std::swap(j[pivot][k], j[col][k]);
            }
            std::swap(b[pivot], b[col]);
        }
        const double inv = 1.0 / j[col][col];
        for (int k = col; k < 6; ++k) {
            j[col][k] *= inv;
        }
        b[col] *= inv;
        for (int row = 0; row < 6; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = j[row][col];
            if (factor == 0.0) {
                continue;
            }
            for (int k = col; k < 6; ++k) {
                j[row][k] -= factor * j[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    for (int i = 0; i < 6; ++i) {
        d[i] = b[i];
    }
    return true;
}

// Intersection of two 2D lines given as point plus direction. Returns false when
// they are parallel.
bool intersectLines(const glm::dvec2& p1, const glm::dvec2& d1, const glm::dvec2& p2, const glm::dvec2& d2,
                    glm::dvec2& out) {
    const double denom = d1.x * d2.y - d1.y * d2.x;
    if (std::abs(denom) < 1e-12) {
        return false;
    }
    const glm::dvec2 dp = p2 - p1;
    const double t = (dp.x * d2.y - dp.y * d2.x) / denom;
    out = p1 + t * d1;
    return true;
}

} // namespace

DoubleWishbone::DoubleWishbone(const DoubleWishboneCorner& corner) : corner_(corner) {
    chassisPoints_ = {corner_.lowerFrontPivot, corner_.lowerRearPivot, corner_.upperFrontPivot, corner_.upperRearPivot,
                      corner_.tieRodInner};
    uprightPoints_ = {corner_.lowerBallJoint, corner_.lowerBallJoint, corner_.upperBallJoint, corner_.upperBallJoint,
                      corner_.tieRodOuter};
    for (std::size_t i = 0; i < lengths_.size(); ++i) {
        lengths_[i] = glm::length(uprightPoints_[i] - chassisPoints_[i]);
    }
}

CornerState DoubleWishbone::finalize(const Vec3& translation, const Mat3& rotation) const {
    CornerState state;
    state.translation = translation;
    state.rotation = rotation;

    const Vec3 axleWorld = rotation * corner_.wheelAxle;
    const Real axleLength = glm::length(axleWorld);
    state.wheelAxle = (axleLength > 0.0) ? axleWorld / axleLength : corner_.wheelAxle;

    state.wheelCentre = rotation * corner_.wheelCentre + translation;
    state.travel = state.wheelCentre.z - corner_.wheelCentre.z;
    state.camber = std::asin(std::clamp(state.wheelAxle.y, -1.0, 1.0));
    state.toe = std::atan2(-state.wheelAxle.z, state.wheelAxle.x);

    if (corner_.hasSpring) {
        state.springLength = glm::length(rotation * corner_.springLower + translation - corner_.springUpper);
    }

    // Steering axis and the alignment quantities it fixes against the ground
    // plane y = 0 (the world up-axis ground).
    const Vec3 lowerWorld = rotation * corner_.lowerBallJoint + translation;
    const Vec3 upperWorld = rotation * corner_.upperBallJoint + translation;
    const Vec3 axisRaw = upperWorld - lowerWorld;
    const Real axisLength = glm::length(axisRaw);
    const Vec3 a = (axisLength > 1e-12) ? axisRaw / axisLength : Vec3(0.0, 1.0, 0.0);
    state.steeringAxis = a;
    state.kingpinInclination = std::atan2(-a.x, a.y);
    state.caster = std::atan2(a.z, a.y);

    const Real t = (std::abs(a.y) > 1e-12) ? -lowerWorld.y / a.y : 0.0;
    const Vec3 groundPoint = lowerWorld + t * a;
    state.scrubRadius = groundPoint.x - state.wheelCentre.x;
    state.mechanicalTrail = state.wheelCentre.z - groundPoint.z;

    // Instant centres: the arm lines run from a (moved) ball joint to the fixed
    // chassis pivot midpoint of that wishbone. The front view is the x-y plane and
    // the side view is the z-y plane.
    const Vec3 lowerPivot = 0.5 * (corner_.lowerFrontPivot + corner_.lowerRearPivot);
    const Vec3 upperPivot = 0.5 * (corner_.upperFrontPivot + corner_.upperRearPivot);

    glm::dvec2 front(0.0);
    if (intersectLines(glm::dvec2(lowerWorld.x, lowerWorld.y),
                       glm::dvec2(lowerPivot.x - lowerWorld.x, lowerPivot.y - lowerWorld.y),
                       glm::dvec2(upperWorld.x, upperWorld.y),
                       glm::dvec2(upperPivot.x - upperWorld.x, upperPivot.y - upperWorld.y), front)) {
        state.frontInstantCentre = Vec3(front.x, front.y, 0.0);
    }

    glm::dvec2 side(0.0);
    if (intersectLines(glm::dvec2(lowerWorld.z, lowerWorld.y),
                       glm::dvec2(lowerPivot.z - lowerWorld.z, lowerPivot.y - lowerWorld.y),
                       glm::dvec2(upperWorld.z, upperWorld.y),
                       glm::dvec2(upperPivot.z - upperWorld.z, upperPivot.y - upperWorld.y), side)) {
        state.sideInstantCentre = Vec3(0.0, side.y, side.x);
    }
    return state;
}

CornerState DoubleWishbone::pose(const Vec3& translation, const Vec3& rotationVector) const {
    return finalize(translation, expMap(rotationVector));
}

CornerState DoubleWishbone::solveForTravel(Real travel, const CornerState& guess) const {
    Vec3 x = guess.translation;
    Mat3 R = guess.rotation;

    const Vec3 wcLocal = corner_.wheelCentre;
    const Real targetZ = corner_.wheelCentre.z + travel;

    // Largest link-length residual for a candidate pose.
    const auto maxResidual = [&](const Vec3& xx, const Mat3& RR) {
        double m = 0.0;
        for (int i = 0; i < 5; ++i) {
            const Real len = glm::length(RR * uprightPoints_[i] + xx - chassisPoints_[i]);
            m = std::max(m, std::abs(len - lengths_[i]));
        }
        const Real wcZ = (RR * wcLocal + xx).z;
        return std::max(m, std::abs(wcZ - targetZ));
    };

    bool converged = maxResidual(x, R) < 1e-12;
    for (int iter = 0; iter < 60 && !converged; ++iter) {
        double j[6][6];
        double b[6];

        for (int i = 0; i < 5; ++i) {
            const Vec3 rq = R * uprightPoints_[i];
            const Vec3 v = rq + x - chassisPoints_[i];
            const Real len = glm::length(v);
            const Vec3 n = (len > 1e-12) ? v / len : Vec3(0.0);
            const Vec3 rot = glm::cross(rq, n);
            for (int k = 0; k < 3; ++k) {
                j[i][k] = n[k];
                j[i][k + 3] = rot[k];
            }
            b[i] = -(len - lengths_[i]);
        }

        const Vec3 rqWc = R * wcLocal;
        const Vec3 axis = Vec3(0.0, 0.0, 1.0);
        const Vec3 rotWc = glm::cross(rqWc, axis);
        for (int k = 0; k < 3; ++k) {
            j[5][k] = axis[k];
            j[5][k + 3] = rotWc[k];
        }
        b[5] = -((rqWc + x).z - targetZ);

        double d[6];
        if (!solve6x6(j, b, d)) {
            break;
        }
        const Vec3 dx(d[0], d[1], d[2]);
        const Vec3 dr(d[3], d[4], d[5]);

        // Backtracking line search: shrink the step until the residual drops.
        const double current = maxResidual(x, R);
        double alpha = 1.0;
        Vec3 nextX = x;
        Mat3 nextR = R;
        double next = current;
        for (int trial = 0; trial < 20; ++trial) {
            nextX = x + alpha * dx;
            nextR = expMap(alpha * dr) * R;
            next = maxResidual(nextX, nextR);
            if (next < current) {
                break;
            }
            alpha *= 0.5;
        }
        x = nextX;
        R = nextR;
        converged = next < 1e-12;
    }

    CornerState state = finalize(x, R);
    state.converged = converged;
    return state;
}

Real DoubleWishbone::motionRatio(Real travel) const {
    const Real h = 1e-4;
    const CornerState low = solveForTravel(travel - h);
    const CornerState high = solveForTravel(travel + h, low);
    return (high.springLength - low.springLength) / (2.0 * h);
}

Vec3 rollCentre(const CornerState& left, const CornerState& right) {
    const glm::dvec2 lp(left.frontInstantCentre.x, left.frontInstantCentre.y);
    const glm::dvec2 lc(left.wheelCentre.x, 0.0);
    const glm::dvec2 rp(right.frontInstantCentre.x, right.frontInstantCentre.y);
    const glm::dvec2 rc(right.wheelCentre.x, 0.0);

    glm::dvec2 out(0.0);
    if (!intersectLines(lp, lc - lp, rp, rc - rp, out)) {
        return Vec3(0.0);
    }
    return Vec3(out.x, out.y, 0.0);
}

Real antiDiveFraction(const CornerState& corner, Real wheelbase, Real cgHeight) {
    const Vec3 ic = corner.sideInstantCentre;
    const Real dz = corner.wheelCentre.z - ic.z;
    if (std::abs(dz) < 1e-9 || cgHeight <= 0.0) {
        return 0.0;
    }
    return (ic.y / dz) * (wheelbase / cgHeight);
}

} // namespace sim::vehicle
