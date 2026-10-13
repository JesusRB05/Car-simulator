// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace sim::vehicle {

using physics::Real;

// Gearbox (PHYSICS_PLAN Section 7.3). Gears are indexed as in the drivetrain
// input: -1 reverse, 0 neutral, 1..N forward. The output torque scales with the
// ratio and the efficiency and the inertias reflect through the square of the
// ratio.
struct Gearbox {
    std::vector<Real> forwardRatios; // gear 1..N, positive and decreasing
    Real reverseRatio = 0.0;         // magnitude; the output turns backwards
    Real efficiency = 0.95;          // eta_g

    // Signed overall ratio for a gear (negative in reverse, zero in neutral).
    Real ratio(int gear) const;

    // Torque at the output shaft from the input torque (N*m).
    Real outputTorque(int gear, Real inputTorque) const;

    // Inertia seen at the input, J_out + g^2 J_in (kg*m^2).
    Real reflectedInertia(int gear, Real inputInertia, Real outputInertia) const;
};

// Torque converter (automatic transmissions). The capacity factor K and the torque
// ratio TR are functions of the speed ratio; here they are given as values for the
// current step.
struct TorqueConverter {
    Real capacityFactor = 0.0; // K(N*m*s^2/rad^2)
    Real torqueRatio = 1.0;    // TR

    Real pumpTorque(Real pumpAngularVelocity) const; // T_p = K * w_p^2
    Real turbineTorque(Real pumpTorque) const;       // T_t = TR * T_p
};

// Sequential shift controller (PHYSICS_PLAN Section 7.3). A request interrupts the
// torque for the shift time (the gears swap at the end of it) and then ramps it
// back in over the blend time, which is the torque interruption a dog or
// synchroniser engagement needs.
class SequentialShift {
public:
    Real shiftTime = 0.2;  // s of torque interruption
    Real blendTime = 0.05; // s to ramp the torque back in

    void request(int gear);
    void step(Real deltaSeconds);

    Real torqueFactor() const; // 0 during the interruption, then ramp to 1
    bool shifting() const { return active_; }
    int gear() const { return currentGear_; }
    int target() const { return targetGear_; }

private:
    int currentGear_ = 0;
    int targetGear_ = 0;
    Real elapsed_ = 0.0;
    bool active_ = false;
};

} // namespace sim::vehicle
