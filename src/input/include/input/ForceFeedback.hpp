// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "input/Types.hpp"

namespace sim::input {

// User-tunable force feedback settings plus the capabilities of the device they
// are applied to. Capabilities gate every effect: a setting for an effect the
// device does not expose is simply never sent.
struct FfbSettings {
    bool enabled = true;
    double masterGain = 0.7;        // [0, 1] scales everything
    double constantForceGain = 1.0; // [0, 1]
    double springGain = 1.0;        // [0, 1]
    double damperGain = 1.0;        // [0, 1]
    double frictionGain = 1.0;      // [0, 1]
    double rumbleGain = 1.0;        // [0, 1]
    bool invertDirection = false;
    double maxForce = 1.0; // [0, 1] hard clamp on the main torque
    FfbCapabilities caps{};
};

// Physics-derived demand for one frame, already normalised by the caller.
struct FfbInput {
    double steeringTorque = 0.0; // [-1, 1] self-aligning torque, sign = direction
    double understeer = 0.0;     // [0, 1] front slip (lightens the wheel)
    double oversteer = 0.0;      // [0, 1]
    double kerbRumble = 0.0;     // [0, 1] road texture / kerb vibration
    double impact = 0.0;         // [0, 1] collision, decays on its own
    double speed = 0.0;          // [0, 1] normalised speed, gates some effects
};

// The concrete effect values to send this frame.
struct FfbCommand {
    bool active = false;        // false = all effects must be stopped
    double constantForce = 0.0; // [-1, 1]
    double spring = 0.0;        // [-1, 1]
    double damper = 0.0;        // [-1, 1]
    double friction = 0.0;      // [-1, 1]
    double rumbleLow = 0.0;     // [0, 1]
    double rumbleHigh = 0.0;    // [0, 1]
};

// Turns physics demand into effect values and enforces safety. It never touches
// the device itself: the caller forwards the returned command to the device's
// IFfbOutput, so the whole controller is unit-testable without hardware.
//
// Safety rules (all of them force `active = false`, i.e. stop everything):
//   - the user disabled force feedback,
//   - no device, or focus lost, or the application is shutting down,
//   - a `requestStop()` was issued.
class ForceFeedbackController {
public:
    FfbSettings& settings() { return settings_; }
    const FfbSettings& settings() const { return settings_; }

    // Latches a stop request (application closing, device error, ...).
    void requestStop() { stopRequested_ = true; }
    // Clears the stop request and resumes normal operation.
    void clearStop() { stopRequested_ = false; }

    // Starts a bounded, safe self-test: a gentle oscillation for `seconds`.
    void startTest(double seconds);
    bool testing() const { return testRemaining_ > 0.0; }

    // Recomputes the command for this frame. `dt` drives the impact decay and
    // the test timer. `devicePresent` and `windowFocused` are safety inputs.
    FfbCommand update(const FfbInput& input, bool devicePresent, bool windowFocused, double dt);

    bool active() const { return active_; }

private:
    FfbSettings settings_{};
    bool stopRequested_ = false;
    bool active_ = false;
    double impactEnvelope_ = 0.0;
    double testRemaining_ = 0.0;
    double testPhase_ = 0.0;
};

} // namespace sim::input
