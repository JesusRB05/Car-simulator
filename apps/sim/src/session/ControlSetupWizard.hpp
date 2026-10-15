// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <vector>

namespace sim::input {
class WheelManager;
enum class ControlRole;
} // namespace sim::input

namespace sim::app {

// The step-by-step peripheral setup shown before the first drive (and from
// Options). It pairs the player's wheel, pedals and shifter with the engine's
// input layer: it never reads the OS itself, it drives sim::input::WheelManager.
//
// Steps: 0 Devices, 1 Compatibility, 2 Steering, 3 Pedals, 4 Buttons, 5 Test.
class ControlSetupWizard {
public:
    enum class Result {
        None,     // still running, keep drawing
        Finished, // the player completed the wizard
        Cancelled // the player backed out without changes being mandatory
    };

    // Starts (or restarts) the wizard at the first step.
    void open();

    // Draws the current step and processes it. Returns None until the player
    // finishes or cancels, then the wizard deactivates itself.
    Result draw(sim::input::WheelManager& wheel);

    // Which step the wizard would show if opened again (skips nothing; kept for
    // the caller to restore a partially completed wizard).
    int step() const { return step_; }

private:
    void initStep(sim::input::WheelManager& wheel);
    void drawDevices(sim::input::WheelManager& wheel);
    void drawCompatibility(sim::input::WheelManager& wheel);
    void drawSteering(sim::input::WheelManager& wheel);
    void drawPedals(sim::input::WheelManager& wheel);
    void drawButtons(sim::input::WheelManager& wheel);
    void drawTest(sim::input::WheelManager& wheel);

    void beginListening(const sim::input::WheelManager& wheel, int role);
    // Scans the selected device for a fresh button / hat / axis event while
    // listening. Returns the role that was bound, or -1.
    int pollListening(sim::input::WheelManager& wheel);
    // True when `button` on the selected device is already used by another role.
    bool buttonConflict(const sim::input::WheelManager& wheel, int role, int button, int& other) const;

    std::size_t clampDevice(const sim::input::WheelManager& wheel) const;

    int step_ = 0;
    int lastInitStep_ = -1;
    int lastDeviceCount_ = -1;

    std::size_t deviceIndex_ = 0;

    // Steering capture.
    int steerAxis_ = 0;
    double steerMin_ = -1.0;
    double steerMax_ = 1.0;
    double steerCenter_ = 0.0;
    bool haveMin_ = false;
    bool haveMax_ = false;
    bool haveCenter_ = false;
    double steeringRange_ = 900.0;
    double steerDeadzone_ = 0.02;
    bool steerInvert_ = false;

    // Pedals.
    double pedalDeadzone_ = 0.02;
    bool combinedPedals_ = false;
    int combinedAxis_ = 1;
    std::vector<double> pedalRest_; // per-axis rest value captured at step entry
    std::vector<double> pedalMin_;  // observed travel, per axis
    std::vector<double> pedalMax_;

    // Button binding.
    int listeningRole_ = -1;
    std::vector<bool> buttonBaseline_;
    std::vector<int> hatBaseline_;

    // Conflict confirmation pending a second click.
    int conflictRole_ = -1;
    int conflictButton_ = -1;
    int conflictOther_ = -1;
};

} // namespace sim::app
