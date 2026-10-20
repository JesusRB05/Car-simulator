// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include "session/ControlSetupWizard.hpp"
#include "session/SessionState.hpp"

namespace sim::input {
class WheelManager;
} // namespace sim::input

namespace sim::app {

// The pre-session flow drawn over the garage backdrop: the main menu, the car
// and circuit pickers, the recap and the control wizard. It only edits the
// SessionState; the application reacts to the action it returns.
class SessionUI {
public:
    enum class Action {
        None,       // stay on the current screen
        StartDrive, // the player pressed Drive on the recap
        Quit,       // the player asked to quit
    };

    SessionUI();

    // Draws the current screen. `wheel` may be null (headless); when it is the
    // controls wizard is unavailable.
    Action draw(SessionState& state, input::WheelManager* wheel);

    bool wizardActive() const { return wizardActive_; }
    void openControlsWizard();

    // True exactly once after the wizard has been completed this frame, so the
    // caller can mark the session as configured and persist it.
    bool consumeWizardFinished();

private:
    void drawMainMenu(SessionState& state, bool haveWheel, Action& action);
    void drawCarSelect(SessionState& state, Action& action);
    void drawTrackSelect(SessionState& state, Action& action);
    void drawSummary(SessionState& state, bool haveWheel, Action& action);

    void drawCarCards(SessionState& state);
    void drawCarSpecs(const struct CarSpec& car);
    void drawSkinPicker(SessionState& state, const struct CarSpec& car);

    ControlSetupWizard wizard_;
    bool wizardActive_ = false;
    bool wizardFinishedThisFrame_ = false;
    bool startAfterWizard_ = false;

    int modeIndex_ = 0;
    int variantIndex_ = 0;
};

} // namespace sim::app
