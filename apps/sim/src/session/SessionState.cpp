// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/SessionState.hpp"

#include "core/Config.hpp"

namespace sim::app {

const char* gameScreenName(GameScreen screen) {
    switch (screen) {
    case GameScreen::MainMenu:
        return "MainMenu";
    case GameScreen::Garage:
        return "Garage";
    case GameScreen::TrackSelect:
        return "TrackSelect";
    case GameScreen::ControlsWizard:
        return "ControlsWizard";
    case GameScreen::Summary:
        return "Summary";
    case GameScreen::Driving:
        return "Driving";
    }
    return "Unknown";
}

void SessionState::load(const core::Config& config) {
    selection.carId = config.getString("session.car", selection.carId);
    selection.trackId = config.getString("session.track", selection.trackId);
    selection.modeId = config.getString("session.mode", selection.modeId);
    selection.weatherId = config.getString("session.weather", selection.weatherId);
    selection.timeOfDayId = config.getString("session.time_of_day", selection.timeOfDayId);
    controlsConfigured = config.getBool("session.controls_configured", controlsConfigured);
    reducedMotion = config.getBool("ui.reduced_motion", reducedMotion);
}

void SessionState::save(core::Config& config) const {
    config.setString("session.car", selection.carId);
    config.setString("session.track", selection.trackId);
    config.setString("session.mode", selection.modeId);
    config.setString("session.weather", selection.weatherId);
    config.setString("session.time_of_day", selection.timeOfDayId);
    config.setBool("session.controls_configured", controlsConfigured);
    config.setBool("ui.reduced_motion", reducedMotion);
}

} // namespace sim::app
