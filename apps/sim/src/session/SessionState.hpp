// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>

namespace sim::core {
class Config;
} // namespace sim::core

namespace sim::app {

// Which screen the windowed app is showing. The session flow is
// MainMenu -> Garage -> TrackSelect -> (ControlsWizard) -> Summary -> Driving,
// and any screen can jump back to the menu with Back.
enum class GameScreen {
    MainMenu,
    Garage,         // car selection with the 3D preview
    TrackSelect,    // circuit selection
    ControlsWizard, // peripheral setup (also reachable from Options)
    Summary,        // persistent recap + "Start session"
    Driving,        // the simulation is running
};

const char* gameScreenName(GameScreen screen);

// What the player picked. Persisted in the user's config so the next launch
// starts where they left off. Ids are resolved against CarCatalog/TrackCatalog;
// unknown ids fall back to the catalog defaults.
struct SessionSelection {
    std::string carId = "voyager";
    std::string trackId = "sunset_circuit";
    std::string modeId = "practice";
    std::string weatherId = "clear";
    std::string timeOfDayId = "day";
};

// The whole pre-session state: the selection, the current screen and the
// first-run flag that decides whether the controls wizard must be completed
// before the first drive.
struct SessionState {
    SessionSelection selection;
    GameScreen screen = GameScreen::MainMenu;
    bool controlsConfigured = false;
    bool reducedMotion = false; // accessibility: shorter/no transitions

    // Loads/creates the persisted fields (missing keys keep the defaults).
    void load(const core::Config& config);
    // Writes the persisted fields into the in-memory config document; the app
    // flushes it to disk on shutdown.
    void save(core::Config& config) const;
};

} // namespace sim::app
