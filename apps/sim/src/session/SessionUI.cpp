// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/SessionUI.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <imgui.h>

#include "input/WheelManager.hpp"
#include "session/CarCatalog.hpp"
#include "session/TrackCatalog.hpp"

namespace sim::app {
namespace {

struct Choice {
    const char* id;
    const char* label;
    bool available;
    const char* note;
};

const Choice kModes[] = {
    {"practice", "Practice", true, ""},
    {"time_trial", "Time trial", false, "not implemented yet"},
    {"race", "Race", false, "not implemented yet"},
};

const Choice kWeather[] = {
    {"clear", "Clear", true, ""},
    {"rain", "Rain", false, "no weather simulation yet"},
    {"storm", "Storm", false, "no weather simulation yet"},
};

const Choice kTimeOfDay[] = {
    {"day", "Day", true, ""},
    {"dusk", "Dusk", false, "no time-of-day simulation yet"},
    {"night", "Night", false, "no time-of-day simulation yet"},
};

const Choice& choiceById(const Choice* table, int count, const std::string& id) {
    for (int i = 0; i < count; ++i) {
        if (id == table[i].id) {
            return table[i];
        }
    }
    return table[0];
}

const char* drivetrainHeadline(const CarSpec& car) {
    return car.drivetrain.empty() ? "Unknown" : car.drivetrain.c_str();
}

// Draws the circuit centreline (already normalised to the unit box) into a box.
void drawMinimap(const TrackSpec& track, ImVec2 size) {
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImVec2 bottom(origin.x + size.x, origin.y + size.y);
    draw->AddRectFilled(origin, bottom, IM_COL32(18, 20, 24, 255), 4.0f);
    draw->AddRect(origin, bottom, IM_COL32(60, 64, 72, 255), 4.0f);

    const float scale = std::min(size.x, size.y) * 0.44f;
    const ImVec2 center(origin.x + size.x * 0.5f, origin.y + size.y * 0.5f);

    if (track.minimap.size() >= 2) {
        std::vector<ImVec2> points;
        points.reserve(track.minimap.size());
        for (const glm::vec2& p : track.minimap) {
            points.emplace_back(center.x + p.x * scale, center.y - p.y * scale);
        }
        draw->AddPolyline(points.data(), static_cast<int>(points.size()), IM_COL32(96, 196, 128, 255), ImDrawFlags_None,
                          2.5f);
        draw->AddCircleFilled(points.front(), 4.0f, IM_COL32(240, 220, 120, 255));
    }
    ImGui::Dummy(size);
}

} // namespace

SessionUI::SessionUI() = default;

void SessionUI::openControlsWizard() {
    wizard_.open();
    wizardActive_ = true;
}

bool SessionUI::consumeWizardFinished() {
    const bool finished = wizardFinishedThisFrame_;
    wizardFinishedThisFrame_ = false;
    return finished;
}

SessionUI::Action SessionUI::draw(SessionState& state, input::WheelManager* wheel) {
    const CarCatalog& cars = CarCatalog::instance();
    const TrackCatalog& tracks = TrackCatalog::instance();

    // Keep the selection pointing at something that exists.
    if (cars.find(state.selection.carId) == nullptr) {
        if (const CarSpec* car = cars.defaultCar()) {
            state.selection.carId = car->ref();
        }
    }
    if (tracks.find(state.selection.trackId) == nullptr) {
        if (const TrackSpec* track = tracks.defaultTrack()) {
            state.selection.trackId = track->ref();
        }
    }

    Action action = Action::None;

    if (wizardActive_) {
        if (wheel == nullptr) {
            wizardActive_ = false;
        } else {
            const ControlSetupWizard::Result result = wizard_.draw(*wheel);
            if (result == ControlSetupWizard::Result::Finished) {
                state.controlsConfigured = true;
                wizardActive_ = false;
                wizardFinishedThisFrame_ = true;
                if (startAfterWizard_) {
                    startAfterWizard_ = false;
                    state.screen = GameScreen::Summary;
                }
            } else if (result == ControlSetupWizard::Result::Cancelled) {
                wizardActive_ = false;
                startAfterWizard_ = false;
            }
        }
        return action;
    }

    switch (state.screen) {
    case GameScreen::MainMenu:
        drawMainMenu(state, wheel != nullptr, action);
        break;
    case GameScreen::Garage:
        drawCarSelect(state, action);
        break;
    case GameScreen::TrackSelect:
        drawTrackSelect(state, action);
        break;
    case GameScreen::Summary:
        drawSummary(state, wheel != nullptr, action);
        break;
    case GameScreen::Driving:
        break;
    }
    return action;
}

void SessionUI::drawMainMenu(SessionState& state, bool haveWheel, Action& action) {
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always,
                            ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(440.0f, 0.0f), ImGuiCond_Always);
    ImGui::Begin("##main_menu", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Dummy(ImVec2(0.0f, 6.0f));
    ImGui::SetWindowFontScale(2.0f);
    ImGui::TextUnformatted("CAR SIMULATOR");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 4.0f));

    const CarSpec* car = CarCatalog::instance().find(state.selection.carId);
    const TrackSpec* track = TrackCatalog::instance().find(state.selection.trackId);
    if (car == nullptr || track == nullptr) {
        ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f),
                           "No content loaded: install a car and a circuit (see mods).");
        ImGui::Dummy(ImVec2(0.0f, 8.0f));
    } else {
        ImGui::TextDisabled("Car:   %s %s", car->manufacturer.c_str(), car->model.c_str());
        ImGui::TextDisabled("Track: %s (%.0f m)", track->name.c_str(), track->lengthMeters);
        ImGui::Dummy(ImVec2(0.0f, 8.0f));
    }

    const ImVec2 button(ImGui::GetContentRegionAvail().x, 34.0f);
    if (ImGui::Button("Start session", button)) {
        if (!state.controlsConfigured && haveWheel) {
            startAfterWizard_ = true;
            openControlsWizard();
        } else {
            state.screen = GameScreen::Summary;
        }
    }
    if (ImGui::Button("Choose car", button)) {
        state.screen = GameScreen::Garage;
    }
    if (ImGui::Button("Choose track", button)) {
        state.screen = GameScreen::TrackSelect;
    }
    ImGui::BeginDisabled(!haveWheel);
    if (ImGui::Button(haveWheel ? "Configure controls" : "Configure controls (no device)", button)) {
        openControlsWizard();
    }
    ImGui::EndDisabled();
    ImGui::Checkbox("Reduced motion", &state.reducedMotion);
    ImGui::TextDisabled("Driver aids");
    ImGui::Checkbox("ABS", &state.abs);
    ImGui::SameLine();
    ImGui::Checkbox("TCS", &state.tcs);
    ImGui::SameLine();
    ImGui::Checkbox("ESC", &state.esc);
    ImGui::SetItemTooltip(
        "Anti-lock brakes, traction and stability control. Brake hard or launch the car to feel them work.");
    ImGui::Dummy(ImVec2(0.0f, 6.0f));
    if (ImGui::Button("Quit", button)) {
        action = Action::Quit;
    }

    if (!state.controlsConfigured && haveWheel) {
        ImGui::Dummy(ImVec2(0.0f, 4.0f));
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "Controls not configured yet.");
    }
    ImGui::End();
}

void SessionUI::drawCarCards(SessionState& state) {
    const CarCatalog& cars = CarCatalog::instance();
    for (const CarSpec& car : cars.cars()) {
        ImGui::PushID(car.id.c_str());
        const bool selected = car.id == state.selection.carId;
        if (ImGui::Selectable((" " + car.manufacturer + " " + car.model).c_str(), selected, 0, ImVec2(0.0f, 34.0f))) {
            state.selection.carId = car.id;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", car.category.c_str());
        ImGui::PopID();
    }
}

void SessionUI::drawCarSpecs(const CarSpec& car) {
    ImGui::Text("%s %s", car.manufacturer.c_str(), car.model.c_str());
    ImGui::Separator();
    ImGui::TextWrapped("%s", car.description.c_str());
    ImGui::Dummy(ImVec2(0.0f, 6.0f));
    if (ImGui::BeginTable("car_specs", 2, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Power");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.1f kW (%.0f hp)", car.peakPowerKw, car.peakPowerKw * 1.34102);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Torque");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.0f Nm", car.peakTorqueNm);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Mass");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.0f kg", car.massKg);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Drivetrain");
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(drivetrainHeadline(car));
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Gearbox");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%d-speed", car.gearCount);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Wheelbase");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.2f m", car.wheelbaseM);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Track width");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.2f m", car.trackWidthM);
        ImGui::EndTable();
    }
}

void SessionUI::drawCarSelect(SessionState& state, Action& action) {
    (void)action;
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(32.0f, 72.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(430.0f, std::max(320.0f, io.DisplaySize.y - 144.0f)), ImGuiCond_Always);
    ImGui::Begin("Choose your car", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    ImGui::TextWrapped("Drag with the right mouse button to rotate the car.");
    ImGui::Separator();
    drawCarCards(state);
    ImGui::Separator();

    const CarSpec* car = CarCatalog::instance().find(state.selection.carId);
    if (car != nullptr) {
        drawCarSpecs(*car);
    }

    ImGui::SetCursorPosY(ImGui::GetWindowHeight() - 44.0f);
    ImGui::Separator();
    if (ImGui::Button("Back")) {
        state.screen = GameScreen::MainMenu;
    }
    ImGui::SameLine();
    if (ImGui::Button("Continue")) {
        state.screen = GameScreen::TrackSelect;
    }
    ImGui::End();
}

void SessionUI::drawTrackSelect(SessionState& state, Action& action) {
    (void)action;
    const TrackCatalog& tracks = TrackCatalog::instance();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always,
                            ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(560.0f, 0.0f), ImGuiCond_Always);
    ImGui::Begin("Choose your track", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_AlwaysAutoResize);

    for (const TrackSpec& track : tracks.tracks()) {
        const bool selected = track.id == state.selection.trackId;
        if (ImGui::RadioButton(track.name.c_str(), selected)) {
            state.selection.trackId = track.id;
        }
    }
    ImGui::Separator();

    const TrackSpec* track = tracks.find(state.selection.trackId);
    if (track != nullptr) {
        if (ImGui::BeginTable("track_layout", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            drawMinimap(*track, ImVec2(220.0f, 160.0f));
            ImGui::TableSetColumnIndex(1);
            ImGui::TextWrapped("%s", track->description.c_str());
            ImGui::Dummy(ImVec2(0.0f, 4.0f));
            ImGui::TextDisabled("Length  %.0f m", track->lengthMeters);
            ImGui::TextDisabled("Width   %.1f m", track->widthMeters);
            ImGui::EndTable();
        }

        ImGui::Dummy(ImVec2(0.0f, 6.0f));
        ImGui::SetNextItemWidth(240.0f);
        if (ImGui::BeginCombo("Layout", track->variants.empty() ? "Full circuit" : track->variants[0].c_str())) {
            for (std::size_t i = 0; i < track->variants.size(); ++i) {
                if (ImGui::Selectable(track->variants[i].c_str(), i == static_cast<std::size_t>(variantIndex_))) {
                    variantIndex_ = static_cast<int>(i);
                }
            }
            ImGui::EndCombo();
        }
    }

    ImGui::Separator();
    // Game mode: only Practice exists today; the rest are listed but disabled.
    const Choice& mode =
        choiceById(kModes, static_cast<int>(sizeof(kModes) / sizeof(kModes[0])), state.selection.modeId);
    ImGui::SetNextItemWidth(240.0f);
    if (ImGui::BeginCombo("Game mode", mode.label)) {
        for (int i = 0; i < static_cast<int>(sizeof(kModes) / sizeof(kModes[0])); ++i) {
            const Choice& c = kModes[i];
            ImGui::BeginDisabled(!c.available);
            if (ImGui::Selectable(c.label, state.selection.modeId == c.id)) {
                state.selection.modeId = c.id;
            }
            if (!c.available && c.note[0] != '\0') {
                ImGui::SameLine();
                ImGui::TextDisabled("(%s)", c.note);
            }
            ImGui::EndDisabled();
        }
        ImGui::EndCombo();
    }

    // Weather and time-of-day: declared but not simulated.
    const Choice& weather =
        choiceById(kWeather, static_cast<int>(sizeof(kWeather) / sizeof(kWeather[0])), state.selection.weatherId);
    ImGui::BeginDisabled(true);
    ImGui::SetNextItemWidth(240.0f);
    if (ImGui::BeginCombo("Weather", weather.label)) {
        ImGui::EndCombo();
    }
    const Choice& timeOfDay = choiceById(kTimeOfDay, static_cast<int>(sizeof(kTimeOfDay) / sizeof(kTimeOfDay[0])),
                                         state.selection.timeOfDayId);
    ImGui::SetNextItemWidth(240.0f);
    if (ImGui::BeginCombo("Time of day", timeOfDay.label)) {
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();
    ImGui::TextDisabled("Weather and time of day are not simulated yet.");

    ImGui::Separator();
    if (ImGui::Button("Back")) {
        state.screen = GameScreen::Garage;
    }
    ImGui::SameLine();
    if (ImGui::Button("Continue")) {
        state.screen = GameScreen::Summary;
    }
    ImGui::End();
}

void SessionUI::drawSummary(SessionState& state, bool haveWheel, Action& action) {
    const CarSpec* car = CarCatalog::instance().find(state.selection.carId);
    const TrackSpec* track = TrackCatalog::instance().find(state.selection.trackId);
    const Choice& mode =
        choiceById(kModes, static_cast<int>(sizeof(kModes) / sizeof(kModes[0])), state.selection.modeId);
    const Choice& weather =
        choiceById(kWeather, static_cast<int>(sizeof(kWeather) / sizeof(kWeather[0])), state.selection.weatherId);
    const Choice& timeOfDay = choiceById(kTimeOfDay, static_cast<int>(sizeof(kTimeOfDay) / sizeof(kTimeOfDay[0])),
                                         state.selection.timeOfDayId);

    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Always,
                            ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(480.0f, 0.0f), ImGuiCond_Always);
    ImGui::Begin("Session", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::BeginTable("summary", 2, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Car");
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(car != nullptr ? (car->manufacturer + " " + car->model).c_str() : "none");
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Track");
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(track != nullptr ? track->name.c_str() : "none");
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Mode");
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(mode.label);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Weather");
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(weather.label);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Time");
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(timeOfDay.label);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Controls");
        ImGui::TableSetColumnIndex(1);
        if (state.controlsConfigured) {
            ImGui::TextUnformatted("configured");
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "keyboard only");
        }
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextDisabled("Driver aids");
        ImGui::TableSetColumnIndex(1);
        char aids[64];
        std::snprintf(aids, sizeof(aids), "%s %s %s", state.abs ? "ABS" : "-", state.tcs ? "TCS" : "-",
                      state.esc ? "ESC" : "-");
        ImGui::TextUnformatted(aids);
        ImGui::EndTable();
    }

    if (!state.controlsConfigured) {
        ImGui::Spacing();
        ImGui::TextWrapped("No wheel has been set up yet. You can drive with the keyboard, or configure your "
                           "controls now.");
        ImGui::BeginDisabled(!haveWheel);
        if (ImGui::Button("Configure controls")) {
            openControlsWizard();
        }
        ImGui::EndDisabled();
    }

    ImGui::Separator();
    if (ImGui::Button("Back")) {
        state.screen = GameScreen::TrackSelect;
    }
    ImGui::SameLine();
    if (ImGui::Button("Drive", ImVec2(160.0f, 32.0f))) {
        action = Action::StartDrive;
    }
    ImGui::End();
}

} // namespace sim::app
