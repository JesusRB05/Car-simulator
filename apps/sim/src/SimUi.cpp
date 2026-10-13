// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "SimUi.hpp"

#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "OrbitCamera.hpp"
#include "Scenes.hpp"
#include "core/Version.hpp"
#include "physics/PhysicsWorld.hpp"

namespace sim::app {
namespace {

constexpr const char* kSettingsType = "SimUi";
constexpr double kHistorySeconds = 10.0;
constexpr float kMinTimeScale = 0.1f;
constexpr float kMaxTimeScale = 4.0f;

// Window titles double as their .ini identifiers; keep them stable.
constexpr const char* kSceneBrowserTitle = "Scenes";
constexpr const char* kSimulationTitle = "Simulation";
constexpr const char* kDebugDrawTitle = "Debug draw";
constexpr const char* kTelemetryTitle = "Telemetry";
constexpr const char* kCameraTitle = "Camera";

struct PanelField
{
    const char* key;
    bool PanelVisibility::*member;
};

constexpr PanelField kPanelFields[] = {
    {"SceneBrowser", &PanelVisibility::sceneBrowser},
    {"Simulation", &PanelVisibility::simulation},
    {"DebugDraw", &PanelVisibility::debugDraw},
    {"Telemetry", &PanelVisibility::telemetry},
    {"Camera", &PanelVisibility::camera},
    {"Vehicle", &PanelVisibility::vehicle},
    {"Mods", &PanelVisibility::mods},
    {"Input", &PanelVisibility::input},
};

struct ShortcutEntry
{
    const char* keys;
    const char* action;
};

constexpr ShortcutEntry kShortcuts[] = {
    {"Space", "Play / pause the physics scene"},
    {".", "Advance one fixed step (pauses)"},
    {"R", "Reset the physics scene"},
    {"F", "Frame the scene"},
    {"G", "Toggle the ground grid"},
    {"C", "Cycle chase / orbit / free camera (driving scene)"},
    {"Left drag", "Orbit the camera (inspection scenes)"},
    {"Right drag", "Pan the camera (inspection scenes)"},
    {"Mouse wheel", "Zoom"},
    {"W/A/S/D, arrows", "Drive (driving scene) / fly (free camera)"},
    {"Space", "Handbrake (driving scene)"},
    {"F1", "Show this list"},
    {"Ctrl+Q", "Quit"},
};

// Places a panel on first use only; afterwards the .ini layout wins. Offsets
// are from the top-left (positive) or bottom-right (negative) corner of the
// area below the main menu bar.
void placeWindow(float x, float y, float width, float height)
{
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    // WorkPos only accounts for the menu bar from the frame after it is first
    // submitted, so reserve its height explicitly.
    const float top = viewport->Pos.y + ImGui::GetFrameHeight();
    const float bottom = viewport->Pos.y + viewport->Size.y;
    const float px = x < 0.0f ? viewport->Pos.x + viewport->Size.x + x - width : viewport->Pos.x + x;
    const float py = y < 0.0f ? bottom + y - height : top + y;
    ImGui::SetNextWindowPos(ImVec2(px, py), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(width, height), ImGuiCond_FirstUseEver);
}

void trimFront(std::vector<double>& values, std::size_t count)
{
    values.erase(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(count));
}

// --- ImGui .ini handler: persists PanelVisibility as "Key=0/1" lines. ---

void* settingsReadOpen(ImGuiContext*, ImGuiSettingsHandler* handler, const char* name)
{
    return std::strcmp(name, "Panels") == 0 ? handler->UserData : nullptr;
}

void settingsReadLine(ImGuiContext*, ImGuiSettingsHandler*, void* entry, const char* line)
{
    auto* panels = static_cast<PanelVisibility*>(entry);
    const char* equals = std::strchr(line, '=');
    if (equals == nullptr)
    {
        return;
    }
    const std::size_t keyLength = static_cast<std::size_t>(equals - line);
    const bool value = std::strtol(equals + 1, nullptr, 10) != 0;
    for (const PanelField& field : kPanelFields)
    {
        if (std::strlen(field.key) == keyLength && std::strncmp(line, field.key, keyLength) == 0)
        {
            panels->*field.member = value;
        }
    }
}

void settingsWriteAll(ImGuiContext*, ImGuiSettingsHandler* handler, ImGuiTextBuffer* out)
{
    const auto* panels = static_cast<const PanelVisibility*>(handler->UserData);
    out->appendf("[%s][Panels]\n", handler->TypeName);
    for (const PanelField& field : kPanelFields)
    {
        out->appendf("%s=%d\n", field.key, panels->*field.member ? 1 : 0);
    }
    out->append("\n");
}

} // namespace

SimUi::SimUi()
{
    registerSettingsHandler(this);
}

SimUi::~SimUi()
{
    if (ImGui::GetCurrentContext() != nullptr && ImGui::FindSettingsHandler(kSettingsType) != nullptr)
    {
        // Flush while the handler can still reach this object, then detach it
        // so a later save (context destruction) never sees a dangling pointer.
        if (ImGui::GetIO().IniFilename != nullptr)
        {
            ImGui::SaveIniSettingsToDisk(ImGui::GetIO().IniFilename);
        }
        ImGui::RemoveSettingsHandler(kSettingsType);
    }
}

void SimUi::registerSettingsHandler(SimUi* self)
{
    ImGuiSettingsHandler handler;
    handler.TypeName = kSettingsType;
    handler.TypeHash = ImHashStr(kSettingsType);
    handler.ReadOpenFn = settingsReadOpen;
    handler.ReadLineFn = settingsReadLine;
    handler.WriteAllFn = settingsWriteAll;
    handler.UserData = &self->panels_;
    ImGui::AddSettingsHandler(&handler);
}

void SimUi::resetHistory()
{
    energyBaselineValid_ = false;
    plotTime_.clear();
    plotEnergy_.clear();
    plotDrift_.clear();
    plotCpu_.clear();
    plotGpu_.clear();
    lastSimTime_ = -1.0;
    stepsPerSecond_ = 0.0;
}

SimUiActions SimUi::draw(const SimUiFrame& frame, OrbitCamera& camera)
{
    SimUiActions actions;
    const PanelVisibility before = panels_;

    handleShortcuts(frame, actions);
    handleCameraMouse(frame, camera);
    updateStatistics(frame);

    drawMenuBar(frame, actions);
    if (panels_.sceneBrowser)
    {
        drawSceneBrowser(frame, actions);
    }
    if (panels_.simulation)
    {
        drawSimulation(frame, actions);
    }
    if (panels_.debugDraw)
    {
        drawDebugDraw(frame);
    }
    if (panels_.telemetry)
    {
        drawTelemetry(frame);
    }
    if (panels_.camera)
    {
        drawCamera(frame, camera, actions);
    }
    drawAbout();
    drawShortcuts();

    if (std::memcmp(&before, &panels_, sizeof(PanelVisibility)) != 0)
    {
        ImGui::MarkIniSettingsDirty();
    }
    return actions;
}

void SimUi::handleShortcuts(const SimUiFrame& frame, SimUiActions& actions)
{
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput)
    {
        return;
    }
    const bool physics = frame.world != nullptr;

    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Q))
    {
        actions.quit = true;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false))
    {
        showShortcuts_ = !showShortcuts_;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_G, false) && !io.KeyCtrl)
    {
        debug_.grid = !debug_.grid;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F, false) && !io.KeyCtrl && !frame.activeScene.empty())
    {
        actions.frameScene = true;
    }
    if (!physics)
    {
        return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false))
    {
        paused_ = !paused_;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Period, true))
    {
        paused_ = true;
        actions.step = true;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_R, false) && !io.KeyCtrl)
    {
        actions.reset = true;
    }
}

void SimUi::handleCameraMouse(const SimUiFrame& frame, OrbitCamera& camera)
{
    // The driving scene keeps its own chase/orbit/free camera controls.
    const ImGuiIO& io = ImGui::GetIO();
    if (frame.activeScene.empty() || io.WantCaptureMouse)
    {
        return;
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f))
    {
        camera.orbit(io.MouseDelta.x, io.MouseDelta.y);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0f) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f))
    {
        camera.pan(io.MouseDelta.x, io.MouseDelta.y, frame.viewportHeight);
    }
    if (io.MouseWheel != 0.0f)
    {
        camera.zoom(io.MouseWheel);
    }
}

void SimUi::updateStatistics(const SimUiFrame& frame)
{
    // Steps per second from the simulated time actually advanced, averaged
    // over half-second windows of wall time.
    if (frame.world != nullptr)
    {
        const double simTime = frame.world->time();
        if (lastSimTime_ < 0.0 || simTime < lastSimTime_)
        {
            lastSimTime_ = simTime;
            rateWindowStart_ = frame.nowSeconds;
            rateWindowSim_ = 0.0;
        }
        rateWindowSim_ += simTime - lastSimTime_;
        lastSimTime_ = simTime;
        const double window = frame.nowSeconds - rateWindowStart_;
        if (window >= 0.5)
        {
            const double dt = frame.world->fixedStep;
            stepsPerSecond_ = dt > 0.0 ? rateWindowSim_ / dt / window : 0.0;
            rateWindowStart_ = frame.nowSeconds;
            rateWindowSim_ = 0.0;
        }
    }
    else
    {
        lastSimTime_ = -1.0;
        stepsPerSecond_ = 0.0;
    }

    double drift = std::numeric_limits<double>::quiet_NaN();
    double energy = std::numeric_limits<double>::quiet_NaN();
    if (frame.hasEnergy)
    {
        energy = frame.totalEnergy;
        if (!energyBaselineValid_)
        {
            energyBaseline_ = energy;
            energyBaselineValid_ = true;
        }
        const double scale = std::max(std::abs(energyBaseline_), 1e-9);
        drift = (energy - energyBaseline_) / scale;
    }

    plotTime_.push_back(frame.nowSeconds);
    plotEnergy_.push_back(energy);
    plotDrift_.push_back(drift);
    plotCpu_.push_back(frame.cpuFrameMs);
    plotGpu_.push_back(frame.gpuFrameMs);

    std::size_t expired = 0;
    while (expired < plotTime_.size() && plotTime_[expired] < frame.nowSeconds - kHistorySeconds)
    {
        ++expired;
    }
    // Trim in batches so the erase cost is amortised.
    if (expired > 64)
    {
        trimFront(plotTime_, expired);
        trimFront(plotEnergy_, expired);
        trimFront(plotDrift_, expired);
        trimFront(plotCpu_, expired);
        trimFront(plotGpu_, expired);
    }
}

void SimUi::drawSceneMenuItems(const SimUiFrame& frame, SimUiActions& actions)
{
    if (ImGui::MenuItem("Driving", nullptr, frame.activeScene.empty()))
    {
        actions.loadScene = true;
        actions.scene.clear();
    }
    for (const SceneCategory category : {SceneCategory::Physics, SceneCategory::RenderTest})
    {
        if (!ImGui::BeginMenu(sceneCategoryLabel(category)))
        {
            continue;
        }
        for (const SceneInfo& info : sceneCatalog())
        {
            if (info.category != category)
            {
                continue;
            }
            if (ImGui::MenuItem(info.name.c_str(), nullptr, frame.activeScene == info.name))
            {
                actions.loadScene = true;
                actions.scene = info.name;
            }
            ImGui::SetItemTooltip("%s", info.description.c_str());
        }
        ImGui::EndMenu();
    }
}

void SimUi::drawMenuBar(const SimUiFrame& frame, SimUiActions& actions)
{
    if (!ImGui::BeginMainMenuBar())
    {
        return;
    }
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Quit", "Ctrl+Q"))
        {
            actions.quit = true;
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View"))
    {
        ImGui::MenuItem("Scene browser", nullptr, &panels_.sceneBrowser);
        ImGui::MenuItem("Simulation", nullptr, &panels_.simulation);
        ImGui::MenuItem("Debug draw", nullptr, &panels_.debugDraw);
        ImGui::MenuItem("Telemetry", nullptr, &panels_.telemetry);
        ImGui::MenuItem("Camera", nullptr, &panels_.camera);
        ImGui::Separator();
        ImGui::MenuItem("Vehicle", nullptr, &panels_.vehicle);
        ImGui::MenuItem("Mods", nullptr, &panels_.mods);
        ImGui::MenuItem("Input", nullptr, &panels_.input);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Scenes"))
    {
        drawSceneMenuItems(frame, actions);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help"))
    {
        ImGui::MenuItem("Keyboard shortcuts", "F1", &showShortcuts_);
        ImGui::MenuItem("About", nullptr, &showAbout_);
        ImGui::EndMenu();
    }

    // Right-aligned status: active scene and frame rate.
    char status[128];
    std::snprintf(status, sizeof(status), "%s%s  |  %.0f FPS", frame.activeScene.empty() ? "driving" : frame.activeScene.c_str(),
                  frame.world != nullptr && paused_ ? " (paused)" : "", frame.fps);
    const float width = ImGui::CalcTextSize(status).x;
    ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowWidth() - width - ImGui::GetStyle().ItemSpacing.x * 2.0f));
    ImGui::TextDisabled("%s", status);
    ImGui::EndMainMenuBar();
}

void SimUi::drawSceneBrowser(const SimUiFrame& frame, SimUiActions& actions)
{
    placeWindow(10.0f, 10.0f, 260.0f, 470.0f);
    if (!ImGui::Begin(kSceneBrowserTitle, &panels_.sceneBrowser))
    {
        ImGui::End();
        return;
    }

    if (ImGui::Selectable("Driving", frame.activeScene.empty()))
    {
        actions.loadScene = true;
        actions.scene.clear();
    }
    ImGui::SetItemTooltip("Interactive vehicle on the physics thread.");

    for (const SceneCategory category : {SceneCategory::Physics, SceneCategory::RenderTest})
    {
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (!ImGui::CollapsingHeader(sceneCategoryLabel(category)))
        {
            continue;
        }
        for (const SceneInfo& info : sceneCatalog())
        {
            if (info.category != category)
            {
                continue;
            }
            if (ImGui::Selectable(info.name.c_str(), frame.activeScene == info.name))
            {
                actions.loadScene = true;
                actions.scene = info.name;
            }
            ImGui::SetItemTooltip("%s", info.description.c_str());
        }
    }

    // Description of the active scene, wrapped at the bottom.
    for (const SceneInfo& info : sceneCatalog())
    {
        if (info.name == frame.activeScene)
        {
            ImGui::Separator();
            ImGui::TextWrapped("%s", info.description.c_str());
        }
    }
    ImGui::End();
}

void SimUi::drawSimulation(const SimUiFrame& frame, SimUiActions& actions)
{
    placeWindow(-10.0f, 10.0f, 320.0f, 200.0f);
    if (!ImGui::Begin(kSimulationTitle, &panels_.simulation))
    {
        ImGui::End();
        return;
    }
    if (frame.world == nullptr)
    {
        ImGui::TextDisabled("No physics scene loaded.");
        ImGui::TextDisabled("Pick one under Scenes > Physics.");
        ImGui::End();
        return;
    }

    if (ImGui::Button(paused_ ? "Play" : "Pause", ImVec2(70.0f, 0.0f)))
    {
        paused_ = !paused_;
    }
    ImGui::SetItemTooltip("Space");
    ImGui::SameLine();
    if (ImGui::Button("Step"))
    {
        paused_ = true;
        actions.step = true;
    }
    ImGui::SetItemTooltip("Advance exactly one fixed step (.)");
    ImGui::SameLine();
    if (ImGui::Button("Reset"))
    {
        actions.reset = true;
    }
    ImGui::SetItemTooltip("Rebuild the scene from its initial state (R)");

    ImGui::SliderFloat("Time scale", &timeScale_, kMinTimeScale, kMaxTimeScale, "%.2fx", ImGuiSliderFlags_Logarithmic);
    ImGui::SetItemTooltip("Scales the wall time fed to the fixed-step integrator.\nThe step size itself never changes.");
    timeScale_ = std::clamp(timeScale_, kMinTimeScale, kMaxTimeScale);
    ImGui::SameLine();
    if (ImGui::SmallButton("1x"))
    {
        timeScale_ = 1.0f;
    }

    ImGui::Separator();
    const double dt = frame.world->fixedStep;
    ImGui::Text("Simulated time: %.3f s", frame.world->time());
    ImGui::Text("Fixed step:     %.3f ms", dt * 1000.0);
    ImGui::Text("Steps/s:        %.0f", stepsPerSecond_);
    ImGui::Text("Bodies:         %zu  Joints: %zu", frame.world->bodyCount(), frame.world->constraintCount());
    char hash[32];
    std::snprintf(hash, sizeof(hash), "%016" PRIx64, frame.world->stateHash());
    ImGui::Text("State hash:     %s", hash);
    ImGui::SetItemTooltip("FNV-1a over every body state: identical inputs and steps give identical hashes.");
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy"))
    {
        ImGui::SetClipboardText(hash);
    }
    ImGui::End();
}

void SimUi::drawDebugDraw(const SimUiFrame& frame)
{
    placeWindow(-10.0f, 220.0f, 320.0f, 230.0f);
    if (!ImGui::Begin(kDebugDrawTitle, &panels_.debugDraw))
    {
        ImGui::End();
        return;
    }
    ImGui::Checkbox("Ground grid (G)", &debug_.grid);
    ImGui::Checkbox("World axes", &debug_.worldAxes);
    ImGui::SeparatorText("Physics scenes");
    ImGui::BeginDisabled(frame.world == nullptr);
    ImGui::Checkbox("Body local axes", &debug_.bodyAxes);
    ImGui::Checkbox("Joints", &debug_.joints);
    ImGui::Checkbox("Springs", &debug_.springs);
    ImGui::Checkbox("Velocity vectors", &debug_.velocities);
    ImGui::Checkbox("Analytic reference", &debug_.reference);
    ImGui::EndDisabled();
    ImGui::End();
}

void SimUi::drawTelemetry(const SimUiFrame& frame)
{
    placeWindow(280.0f, -10.0f, 620.0f, 330.0f);
    if (!ImGui::Begin(kTelemetryTitle, &panels_.telemetry))
    {
        ImGui::End();
        return;
    }

    ImGui::Text("CPU %.2f ms  GPU %.2f ms  %.0f FPS", frame.cpuFrameMs, frame.gpuFrameMs, frame.fps);
    if (frame.hasEnergy && energyBaselineValid_)
    {
        ImGui::SameLine();
        ImGui::Text("  |  E = %.4f J  drift %+.3e", frame.totalEnergy,
                    (frame.totalEnergy - energyBaseline_) / std::max(std::abs(energyBaseline_), 1e-9));
    }

    const int count = static_cast<int>(plotTime_.size());
    const double xMax = frame.nowSeconds;
    const double xMin = xMax - kHistorySeconds;
    const float plotHeight = std::max(80.0f, (ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y * 2.0f) / 3.0f);
    const ImPlotAxisFlags xFlags = ImPlotAxisFlags_NoTickLabels;
    const ImPlotAxisFlags yFlags = ImPlotAxisFlags_AutoFit;

    if (ImPlot::BeginPlot("Total energy", ImVec2(-1.0f, plotHeight)))
    {
        ImPlot::SetupAxes(nullptr, "J", xFlags, yFlags);
        ImPlot::SetupAxisLimits(ImAxis_X1, xMin, xMax, ImPlotCond_Always);
        if (count > 0)
        {
            ImPlot::PlotLine("E", plotTime_.data(), plotEnergy_.data(), count);
        }
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Relative energy drift", ImVec2(-1.0f, plotHeight)))
    {
        ImPlot::SetupAxes(nullptr, "(E-E0)/|E0|", xFlags, yFlags);
        ImPlot::SetupAxisLimits(ImAxis_X1, xMin, xMax, ImPlotCond_Always);
        if (count > 0)
        {
            ImPlot::PlotLine("drift", plotTime_.data(), plotDrift_.data(), count);
        }
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Frame time", ImVec2(-1.0f, plotHeight)))
    {
        ImPlot::SetupAxes("last 10 s", "ms", xFlags, yFlags);
        ImPlot::SetupAxisLimits(ImAxis_X1, xMin, xMax, ImPlotCond_Always);
        if (count > 0)
        {
            ImPlot::PlotLine("CPU", plotTime_.data(), plotCpu_.data(), count);
            ImPlot::PlotLine("GPU", plotTime_.data(), plotGpu_.data(), count);
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}

void SimUi::drawCamera(const SimUiFrame& frame, OrbitCamera& camera, SimUiActions& actions)
{
    placeWindow(-10.0f, 460.0f, 320.0f, 190.0f);
    if (!ImGui::Begin(kCameraTitle, &panels_.camera))
    {
        ImGui::End();
        return;
    }
    ImGui::SliderFloat("FOV", &camera.fovDegrees, 20.0f, 110.0f, "%.0f deg");
    if (frame.activeScene.empty())
    {
        ImGui::TextDisabled("Driving: C cycles chase/orbit/free,");
        ImGui::TextDisabled("right-drag looks, wheel zooms.");
        ImGui::End();
        return;
    }
    if (ImGui::Button("Frame scene"))
    {
        actions.frameScene = true;
    }
    ImGui::SetItemTooltip("F");
    ImGui::DragFloat3("Target", &camera.target.x, 0.05f, 0.0f, 0.0f, "%.2f");
    float yawDegrees = glm::degrees(camera.yaw);
    float pitchDegrees = glm::degrees(camera.pitch);
    if (ImGui::DragFloat("Yaw", &yawDegrees, 0.5f, 0.0f, 0.0f, "%.1f deg"))
    {
        camera.yaw = glm::radians(yawDegrees);
    }
    if (ImGui::DragFloat("Pitch", &pitchDegrees, 0.5f, -88.0f, 88.0f, "%.1f deg"))
    {
        camera.pitch = glm::radians(pitchDegrees);
    }
    ImGui::DragFloat("Distance", &camera.distance, 0.05f, 0.5f, 500.0f, "%.2f m");
    ImGui::TextDisabled("Left-drag orbit, right-drag pan, wheel zoom.");
    ImGui::End();
}

void SimUi::drawAbout()
{
    if (!showAbout_)
    {
        return;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin("About", &showAbout_, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse))
    {
        ImGui::Text("Car Simulator %s", core::kVersionString);
        ImGui::Separator();
        ImGui::TextUnformatted("Driving simulator with its own deterministic rigid-body physics");
        ImGui::TextUnformatted("and a Vulkan renderer.");
        ImGui::TextUnformatted("Licensed under the GNU GPL v3.0 or later.");
        ImGui::Text("Dear ImGui %s, ImPlot", IMGUI_VERSION);
    }
    ImGui::End();
}

void SimUi::drawShortcuts()
{
    if (!showShortcuts_)
    {
        return;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::Begin("Keyboard shortcuts", &showShortcuts_, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse))
    {
        if (ImGui::BeginTable("shortcuts", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV))
        {
            for (const ShortcutEntry& entry : kShortcuts)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(entry.keys);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(entry.action);
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

} // namespace sim::app
