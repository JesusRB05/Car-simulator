// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

namespace sim::physics {
class PhysicsWorld;
} // namespace sim::physics

namespace sim::input {
class WheelManager;
} // namespace sim::input

namespace sim::app {

struct OrbitCamera;

// Debug overlay layers. Everything defaults to on; the physics layers only
// apply to physics scenes.
struct DebugDrawFlags {
    bool grid = true;
    bool worldAxes = true;
    bool bodyAxes = true;
    bool joints = true;
    bool springs = true;
    bool velocities = true;
    bool reference = true;
    bool driveForces = true; // drive scene: suspension rays, contacts, tyre forces
};

// Which panels are open. Persisted in the ImGui .ini file under [SimUi][Panels].
struct PanelVisibility {
    bool sceneBrowser = true;
    bool simulation = true;
    bool debugDraw = true;
    bool telemetry = true;
    bool camera = true;
    bool vehicle = false;
    bool mods = false;
    bool input = false;
};

// Per-frame read-only state shown by the panels.
struct SimUiFrame {
    std::string activeScene;                      // empty = interactive driving scene
    const physics::PhysicsWorld* world = nullptr; // the live physics scene, if any
    bool driveActive = false;                     // the keyboard-drivable scene owns keys and camera
    bool hasEnergy = false;
    double totalEnergy = 0.0; // J
    double cpuFrameMs = 0.0;
    double gpuFrameMs = 0.0;
    double fps = 0.0;
    double nowSeconds = 0.0;              // wall clock, for the plots
    float viewportHeight = 1.0f;          // pixels, for panning
    input::WheelManager* wheel = nullptr; // live wheel/pedal stack, if any
};

// What the user asked for this frame. Applied by the Application.
struct SimUiActions {
    bool quit = false;
    bool loadScene = false;
    std::string scene;  // with loadScene; empty = driving scene
    bool step = false;  // advance exactly one fixed step (while paused)
    bool reset = false; // rebuild the physics scene from its initial state
    bool frameScene = false;
};

// The windowed app's ImGui interface: main menu bar, scene browser,
// simulation controls, debug-draw toggles, telemetry plots and camera panel,
// plus the keyboard shortcuts. Only ever constructed in windowed mode; must be
// created after the ImGui context and before the first ImGui::NewFrame() so
// its .ini settings are read.
class SimUi {
public:
    SimUi();
    ~SimUi();

    SimUi(const SimUi&) = delete;
    SimUi& operator=(const SimUi&) = delete;

    // Draws every visible panel and handles shortcuts and scene-camera mouse
    // input. Call between ImGui::NewFrame() and ImGui::Render().
    SimUiActions draw(const SimUiFrame& frame, OrbitCamera& camera);

    PanelVisibility& panels() { return panels_; }
    DebugDrawFlags& debugFlags() { return debug_; }
    const DebugDrawFlags& debugFlags() const { return debug_; }

    bool paused() const { return paused_; }
    // Multiplier on the wall time fed to PhysicsWorld::advance(); never
    // changes the fixed step itself.
    float timeScale() const { return timeScale_; }

    // Restarts the energy drift reference and the plot history (scene loads
    // and resets).
    void resetHistory();

private:
    void handleShortcuts(const SimUiFrame& frame, SimUiActions& actions);
    void handleCameraMouse(const SimUiFrame& frame, OrbitCamera& camera);
    void updateStatistics(const SimUiFrame& frame);

    void drawMenuBar(const SimUiFrame& frame, SimUiActions& actions);
    void drawSceneMenuItems(const SimUiFrame& frame, SimUiActions& actions);
    void drawSceneBrowser(const SimUiFrame& frame, SimUiActions& actions);
    void drawSimulation(const SimUiFrame& frame, SimUiActions& actions);
    void drawDebugDraw(const SimUiFrame& frame);
    void drawTelemetry(const SimUiFrame& frame);
    void drawCamera(const SimUiFrame& frame, OrbitCamera& camera, SimUiActions& actions);
    void drawInput(const SimUiFrame& frame);
    void drawAbout();
    void drawShortcuts();

    static void registerSettingsHandler(SimUi* self);

    PanelVisibility panels_;
    DebugDrawFlags debug_;
    bool showAbout_ = false;
    bool showShortcuts_ = false;

    bool paused_ = false;
    float timeScale_ = 1.0f;

    // Steps per second, measured from the world's simulated time.
    double lastSimTime_ = -1.0;
    double rateWindowStart_ = 0.0;
    double rateWindowSim_ = 0.0;
    double stepsPerSecond_ = 0.0;

    // Rolling ~10 s histories for the telemetry plots (x = wall seconds).
    bool energyBaselineValid_ = false;
    double energyBaseline_ = 0.0;
    std::vector<double> plotTime_;
    std::vector<double> plotEnergy_;
    std::vector<double> plotDrift_;
    std::vector<double> plotCpu_;
    std::vector<double> plotGpu_;
};

} // namespace sim::app
