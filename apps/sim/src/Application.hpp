// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <glm/glm.hpp>

#include "audio/AudioEngine.hpp"
#include "core/Clock.hpp"
#include "core/Config.hpp"
#include "modding/ModManager.hpp"
#include "net/NetworkSystem.hpp"
#include "physics/PhysicsWorld.hpp"
#include "platform/Input.hpp"
#include "platform/Sdl.hpp"
#include "platform/Window.hpp"
#include "plugin_api/PluginLoader.hpp"
#include "render/DebugDraw.hpp"
#include "render/Renderer.hpp"
#include "render/Scene.hpp"
#include "rhi/Device.hpp"
#include "rhi/Frame.hpp"
#include "rhi/GpuTimer.hpp"
#include "rhi/Swapchain.hpp"
#include "scripting/LuaSandbox.hpp"
#include "scripting/ScriptHost.hpp"
#include "telemetry/Telemetry.hpp"
#include "vehicle/GroundQuery.hpp"
#include "vehicle/VehicleModel.hpp"
#include "vfs/VirtualFileSystem.hpp"

#include "OrbitCamera.hpp"

#include <volk.h>

namespace sim::app {

class ImGuiLayer;
class SimUi;

namespace drive {
class DriveScene;
} // namespace drive

// One physics state, produced by the physics thread and consumed by the render
// thread. Copied through a lock-free single-producer/single-consumer ring.
struct SimSnapshot {
    double simTime = 0.0;
    double wallTime = 0.0;
    int steps = 0;
    glm::dvec3 vehiclePosition{0.0};
    double heading = 0.0;
    double speed = 0.0;
    double rpm = 0.0;
    double steer = 0.0;
    std::array<double, 4> wheelSpin{0.0, 0.0, 0.0, 0.0};
    // Totals over every dynamic body, filled on the physics thread so the energy
    // panel never touches live simulation state.
    double kineticEnergy = 0.0;
    double potentialEnergy = 0.0;
    int bodyCount = 0;
};

class SnapshotRing {
public:
    static constexpr std::size_t kCapacity = 128;

    void push(const SimSnapshot& snapshot) {
        const std::uint64_t write = write_.load(std::memory_order_relaxed);
        buffer_[write % kCapacity] = snapshot;
        write_.store(write + 1, std::memory_order_release);
    }

    bool pop(SimSnapshot& out) {
        const std::uint64_t read = read_.load(std::memory_order_relaxed);
        if (read >= write_.load(std::memory_order_acquire)) {
            return false;
        }
        out = buffer_[read % kCapacity];
        read_.store(read + 1, std::memory_order_release);
        return true;
    }

private:
    std::array<SimSnapshot, kCapacity> buffer_{};
    std::atomic<std::uint64_t> write_{0};
    std::atomic<std::uint64_t> read_{0};
};

class Application {
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    int run();

    // Scene loaded right after start-up (`sim --scene <name>`): "drive", any
    // entry of sceneCatalog(), or "driving" for the interactive vehicle.
    void setStartupScene(const std::string& name) { startupScene_ = name; }

private:
    bool initialise();
    void shutdown();

    void startPhysicsThread();
    void stopPhysicsThread();
    void physicsLoop();

    void pumpInput();
    void gatherSnapshots(SimSnapshot& interpolated);
    void stepScenePhysics(double delta);
    void updateScene(const SimSnapshot& interpolated);
    void updateDemoCamera(const glm::vec3& target);
    void handleCameraKeys();
    void drawUi();
    void loadScene(const std::string& name);
    void frameScene();
    void drawDebugOverlay(rhi::CommandList& commandList);
    void stepDriveScene(double delta);
    bool driveActive() const { return drive_ != nullptr; }

    enum class CameraMode {
        Chase,
        Orbit,
        Free,
    };

    std::filesystem::path locateRoot() const;
    static double wallNow();

    // Platform / infrastructure (declared in construction order).
    std::unique_ptr<platform::SdlContext> sdl_;
    std::unique_ptr<platform::Window> window_;
    std::unique_ptr<platform::InputManager> input_;
    std::filesystem::path rootDirectory_;

    std::unique_ptr<vfs::VirtualFileSystem> vfs_;
    std::unique_ptr<core::Config> config_;
    std::unique_ptr<modding::ModManager> mods_;
    std::unique_ptr<scripting::LuaSandbox> lua_;
    std::unique_ptr<scripting::ScriptHost> scripts_;
    std::unique_ptr<plugin::PluginLoader> plugins_;
    std::unique_ptr<audio::AudioEngine> audio_;
    std::unique_ptr<net::NetworkSystem> net_;

    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    std::unique_ptr<rhi::Device> device_;
    std::unique_ptr<rhi::Swapchain> swapchain_;
    std::unique_ptr<rhi::Frame> frame_;
    std::unique_ptr<render::Renderer> renderer_;
    std::unique_ptr<render::DebugDraw> debugDraw_;
    std::unique_ptr<ImGuiLayer> imgui_;
    std::unique_ptr<telemetry::Telemetry> telemetry_;
    std::unique_ptr<rhi::GpuTimer> gpuTimer_;

    std::shared_ptr<render::Mesh> cubeMesh_;
    std::shared_ptr<render::Mesh> sphereMesh_;
    std::shared_ptr<render::Mesh> cylinderMesh_;
    std::shared_ptr<render::Mesh> groundMesh_;
    std::array<glm::vec3, 4> wheelOffsets_{};
    render::Scene scene_;

    // Camera modes (phase C). Chase follows the car; Orbit and Free are debug
    // views driven from the ImGui IO so the panels stay interactive.
    CameraMode cameraMode_ = CameraMode::Chase;
    float orbitYaw_ = 0.6f;
    float orbitPitch_ = 0.30f;
    float orbitDistance_ = 14.0f;
    float freeYaw_ = 0.0f;
    float freePitch_ = -0.15f;
    glm::vec3 freePosition_{0.0f, 3.0f, 8.0f};

    // Active scene: empty is the interactive driving scene, otherwise an entry
    // of sceneCatalog() rendered for inspection (static render-test scenes, or
    // live physics scenes stepped on the main thread).
    std::string sceneName_;

    // Physics demo scenes run in their own world on the main thread, separate
    // from the vehicle world on the physics thread. Wall time (scaled by the
    // UI time scale) is fed to PhysicsWorld::advance(), which keeps the step
    // fixed.
    std::unique_ptr<physics::PhysicsWorld> sceneWorld_;

    // Mouse-driven camera of the inspection scenes (also owns the FOV used by
    // the driving cameras).
    OrbitCamera sceneCamera_;

    // Keyboard-drivable circuit (apps/sim/src/drive), simulated on the main
    // thread while the scene named "drive" is active. Its own chase camera
    // replaces sceneCamera_ for as long as it exists.
    std::unique_ptr<drive::DriveScene> drive_;
    double frameDelta_ = 0.0;
    std::string startupScene_;

    // Windowed-mode ImGui interface (menus, panels, shortcuts).
    std::unique_ptr<SimUi> ui_;
    double lastGpuMs_ = 0.0;

    // Snapshot save/restore is requested by the UI thread and executed on the
    // physics thread so it never races the simulation.
    std::atomic<bool> snapshotSaveRequest_{false};
    std::atomic<bool> snapshotRestoreRequest_{false};
    std::atomic<bool> snapshotValid_{false};
    std::mutex snapshotMutex_;
    physics::WorldSnapshot savedWorld_;

    // Simulation.
    physics::PhysicsWorld world_;
    vehicle::FlatGroundQuery vehicleGround_;   // the flat demo ground
    vehicle::VehicleModel* vehicle_ = nullptr; // the assembled car, owned by world_
    std::thread physicsThread_;
    std::atomic<bool> physicsRunning_{false};
    core::FixedStepClock physicsClock_{1.0 / 1000.0, 16};
    SnapshotRing snapshots_;
    std::atomic<double> inputThrottle_{0.0};
    std::atomic<double> inputBrake_{0.0};
    std::atomic<double> inputSteer_{0.0};
    std::atomic<bool> inputHandbrake_{false};
    std::atomic<double> physicsTimeMs_{0.0};
    std::atomic<int> physicsSteps_{0};

    // Render-side interpolation state.
    SimSnapshot previous_{};
    SimSnapshot current_{};
    bool hasSnapshot_ = false;

    core::FrameClock frameClock_;
    bool vsync_ = true;
    bool running_ = true;
};

} // namespace sim::app
