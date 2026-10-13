// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "Application.hpp"

#include <imgui.h>
#include <implot.h>

#include <glm/gtc/matrix_transform.hpp>

#include <SDL3/SDL.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "ImGuiLayer.hpp"
#include "PhysicsScenes.hpp"
#include "Scenes.hpp"
#include "core/Log.hpp"
#include "core/Version.hpp"

namespace sim::app {
namespace {

constexpr std::uint32_t kFramesInFlight = 2;

// Validation layers default to on in Debug builds (they are the whole point of
// a debug build) and off in Release; both can be overridden with the
// render.validation config key.
#ifdef NDEBUG
constexpr bool kDefaultValidation = false;
#else
constexpr bool kDefaultValidation = true;
#endif

glm::dvec3 toDvec(const physics::Vec3& value)
{
    return {value.x, value.y, value.z};
}

std::filesystem::path executableDirectory()
{
    const char* base = SDL_GetBasePath();
    if (base == nullptr)
    {
        return std::filesystem::current_path();
    }
    std::filesystem::path path(base);
    SDL_free(const_cast<char*>(base));
    return path;
}

std::filesystem::path searchForRoot()
{
    std::vector<std::filesystem::path> starts{std::filesystem::current_path(), executableDirectory()};
    for (const std::filesystem::path& start : starts)
    {
        std::filesystem::path dir = start;
        for (int depth = 0; depth < 8 && !dir.empty(); ++depth)
        {
            if (std::filesystem::exists(dir / "content" / "base"))
            {
                return dir;
            }
            const std::filesystem::path parent = dir.parent_path();
            if (parent == dir)
            {
                break;
            }
            dir = parent;
        }
    }
    return executableDirectory();
}

bool keyDown(const platform::InputManager* input, SDL_Scancode scancode)
{
    return input != nullptr && input->keyDown(scancode);
}

} // namespace

double Application::wallNow()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

Application::Application() = default;
Application::~Application() = default;

std::filesystem::path Application::locateRoot() const
{
    return searchForRoot();
}

bool Application::initialise()
{
    sdl_ = std::make_unique<platform::SdlContext>();
    if (!sdl_->valid())
    {
        return false;
    }

    rootDirectory_ = locateRoot();

    core::Log::Config logConfig;
    logConfig.appName = "sim";
    logConfig.logFile = rootDirectory_ / "sim.log";
    core::Log::init(logConfig);
    auto log = core::Log::get();

    log->info("Car Simulator {} starting", core::kVersionString);
    log->info("root directory: {}", rootDirectory_.string());

    platform::WindowDesc windowDesc;
    windowDesc.title = std::string("Car Simulator ") + core::kVersionString;
    windowDesc.width = 1600;
    windowDesc.height = 900;
    window_ = std::make_unique<platform::Window>(windowDesc);

    input_ = std::make_unique<platform::InputManager>();
    window_->setInput(input_.get());

    vfs_ = std::make_unique<vfs::VirtualFileSystem>();
    vfs_->setWriteDirectory(rootDirectory_ / "userdata");
    const std::filesystem::path exeDir = executableDirectory();

    config_ = std::make_unique<core::Config>(rootDirectory_ / "config.toml");
    config_->load();

    modding::ModManager::Paths modPaths;
    modPaths.baseContent = rootDirectory_ / "content" / "base";
    modPaths.modsDirectory = rootDirectory_ / "mods";
    mods_ = std::make_unique<modding::ModManager>(*vfs_, *config_, modPaths);
    mods_->reload();
    log->info("mods: {}", mods_->report().toString());

    // ModManager rebuilds the mount table from scratch (base + mods), so the
    // engine's own runtime directory is mounted afterwards, at the lowest
    // priority, to make shaders/*.spv resolvable without shadowing content.
    vfs_->mount(exeDir, "/", true);

    net_ = std::make_unique<net::NetworkSystem>();
    net_->start(0);

    const double fixedDt = config_->getDouble("physics.fixed_dt", 1.0 / 1000.0);
    physicsClock_.setFixedDt(fixedDt);
    vehicle_.reset(physics::Vec3(0.0, 0.0, 0.0), 0.0);
    for (std::size_t i = 0; i < wheelOffsets_.size() && i < vehicle_.wheels().size(); ++i)
    {
        const physics::Vec3 local = vehicle_.wheels()[i].localPosition;
        wheelOffsets_[i] = glm::vec3(static_cast<float>(local.x), static_cast<float>(local.y), static_cast<float>(local.z));
    }

    vsync_ = config_->getBool("render.vsync", true);

    rhi::DeviceConfig deviceConfig;
    deviceConfig.enableValidation = config_->getBool("render.validation", kDefaultValidation);
    deviceConfig.extraInstanceExtensions = platform::Window::requiredVulkanInstanceExtensions();
    deviceConfig.createSurface = [this](VkInstance instance) { return window_->createVulkanSurface(instance); };
    device_ = std::make_unique<rhi::Device>(deviceConfig);
    log->info("device: {}", device_->deviceName());

    int width = windowDesc.width;
    int height = windowDesc.height;
    window_->getDrawableSize(width, height);

    rhi::SwapchainDesc swapDesc;
    swapDesc.extent = {static_cast<std::uint32_t>(std::max(width, 1)), static_cast<std::uint32_t>(std::max(height, 1))};
    swapDesc.vsync = vsync_;
    swapchain_ = std::make_unique<rhi::Swapchain>(*device_, swapDesc);

    frame_ = std::make_unique<rhi::Frame>(*device_, *swapchain_, kFramesInFlight);
    gpuTimer_ = std::make_unique<rhi::GpuTimer>(*device_, kFramesInFlight);
    renderer_ = std::make_unique<render::Renderer>(*device_, *vfs_, kFramesInFlight);
    renderer_->ensureTargets(swapchain_->extent(), frame_->colorFormat());
    debugDraw_ = std::make_unique<render::DebugDraw>(*device_, *vfs_);
    debugDraw_->ensureTargets(frame_->colorFormat(), swapchain_->extent());

    imgui_ = std::make_unique<ImGuiLayer>(*device_, *swapchain_, window_->handle());
    window_->setEventHook([this](const SDL_Event& event) { imgui_->processEvent(event); });

    telemetry_ = std::make_unique<telemetry::Telemetry>(600);

    audio_ = std::make_unique<audio::AudioEngine>();
    plugins_ = std::make_unique<plugin::PluginLoader>(*vfs_);
    plugins_->loadDirectory(exeDir / "plugins");

    lua_ = std::make_unique<scripting::LuaSandbox>();
    scripts_ = std::make_unique<scripting::ScriptHost>(*lua_, *vfs_);
    if (vfs_->exists("scripts/main.lua"))
    {
        scripts_->loadScript("base", "scripts/main.lua");
        scripts_->callOnLoad();
    }

    cubeMesh_ = render::Mesh::makeCube(*device_);
    sphereMesh_ = render::Mesh::makeSphere(*device_);
    cylinderMesh_ = render::Mesh::makeCylinder(*device_);
    groundMesh_ = render::Mesh::makeGroundPlane(*device_, 200.0f);
    scene_.setDirectionalLight({-0.4f, -1.0f, -0.35f}, {1.0f, 0.98f, 0.95f});

    startPhysicsThread();
    return true;
}

void Application::shutdown()
{
    stopPhysicsThread();

    if (scripts_ != nullptr)
    {
        scripts_->clear();
    }
    scripts_.reset();
    lua_.reset();
    plugins_.reset();
    audio_.reset();
    telemetry_.reset();

    if (device_ != nullptr)
    {
        device_->waitIdle();
    }

    imgui_.reset();
    debugDraw_.reset();
    renderer_.reset();
    frame_.reset();
    gpuTimer_.reset();
    groundMesh_.reset();
    cubeMesh_.reset();
    cylinderMesh_.reset();
    sphereMesh_.reset();
    swapchain_.reset();
    device_.reset();

    net_.reset();
    mods_.reset();
    config_.reset();
    vfs_.reset();
    input_.reset();
    window_.reset();
    sdl_.reset();

    core::Log::shutdown();
}

void Application::startPhysicsThread()
{
    physicsRunning_.store(true, std::memory_order_release);
    physicsThread_ = std::thread([this] { physicsLoop(); });
}

void Application::stopPhysicsThread()
{
    if (!physicsThread_.joinable())
    {
        return;
    }
    physicsRunning_.store(false, std::memory_order_release);
    physicsThread_.join();
}

void Application::physicsLoop()
{
    const double dt = physicsClock_.fixedDt();
    double last = wallNow();

    while (physicsRunning_.load(std::memory_order_acquire))
    {
        if (snapshotSaveRequest_.exchange(false))
        {
            std::lock_guard<std::mutex> lock(snapshotMutex_);
            savedWorld_ = world_.saveSnapshot();
            snapshotValid_.store(true, std::memory_order_release);
        }
        if (snapshotRestoreRequest_.exchange(false))
        {
            std::lock_guard<std::mutex> lock(snapshotMutex_);
            if (snapshotValid_.load(std::memory_order_acquire))
            {
                world_.restoreSnapshot(savedWorld_);
            }
        }

        const double now = wallNow();
        double frameDelta = now - last;
        last = now;
        if (frameDelta < 0.0)
        {
            frameDelta = 0.0;
        }
        if (frameDelta > 0.25)
        {
            frameDelta = 0.25;
        }

        const core::StepResult result = physicsClock_.advance(frameDelta);

        if (result.steps > 0)
        {
            vehicle::VehicleInput input;
            input.throttle = inputThrottle_.load(std::memory_order_relaxed);
            input.brake = inputBrake_.load(std::memory_order_relaxed);
            input.steer = inputSteer_.load(std::memory_order_relaxed);
            input.handbrake = inputHandbrake_.load(std::memory_order_relaxed);
            vehicle_.setInput(input);

            const double startMs = wallNow();
            for (std::uint32_t i = 0; i < result.steps; ++i)
            {
                vehicle_.step(dt);
                world_.step(dt);
            }
            physicsTimeMs_.store((wallNow() - startMs) * 1000.0, std::memory_order_relaxed);
            physicsSteps_.store(static_cast<int>(result.steps), std::memory_order_relaxed);

            SimSnapshot snapshot;
            snapshot.simTime = physicsClock_.simTime();
            snapshot.wallTime = wallNow();
            snapshot.steps = static_cast<int>(result.steps);
            snapshot.vehiclePosition = toDvec(vehicle_.position());
            snapshot.heading = vehicle_.heading();
            snapshot.speed = vehicle_.speed();
            snapshot.rpm = vehicle_.engineRpm();
            snapshot.steer = vehicle_.steerAngle();
            for (std::size_t i = 0; i < snapshot.wheelSpin.size(); ++i)
            {
                snapshot.wheelSpin[i] = vehicle_.wheels()[i].spinAngle;
            }
            for (std::size_t i = 0; i < world_.bodyCount(); ++i)
            {
                const physics::RigidBody& body = world_.body(i);
                if (!body.isDynamic())
                {
                    continue;
                }
                snapshot.kineticEnergy += body.kineticEnergy();
                snapshot.potentialEnergy += body.mass * (-world_.gravity.y) * body.position.y;
                ++snapshot.bodyCount;
            }
            snapshots_.push(snapshot);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void Application::pumpInput()
{
    input_->refresh();

    double throttle = 0.0;
    double brake = 0.0;
    double steer = 0.0;
    bool handbrake = false;

    if (keyDown(input_.get(), SDL_SCANCODE_W) || keyDown(input_.get(), SDL_SCANCODE_UP))
    {
        throttle = 1.0;
    }
    if (keyDown(input_.get(), SDL_SCANCODE_S) || keyDown(input_.get(), SDL_SCANCODE_DOWN))
    {
        brake = 1.0;
    }
    if (keyDown(input_.get(), SDL_SCANCODE_A) || keyDown(input_.get(), SDL_SCANCODE_LEFT))
    {
        steer += 1.0;
    }
    if (keyDown(input_.get(), SDL_SCANCODE_D) || keyDown(input_.get(), SDL_SCANCODE_RIGHT))
    {
        steer -= 1.0;
    }
    handbrake = keyDown(input_.get(), SDL_SCANCODE_SPACE);

    for (const platform::InputDevice& device : input_->devices())
    {
        if (!device.isGamepad || device.axes.size() < 2)
        {
            continue;
        }
        const double axisSteer = static_cast<double>(device.axes[0]);
        const double axisThrottle = static_cast<double>(-device.axes[1]);
        if (std::abs(axisSteer) > 0.08)
        {
            steer -= axisSteer;
        }
        if (axisThrottle > 0.05)
        {
            throttle = std::max(throttle, axisThrottle);
        }
        else if (axisThrottle < -0.05)
        {
            brake = std::max(brake, -axisThrottle);
        }
        break;
    }

    inputThrottle_.store(std::clamp(throttle, 0.0, 1.0), std::memory_order_relaxed);
    inputBrake_.store(std::clamp(brake, 0.0, 1.0), std::memory_order_relaxed);
    inputSteer_.store(std::clamp(steer, -1.0, 1.0), std::memory_order_relaxed);
    inputHandbrake_.store(handbrake, std::memory_order_relaxed);
}

void Application::gatherSnapshots(SimSnapshot& interpolated)
{
    SimSnapshot snapshot;
    while (snapshots_.pop(snapshot))
    {
        previous_ = current_;
        current_ = snapshot;
        hasSnapshot_ = true;
    }

    if (!hasSnapshot_)
    {
        interpolated = SimSnapshot{};
        return;
    }

    const double dt = physicsClock_.fixedDt();
    double alpha = 0.0;
    if (dt > 0.0)
    {
        alpha = std::clamp((wallNow() - current_.wallTime) / dt, 0.0, 1.0);
    }

    interpolated = current_;
    interpolated.simTime = previous_.simTime + (current_.simTime - previous_.simTime) * alpha;
    interpolated.vehiclePosition = previous_.vehiclePosition + (current_.vehiclePosition - previous_.vehiclePosition) * alpha;
    interpolated.heading = previous_.heading + (current_.heading - previous_.heading) * alpha;
    interpolated.speed = previous_.speed + (current_.speed - previous_.speed) * alpha;
    interpolated.rpm = previous_.rpm + (current_.rpm - previous_.rpm) * alpha;
    interpolated.steer = previous_.steer + (current_.steer - previous_.steer) * alpha;
    for (std::size_t i = 0; i < interpolated.wheelSpin.size(); ++i)
    {
        interpolated.wheelSpin[i] = previous_.wheelSpin[i] + (current_.wheelSpin[i] - previous_.wheelSpin[i]) * alpha;
    }
}

void Application::stepScenePhysics(double delta)
{
    if (sceneSelection_ <= 0 || sceneWorld_ == nullptr)
    {
        return;
    }
    const double dt = sceneWorld_->fixedStep;
    sceneWorldRemainder_ += delta;
    int steps = 0;
    while (sceneWorldRemainder_ >= dt && steps < 64)
    {
        sceneWorld_->step(dt);
        sceneWorldRemainder_ -= dt;
        ++steps;
    }
    if (steps >= 64)
    {
        sceneWorldRemainder_ = 0.0;
    }
}

void Application::updateScene(const SimSnapshot& interpolated)
{
    const float aspect = swapchain_->extent().height > 0
                             ? static_cast<float>(swapchain_->extent().width) / static_cast<float>(swapchain_->extent().height)
                             : 16.0f / 9.0f;

    if (sceneSelection_ > 0)
    {
        const std::vector<std::string> names = sceneNames();
        const int index = std::clamp(sceneSelection_ - 1, 0, static_cast<int>(names.size()) - 1);
        const std::string name = names[static_cast<std::size_t>(index)];

        if (isPhysicsScene(name))
        {
            if (sceneWorld_ == nullptr || sceneWorldName_ != name)
            {
                sceneWorld_ = std::make_unique<physics::PhysicsWorld>();
                buildPhysicsWorld(name, *sceneWorld_);
                sceneWorldRemainder_ = 0.0;
                sceneWorldName_ = name;
            }
            buildSceneFromWorld(*sceneWorld_, scene_, cubeMesh_, sphereMesh_, cylinderMesh_, groundMesh_);
            physicsSceneCamera(name, scene_, aspect);
            return;
        }

        sceneWorld_.reset();
        sceneWorldName_.clear();
        buildScene(name, scene_, cubeMesh_, groundMesh_);
        return;
    }

    scene_.clear();

    const glm::vec3 position(static_cast<float>(interpolated.vehiclePosition.x),
                             static_cast<float>(interpolated.vehiclePosition.y),
                             static_cast<float>(interpolated.vehiclePosition.z));
    const float heading = static_cast<float>(interpolated.heading);
    const float steer = static_cast<float>(interpolated.steer);

    const glm::mat4 headingRotation = glm::rotate(glm::mat4(1.0f), heading, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::mat4 base = glm::translate(glm::mat4(1.0f), position) * headingRotation;

    scene_.add(groundMesh_, glm::mat4(1.0f), glm::vec4(0.32f, 0.34f, 0.30f, 1.0f));
    scene_.add(cubeMesh_, base * glm::scale(glm::mat4(1.0f), glm::vec3(1.9f, 1.1f, 4.2f)),
               glm::vec4(0.80f, 0.15f, 0.12f, 1.0f));

    for (std::size_t i = 0; i < wheelOffsets_.size(); ++i)
    {
        glm::mat4 wheel = base * glm::translate(glm::mat4(1.0f), wheelOffsets_[i]);
        if (i < 2)
        {
            wheel = wheel * glm::rotate(glm::mat4(1.0f), steer, glm::vec3(0.0f, 1.0f, 0.0f));
        }
        wheel = wheel * glm::rotate(glm::mat4(1.0f), static_cast<float>(interpolated.wheelSpin[i]), glm::vec3(1.0f, 0.0f, 0.0f));
        wheel = wheel * glm::scale(glm::mat4(1.0f), glm::vec3(0.68f));
        scene_.add(cubeMesh_, wheel, glm::vec4(0.08f, 0.08f, 0.09f, 1.0f));
    }

    scene_.camera().setPerspective(60.0f, aspect, 0.1f, 1000.0f);

    const glm::vec3 focus = position + glm::vec3(0.0f, 0.8f, 0.0f);
    if (cameraMode_ == CameraMode::Orbit)
    {
        const float cp = std::cos(orbitPitch_);
        const glm::vec3 offset(orbitDistance_ * cp * std::sin(orbitYaw_), orbitDistance_ * std::sin(orbitPitch_),
                               orbitDistance_ * cp * std::cos(orbitYaw_));
        scene_.camera().lookAt(focus + offset, focus, glm::vec3(0.0f, 1.0f, 0.0f));
    }
    else if (cameraMode_ == CameraMode::Free)
    {
        const glm::vec3 direction(std::cos(freePitch_) * std::sin(freeYaw_), std::sin(freePitch_),
                                  std::cos(freePitch_) * std::cos(freeYaw_));
        scene_.camera().lookAt(freePosition_, freePosition_ + direction, glm::vec3(0.0f, 1.0f, 0.0f));
    }
    else
    {
        const glm::vec3 forward(std::sin(heading), 0.0f, std::cos(heading));
        const glm::vec3 eye = position - forward * 8.0f + glm::vec3(0.0f, 3.4f, 0.0f);
        scene_.camera().lookAt(eye, focus, glm::vec3(0.0f, 1.0f, 0.0f));
    }
}

void Application::handleCameraKeys()
{
    ImGuiIO& io = ImGui::GetIO();

    if (ImGui::IsKeyPressed(ImGuiKey_C, false))
    {
        cameraMode_ = cameraMode_ == CameraMode::Chase    ? CameraMode::Orbit
                      : cameraMode_ == CameraMode::Orbit  ? CameraMode::Free
                                                          : CameraMode::Chase;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_G, false))
    {
        debugDrawEnabled_ = !debugDrawEnabled_;
    }

    const bool looking = ImGui::IsMouseDown(ImGuiMouseButton_Right) && !io.WantCaptureMouse;
    if (looking)
    {
        orbitYaw_ -= io.MouseDelta.x * 0.006f;
        orbitPitch_ = std::clamp(orbitPitch_ - io.MouseDelta.y * 0.006f, -1.45f, 1.45f);
        freeYaw_ -= io.MouseDelta.x * 0.006f;
        freePitch_ = std::clamp(freePitch_ - io.MouseDelta.y * 0.006f, -1.45f, 1.45f);
    }

    if (cameraMode_ == CameraMode::Orbit)
    {
        orbitDistance_ = std::clamp(orbitDistance_ - io.MouseWheel * 1.5f, 2.0f, 250.0f);
    }
    else if (cameraMode_ == CameraMode::Free && !io.WantCaptureKeyboard)
    {
        const glm::vec3 forward(std::cos(freePitch_) * std::sin(freeYaw_), std::sin(freePitch_),
                                std::cos(freePitch_) * std::cos(freeYaw_));
        const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
        const float speed = 14.0f * static_cast<float>(io.DeltaTime);
        if (ImGui::IsKeyDown(ImGuiKey_W))
        {
            freePosition_ += forward * speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_S))
        {
            freePosition_ -= forward * speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_A))
        {
            freePosition_ -= right * speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_D))
        {
            freePosition_ += right * speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_Q))
        {
            freePosition_.y -= speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_E))
        {
            freePosition_.y += speed;
        }
    }
}

void Application::drawUi()
{
    if (ImGui::Begin("Overview"))
    {
        ImGui::Text("Car Simulator %s", core::kVersionString);
        ImGui::Text("FPS: %.1f (%.2f ms)", telemetry_->fps(), telemetry_->frameTimeMs());
        ImGui::Text("Frame avg: %.2f ms", telemetry_->averageFrameTimeMs());
        ImGui::Separator();
        ImGui::Text("Physics: %.3f ms / %d steps", telemetry_->physicsTimeMs(), telemetry_->physicsSteps());
        ImGui::Text("Render: %.3f ms", telemetry_->renderTimeMs());
        ImGui::Text("GPU: %.3f ms", telemetry_->gpuTimeMs());
        ImGui::Text("UI: %.3f ms", telemetry_->uiTimeMs());
        ImGui::Separator();
        ImGui::Text("Speed: %.1f km/h", current_.speed * 3.6);
        ImGui::Text("RPM: %.0f", current_.rpm);
        ImGui::Text("Sim time: %.2f s", current_.simTime);
        ImGui::Text("Position: (%.1f, %.1f, %.1f)", current_.vehiclePosition.x, current_.vehiclePosition.y, current_.vehiclePosition.z);
    }
    ImGui::End();

    if (ImGui::Begin("Mods"))
    {
        const modding::ModReport& report = mods_->report();
        for (const modding::ModStatus& mod : report.mods)
        {
            const char* state = mod.state == modding::ModState::Active ? "active"
                                : mod.state == modding::ModState::Disabled ? "disabled"
                                                                            : "invalid";
            ImGui::BulletText("%s %s [%s]", mod.name.c_str(), mod.version.c_str(), state);
        }
        if (report.mods.empty())
        {
            ImGui::TextUnformatted("No mods found.");
        }
    }
    ImGui::End();

    if (ImGui::Begin("Input"))
    {
        ImGui::Text("Devices: %zu", input_->joystickCount());
        for (std::size_t index = 0; index < input_->devices().size(); ++index)
        {
            const platform::InputDevice& device = input_->devices()[index];
            ImGui::PushID(static_cast<int>(index));
            ImGui::BulletText("%s%s", device.name.c_str(),
                              device.isGamepad ? " (gamepad)"
                              : device.axes.size() >= 4 ? " (wheel/pedals)"
                                                        : " (joystick)");
            if (ImGui::TreeNode("Details"))
            {
                for (std::size_t axis = 0; axis < device.axes.size(); ++axis)
                {
                    ImGui::Text("Axis %zu", axis);
                    ImGui::SameLine();
                    ImGui::ProgressBar((device.axes[axis] + 1.0f) * 0.5f, ImVec2(-1.0f, 0.0f));
                }
                ImGui::Text("Buttons (%zu)", device.buttons.size());
                for (std::size_t button = 0; button < device.buttons.size(); ++button)
                {
                    ImGui::SameLine();
                    const bool pressed = device.buttons[button];
                    ImGui::TextColored(pressed ? ImVec4(0.3f, 1.0f, 0.3f, 1.0f) : ImVec4(0.4f, 0.4f, 0.4f, 1.0f), "%zu", button);
                }
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        if (input_->devices().empty())
        {
            ImGui::TextUnformatted("No wheels, pedals or gamepads connected.");
        }
        ImGui::Separator();
        ImGui::Text("Network: %.*s", static_cast<int>(net_->status().size()), net_->status().data());
        ImGui::Text("Plugins: %zu", plugins_->plugins().size());
    }
    ImGui::End();

    if (ImGui::Begin("View"))
    {
        const char* modeNames[] = {"Chase", "Orbit", "Free"};
        int mode = static_cast<int>(cameraMode_);
        if (ImGui::Combo("Camera (C)", &mode, modeNames, 3))
        {
            cameraMode_ = static_cast<CameraMode>(mode);
        }
        ImGui::Text("Right-drag to look, wheel to zoom (Orbit/Free).");

        ImGui::Separator();
        const std::vector<std::string> names = sceneNames();
        std::vector<const char*> labels;
        labels.reserve(names.size() + 1);
        labels.push_back("driving");
        for (const std::string& name : names)
        {
            labels.push_back(name.c_str());
        }
        ImGui::Combo("Scene", &sceneSelection_, labels.data(), static_cast<int>(labels.size()));

        ImGui::Separator();
        ImGui::Checkbox("Debug grid (G)", &debugDrawEnabled_);

        ImGui::Separator();
        if (ImGui::Button("Save snapshot"))
        {
            snapshotSaveRequest_.store(true, std::memory_order_relaxed);
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!snapshotValid_.load(std::memory_order_relaxed));
        if (ImGui::Button("Restore snapshot"))
        {
            snapshotRestoreRequest_.store(true, std::memory_order_relaxed);
        }
        ImGui::EndDisabled();
        ImGui::Text("Snapshot: %s", snapshotValid_.load() ? "available" : "none");
    }
    ImGui::End();

    if (ImGui::Begin("Energy"))
    {
        ImGui::Text("Bodies: %d  Kinetic: %.2f J  Potential: %.2f J", current_.bodyCount, current_.kineticEnergy,
                    current_.potentialEnergy);
        ImGui::Text("Total: %.2f J", current_.kineticEnergy + current_.potentialEnergy);
        if (!energyKinetic_.empty() && ImPlot::BeginPlot("##energy", ImVec2(-1.0f, 220.0f)))
        {
            ImPlot::SetupAxes("frames", "joules");
            const int count = static_cast<int>(energyKinetic_.size());
            ImPlot::PlotLine("kinetic", energyKinetic_.data(), count);
            ImPlot::PlotLine("potential", energyPotential_.data(), count);
            ImPlot::EndPlot();
        }
    }
    ImGui::End();
}

int Application::run()
{
    if (!initialise())
    {
        core::Log::get()->critical("initialisation failed");
        return 1;
    }

    auto log = core::Log::get();

    while (running_)
    {
        const double frameStart = wallNow();
        telemetry_->beginFrame(frameStart);

        frameClock_.tick();
        double delta = frameClock_.lastDelta();
        if (delta <= 0.0)
        {
            delta = 1.0 / 60.0;
        }

        if (!window_->pumpEvents())
        {
            running_ = false;
        }
        if (window_->shouldClose())
        {
            running_ = false;
        }

        pumpInput();
        handleCameraKeys();
        scripts_->callOnUpdate(delta);
        plugins_->update(delta);
        net_->update(delta);
        audio_->update();

        SimSnapshot interpolated;
        gatherSnapshots(interpolated);
        stepScenePhysics(delta);
        updateScene(interpolated);

        energyKinetic_.push_back(static_cast<float>(current_.kineticEnergy));
        energyPotential_.push_back(static_cast<float>(current_.potentialEnergy));
        if (energyKinetic_.size() > 600)
        {
            energyKinetic_.erase(energyKinetic_.begin());
            energyPotential_.erase(energyPotential_.begin());
        }

        const bool resized = window_->wasResized() || swapchain_->framebufferResized();
        if (resized)
        {
            int width = 0;
            int height = 0;
            window_->getDrawableSize(width, height);
            if (width > 0 && height > 0)
            {
                swapchain_->recreate({static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height)}, vsync_);
                swapchain_->clearResizedFlag();
                window_->clearResized();
                imgui_->onSwapchainRecreated(*swapchain_);
                renderer_->ensureTargets(swapchain_->extent(), frame_->colorFormat());
                debugDraw_->ensureTargets(frame_->colorFormat(), swapchain_->extent());
            }
        }

        if (!frame_->begin())
        {
            telemetry_->endFrame(wallNow());
            continue;
        }

        rhi::CommandList& commandList = frame_->commandList();

        imgui_->beginFrame();
        drawUi();

        const double renderStart = wallNow();
        const double gpuMs = gpuTimer_->readMilliseconds(frame_->slot());
        renderer_->updateFrameUniforms(frame_->slot(), scene_.camera(), scene_.lightDirection(), scene_.lightColor());
        gpuTimer_->begin(commandList, frame_->slot());
        renderer_->renderScene(commandList, scene_, frame_->swapchainAttachment(), frame_->slot());
        gpuTimer_->end(commandList, frame_->slot());
        const bool physicsOverlay = !sceneWorldName_.empty();
        if ((debugDrawEnabled_ || physicsOverlay) && debugDraw_ != nullptr)
        {
            debugDraw_->clear();
            debugDraw_->addGrid(20.0f, 1.0f, {0.55f, 0.58f, 0.62f, 1.0f}, {0.75f, 0.30f, 0.20f, 1.0f});
            debugDraw_->addAxes({0.0f, 0.02f, 0.0f}, 3.0f);
            if (physicsOverlay)
            {
                drawPhysicsSceneDebug(sceneWorldName_, *sceneWorld_, *debugDraw_);
            }
            rhi::ColorAttachment overlay = frame_->swapchainAttachment();
            overlay.loadOp = rhi::LoadOp::Load;
            debugDraw_->render(commandList, scene_.camera(), overlay);
        }
        const double uiStart = wallNow();
        imgui_->endFrame(commandList.handle());
        const double uiEnd = wallNow();

        frame_->submitAndPresent();

        telemetry_->recordPhysics(physicsTimeMs_.load(std::memory_order_relaxed), physicsSteps_.load(std::memory_order_relaxed));
        telemetry_->recordRender((uiStart - renderStart) * 1000.0);
        telemetry_->recordGpu(gpuMs);
        telemetry_->recordUi((uiEnd - uiStart) * 1000.0);
        telemetry_->setSimulationTime(interpolated.simTime);
        telemetry_->endFrame(wallNow());
    }

    log->info("shutting down");
    shutdown();
    return 0;
}

} // namespace sim::app
