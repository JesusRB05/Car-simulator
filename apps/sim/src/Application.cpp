// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "Application.hpp"

#include <imgui.h>

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
#include "Paths.hpp"
#include "PhysicsScenes.hpp"
#include "Scenes.hpp"
#include "SimUi.hpp"
#include "core/Log.hpp"
#include "core/Version.hpp"
#include "drive/DriveScene.hpp"
#include "modding/Collection.hpp"
#include "modding/Composition.hpp"
#include "modding/ModInstaller.hpp"
#include "modding/Package.hpp"
#include "session/CarCatalog.hpp"
#include "session/GarageScene.hpp"
#include "session/SessionUI.hpp"
#include "session/TrackCatalog.hpp"

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

glm::dvec3 toDvec(const physics::Vec3& value) {
    return {value.x, value.y, value.z};
}

bool keyDown(const platform::InputManager* input, SDL_Scancode scancode) {
    return input != nullptr && input->keyDown(scancode);
}

} // namespace

double Application::wallNow() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

Application::Application() = default;
Application::~Application() = default;

std::filesystem::path Application::locateRoot() const {
    return locateRootDirectory();
}

void Application::reloadContent() {
    auto log = core::Log::get("app");

    // The content catalogs read their descriptors through the VFS, so they see
    // the official cars/tracks and every enabled mod's. The owner resolver gives
    // each descriptor its namespaced reference (`<mod-id>/<content-id>`). A
    // descriptor that fails validation is logged and skipped.
    const ContentOwner owner = [this](std::string_view path) { return mods_->modIdFor(path); };

    // Additive composition: the patches, skins and parts every mod adds without
    // copying anything, plus the conflicts between mods. Loaded before the
    // catalogs so a patch is applied to its target before validation.
    modding::Composition composition = modding::loadComposition(*vfs_, owner);
    std::vector<std::pair<std::string, std::string>> overrides;
    for (const modding::ModStatus& mod : mods_->report().mods) {
        for (const std::string& path : mod.overrides) {
            overrides.emplace_back(mod.id, path);
        }
    }
    modding::detectConflicts(composition, overrides);
    for (const std::string& error : composition.errors) {
        log->warn("composition: {}", error);
    }
    for (const modding::CompositionConflict& conflict : composition.conflicts) {
        log->warn("mod conflict: {}", conflict.message);
    }

    CarCatalog::instance().reload(*vfs_, owner, &composition);
    TrackCatalog::instance().reload(*vfs_, owner, &composition);
}

bool Application::initialise() {
    sdl_ = std::make_unique<platform::SdlContext>();
    if (!sdl_->valid()) {
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

    // A crafted archive can claim an enormous uncompressed size; cap how far a
    // single file may expand (256 MiB by default, override with vfs.max_read_bytes).
    vfs_->setMaxReadSize(static_cast<std::uint64_t>(
        config_->getInt("vfs.max_read_bytes", static_cast<std::int64_t>(vfs::VirtualFileSystem::kDefaultMaxReadSize))));

    session_.load(*config_);

    // Driver aids start from the persisted session state (all three on by
    // default); the physics thread applies the mask to the car's control unit.
    applySessionAids();

    inputProfiles_ = std::make_unique<input::ProfileStore>(*config_);
    wheel_ = std::make_unique<input::WheelManager>(*input_, input::CompatibilityDatabase::instance(), *inputProfiles_);
    wheel_->ffb().settings() = inputProfiles_->loadFfbSettings();
    wheel_->ffb().settings().caps = {};

    modding::ModManager::Paths modPaths;
    modPaths.baseContent = rootDirectory_ / "content" / "base";
    modPaths.modsDirectory = locateModsDirectory(*config_);
    // The bundled `mods/` folder (the example mod, hand-installed mods) is still
    // scanned, below the user's own directory.
    modPaths.extraModDirectories.push_back(rootDirectory_ / "mods");
    modPaths.profilesFile = rootDirectory_ / "userdata" / modding::kProfilesFileName;
    log->info("mods directory: {}", modPaths.modsDirectory.string());
    exeDir_ = exeDir;
    installer_ = std::make_unique<modding::ModInstaller>(*vfs_, modPaths.modsDirectory);
    mods_ = std::make_unique<modding::ModManager>(*vfs_, *config_, modPaths);
    mods_->reload();
    log->info("mods: {}", mods_->report().toString());

    // ModManager rebuilds the mount table from scratch (base + mods), so the
    // engine's own runtime directory is mounted afterwards, at the lowest
    // priority, to make shaders/*.spv resolvable without shadowing content.
    vfs_->mount(exeDir_, "/", true);

    reloadContent();

    net_ = std::make_unique<net::NetworkSystem>();
    net_->start(0);

    const double fixedDt = config_->getDouble("physics.fixed_dt", 1.0 / 1000.0);
    physicsClock_.setFixedDt(fixedDt);
    world_.fixedStep = fixedDt;
    const CarSpec* selectedCar = CarCatalog::instance().find(session_.selection.carId);
    if (selectedCar == nullptr) {
        selectedCar = CarCatalog::instance().defaultCar();
        if (selectedCar == nullptr) {
            core::Log::get("app")->error("no car descriptor loaded; falling back to the library default setup");
        }
    }
    vehicle_ = &vehicle::VehicleModel::spawn(
        world_, selectedCar != nullptr ? selectedCar->setup : vehicle::makeDefaultVehicleSetup(), vehicleGround_);
    // The car's driver aids: attach the unit and honour the persisted mask.
    vehicle_->setElectronics(&electronics_);
    for (std::size_t i = 0; i < wheelOffsets_.size() && i < vehicle_->corners().size(); ++i) {
        const physics::Vec3 local = vehicle_->corners()[i].localMount;
        wheelOffsets_[i] =
            glm::vec3(static_cast<float>(local.x), static_cast<float>(local.y), static_cast<float>(local.z));
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
    window_->setEventHook([this](const SDL_Event& event) {
        imgui_->processEvent(event);
        // A `.simmod` (or a folder with a mod.toml) dropped on the window is
        // inspected and offered for confirmation. Nothing is installed until
        // the user clicks: dropping a file is never silently destructive.
        if (event.type == SDL_EVENT_DROP_FILE && event.drop.data != nullptr) {
            pendingDrops_.emplace_back(event.drop.data);
        }
    });
    ui_ = std::make_unique<SimUi>();

    telemetry_ = std::make_unique<telemetry::Telemetry>(600);

    audio_ = std::make_unique<audio::AudioEngine>();

    // Native plugins are opt-in: nothing in `plugins/` is loaded unless the user
    // sets `plugins.enabled = true` and lists the file in `plugins.allow`. Every
    // library found is fingerprinted (SHA-256) and logged either way.
    plugin::PluginLoadOptions pluginOptions;
    pluginOptions.enabled = config_->getBool("plugins.enabled", false);
    pluginOptions.allow = config_->getStringArray("plugins.allow");
    plugins_ = std::make_unique<plugin::PluginLoader>(*vfs_);
    plugins_->loadDirectory(exeDir / "plugins", pluginOptions);

    scripts_ = std::make_unique<scripting::ScriptHost>(*vfs_);
    if (vfs_->exists("apps/main.lua")) {
        scripts_->loadScript(std::string(modding::kBaseModId), "apps/main.lua");
        scripts_->callOnLoad();
    }

    cubeMesh_ = render::Mesh::makeCube(*device_);
    sphereMesh_ = render::Mesh::makeSphere(*device_);
    cylinderMesh_ = render::Mesh::makeCylinder(*device_);
    groundMesh_ = render::Mesh::makeGroundPlane(*device_, 200.0f);
    scene_.setDirectionalLight({-0.4f, -1.0f, -0.35f}, {1.0f, 0.98f, 0.95f});

    sessionUi_ = std::make_unique<SessionUI>();
    garage_ = std::make_unique<GarageScene>(*device_);
    if (const CarSpec* car = CarCatalog::instance().find(session_.selection.carId)) {
        garage_->setCar(*car);
    }
    // A scene requested on the command line skips the menus and drives at once.
    if (!startupScene_.empty()) {
        session_.screen = GameScreen::Driving;
    }

    startPhysicsThread();
    return true;
}

void Application::shutdown() {
    stopPhysicsThread();

    if (scripts_ != nullptr) {
        scripts_->clear();
    }
    scripts_.reset();
    plugins_.reset();
    audio_.reset();
    telemetry_.reset();

    scene_.clear();
    if (device_ != nullptr) {
        device_->waitIdle();
    }

    imgui_.reset();
    debugDraw_.reset();
    drive_.reset();
    garage_.reset();
    sessionUi_.reset();
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
    if (wheel_ != nullptr) {
        wheel_->stopForceFeedback();
        wheel_->storeProfiles();
        wheel_.reset();
    }
    inputProfiles_.reset();
    if (config_ != nullptr) {
        session_.save(*config_);
        config_->save();
    }
    config_.reset();
    vfs_.reset();
    input_.reset();
    window_.reset();
    sdl_.reset();

    core::Log::shutdown();
}

void Application::startPhysicsThread() {
    physicsRunning_.store(true, std::memory_order_release);
    physicsThread_ = std::thread([this] { physicsLoop(); });
}

void Application::stopPhysicsThread() {
    if (!physicsThread_.joinable()) {
        return;
    }
    physicsRunning_.store(false, std::memory_order_release);
    physicsThread_.join();
}

void Application::physicsLoop() {
    const double dt = physicsClock_.fixedDt();
    double last = wallNow();

    while (physicsRunning_.load(std::memory_order_acquire)) {
        if (snapshotSaveRequest_.exchange(false)) {
            std::lock_guard<std::mutex> lock(snapshotMutex_);
            savedWorld_ = world_.saveSnapshot();
            snapshotValid_.store(true, std::memory_order_release);
        }
        if (snapshotRestoreRequest_.exchange(false)) {
            std::lock_guard<std::mutex> lock(snapshotMutex_);
            if (snapshotValid_.load(std::memory_order_acquire)) {
                world_.restoreSnapshot(savedWorld_);
            }
        }

        const double now = wallNow();
        double frameDelta = now - last;
        last = now;
        if (frameDelta < 0.0) {
            frameDelta = 0.0;
        }
        if (frameDelta > 0.25) {
            frameDelta = 0.25;
        }

        const core::StepResult result = physicsClock_.advance(frameDelta);

        if (result.steps > 0) {
            // Apply the driver-aid switches before stepping; the unit is then
            // sampled by the model once per fixed step.
            const unsigned aids = aidsMask_.load(std::memory_order_relaxed);
            electronics_.setEnabled((aids & kAidAbs) != 0u, (aids & kAidTcs) != 0u, (aids & kAidEsc) != 0u);

            vehicle::VehicleInput input;
            input.throttle = inputThrottle_.load(std::memory_order_relaxed);
            input.brake = inputBrake_.load(std::memory_order_relaxed);
            input.steer = inputSteer_.load(std::memory_order_relaxed);
            input.handbrake = inputHandbrake_.load(std::memory_order_relaxed);
            vehicle_->setInput(input);

            const double startMs = wallNow();
            for (std::uint32_t i = 0; i < result.steps; ++i) {
                // VehicleModel advances the world it was spawned into, so the
                // chassis and the rest of the world share one fixed step.
                vehicle_->step(dt);
            }
            physicsTimeMs_.store((wallNow() - startMs) * 1000.0, std::memory_order_relaxed);
            physicsSteps_.store(static_cast<int>(result.steps), std::memory_order_relaxed);

            SimSnapshot snapshot;
            snapshot.simTime = physicsClock_.simTime();
            snapshot.wallTime = wallNow();
            snapshot.steps = static_cast<int>(result.steps);
            snapshot.vehiclePosition = toDvec(vehicle_->position());
            snapshot.heading = vehicle_->heading();
            snapshot.speed = vehicle_->forwardSpeed();
            snapshot.rpm = vehicle_->engineSpeed() * 9.549296585513721; // rad/s -> rpm
            snapshot.steer = vehicle_->steerAngle();
            snapshot.absActive = electronics_.absActive();
            snapshot.tcsActive = electronics_.tcsActive();
            snapshot.escActive = electronics_.escActive();
            snapshot.aidsThrottleScale = electronics_.throttleScale();
            for (std::size_t i = 0; i < snapshot.wheelSpin.size(); ++i) {
                snapshot.wheelSpin[i] = vehicle_->corners()[i].spinAngle;
            }
            for (std::size_t i = 0; i < world_.bodyCount(); ++i) {
                const physics::RigidBody& body = world_.body(i);
                if (!body.isDynamic()) {
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

void Application::pumpInput() {
    input_->refresh();
    wheel_->update(frameDelta_);

    double throttle = 0.0;
    double brake = 0.0;
    double steer = 0.0;
    bool handbrake = false;

    if (keyDown(input_.get(), SDL_SCANCODE_W) || keyDown(input_.get(), SDL_SCANCODE_UP)) {
        throttle = 1.0;
    }
    if (keyDown(input_.get(), SDL_SCANCODE_S) || keyDown(input_.get(), SDL_SCANCODE_DOWN)) {
        brake = 1.0;
    }
    if (keyDown(input_.get(), SDL_SCANCODE_A) || keyDown(input_.get(), SDL_SCANCODE_LEFT)) {
        steer += 1.0;
    }
    if (keyDown(input_.get(), SDL_SCANCODE_D) || keyDown(input_.get(), SDL_SCANCODE_RIGHT)) {
        steer -= 1.0;
    }
    handbrake = keyDown(input_.get(), SDL_SCANCODE_SPACE);

    // The keyboard is the fallback; any connected wheel, pedal set or gamepad
    // contributes on top of it (largest throttle/brake wins, the wheel owns the
    // steering once it moves past its deadzone).
    const input::Controls& device = wheel_->controls();
    if (device.hasDrivingDevice) {
        if (std::abs(device.steer) > 0.02) {
            steer = device.steer;
        }
        throttle = std::max(throttle, device.throttle);
        brake = std::max(brake, device.brake);
        handbrake = handbrake || device.handbrake > 0.5;
    }

    inputThrottle_.store(std::clamp(throttle, 0.0, 1.0), std::memory_order_relaxed);
    inputBrake_.store(std::clamp(brake, 0.0, 1.0), std::memory_order_relaxed);
    inputSteer_.store(std::clamp(steer, -1.0, 1.0), std::memory_order_relaxed);
    inputHandbrake_.store(handbrake, std::memory_order_relaxed);

    // Force feedback from the last physics snapshot: a basic self-aligning
    // torque that grows with speed and steering angle. The richer model
    // (understeer/oversteer, kerbs, impacts) is a documented follow-up.
    input::FfbInput ffb;
    const double speedFactor = std::clamp(std::abs(current_.speed) / 45.0, 0.0, 1.0);
    const double normalizedSteer = std::clamp(current_.steer / 0.5, -1.0, 1.0);
    ffb.speed = speedFactor;
    ffb.steeringTorque = std::clamp(normalizedSteer * speedFactor, -1.0, 1.0);
    wheel_->setFfbInput(ffb);
}

void Application::applySessionAids() {
    unsigned aids = 0;
    aids |= session_.abs ? kAidAbs : 0u;
    aids |= session_.tcs ? kAidTcs : 0u;
    aids |= session_.esc ? kAidEsc : 0u;
    aidsMask_.store(aids, std::memory_order_relaxed);
}

void Application::gatherSnapshots(SimSnapshot& interpolated) {
    SimSnapshot snapshot;
    while (snapshots_.pop(snapshot)) {
        previous_ = current_;
        current_ = snapshot;
        hasSnapshot_ = true;
    }

    if (!hasSnapshot_) {
        interpolated = SimSnapshot{};
        return;
    }

    const double dt = physicsClock_.fixedDt();
    double alpha = 0.0;
    if (dt > 0.0) {
        alpha = std::clamp((wallNow() - current_.wallTime) / dt, 0.0, 1.0);
    }

    interpolated = current_;
    interpolated.simTime = previous_.simTime + (current_.simTime - previous_.simTime) * alpha;
    interpolated.vehiclePosition =
        previous_.vehiclePosition + (current_.vehiclePosition - previous_.vehiclePosition) * alpha;
    interpolated.heading = previous_.heading + (current_.heading - previous_.heading) * alpha;
    interpolated.speed = previous_.speed + (current_.speed - previous_.speed) * alpha;
    interpolated.rpm = previous_.rpm + (current_.rpm - previous_.rpm) * alpha;
    interpolated.steer = previous_.steer + (current_.steer - previous_.steer) * alpha;
    for (std::size_t i = 0; i < interpolated.wheelSpin.size(); ++i) {
        interpolated.wheelSpin[i] = previous_.wheelSpin[i] + (current_.wheelSpin[i] - previous_.wheelSpin[i]) * alpha;
    }
}

void Application::stepDriveScene(double delta) {
    // The circuit's car holds the shared control unit; the aid setting is the
    // same session state the menu and the Vehicle panel edit.
    const unsigned aids = aidsMask_.load(std::memory_order_relaxed);
    drive_->setAidsEnabled((aids & kAidAbs) != 0u, (aids & kAidTcs) != 0u, (aids & kAidEsc) != 0u);

    // The drive keys take priority over the UI shortcuts; ImGui widgets that
    // want the keyboard (text fields) still win.
    drive::DriveKeys keys;
    if (!ImGui::GetIO().WantCaptureKeyboard) {
        keys.throttle = keyDown(input_.get(), SDL_SCANCODE_W) || keyDown(input_.get(), SDL_SCANCODE_UP);
        keys.brake = keyDown(input_.get(), SDL_SCANCODE_S) || keyDown(input_.get(), SDL_SCANCODE_DOWN);
        keys.left = keyDown(input_.get(), SDL_SCANCODE_A) || keyDown(input_.get(), SDL_SCANCODE_LEFT);
        keys.right = keyDown(input_.get(), SDL_SCANCODE_D) || keyDown(input_.get(), SDL_SCANCODE_RIGHT);
        keys.handbrake = keyDown(input_.get(), SDL_SCANCODE_SPACE);
    }
    const input::Controls& device = wheel_->controls();
    if (device.hasDrivingDevice) {
        keys.throttleAnalog = device.throttle;
        keys.brakeAnalog = device.brake;
        keys.steerAnalog = device.steer;
        keys.handbrakeAnalog = device.handbrake;
    }
    drive_->setKeys(keys);
    drive_->advance(delta); // the fixed 1 ms step is internal; no pause / time scale
}

void Application::stepScenePhysics(double delta) {
    if (driveActive()) {
        stepDriveScene(delta);
        return;
    }
    if (sceneWorld_ == nullptr || ui_ == nullptr || ui_->paused()) {
        return;
    }
    // The time scale only changes how much wall time is fed to the fixed-step
    // integrator; dt stays sceneWorld_->fixedStep. Long hitches are clamped so
    // a stalled frame never triggers a burst of thousands of steps.
    const double wall = std::min(delta, 0.1);
    sceneWorld_->advance(wall * static_cast<double>(ui_->timeScale()));
}

void Application::loadScene(const std::string& name) {
    sceneName_ = name;
    sceneWorld_.reset();
    if (drive_ != nullptr) {
        // Leaving the drive scene hands the camera back to sceneCamera_. Its
        // meshes may still be in flight on the GPU: drop every reference first.
        scene_.clear();
        device_->waitIdle();
        drive_.reset();
    }
    if (ui_ != nullptr) {
        ui_->resetHistory();
    }
    if (name.empty()) {
        return;
    }
    if (drive::isDriveScene(name)) {
        const CarSpec* car = CarCatalog::instance().find(session_.selection.carId);
        if (car == nullptr) {
            car = CarCatalog::instance().defaultCar();
        }
        const TrackSpec* track = TrackCatalog::instance().find(session_.selection.trackId);
        if (track == nullptr) {
            track = TrackCatalog::instance().defaultTrack();
        }
        // Fallbacks: the vehicle library default setup and the reference circuit
        // description, used only when no content loaded at all.
        static const drive::TrackDescription kReferenceTrack = drive::defaultTrackDescription();
        drive_ = std::make_unique<drive::DriveScene>(*device_, track != nullptr ? track->track : kReferenceTrack,
                                                     car != nullptr ? car->setup : vehicle::makeDefaultVehicleSetup());
        drive_->snapCamera();
        return;
    }

    // Build once with the curated camera, then adopt it as the orbit start.
    const float aspect = swapchain_->extent().height > 0 ? static_cast<float>(swapchain_->extent().width) /
                                                               static_cast<float>(swapchain_->extent().height)
                                                         : 16.0f / 9.0f;
    if (isPhysicsScene(name)) {
        sceneWorld_ = std::make_unique<physics::PhysicsWorld>();
        buildPhysicsWorld(name, *sceneWorld_);
        buildSceneFromWorld(*sceneWorld_, scene_, cubeMesh_, sphereMesh_, cylinderMesh_, groundMesh_);
        physicsSceneCamera(name, scene_, aspect);
    } else {
        buildScene(name, scene_, cubeMesh_, groundMesh_);
    }
    glm::vec3 center(0.0f);
    float radius = 1.0f;
    if (!sceneBounds(scene_, groundMesh_.get(), center, radius)) {
        center = glm::vec3(0.0f);
    }
    sceneCamera_.fromCamera(scene_.camera(), center);
}

void Application::startSession() {
    session_.screen = GameScreen::Driving;
    // Only one circuit and one car exist today, so every track id maps to the
    // same "drive" scene; the catalog is ready for more.
    loadScene("drive");
}

void Application::handleGarageCamera() {
    if (garage_ == nullptr) {
        return;
    }
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse || io.WantTextInput) {
        return;
    }
    OrbitCamera& camera = garage_->camera();
    if (ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        camera.orbit(io.MouseDelta.x, io.MouseDelta.y);
    }
    camera.zoom(io.MouseWheel);
}

void Application::frameScene() {
    glm::vec3 center(0.0f);
    float radius = 1.0f;
    if (!sceneName_.empty() && sceneBounds(scene_, groundMesh_.get(), center, radius)) {
        sceneCamera_.frame(center, radius);
    }
}

void Application::updateScene(const SimSnapshot& interpolated) {
    const float aspect = swapchain_->extent().height > 0 ? static_cast<float>(swapchain_->extent().width) /
                                                               static_cast<float>(swapchain_->extent().height)
                                                         : 16.0f / 9.0f;

    // Menu screens: the simulation is paused and the showroom fills the frame.
    if (session_.screen != GameScreen::Driving && garage_ != nullptr) {
        garage_->buildScene(scene_, aspect);
        return;
    }

    if (driveActive()) {
        drive_->buildScene(scene_, aspect, frameDelta_);
        return;
    }

    if (!sceneName_.empty()) {
        if (sceneWorld_ != nullptr) {
            buildSceneFromWorld(*sceneWorld_, scene_, cubeMesh_, sphereMesh_, cylinderMesh_, groundMesh_);
        } else {
            buildScene(sceneName_, scene_, cubeMesh_, groundMesh_);
        }
        sceneCamera_.apply(scene_.camera(), aspect);
        return;
    }

    scene_.clear();

    const glm::vec3 position(static_cast<float>(interpolated.vehiclePosition.x),
                             static_cast<float>(interpolated.vehiclePosition.y),
                             static_cast<float>(interpolated.vehiclePosition.z));
    const float heading = static_cast<float>(interpolated.heading);
    const float steer = static_cast<float>(interpolated.steer);

    // Heading is a plain yaw about +Y; the car body and wheel offsets are
    // authored with -Z forward, so no axis conversion is needed.
    const glm::mat4 headingRotation = glm::rotate(glm::mat4(1.0f), heading, glm::vec3(0.0f, 1.0f, 0.0f));
    const glm::mat4 base = glm::translate(glm::mat4(1.0f), position) * headingRotation;

    scene_.add(groundMesh_, glm::mat4(1.0f), glm::vec4(0.32f, 0.34f, 0.30f, 1.0f));
    scene_.add(cubeMesh_, base * glm::scale(glm::mat4(1.0f), glm::vec3(1.9f, 1.1f, 4.2f)),
               glm::vec4(0.80f, 0.15f, 0.12f, 1.0f));

    for (std::size_t i = 0; i < wheelOffsets_.size(); ++i) {
        glm::mat4 wheel = base * glm::translate(glm::mat4(1.0f), wheelOffsets_[i]);
        if (i < 2) {
            wheel = wheel * glm::rotate(glm::mat4(1.0f), steer, glm::vec3(0.0f, 1.0f, 0.0f));
        }
        // Rolling forward (towards -Z) is a rotation about body -X.
        wheel = wheel * glm::rotate(glm::mat4(1.0f), static_cast<float>(interpolated.wheelSpin[i]),
                                    glm::vec3(-1.0f, 0.0f, 0.0f));
        wheel = wheel * glm::scale(glm::mat4(1.0f), glm::vec3(0.68f));
        scene_.add(cubeMesh_, wheel, glm::vec4(0.08f, 0.08f, 0.09f, 1.0f));
    }

    scene_.camera().setPerspective(sceneCamera_.fovDegrees, aspect, 0.1f, 1000.0f);

    const glm::vec3 focus = position + glm::vec3(0.0f, 0.8f, 0.0f);
    if (cameraMode_ == CameraMode::Orbit) {
        const float cp = std::cos(orbitPitch_);
        const glm::vec3 offset(orbitDistance_ * cp * std::sin(orbitYaw_), orbitDistance_ * std::sin(orbitPitch_),
                               orbitDistance_ * cp * std::cos(orbitYaw_));
        scene_.camera().lookAt(focus + offset, focus, glm::vec3(0.0f, 1.0f, 0.0f));
    } else if (cameraMode_ == CameraMode::Free) {
        const glm::vec3 direction(std::cos(freePitch_) * std::sin(freeYaw_), std::sin(freePitch_),
                                  std::cos(freePitch_) * std::cos(freeYaw_));
        scene_.camera().lookAt(freePosition_, freePosition_ + direction, glm::vec3(0.0f, 1.0f, 0.0f));
    } else {
        // Vehicle forward is body -Z rotated by heading about +Y (heading 0 faces -Z).
        const glm::vec3 forward(-std::sin(heading), 0.0f, -std::cos(heading));
        const glm::vec3 eye = position - forward * 8.0f + glm::vec3(0.0f, 3.4f, 0.0f);
        scene_.camera().lookAt(eye, focus, glm::vec3(0.0f, 1.0f, 0.0f));
    }
}

void Application::handleCameraKeys() {
    ImGuiIO& io = ImGui::GetIO();

    if (driveActive()) {
        if (!io.WantTextInput) {
            if (ImGui::IsKeyPressed(ImGuiKey_C, false)) {
                drive_->cycleCamera();
            }
            if (ImGui::IsKeyPressed(ImGuiKey_R, false) && !io.KeyCtrl) {
                drive_->reset();
            }
            if (ImGui::IsKeyPressed(ImGuiKey_G, false) && !io.KeyCtrl) {
                ui_->debugFlags().driveForces = !ui_->debugFlags().driveForces;
            }
        }
        if (!io.WantCaptureMouse) {
            const bool looking = ImGui::IsMouseDown(ImGuiMouseButton_Right);
            drive_->orbitInput(looking ? io.MouseDelta.x : 0.0f, looking ? io.MouseDelta.y : 0.0f, io.MouseWheel);
        }
        return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_C, false)) {
        cameraMode_ = cameraMode_ == CameraMode::Chase   ? CameraMode::Orbit
                      : cameraMode_ == CameraMode::Orbit ? CameraMode::Free
                                                         : CameraMode::Chase;
    }

    if (!sceneName_.empty()) {
        return; // inspection scenes: SimUi drives sceneCamera_
    }

    const bool looking = ImGui::IsMouseDown(ImGuiMouseButton_Right) && !io.WantCaptureMouse;
    if (looking) {
        orbitYaw_ -= io.MouseDelta.x * 0.006f;
        orbitPitch_ = std::clamp(orbitPitch_ - io.MouseDelta.y * 0.006f, -1.45f, 1.45f);
        freeYaw_ -= io.MouseDelta.x * 0.006f;
        freePitch_ = std::clamp(freePitch_ - io.MouseDelta.y * 0.006f, -1.45f, 1.45f);
    }

    if (cameraMode_ == CameraMode::Orbit) {
        orbitDistance_ = std::clamp(orbitDistance_ - io.MouseWheel * 1.5f, 2.0f, 250.0f);
    } else if (cameraMode_ == CameraMode::Free && !io.WantCaptureKeyboard) {
        const glm::vec3 forward(std::cos(freePitch_) * std::sin(freeYaw_), std::sin(freePitch_),
                                std::cos(freePitch_) * std::cos(freeYaw_));
        const glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
        const float speed = 14.0f * static_cast<float>(io.DeltaTime);
        if (ImGui::IsKeyDown(ImGuiKey_W)) {
            freePosition_ += forward * speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_S)) {
            freePosition_ -= forward * speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_A)) {
            freePosition_ -= right * speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_D)) {
            freePosition_ += right * speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_Q)) {
            freePosition_.y -= speed;
        }
        if (ImGui::IsKeyDown(ImGuiKey_E)) {
            freePosition_.y += speed;
        }
    }
}

void Application::updateModInstall() {
    // A finished install is applied here, at a frame boundary, so no descriptor
    // pointer handed out during the previous frame is still in use.
    if (reloadContentPending_) {
        reloadContentPending_ = false;
        mods_->reload();
        vfs_->mount(exeDir_, "/", true);
        reloadContent();
    }
    if (pendingInstall_ != nullptr || pendingDrops_.empty() || installer_ == nullptr) {
        return;
    }
    const std::filesystem::path source = pendingDrops_.front();
    pendingDrops_.erase(pendingDrops_.begin());
    auto plan = std::make_unique<modding::InstallPlan>(installer_->inspect(source));
    installSummary_ = plan->summary();
    if (plan->collection) {
        // What the pack still needs, checked against what is installed, so the
        // user sees the whole picture before confirming.
        std::vector<modding::InstalledMod> installed;
        for (const modding::ModStatus& mod : mods_->report().mods) {
            installed.push_back({mod.id, mod.version});
        }
        for (const std::string& line : modding::planCollection(*plan->collection, installed).messages()) {
            installSummary_ += "\n  ! " + line;
        }
    }
    pendingInstall_ = std::move(plan);
}

void Application::drawInstallConfirm() {
    if (pendingInstall_ == nullptr) {
        return;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    bool open = true;
    if (ImGui::Begin("Install mod", &open, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextUnformatted(installSummary_.c_str());
        ImGui::Separator();
        if (pendingInstall_->ok) {
            if (ImGui::Button("Install")) {
                finishInstall(true);
            }
            ImGui::SameLine();
        }
        if (ImGui::Button(pendingInstall_->ok ? "Cancel" : "Close")) {
            finishInstall(false);
        }
    }
    ImGui::End();
    if (!open) {
        finishInstall(false);
    }
}

void Application::finishInstall(bool confirm) {
    if (pendingInstall_ == nullptr) {
        return;
    }
    auto log = core::Log::get("modding");
    if (confirm) {
        const modding::InstallResult result = installer_->install(*pendingInstall_);
        if (!result.ok) {
            log->error("install failed: {}", result.error);
        } else {
            log->info("installed '{}'", pendingInstall_->modId);
            reloadContentPending_ = true;
        }
    }
    pendingInstall_.reset();
    installSummary_.clear();
}

void Application::drawUi() {
    // The driver aids are edited in the main menu and in the Vehicle panel; the
    // physics thread only ever sees the derived mask.
    applySessionAids();
    updateModInstall();
    drawInstallConfirm();
    if (session_.screen != GameScreen::Driving) {
        drawSessionUi();
        return;
    }

    SimUiFrame frame;
    frame.activeScene = sceneName_;
    frame.driveActive = driveActive();
    frame.world = sceneWorld_.get();
    if (sceneWorld_ != nullptr) {
        frame.hasEnergy = true;
        frame.totalEnergy = physicsSceneEnergy(sceneName_, *sceneWorld_);
    } else if (sceneName_.empty() && current_.bodyCount > 0) {
        frame.hasEnergy = true;
        frame.totalEnergy = current_.kineticEnergy + current_.potentialEnergy;
    }
    frame.cpuFrameMs = telemetry_->frameTimeMs();
    frame.gpuFrameMs = lastGpuMs_;
    frame.fps = telemetry_->fps();
    frame.nowSeconds = wallNow();
    frame.viewportHeight = static_cast<float>(swapchain_->extent().height);
    frame.wheel = wheel_.get();

    const SimUiActions actions = ui_->draw(frame, sceneCamera_);
    if (driveActive()) {
        drive_->drawHud();
    }
    if (actions.quit) {
        window_->requestClose();
    }
    if (actions.loadScene) {
        loadScene(actions.scene);
    }
    if (actions.reset && sceneWorld_ != nullptr) {
        const glm::vec3 target = sceneCamera_.target;
        const float yaw = sceneCamera_.yaw;
        const float pitch = sceneCamera_.pitch;
        const float distance = sceneCamera_.distance;
        loadScene(sceneName_);
        // A reset restarts the physics, not the user's view.
        sceneCamera_.target = target;
        sceneCamera_.yaw = yaw;
        sceneCamera_.pitch = pitch;
        sceneCamera_.distance = distance;
    }
    if (actions.step && sceneWorld_ != nullptr) {
        sceneWorld_->step(sceneWorld_->fixedStep);
    }
    if (actions.frameScene) {
        frameScene();
    }
    if (actions.loadScene || actions.reset || actions.step) {
        updateScene(current_); // show the new state this frame
    }

    PanelVisibility& panels = ui_->panels();
    if (panels.vehicle) {
        if (ImGui::Begin("Vehicle", &panels.vehicle)) {
            ImGui::Text("Speed: %.1f km/h", current_.speed * 3.6);
            ImGui::Text("RPM: %.0f", current_.rpm);
            ImGui::Text("Sim time: %.2f s", current_.simTime);
            ImGui::Text("Position: (%.1f, %.1f, %.1f)", current_.vehiclePosition.x, current_.vehiclePosition.y,
                        current_.vehiclePosition.z);
            ImGui::Text("Physics thread: %.3f ms / %d steps", telemetry_->physicsTimeMs(), telemetry_->physicsSteps());
            ImGui::Text("Bodies: %d  Kinetic: %.2f J  Potential: %.2f J", current_.bodyCount, current_.kineticEnergy,
                        current_.potentialEnergy);

            ImGui::Separator();
            ImGui::TextUnformatted("Driver aids (ABS / TCS / ESC)");
            // The same persisted setting the main menu edits; the mask the
            // physics thread reads is derived from it every frame.
            ImGui::Checkbox("ABS", &session_.abs);
            ImGui::SameLine();
            ImGui::Checkbox("TCS", &session_.tcs);
            ImGui::SameLine();
            ImGui::Checkbox("ESC", &session_.esc);
            ImGui::SameLine();
            ImGui::TextDisabled("throttle x%.2f", current_.aidsThrottleScale);
            const auto aidState = [](bool active, const char* name) {
                ImGui::TextColored(active ? ImVec4(0.40f, 0.90f, 0.45f, 1.0f) : ImVec4(0.55f, 0.55f, 0.55f, 1.0f),
                                   "%s%s", name, active ? " active" : "");
            };
            aidState(current_.absActive, "ABS");
            ImGui::SameLine();
            aidState(current_.tcsActive, "TCS");
            ImGui::SameLine();
            aidState(current_.escActive, "ESC");

            ImGui::Separator();
            const char* modeNames[] = {"Chase", "Orbit", "Free"};
            int mode = static_cast<int>(cameraMode_);
            if (ImGui::Combo("Camera (C)", &mode, modeNames, 3)) {
                cameraMode_ = static_cast<CameraMode>(mode);
            }

            ImGui::Separator();
            if (ImGui::Button("Save snapshot")) {
                snapshotSaveRequest_.store(true, std::memory_order_relaxed);
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(!snapshotValid_.load(std::memory_order_relaxed));
            if (ImGui::Button("Restore snapshot")) {
                snapshotRestoreRequest_.store(true, std::memory_order_relaxed);
            }
            ImGui::EndDisabled();
            ImGui::Text("Snapshot: %s", snapshotValid_.load() ? "available" : "none");
        }
        ImGui::End();
    }

    if (panels.mods) {
        if (ImGui::Begin("Mods", &panels.mods)) {
            const modding::ModReport& report = mods_->report();
            for (const modding::ModStatus& mod : report.mods) {
                const char* state = mod.state == modding::ModState::Active     ? "active"
                                    : mod.state == modding::ModState::Disabled ? "disabled"
                                                                               : "invalid";
                ImGui::BulletText("%s %s [%s]", mod.name.c_str(), mod.version.c_str(), state);
            }
            if (report.mods.empty()) {
                ImGui::TextUnformatted("No mods found.");
            }

            ImGui::Separator();
            ImGui::TextUnformatted("Native plugins");
            ImGui::TextWrapped("Native plugins run with the full permissions of the game. Only load code you trust.");
            if (!plugins_->enabled()) {
                ImGui::TextColored({1.0f, 0.75f, 0.25f, 1.0f},
                                   "Disabled: set plugins.enabled = true and list each trusted file in "
                                   "plugins.allow in config.toml.");
            }
            for (const plugin::PluginRecord& record : plugins_->plugins()) {
                const char* state = record.state == plugin::PluginState::Loaded    ? "loaded"
                                    : record.state == plugin::PluginState::Skipped ? "skipped"
                                                                                   : "failed";
                ImGui::BulletText("%s [%s]", record.path.filename().string().c_str(), state);
                ImGui::Text("    sha256 %s", record.sha256.c_str());
                if (!record.error.empty()) {
                    ImGui::TextWrapped("    %s", record.error.c_str());
                }
            }
            if (plugins_->plugins().empty()) {
                ImGui::TextUnformatted("No native plugins found.");
            }
        }
        ImGui::End();
    }
}

void Application::drawSessionUi() {
    const SessionUI::Action action = sessionUi_->draw(session_, wheel_.get());
    if (sessionUi_->consumeWizardFinished()) {
        session_.save(*config_);
    }
    if (action == SessionUI::Action::Quit) {
        window_->requestClose();
    } else if (action == SessionUI::Action::StartDrive) {
        startSession();
    }
}

void Application::drawDebugOverlay(rhi::CommandList& commandList) {
    if (debugDraw_ == nullptr || ui_ == nullptr) {
        return;
    }
    const DebugDrawFlags& flags = ui_->debugFlags();
    debugDraw_->clear();
    if (driveActive()) {
        if (flags.driveForces) {
            drive_->drawDebug(*debugDraw_);
        }
    } else if (flags.grid) {
        debugDraw_->addGrid(20.0f, 1.0f, {0.55f, 0.58f, 0.62f, 1.0f}, {0.75f, 0.30f, 0.20f, 1.0f});
    }
    if (!driveActive() && flags.worldAxes) {
        debugDraw_->addAxes({0.0f, 0.02f, 0.0f}, 3.0f);
    }
    if (sceneWorld_ != nullptr) {
        PhysicsDebugOptions options;
        options.reference = flags.reference;
        options.joints = flags.joints;
        options.springs = flags.springs;
        options.bodyAxes = flags.bodyAxes;
        options.velocities = flags.velocities;
        drawPhysicsSceneDebug(sceneName_, *sceneWorld_, *debugDraw_, options);
    }
    if (debugDraw_->empty()) {
        return;
    }
    rhi::ColorAttachment overlay = frame_->swapchainAttachment();
    overlay.loadOp = rhi::LoadOp::Load;
    debugDraw_->render(commandList, scene_.camera(), overlay);
}

int Application::run() {
    if (!initialise()) {
        core::Log::get()->critical("initialisation failed");
        return 1;
    }

    auto log = core::Log::get();

    if (!startupScene_.empty() && startupScene_ != "driving") {
        const std::vector<SceneInfo>& catalog = sceneCatalog();
        const bool known = std::any_of(catalog.begin(), catalog.end(),
                                       [&](const SceneInfo& info) { return info.name == startupScene_; });
        if (known) {
            loadScene(startupScene_);
        } else {
            log->warn("unknown scene '{}', starting the interactive driving scene", startupScene_);
        }
    }

    while (running_) {
        const double frameStart = wallNow();
        telemetry_->beginFrame(frameStart);

        frameClock_.tick();
        double delta = frameClock_.lastDelta();
        if (delta <= 0.0) {
            delta = 1.0 / 60.0;
        }
        frameDelta_ = delta;

        if (!window_->pumpEvents()) {
            running_ = false;
        }
        if (window_->shouldClose()) {
            running_ = false;
        }

        pumpInput();
        if (session_.screen == GameScreen::Driving) {
            handleCameraKeys();
            // Escape returns to the main menu (unless a text field wants it).
            if (!ImGui::GetIO().WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                session_.screen = GameScreen::MainMenu;
                loadScene("");
            }
        } else {
            handleGarageCamera();
        }
        scripts_->callOnUpdate(delta);
        plugins_->update(delta);
        net_->update(delta);
        audio_->update();

        SimSnapshot interpolated;
        gatherSnapshots(interpolated);
        if (session_.screen == GameScreen::Driving) {
            stepScenePhysics(delta);
        }
        updateScene(interpolated);

        const bool resized = window_->wasResized() || swapchain_->framebufferResized();
        if (resized) {
            int width = 0;
            int height = 0;
            window_->getDrawableSize(width, height);
            if (width > 0 && height > 0) {
                swapchain_->recreate({static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height)}, vsync_);
                swapchain_->clearResizedFlag();
                window_->clearResized();
                imgui_->onSwapchainRecreated(*swapchain_);
                renderer_->ensureTargets(swapchain_->extent(), frame_->colorFormat());
                debugDraw_->ensureTargets(frame_->colorFormat(), swapchain_->extent());
            }
        }

        if (!frame_->begin()) {
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
        const glm::vec4& sky = scene_.skyColor();
        renderer_->renderScene(commandList, scene_,
                               frame_->swapchainAttachment(rhi::LoadOp::Clear, sky.r, sky.g, sky.b, sky.a),
                               frame_->slot());
        gpuTimer_->end(commandList, frame_->slot());
        lastGpuMs_ = gpuMs;
        drawDebugOverlay(commandList);
        const double uiStart = wallNow();
        imgui_->endFrame(commandList.handle());
        const double uiEnd = wallNow();

        frame_->submitAndPresent();

        telemetry_->recordPhysics(physicsTimeMs_.load(std::memory_order_relaxed),
                                  physicsSteps_.load(std::memory_order_relaxed));
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
