// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "drive/Obstacles.hpp"
#include "drive/Track.hpp"
#include "drive/TrackGroundQuery.hpp"
#include "physics/PhysicsWorld.hpp"
#include "vehicle/VehicleModel.hpp"

namespace sim::rhi {
class Device;
} // namespace sim::rhi

namespace sim::render {
class DebugDraw;
class Mesh;
class Scene;
} // namespace sim::render

namespace sim::app::drive {

// Name of the drivable scene (`sim --scene drive`).
inline constexpr const char* kDriveSceneName = "drive";
inline bool isDriveScene(const std::string& name) {
    return name == kDriveSceneName;
}

// Raw keyboard state for one frame; DriveScene turns it into VehicleInput.
// A connected wheel, pedal set or gamepad contributes analog values on top of
// the digital keys (the analog value wins for steering and handbrake, and is
// taken alongside the keys for throttle and brake).
struct DriveKeys {
    bool throttle = false;
    bool brake = false;
    bool left = false;
    bool right = false;
    bool handbrake = false;
    double throttleAnalog = 0.0;  // [0, 1]
    double brakeAnalog = 0.0;     // [0, 1]
    double steerAnalog = 0.0;     // [-1, 1], + = left
    double handbrakeAnalog = 0.0; // [0, 1]
};

enum class DriveCamera {
    Chase,
    Cockpit,
    Orbit,
};

// The keyboard-drivable test scene: a procedural circuit, the assembled
// VehicleModel in its own PhysicsWorld stepped with the fixed 1 ms step, a
// smoothed chase camera, an ImGui HUD and a debug overlay (suspension rays,
// contact points, tyre forces). Simulation and rendering live on the calling
// (main) thread.
class DriveScene {
public:
    explicit DriveScene(rhi::Device& device);
    ~DriveScene();

    DriveScene(const DriveScene&) = delete;
    DriveScene& operator=(const DriveScene&) = delete;

    // Puts the car back on the start line at rest and clears the lap timer.
    void reset();
    void cycleCamera();
    DriveCamera camera() const { return cameraMode_; }

    void setKeys(const DriveKeys& keys);
    // Mouse look for the orbit camera (pixels, wheel notches).
    void orbitInput(float deltaX, float deltaY, float wheel);

    // Advances the simulation by the fixed steps that fit into `wallDelta`
    // (capped so a stalled frame cannot trigger a spiral of death).
    void advance(double wallDelta);
    // Advances exactly `seconds` of simulated time (headless captures).
    void simulate(double seconds);

    // Rebuilds `scene` (track, terrain, car) and positions the camera. The
    // chase camera eases towards its target over `frameDelta` seconds.
    void buildScene(render::Scene& scene, float aspect, double frameDelta);
    // Places the camera exactly on its target (no smoothing) on the next build.
    void snapCamera() { cameraSnap_ = true; }

    void drawHud() const;
    void drawDebug(render::DebugDraw& draw) const;

    const vehicle::VehicleModel& car() const { return *car_; }
    const Track& track() const { return track_; }
    double simTime() const { return world_.time(); }

private:
    void buildMeshes(rhi::Device& device);
    void buildObstacles(rhi::Device& device);
    void updateLapTimer();
    void updateCamera(float aspect, double frameDelta, render::Scene& scene);

    // A merged obstacle mesh plus the colour it is drawn with. The hundreds of
    // static blocks collapse into one batch per colour, like the kerbs.
    struct ObstacleBatch {
        std::shared_ptr<render::Mesh> mesh;
        glm::vec4 color{1.0f};
    };

    Track track_;
    TrackGroundQuery ground_{track_};
    physics::PhysicsWorld world_;
    vehicle::VehicleModel* car_ = nullptr;
    double remainder_ = 0.0;

    std::vector<Obstacle> obstacles_;
    std::vector<ObstacleBatch> obstacleBatches_;

    std::shared_ptr<render::Mesh> cube_;
    std::shared_ptr<render::Mesh> cylinder_;
    std::shared_ptr<render::Mesh> terrain_;
    std::shared_ptr<render::Mesh> farGround_;
    std::shared_ptr<render::Mesh> asphalt_;
    std::shared_ptr<render::Mesh> verge_;
    std::shared_ptr<render::Mesh> edgeLines_;
    std::shared_ptr<render::Mesh> kerbRed_;
    std::shared_ptr<render::Mesh> kerbWhite_;
    std::shared_ptr<render::Mesh> startLine_;
    std::shared_ptr<render::Mesh> startLineDark_;

    DriveCamera cameraMode_ = DriveCamera::Chase;
    bool cameraSnap_ = true;
    glm::vec3 chaseEye_{0.0f};
    glm::vec3 chaseTarget_{0.0f};
    float orbitYaw_ = 0.8f;
    float orbitPitch_ = 0.35f;
    float orbitDistance_ = 9.0f;

    // Lap timing: a lap counts when the car crosses the start line forwards
    // after having driven past the far half of the circuit.
    bool lapStarted_ = false;
    bool halfwayReached_ = false;
    double lapStartTime_ = 0.0;
    double lastLap_ = 0.0;
    double bestLap_ = 0.0;
    int laps_ = 0;
    double previousAlong_ = 0.0;
    Surface lastSurface_ = Surface::Asphalt;
};

} // namespace sim::app::drive
