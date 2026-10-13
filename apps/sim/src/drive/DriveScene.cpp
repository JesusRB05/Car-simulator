// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/DriveScene.hpp"

#include <imgui.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "Scenes.hpp"
#include "physics/Math.hpp"
#include "render/DebugDraw.hpp"
#include "render/Mesh.hpp"
#include "render/Scene.hpp"
#include "rhi/Device.hpp"

namespace sim::app::drive {
namespace {

const glm::vec4 kGrassColor{0.24f, 0.42f, 0.20f, 1.0f};
const glm::vec4 kFarGrassColor = kGrassColor;
const glm::vec4 kAsphaltColor{0.20f, 0.20f, 0.22f, 1.0f};
const glm::vec4 kKerbRed{0.85f, 0.10f, 0.08f, 1.0f};
const glm::vec4 kKerbWhite{0.92f, 0.92f, 0.92f, 1.0f};
const glm::vec4 kLineWhite{0.95f, 0.95f, 0.95f, 1.0f};
const glm::vec4 kBodyColor{0.10f, 0.35f, 0.80f, 1.0f};
const glm::vec4 kCabinColor{0.10f, 0.12f, 0.16f, 1.0f};
const glm::vec4 kTyreColor{0.06f, 0.06f, 0.07f, 1.0f};
const glm::vec4 kHubColor{0.75f, 0.76f, 0.80f, 1.0f};
const glm::vec4 kHeadlightColor{1.0f, 0.95f, 0.70f, 1.0f};
const glm::vec4 kTaillightColor{0.90f, 0.05f, 0.05f, 1.0f};

// The visual grass sits a few centimetres under the analytic terrain so the
// asphalt ribbon (drawn exactly on it) never z-fights with it.
constexpr double kGrassDrop = 0.03;
constexpr double kKerbRaise = 0.012;
constexpr double kLineRaise = 0.008;
constexpr double kMaxFrameDelta = 0.1;

glm::vec3 toVec3(const Vec3& v) {
    return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

// CPU mesh builder. Every triangle is emitted with both windings so the flat
// track and terrain surfaces render regardless of the culling convention.
struct MeshBuilder {
    std::vector<render::Vertex> vertices;
    std::vector<std::uint32_t> indices;

    std::uint32_t vertex(const Vec3& position, const Vec3& normal) {
        render::Vertex v;
        v.position = toVec3(position);
        v.normal = toVec3(normal);
        vertices.push_back(v);
        return static_cast<std::uint32_t>(vertices.size() - 1);
    }

    void triangle(std::uint32_t a, std::uint32_t b, std::uint32_t c) {
        indices.insert(indices.end(), {a, b, c, a, c, b});
    }

    void quad(std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d) {
        triangle(a, b, c);
        triangle(a, c, d);
    }

    std::shared_ptr<render::Mesh> build(rhi::Device& device) const {
        return std::make_shared<render::Mesh>(device, vertices, indices);
    }
};

Vec3 onTerrain(const Vec3& p, double raise) {
    return Vec3(p.x, Track::terrainHeight(p.x, p.z) + raise, p.z);
}

glm::mat4 bodyMatrix(const physics::RigidBody& body) {
    return glm::translate(glm::mat4(1.0f), toVec3(body.position)) * glm::mat4_cast(glm::quat(body.orientation));
}

glm::mat4 boxAt(const glm::mat4& frame, const glm::vec3& centre, const glm::vec3& size) {
    return frame * glm::translate(glm::mat4(1.0f), centre) * glm::scale(glm::mat4(1.0f), size);
}

const char* gearLabel(int gear) {
    static const char* labels[] = {"R", "N", "1", "2", "3", "4", "5", "6", "7", "8"};
    const int index = std::clamp(gear + 1, 0, 9);
    return labels[index];
}

std::string formatLap(double seconds) {
    if (seconds <= 0.0) {
        return "--:--.---";
    }
    const int minutes = static_cast<int>(seconds / 60.0);
    const double rest = seconds - 60.0 * minutes;
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%d:%06.3f", minutes, rest);
    return buffer;
}

} // namespace

DriveScene::DriveScene(rhi::Device& device) {
    world_.fixedStep = 1.0 / 1000.0;
    car_ = &RaycastCar::spawn(world_, CarParams{}, track_);
    buildMeshes(device);
    reset();
}

DriveScene::~DriveScene() = default;

void DriveScene::reset() {
    car_->reset(track_.startPosition(), track_.startHeading());
    remainder_ = 0.0;
    lapStarted_ = false;
    halfwayReached_ = false;
    lapStartTime_ = 0.0;
    previousAlong_ = track_.project(track_.startPosition().x, track_.startPosition().z).along;
    cameraSnap_ = true;
}

void DriveScene::cycleCamera() {
    cameraMode_ = cameraMode_ == DriveCamera::Chase     ? DriveCamera::Cockpit
                  : cameraMode_ == DriveCamera::Cockpit ? DriveCamera::Orbit
                                                        : DriveCamera::Chase;
    cameraSnap_ = true;
}

void DriveScene::setKeys(const DriveKeys& keys) {
    DriverInput input;
    input.throttle = keys.throttle ? 1.0 : 0.0;
    input.brake = keys.brake ? 1.0 : 0.0;
    input.steer = (keys.left ? 1.0 : 0.0) - (keys.right ? 1.0 : 0.0);
    input.handbrake = keys.handbrake;
    car_->setInput(input);
}

void DriveScene::orbitInput(float deltaX, float deltaY, float wheel) {
    orbitYaw_ -= deltaX * 0.006f;
    orbitPitch_ = std::clamp(orbitPitch_ + deltaY * 0.006f, -0.2f, 1.45f);
    orbitDistance_ = std::clamp(orbitDistance_ - wheel * 1.0f, 3.0f, 60.0f);
}

void DriveScene::advance(double wallDelta) {
    remainder_ += std::clamp(wallDelta, 0.0, kMaxFrameDelta);
    const double dt = world_.fixedStep;
    while (remainder_ >= dt) {
        world_.step(dt);
        updateLapTimer();
        remainder_ -= dt;
    }
}

void DriveScene::simulate(double seconds) {
    const int steps = static_cast<int>(seconds / world_.fixedStep + 0.5);
    for (int i = 0; i < steps; ++i) {
        world_.step(world_.fixedStep);
        updateLapTimer();
    }
}

void DriveScene::updateLapTimer() {
    const Vec3& position = car_->chassis().position;
    const Track::Projection projection = track_.project(position.x, position.z);
    lastSurface_ = track_.sample(position.x, position.z).surface;
    if (!projection.found) {
        return;
    }
    const double length = track_.length();
    const double startDistance = track_.samples()[track_.startIndex()].distance;
    const auto relative = [&](double along) { return std::fmod(along - startDistance + 2.0 * length, length); };
    const double previous = relative(previousAlong_);
    const double current = relative(projection.along);
    previousAlong_ = projection.along;

    if (current > 0.4 * length && current < 0.6 * length) {
        halfwayReached_ = true;
    }
    const bool crossedForwards = previous > length - 50.0 && current < 50.0;
    if (!crossedForwards) {
        return;
    }
    const double now = world_.time();
    if (lapStarted_ && halfwayReached_) {
        lastLap_ = now - lapStartTime_;
        bestLap_ = bestLap_ <= 0.0 ? lastLap_ : std::min(bestLap_, lastLap_);
        ++laps_;
    }
    if (!lapStarted_ || halfwayReached_) {
        lapStarted_ = true;
        lapStartTime_ = now;
        halfwayReached_ = false;
    }
}

void DriveScene::buildMeshes(rhi::Device& device) {
    const std::vector<TrackSample>& samples = track_.samples();
    const std::size_t count = samples.size();
    const double halfWidth = track_.halfWidth();
    const double kerbWidth = track_.kerbWidth();

    // Shared unit primitives (cube +-0.5, cylinder radius 1 / half-height 1 on +Y)
    // for the car and the gantry. The flat terrain/track meshes below are built
    // double-sided.
    cube_ = render::Mesh::makeCube(device);
    cylinder_ = render::Mesh::makeCylinder(device);

    // Terrain: a regular grid over the circuit, sampled from the analytic height.
    {
        MeshBuilder builder;
        constexpr double kMinX = -160.0;
        constexpr double kMaxX = 290.0;
        constexpr double kMinZ = -340.0;
        constexpr double kMaxZ = 340.0;
        constexpr double kCell = 5.0;
        const int columns = static_cast<int>((kMaxX - kMinX) / kCell);
        const int rows = static_cast<int>((kMaxZ - kMinZ) / kCell);
        for (int row = 0; row <= rows; ++row) {
            for (int column = 0; column <= columns; ++column) {
                const double x = kMinX + kCell * column;
                const double z = kMinZ + kCell * row;
                builder.vertex(Vec3(x, Track::terrainHeight(x, z) - kGrassDrop, z), Track::terrainNormal(x, z));
            }
        }
        const auto index = [&](int column, int row) {
            return static_cast<std::uint32_t>(row * (columns + 1) + column);
        };
        for (int row = 0; row < rows; ++row) {
            for (int column = 0; column < columns; ++column) {
                builder.quad(index(column, row), index(column + 1, row), index(column + 1, row + 1),
                             index(column, row + 1));
            }
        }
        terrain_ = builder.build(device);
    }

    // Far ground out to the horizon, slightly lower than the terrain edge.
    {
        MeshBuilder builder;
        constexpr double kHalf = 2500.0;
        const Vec3 up(0.0, 1.0, 0.0);
        builder.quad(builder.vertex(Vec3(-kHalf, -0.10, -kHalf), up), builder.vertex(Vec3(kHalf, -0.10, -kHalf), up),
                     builder.vertex(Vec3(kHalf, -0.10, kHalf), up), builder.vertex(Vec3(-kHalf, -0.10, kHalf), up));
        farGround_ = builder.build(device);
    }

    // Asphalt ribbon: two vertices per centreline sample, on the terrain.
    {
        MeshBuilder builder;
        for (const TrackSample& sample : samples) {
            const Vec3 left = onTerrain(sample.position - sample.right * halfWidth, 0.0);
            const Vec3 right = onTerrain(sample.position + sample.right * halfWidth, 0.0);
            builder.vertex(left, Track::terrainNormal(left.x, left.z));
            builder.vertex(right, Track::terrainNormal(right.x, right.z));
        }
        for (std::size_t i = 0; i < count; ++i) {
            const std::uint32_t a = static_cast<std::uint32_t>(2 * i);
            const std::uint32_t b = static_cast<std::uint32_t>(2 * ((i + 1) % count));
            builder.quad(a, a + 1, b + 1, b);
        }
        asphalt_ = builder.build(device);
    }

    // Kerbs: 3 m red/white blocks on both edges where the corner is tight.
    {
        MeshBuilder red;
        MeshBuilder white;
        for (std::size_t i = 0; i < count; ++i) {
            if (!samples[i].kerb) {
                continue;
            }
            const TrackSample& a = samples[i];
            const TrackSample& b = samples[(i + 1) % count];
            MeshBuilder& builder = ((i / 3) % 2 == 0) ? red : white;
            for (const double side : {-1.0, 1.0}) {
                const Vec3 a0 = onTerrain(a.position + a.right * (side * halfWidth), kKerbRaise);
                const Vec3 a1 = onTerrain(a.position + a.right * (side * (halfWidth + kerbWidth)), kKerbRaise);
                const Vec3 b0 = onTerrain(b.position + b.right * (side * halfWidth), kKerbRaise);
                const Vec3 b1 = onTerrain(b.position + b.right * (side * (halfWidth + kerbWidth)), kKerbRaise);
                const Vec3 n = Track::terrainNormal(a0.x, a0.z);
                builder.quad(builder.vertex(a0, n), builder.vertex(a1, n), builder.vertex(b1, n),
                             builder.vertex(b0, n));
            }
        }
        kerbRed_ = red.build(device);
        kerbWhite_ = white.build(device);
    }

    // Start/finish line: a 1 m white band across the asphalt.
    {
        MeshBuilder builder;
        const TrackSample& start = samples[track_.startIndex()];
        const Vec3 along = start.tangent * 0.5;
        const Vec3 across = start.right * halfWidth;
        const Vec3 n = Track::terrainNormal(start.position.x, start.position.z);
        builder.quad(builder.vertex(onTerrain(start.position - across - along, kLineRaise), n),
                     builder.vertex(onTerrain(start.position + across - along, kLineRaise), n),
                     builder.vertex(onTerrain(start.position + across + along, kLineRaise), n),
                     builder.vertex(onTerrain(start.position - across + along, kLineRaise), n));
        startLine_ = builder.build(device);
    }
}

void DriveScene::buildScene(render::Scene& scene, float aspect, double frameDelta) {
    scene.clear();
    setDefaultLight(scene);
    scene.add(farGround_, glm::mat4(1.0f), kFarGrassColor);
    scene.add(terrain_, glm::mat4(1.0f), kGrassColor);
    scene.add(asphalt_, glm::mat4(1.0f), kAsphaltColor);
    scene.add(kerbRed_, glm::mat4(1.0f), kKerbRed);
    scene.add(kerbWhite_, glm::mat4(1.0f), kKerbWhite);
    scene.add(startLine_, glm::mat4(1.0f), kLineWhite);

    // Start gantry posts, so the line reads from a distance.
    const TrackSample& start = track_.samples()[track_.startIndex()];
    for (const double side : {-1.0, 1.0}) {
        const Vec3 base = onTerrain(start.position + start.right * (side * (track_.halfWidth() + 2.0)), 0.0);
        scene.add(cube_, boxAt(glm::mat4(1.0f), toVec3(base) + glm::vec3(0.0f, 2.5f, 0.0f), {0.3f, 5.0f, 0.3f}),
                  kLineWhite);
    }
    const Vec3 beam = onTerrain(start.position, 0.0);
    const glm::mat4 beamFrame =
        glm::translate(glm::mat4(1.0f), toVec3(beam) + glm::vec3(0.0f, 5.0f, 0.0f)) *
        glm::rotate(glm::mat4(1.0f), static_cast<float>(track_.startHeading()), glm::vec3(0.0f, 1.0f, 0.0f));
    scene.add(cube_,
              boxAt(beamFrame, glm::vec3(0.0f), {2.0f * static_cast<float>(track_.halfWidth()) + 4.3f, 0.6f, 0.3f}),
              kKerbRed);

    // Car: body, cabin, lights and four wheels.
    const physics::RigidBody& body = car_->chassis();
    const glm::mat4 frame = bodyMatrix(body);
    scene.add(cube_, boxAt(frame, {0.0f, -0.07f, 0.0f}, {1.76f, 0.52f, 4.30f}), kBodyColor);
    scene.add(cube_, boxAt(frame, {0.0f, 0.40f, 0.25f}, {1.45f, 0.44f, 2.0f}), kCabinColor);
    for (const float side : {-1.0f, 1.0f}) {
        scene.add(cube_, boxAt(frame, {side * 0.60f, -0.02f, -2.16f}, {0.32f, 0.12f, 0.04f}), kHeadlightColor);
        scene.add(cube_, boxAt(frame, {side * 0.62f, 0.00f, 2.16f}, {0.30f, 0.10f, 0.04f}), kTaillightColor);
    }

    const CarParams& params = car_->params();
    const float radius = static_cast<float>(params.wheelRadius);
    const float halfWidth = static_cast<float>(params.wheelHalfWidth);
    const glm::mat4 chassisRotation = glm::mat4_cast(glm::quat(body.orientation));
    for (const WheelState& wheel : car_->wheels()) {
        const glm::mat4 wheelFrame =
            glm::translate(glm::mat4(1.0f), toVec3(wheel.wheelCentre)) * chassisRotation *
            glm::rotate(glm::mat4(1.0f), static_cast<float>(wheel.steerAngle), glm::vec3(0.0f, 1.0f, 0.0f)) *
            glm::rotate(glm::mat4(1.0f), -static_cast<float>(wheel.spinAngle), glm::vec3(1.0f, 0.0f, 0.0f));
        const glm::mat4 tyre = wheelFrame * glm::rotate(glm::mat4(1.0f), 1.5707963f, glm::vec3(0.0f, 0.0f, 1.0f)) *
                               glm::scale(glm::mat4(1.0f), glm::vec3(radius, halfWidth, radius));
        scene.add(cylinder_, tyre, kTyreColor);
        const float outer = wheel.left ? -(halfWidth + 0.01f) : (halfWidth + 0.01f);
        scene.add(cube_, boxAt(wheelFrame, {outer, 0.0f, 0.0f}, {0.02f, 0.40f, 0.08f}), kHubColor);
    }

    updateCamera(aspect, frameDelta, scene);
}

void DriveScene::updateCamera(float aspect, double frameDelta, render::Scene& scene) {
    const physics::RigidBody& body = car_->chassis();
    const glm::vec3 position = toVec3(body.position);
    const glm::vec3 forward = toVec3(body.localToWorldDirection(Vec3(0.0, 0.0, -1.0)));
    glm::vec3 flatForward(forward.x, 0.0f, forward.z);
    flatForward = glm::length(flatForward) > 1e-3f ? glm::normalize(flatForward) : glm::vec3(0.0f, 0.0f, -1.0f);

    render::Camera& camera = scene.camera();
    if (cameraMode_ == DriveCamera::Cockpit) {
        camera.setPerspective(70.0f, aspect, 0.05f, 3000.0f);
        const glm::vec3 eye = toVec3(body.localToWorldPoint(Vec3(-0.35, 0.45, 0.10)));
        const glm::vec3 target = toVec3(body.localToWorldPoint(Vec3(-0.35, 0.20, -10.0)));
        camera.lookAt(eye, target, toVec3(body.localToWorldDirection(Vec3(0.0, 1.0, 0.0))));
        cameraSnap_ = true; // re-snap the chase camera when switching back
        return;
    }

    camera.setPerspective(60.0f, aspect, 0.1f, 3000.0f);
    if (cameraMode_ == DriveCamera::Orbit) {
        const glm::vec3 focus = position + glm::vec3(0.0f, 0.6f, 0.0f);
        const float cp = std::cos(orbitPitch_);
        const glm::vec3 offset(orbitDistance_ * cp * std::sin(orbitYaw_), orbitDistance_ * std::sin(orbitPitch_),
                               orbitDistance_ * cp * std::cos(orbitYaw_));
        camera.lookAt(focus + offset, focus, glm::vec3(0.0f, 1.0f, 0.0f));
        cameraSnap_ = true;
        return;
    }

    // Chase: exponential easing of eye and target towards a point behind and
    // above the car, so the view follows smoothly through corners.
    const glm::vec3 desiredEye = position - flatForward * 6.5f + glm::vec3(0.0f, 2.2f, 0.0f);
    const glm::vec3 desiredTarget = position + flatForward * 2.0f + glm::vec3(0.0f, 0.9f, 0.0f);
    if (cameraSnap_) {
        chaseEye_ = desiredEye;
        chaseTarget_ = desiredTarget;
        cameraSnap_ = false;
    } else {
        const float delta = static_cast<float>(std::clamp(frameDelta, 0.0, kMaxFrameDelta));
        const float eyeBlend = 1.0f - std::exp(-delta * 6.0f);
        const float targetBlend = 1.0f - std::exp(-delta * 12.0f);
        chaseEye_ += (desiredEye - chaseEye_) * eyeBlend;
        chaseTarget_ += (desiredTarget - chaseTarget_) * targetBlend;
    }
    const float ground = static_cast<float>(Track::terrainHeight(chaseEye_.x, chaseEye_.z)) + 0.5f;
    chaseEye_.y = std::max(chaseEye_.y, ground);
    camera.lookAt(chaseEye_, chaseTarget_, glm::vec3(0.0f, 1.0f, 0.0f));
}

void DriveScene::drawHud() const {
    const RaycastCar& car = *car_;
    ImGui::SetNextWindowPos(ImVec2(290.0f, 40.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400.0f, 0.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Drive")) {
        ImGui::End();
        return;
    }

    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 2.0f);
    ImGui::Text("%5.0f km/h   %s", car.forwardSpeed() * 3.6 * (car.gear() < 0 ? -1.0 : 1.0), gearLabel(car.gear()));
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::TextDisabled("%s", car.shifting() ? "(shifting)" : "");

    const CarParams& params = car.params();
    char rpmText[32];
    std::snprintf(rpmText, sizeof(rpmText), "%.0f rpm", car.engineRpm());
    ImGui::ProgressBar(static_cast<float>(car.engineRpm() / params.maxRpm), ImVec2(-1.0f, 0.0f), rpmText);
    ImGui::Text("Throttle");
    ImGui::SameLine(80.0f);
    ImGui::ProgressBar(static_cast<float>(car.throttle()), ImVec2(-1.0f, 0.0f), "");
    ImGui::Text("Brake");
    ImGui::SameLine(80.0f);
    ImGui::ProgressBar(static_cast<float>(car.brake()), ImVec2(-1.0f, 0.0f), car.input().handbrake ? "handbrake" : "");
    ImGui::Text("Steer %.1f deg   Surface: %s", car.steerAngle() * 57.29578, surfaceName(lastSurface_));

    ImGui::Separator();
    const double current = lapStarted_ ? world_.time() - lapStartTime_ : 0.0;
    ImGui::Text("Lap %d   current %s", laps_ + 1, formatLap(current).c_str());
    ImGui::Text("Last %s   Best %s", formatLap(lastLap_).c_str(), formatLap(bestLap_).c_str());

    ImGui::Separator();
    if (ImGui::BeginTable("wheels", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("FL");
        ImGui::TableSetupColumn("FR");
        ImGui::TableSetupColumn("RL");
        ImGui::TableSetupColumn("RR");
        ImGui::TableHeadersRow();
        const auto row = [&](const char* label, auto&& cell) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            for (std::size_t i = 0; i < car.wheels().size(); ++i) {
                ImGui::TableSetColumnIndex(static_cast<int>(i + 1));
                cell(car.wheels()[i]);
            }
        };
        row("Load N", [](const WheelState& w) { ImGui::Text("%.0f", w.load); });
        row("Slip %", [](const WheelState& w) { ImGui::Text("%.1f", w.slipRatio * 100.0); });
        row("Angle deg", [](const WheelState& w) { ImGui::Text("%.1f", w.slipAngle * 57.29578); });
        row("Fx N", [](const WheelState& w) { ImGui::Text("%.0f", w.longitudinalForce); });
        row("Fy N", [](const WheelState& w) { ImGui::Text("%.0f", w.lateralForce); });
        row("Comp mm", [](const WheelState& w) { ImGui::Text("%.0f", w.compression * 1000.0); });
        row("Surface", [](const WheelState& w) { ImGui::TextUnformatted(w.contact ? surfaceName(w.surface) : "air"); });
        ImGui::EndTable();
    }

    ImGui::Separator();
    const char* cameraNames[] = {"chase", "cockpit", "orbit"};
    ImGui::Text("Camera: %s   Sim time %.1f s", cameraNames[static_cast<int>(cameraMode_)], world_.time());
    ImGui::TextDisabled("W/Up throttle  S/Down brake/reverse  A/D steer");
    ImGui::TextDisabled("Space handbrake  R reset  C camera  G debug draw");
    ImGui::End();
}

void DriveScene::drawDebug(render::DebugDraw& draw) const {
    const glm::vec4 rayContact{0.30f, 0.95f, 0.35f, 1.0f};
    const glm::vec4 rayFree{0.55f, 0.55f, 0.55f, 1.0f};
    const glm::vec4 contactColor{1.0f, 0.95f, 0.20f, 1.0f};
    const glm::vec4 tyreColor{1.0f, 0.35f, 0.10f, 1.0f};
    const glm::vec4 loadColor{0.25f, 0.75f, 1.0f, 1.0f};
    constexpr float kForceScale = 1.0f / 2500.0f; // metres per newton

    for (const WheelState& wheel : car_->wheels()) {
        const glm::vec3 mount = toVec3(wheel.mountWorld);
        const glm::vec3 contact = toVec3(wheel.contactPoint);
        draw.addLine(mount, contact, wheel.contact ? rayContact : rayFree);
        if (!wheel.contact) {
            continue;
        }
        constexpr float s = 0.12f;
        draw.addLine(contact - glm::vec3(s, 0.0f, 0.0f), contact + glm::vec3(s, 0.0f, 0.0f), contactColor);
        draw.addLine(contact - glm::vec3(0.0f, 0.0f, s), contact + glm::vec3(0.0f, 0.0f, s), contactColor);
        draw.addLine(contact, contact + toVec3(wheel.tyreForce) * kForceScale, tyreColor);
        draw.addLine(contact, contact + toVec3(wheel.contactNormal) * static_cast<float>(wheel.load) * kForceScale,
                     loadColor);
    }

    // Velocity of the centre of mass.
    const physics::RigidBody& body = car_->chassis();
    draw.addLine(toVec3(body.position), toVec3(body.position + body.linearVelocity * 0.2), glm::vec4(1.0f));
}

} // namespace sim::app::drive
