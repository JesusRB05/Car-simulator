// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/DriveScene.hpp"

#include <imgui.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "Scenes.hpp"
#include "physics/Math.hpp"
#include "render/DebugDraw.hpp"
#include "render/Mesh.hpp"
#include "render/Scene.hpp"
#include "rhi/Device.hpp"
#include "session/CarVisual.hpp"

namespace sim::app::drive {

using vehicle::VehicleCorner;
using vehicle::VehicleInput;
using vehicle::VehicleModel;

namespace {

constexpr double kTwoPi = 6.283185307179586;
constexpr float kTyreHalfWidth = 0.12f;

const glm::vec4 kGrassColor{0.24f, 0.42f, 0.20f, 1.0f};
const glm::vec4 kVergeColor{0.21f, 0.37f, 0.17f, 1.0f};
const glm::vec4 kFarGrassColor = kGrassColor;
const glm::vec4 kAsphaltColor{0.20f, 0.20f, 0.22f, 1.0f};
const glm::vec4 kCheckerDark{0.07f, 0.07f, 0.08f, 1.0f};
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

// CPU mesh builder. Triangles are wound counter-clockwise seen from outside
// (matching Mesh.hpp and the scene pipeline's back-face culling), so each
// triangle is emitted once with a single winding.
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

    void triangle(std::uint32_t a, std::uint32_t b, std::uint32_t c) { indices.insert(indices.end(), {a, b, c}); }

    void quad(std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d) {
        triangle(a, b, c);
        triangle(a, c, d);
    }

    // A near-horizontal quad, wound so its face normal points up (+Y) whatever
    // order the corners were given in. Every track surface is flat or gently
    // banked, so this keeps the single winding correct without having to reason
    // about the direction of travel versus the driver's left and right at each
    // call site.
    void quadUp(std::uint32_t a, std::uint32_t b, std::uint32_t c, std::uint32_t d) {
        const glm::vec3 normal =
            glm::cross(vertices[b].position - vertices[a].position, vertices[c].position - vertices[a].position);
        if (normal.y >= 0.0f) {
            triangle(a, b, c);
            triangle(a, c, d);
        } else {
            triangle(a, c, b);
            triangle(a, d, c);
        }
    }

    std::shared_ptr<render::Mesh> build(rhi::Device& device) const {
        return std::make_shared<render::Mesh>(device, vertices, indices);
    }
};

// Appends a transformed copy of `source` (e.g. the unit cube or cylinder) to a
// builder, so many obstacles share one upload. The normal matrix handles the
// non-uniform scaling the obstacle boxes use.
void appendMesh(MeshBuilder& out, const render::MeshData& source, const glm::mat4& transform) {
    const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(transform)));
    const auto base = static_cast<std::uint32_t>(out.vertices.size());
    for (const render::Vertex& vertex : source.vertices) {
        render::Vertex transformed = vertex;
        transformed.position = glm::vec3(transform * glm::vec4(vertex.position, 1.0f));
        transformed.normal = glm::normalize(normalMatrix * vertex.normal);
        out.vertices.push_back(transformed);
    }
    for (const std::uint32_t index : source.indices) {
        out.indices.push_back(base + index);
    }
}

glm::vec4 unpackColor(std::uint32_t rgba) {
    return glm::vec4(static_cast<float>(rgba & 0xFFu) / 255.0f, static_cast<float>((rgba >> 8) & 0xFFu) / 255.0f,
                     static_cast<float>((rgba >> 16) & 0xFFu) / 255.0f,
                     static_cast<float>((rgba >> 24) & 0xFFu) / 255.0f);
}

Vec3 onTerrain(const Track& track, const Vec3& p, double raise) {
    return Vec3(p.x, track.heightAt(p.x, p.z) + raise, p.z);
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

DriveScene::DriveScene(rhi::Device& device, const TrackDescription& description, const vehicle::VehicleSetup& car,
                       std::shared_ptr<CarVisual> visual, std::shared_ptr<const DriveTrackAssets> circuit)
    : description_(description), track_(description_), circuit_(std::move(circuit)), visual_(std::move(visual)) {
    world_.fixedStep = 1.0 / 1000.0;
    // The wheels query the imported drivable surface when the circuit ships a
    // model; otherwise they query the analytic collision mesh.
    if (circuit_ && circuit_->model) {
        surface_ = std::make_unique<TrackSurfaceQuery>(circuit_->model->collision, circuit_->model->triangleFriction);
        fallbackGround_ = std::make_unique<TrackGroundQuery>(track_);
        ground_ = std::make_unique<TrackSurfaceGroundQuery>(*surface_, *fallbackGround_);
    } else {
        trackMesh_ = std::make_unique<TrackCollisionMesh>(track_);
        ground_ = std::make_unique<TrackMeshGroundQuery>(track_, *trackMesh_);
    }
    car_ = &VehicleModel::spawn(world_, car, *ground_);
    cogHeight_ = vehicle::staticRideHeight(car);
    // Driver aids (ABS/TCS/ESC) are attached to the car here and enabled by the
    // app from the session setting (all three are on by default). The unit is
    // sampled by the model every fixed step.
    car_->setElectronics(&aids_);
    setAidsEnabled(true, true, true);
    buildMeshes(device);
    buildObstacles(device);
    reset();
}

DriveScene::~DriveScene() = default;

void DriveScene::reset() {
    Vec3 position = track_.startPosition();
    Real heading = track_.startHeading();
    if (circuit_ && circuit_->hasStart) {
        // The layout knows where the line is; reset samples the ground itself,
        // so only x and z matter here.
        position = circuit_->startPosition;
        heading = static_cast<Real>(circuit_->startHeading);
    }
    car_->reset(position, heading);
    remainder_ = 0.0;
    lapStarted_ = false;
    halfwayReached_ = false;
    lapStartTime_ = 0.0;
    previousAlong_ = track_.project(position.x, position.z).along;
    cameraSnap_ = true;
}

void DriveScene::cycleCamera() {
    cameraMode_ = cameraMode_ == DriveCamera::Chase     ? DriveCamera::Cockpit
                  : cameraMode_ == DriveCamera::Cockpit ? DriveCamera::Orbit
                  : cameraMode_ == DriveCamera::Orbit   ? DriveCamera::Side
                                                        : DriveCamera::Chase;
    cameraSnap_ = true;
}

void DriveScene::setCamera(DriveCamera mode) {
    cameraMode_ = mode;
    cameraSnap_ = true;
}

void DriveScene::setOrbit(float yaw, float pitch) {
    orbitYaw_ = yaw;
    orbitPitch_ = std::clamp(pitch, -0.2f, 1.45f);
    cameraSnap_ = true;
}

void DriveScene::setKeys(const DriveKeys& keys) {
    VehicleInput input;
    input.throttle = std::max(keys.throttle ? 1.0 : 0.0, keys.throttleAnalog);
    input.brake = std::max(keys.brake ? 1.0 : 0.0, keys.brakeAnalog);
    const double steerDigital = (keys.left ? 1.0 : 0.0) - (keys.right ? 1.0 : 0.0);
    input.steer = std::abs(keys.steerAnalog) > 0.02 ? keys.steerAnalog : steerDigital;
    input.handbrake = keys.handbrake || keys.handbrakeAnalog > 0.5;
    car_->setInput(input);
}

void DriveScene::setAidsEnabled(bool abs, bool tcs, bool esc) {
    if (abs == absEnabled_ && tcs == tcsEnabled_ && esc == escEnabled_) {
        return;
    }
    absEnabled_ = abs;
    tcsEnabled_ = tcs;
    escEnabled_ = esc;
    aids_.setEnabled(absEnabled_, tcsEnabled_, escEnabled_);
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
        // The model owns the world: stepping it advances both the chassis and
        // the static scenery in the same fixed step.
        car_->step(dt);
        updateLapTimer();
        remainder_ -= dt;
    }
}

void DriveScene::simulate(double seconds) {
    const int steps = static_cast<int>(seconds / world_.fixedStep + 0.5);
    for (int i = 0; i < steps; ++i) {
        car_->step(world_.fixedStep);
        updateLapTimer();
    }
}

void DriveScene::autopilot(double seconds) {
    const int steps = static_cast<int>(seconds / world_.fixedStep + 0.5);
    for (int i = 0; i < steps; ++i) {
        car_->setInput(autopilotInput());
        car_->step(world_.fixedStep);
        updateLapTimer();
    }
}

// Pure-pursuit driver: aim at a point a speed-dependent distance ahead on the
// centreline and pick a target speed from the curvature it is about to meet.
// Good enough for a capture, deterministic (no wall clock) and it never needs a
// human at the keyboard.
vehicle::VehicleInput DriveScene::autopilotInput() const {
    vehicle::VehicleInput input;

    const Vec3& position = car_->chassis().position;
    const Track::Projection projection = track_.project(position.x, position.z);
    if (!projection.found) {
        input.brake = 0.4; // lost the circuit: slow down and let it rejoin
        return input;
    }

    const std::vector<TrackSample>& samples = track_.samples();
    const double spacing = std::max(0.1, static_cast<double>(track_.description().sampleSpacing));
    const double speed = std::abs(car_->forwardSpeed());

    // Look further ahead the faster we go, so the steering stays calm.
    const double lookahead = std::clamp(4.0 + 0.9 * speed, 8.0, 40.0);
    const std::size_t ahead = static_cast<std::size_t>(std::max(1.0, lookahead / spacing));

    // Steer towards the look-ahead point. `steer` is + to the left, so a target
    // to the car's left must give a positive angle.
    const Vec3 forwardDir = car_->chassis().localToWorldDirection(Vec3(0.0, 0.0, -1.0));
    glm::vec3 forward(static_cast<float>(forwardDir.x), 0.0f, static_cast<float>(forwardDir.z));
    forward = glm::length(forward) > 1e-3f ? glm::normalize(forward) : glm::vec3(0.0f, 0.0f, -1.0f);
    const TrackSample& aim = samples[(projection.segment + ahead) % samples.size()];
    const glm::vec3 toTarget(static_cast<float>(aim.position.x - position.x), 0.0f,
                             static_cast<float>(aim.position.z - position.z));
    const double cross = static_cast<double>(forward.z) * toTarget.x - static_cast<double>(forward.x) * toTarget.z;
    const double dot = static_cast<double>(forward.x) * toTarget.x + static_cast<double>(forward.z) * toTarget.z;
    input.steer = std::clamp(std::atan2(cross, dot) * 2.0, -1.0, 1.0);

    // Target speed from the tightest curvature on the way to the aim point.
    double maxCurvature = 0.0;
    for (std::size_t i = 0; i <= ahead; ++i) {
        maxCurvature = std::max(maxCurvature, std::abs(samples[(projection.segment + i) % samples.size()].curvature));
    }
    constexpr double kLateralLimit = 7.0; // m/s^2 the dry tyres can hold
    double targetSpeed = 45.0;
    if (maxCurvature > 1e-4) {
        targetSpeed = std::min(targetSpeed, std::sqrt(kLateralLimit / maxCurvature));
    }
    targetSpeed = std::max(targetSpeed, 8.0);

    const double error = targetSpeed - speed;
    if (error > 0.2) {
        input.throttle = std::clamp(error / 4.0, 0.0, 1.0);
    } else if (error < -0.2) {
        input.brake = std::clamp(-error / 4.0, 0.0, 1.0);
    }
    return input;
}

void DriveScene::updateLapTimer() {
    const Vec3& position = car_->chassis().position;
    const Track::Projection projection = track_.project(position.x, position.z);

    // Surface for the HUD, derived from the projection we already have instead of
    // a second full track sample (which would re-project and re-evaluate the
    // terrain) every physics step.
    lastSurface_ = Surface::Grass;
    if (projection.found) {
        const TrackSample& sample = track_.samples()[projection.segment];
        const double offset = std::abs(projection.lateral);
        if (offset <= sample.halfWidth) {
            lastSurface_ = Surface::Asphalt;
        } else if (offset <= sample.halfWidth + track_.kerbWidth() && sample.kerb) {
            lastSurface_ = Surface::Kerb;
        }
    }

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
    // Shared unit primitives (cube +-0.5, cylinder radius 1 / half-height 1 on +Y)
    // for the car and the gantry. The flat terrain/track meshes below are built
    // single-sided.
    cube_ = render::Mesh::makeCube(device);
    cylinder_ = render::Mesh::makeCylinder(device);

    // An imported circuit brings its own road, kerbs, run-off, terrain and
    // scenery, so the procedural meshes are skipped; only the horizon plane is
    // kept, dropped below the model so it never pokes through.
    const bool imported = circuit_ && circuit_->visual;
    const double farGroundY = imported ? static_cast<double>(circuit_->model->boundsMin.y) - 1.0 : -0.10;
    {
        MeshBuilder builder;
        constexpr double kHalf = 2500.0;
        const Vec3 up(0.0, 1.0, 0.0);
        builder.quadUp(
            builder.vertex(Vec3(-kHalf, farGroundY, -kHalf), up), builder.vertex(Vec3(kHalf, farGroundY, -kHalf), up),
            builder.vertex(Vec3(kHalf, farGroundY, kHalf), up), builder.vertex(Vec3(-kHalf, farGroundY, kHalf), up));
        farGround_ = builder.build(device);
    }
    if (imported) {
        return;
    }

    const std::vector<TrackSample>& samples = track_.samples();
    const std::size_t count = samples.size();
    const double kerbWidth = track_.kerbWidth();

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
                builder.vertex(Vec3(x, track_.heightAt(x, z) - kGrassDrop, z), track_.normalAt(x, z));
            }
        }
        const auto index = [&](int column, int row) {
            return static_cast<std::uint32_t>(row * (columns + 1) + column);
        };
        for (int row = 0; row < rows; ++row) {
            for (int column = 0; column < columns; ++column) {
                builder.quadUp(index(column, row), index(column + 1, row), index(column + 1, row + 1),
                               index(column, row + 1));
            }
        }
        terrain_ = builder.build(device);
    }

    // Asphalt ribbon: two vertices per centreline sample, following the
    // per-sample width and banking that Track also feeds to the wheels.
    {
        MeshBuilder builder;
        for (const TrackSample& sample : samples) {
            const Vec3 left = track_.roadPoint(sample, -sample.halfWidth, 0.0);
            const Vec3 right = track_.roadPoint(sample, sample.halfWidth, 0.0);
            builder.vertex(left, track_.roadNormal(sample, -sample.halfWidth));
            builder.vertex(right, track_.roadNormal(sample, sample.halfWidth));
        }
        for (std::size_t i = 0; i < count; ++i) {
            const std::uint32_t a = static_cast<std::uint32_t>(2 * i);
            const std::uint32_t b = static_cast<std::uint32_t>(2 * ((i + 1) % count));
            builder.quadUp(a, a + 1, b + 1, b);
        }
        asphalt_ = builder.build(device);
    }

    // Grass verge: a strip from the asphalt edge down to the terrain, so the
    // raised side of a banked corner never shows a step.
    {
        MeshBuilder builder;
        constexpr double kVergeWidth = 3.0;
        for (const double side : {-1.0, 1.0}) {
            const auto base = static_cast<std::uint32_t>(builder.vertices.size());
            for (const TrackSample& sample : samples) {
                const Vec3 edge = track_.roadPoint(sample, side * sample.halfWidth, 0.0);
                const Vec3 outer = sample.position + sample.right * (side * (sample.halfWidth + kVergeWidth));
                builder.vertex(edge, track_.roadNormal(sample, side * sample.halfWidth));
                builder.vertex(Vec3(outer.x, track_.heightAt(outer.x, outer.z), outer.z),
                               track_.normalAt(outer.x, outer.z));
            }
            for (std::size_t i = 0; i < count; ++i) {
                const std::uint32_t a = base + static_cast<std::uint32_t>(2 * i);
                const std::uint32_t b = base + static_cast<std::uint32_t>(2 * ((i + 1) % count));
                builder.quadUp(a, a + 1, b + 1, b);
            }
        }
        verge_ = builder.build(device);
    }

    // White edge lines just inside both asphalt edges.
    {
        MeshBuilder builder;
        constexpr double kInset = 0.30;
        constexpr double kLineWidth = 0.15;
        for (const double side : {-1.0, 1.0}) {
            const auto base = static_cast<std::uint32_t>(builder.vertices.size());
            for (const TrackSample& sample : samples) {
                const double outer = side * (sample.halfWidth - kInset);
                const double inner = side * (sample.halfWidth - kInset - kLineWidth);
                builder.vertex(track_.roadPoint(sample, inner, kLineRaise), track_.roadNormal(sample, inner));
                builder.vertex(track_.roadPoint(sample, outer, kLineRaise), track_.roadNormal(sample, outer));
            }
            for (std::size_t i = 0; i < count; ++i) {
                const std::uint32_t a = base + static_cast<std::uint32_t>(2 * i);
                const std::uint32_t b = base + static_cast<std::uint32_t>(2 * ((i + 1) % count));
                builder.quadUp(a, a + 1, b + 1, b);
            }
        }
        edgeLines_ = builder.build(device);
    }

    // Kerbs: red/white blocks on both edges where the corner is tight.
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
                const Vec3 a0 = track_.roadPoint(a, side * a.halfWidth, kKerbRaise);
                const Vec3 a1 = track_.roadPoint(a, side * (a.halfWidth + kerbWidth), kKerbRaise);
                const Vec3 b0 = track_.roadPoint(b, side * b.halfWidth, kKerbRaise);
                const Vec3 b1 = track_.roadPoint(b, side * (b.halfWidth + kerbWidth), kKerbRaise);
                const Vec3 n = track_.roadNormal(a, side * a.halfWidth);
                builder.quadUp(builder.vertex(a0, n), builder.vertex(a1, n), builder.vertex(b1, n),
                               builder.vertex(b0, n));
            }
        }
        kerbRed_ = red.build(device);
        kerbWhite_ = white.build(device);
    }

    // Start/finish line: a black-and-white chequer across the asphalt.
    {
        MeshBuilder light;
        MeshBuilder dark;
        const TrackSample& start = samples[track_.startIndex()];
        constexpr int kAcross = 10;
        constexpr int kAlong = 2;
        const double tileWidth = 2.0 * start.halfWidth / kAcross;
        const double tileLength = 0.6 / kAlong;
        const Vec3 n = track_.roadNormal(start, 0.0);
        const auto point = [&](double lateral, double along) {
            const Vec3 p = track_.roadPoint(start, lateral, kLineRaise);
            return Vec3(p.x + start.tangent.x * along, p.y, p.z + start.tangent.z * along);
        };
        for (int row = 0; row < kAlong; ++row) {
            for (int column = 0; column < kAcross; ++column) {
                const double l0 = -start.halfWidth + tileWidth * column;
                const double l1 = l0 + tileWidth;
                const double t0 = (row - 0.5 * kAlong) * tileLength;
                const double t1 = t0 + tileLength;
                MeshBuilder& builder = ((row + column) % 2 == 0) ? light : dark;
                builder.quadUp(builder.vertex(point(l0, t0), n), builder.vertex(point(l1, t0), n),
                               builder.vertex(point(l1, t1), n), builder.vertex(point(l0, t1), n));
            }
        }
        startLine_ = light.build(device);
        startLineDark_ = dark.build(device);
    }
}

void DriveScene::buildObstacles(rhi::Device& device) {
    // The imported circuit ships its own barriers and scenery as geometry, and
    // their analytic placeholders would sit at the wrong elevation.
    if (circuit_ && circuit_->visual) {
        return;
    }
    obstacles_ = buildObstacleLayout(track_);
    addObstaclesToWorld(world_, obstacles_);

    const render::MeshData unitCube = render::cubeData();
    const render::MeshData unitCylinder = render::cylinderData();

    // Merge into one mesh per colour: a barrier wall around a 1.2 km circuit is
    // hundreds of blocks, and one draw call per colour keeps it cheap.
    std::vector<std::uint32_t> colors;
    std::vector<MeshBuilder> builders;
    const auto builderFor = [&](std::uint32_t rgba) -> MeshBuilder& {
        for (std::size_t i = 0; i < colors.size(); ++i) {
            if (colors[i] == rgba) {
                return builders[i];
            }
        }
        colors.push_back(rgba);
        builders.emplace_back();
        return builders.back();
    };

    for (const Obstacle& obstacle : obstacles_) {
        const glm::mat4 frame = glm::translate(glm::mat4(1.0f), toVec3(obstacle.position)) *
                                glm::mat4_cast(glm::quat(obstacle.orientation));
        MeshBuilder& builder = builderFor(obstacle.rgba);
        if (obstacle.shape == physics::ShapeType::Cylinder) {
            const glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(obstacle.radius),
                                                                          static_cast<float>(obstacle.halfHeight),
                                                                          static_cast<float>(obstacle.radius)));
            appendMesh(builder, unitCylinder, frame * scale);
        } else {
            appendMesh(builder, unitCube, frame * glm::scale(glm::mat4(1.0f), toVec3(2.0 * obstacle.halfExtents)));
        }
    }

    obstacleBatches_.clear();
    for (std::size_t i = 0; i < builders.size(); ++i) {
        obstacleBatches_.push_back(ObstacleBatch{builders[i].build(device), unpackColor(colors[i])});
    }
}

void DriveScene::buildScene(render::Scene& scene, float aspect, double frameDelta) {
    scene.clear();
    setDefaultLight(scene);
    scene.add(farGround_, glm::mat4(1.0f), kFarGrassColor);
    if (circuit_ && circuit_->visual) {
        // The content's circuit: road, kerbs, run-off, terrain, barriers,
        // grandstands and trees, all from its glTF.
        addTrackVisual(scene, *circuit_->visual);
    } else {
        scene.add(terrain_, glm::mat4(1.0f), kGrassColor);
        scene.add(verge_, glm::mat4(1.0f), kVergeColor);
        scene.add(asphalt_, glm::mat4(1.0f), kAsphaltColor);
        scene.add(edgeLines_, glm::mat4(1.0f), kLineWhite);
        scene.add(kerbRed_, glm::mat4(1.0f), kKerbRed);
        scene.add(kerbWhite_, glm::mat4(1.0f), kKerbWhite);
        scene.add(startLine_, glm::mat4(1.0f), kLineWhite);
        scene.add(startLineDark_, glm::mat4(1.0f), kCheckerDark);

        // Static collision scenery (barrier walls + the run-off test yard).
        for (const ObstacleBatch& batch : obstacleBatches_) {
            scene.add(batch.mesh, glm::mat4(1.0f), batch.color);
        }

        // Start gantry posts, so the line reads from a distance.
        const TrackSample& start = track_.samples()[track_.startIndex()];
        for (const double side : {-1.0, 1.0}) {
            const Vec3 base =
                onTerrain(track_, start.position + start.right * (side * (track_.halfWidth() + 2.0)), 0.0);
            scene.add(cube_, boxAt(glm::mat4(1.0f), toVec3(base) + glm::vec3(0.0f, 2.5f, 0.0f), {0.3f, 5.0f, 0.3f}),
                      kLineWhite);
        }
        const Vec3 beam = onTerrain(track_, start.position, 0.0);
        const glm::mat4 beamFrame =
            glm::translate(glm::mat4(1.0f), toVec3(beam) + glm::vec3(0.0f, 5.0f, 0.0f)) *
            glm::rotate(glm::mat4(1.0f), static_cast<float>(track_.startHeading()), glm::vec3(0.0f, 1.0f, 0.0f));
        scene.add(cube_,
                  boxAt(beamFrame, glm::vec3(0.0f), {2.0f * static_cast<float>(track_.halfWidth()) + 4.3f, 0.6f, 0.3f}),
                  kKerbRed);
    }

    // Car: the content model when it ships one, the primitive stand-in
    // otherwise.
    const physics::RigidBody& body = car_->chassis();
    const glm::mat4 frame = bodyMatrix(body);
    if (visual_ && visual_->valid()) {
        // The model's origin is on the ground; the physics origin is the centre
        // of mass, so drop the model by the resting ride height.
        const glm::mat4 chassis = glm::translate(frame, glm::vec3(0.0f, -static_cast<float>(cogHeight_), 0.0f));
        const glm::mat4 rotation = glm::mat4_cast(glm::quat(body.orientation));
        std::array<glm::mat4, 4> hubs;
        std::array<float, 4> spin{};
        for (std::size_t i = 0; i < hubs.size(); ++i) {
            const VehicleCorner& corner = car_->corners()[i];
            // The physical hub: where the contact patch puts the wheel, the
            // body's attitude and the steer angle, times the model's own pivot
            // orientation.
            hubs[i] = glm::translate(glm::mat4(1.0f), toVec3(corner.wheelCentre)) * rotation *
                      glm::rotate(glm::mat4(1.0f), static_cast<float>(corner.steerAngle), glm::vec3(0.0f, 1.0f, 0.0f)) *
                      glm::mat4(visual_->wheels[i].rotation);
            spin[i] = static_cast<float>(corner.spinAngle);
        }
        addCarVisual(scene, *visual_, chassis, hubs, spin);
    } else {
        scene.add(cube_, boxAt(frame, {0.0f, -0.07f, 0.0f}, {1.76f, 0.52f, 4.30f}), kBodyColor);
        scene.add(cube_, boxAt(frame, {0.0f, 0.40f, 0.25f}, {1.45f, 0.44f, 2.0f}), kCabinColor);
        for (const float side : {-1.0f, 1.0f}) {
            scene.add(cube_, boxAt(frame, {side * 0.60f, -0.02f, -2.16f}, {0.32f, 0.12f, 0.04f}), kHeadlightColor);
            scene.add(cube_, boxAt(frame, {side * 0.62f, 0.00f, 2.16f}, {0.30f, 0.10f, 0.04f}), kTaillightColor);
        }

        const glm::mat4 chassisRotation = glm::mat4_cast(glm::quat(body.orientation));
        for (const VehicleCorner& corner : car_->corners()) {
            const float radius = static_cast<float>(corner.wheelRadius);
            const glm::mat4 wheelFrame =
                glm::translate(glm::mat4(1.0f), toVec3(corner.wheelCentre)) * chassisRotation *
                glm::rotate(glm::mat4(1.0f), static_cast<float>(corner.steerAngle), glm::vec3(0.0f, 1.0f, 0.0f)) *
                glm::rotate(glm::mat4(1.0f), -static_cast<float>(corner.spinAngle), glm::vec3(1.0f, 0.0f, 0.0f));
            const glm::mat4 tyre = wheelFrame * glm::rotate(glm::mat4(1.0f), 1.5707963f, glm::vec3(0.0f, 0.0f, 1.0f)) *
                                   glm::scale(glm::mat4(1.0f), glm::vec3(radius, kTyreHalfWidth, radius));
            scene.add(cylinder_, tyre, kTyreColor);
            const float outer = corner.left ? -(kTyreHalfWidth + 0.01f) : (kTyreHalfWidth + 0.01f);
            scene.add(cube_, boxAt(wheelFrame, {outer, 0.0f, 0.0f}, {0.02f, 0.40f, 0.08f}), kHubColor);
        }
    }

    // Cars standing on the grid, drawn with the same model at rest. They have no
    // physics: this fills a screenshot or a frame-time measurement.
    if (visual_ && visual_->valid()) {
        for (const ParkedCar& parked : parkedCars_) {
            const vehicle::GroundSample ground = ground_->sample(parked.position.x, parked.position.z);
            const glm::mat4 chassis =
                glm::translate(glm::mat4(1.0f),
                               glm::vec3(static_cast<float>(parked.position.x), static_cast<float>(ground.height),
                                         static_cast<float>(parked.position.z))) *
                glm::rotate(glm::mat4(1.0f), static_cast<float>(parked.headingRadians), glm::vec3(0.0f, 1.0f, 0.0f));
            addCarVisual(scene, *visual_, chassis, restHubFrames(*visual_, chassis), std::array<float, 4>{});
        }
    }

    updateCamera(aspect, frameDelta, scene);
}

void DriveScene::setParkedCars(std::vector<ParkedCar> cars) {
    parkedCars_ = std::move(cars);
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
        const glm::vec3 eye = toVec3(body.localToWorldPoint(Vec3(-0.35, 0.50, 0.05)));
        const glm::vec3 target = toVec3(body.localToWorldPoint(Vec3(-0.35, 0.32, -14.0)));
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

    if (cameraMode_ == DriveCamera::Side) {
        // Broadside tracking shot: off the driver's right, slightly behind and
        // above, so the car's attitude through a corner reads clearly.
        const glm::vec3 right = glm::normalize(glm::cross(flatForward, glm::vec3(0.0f, 1.0f, 0.0f)));
        const glm::vec3 eye = position + right * 7.5f + glm::vec3(0.0f, 2.0f, 0.0f) - flatForward * 1.5f;
        const glm::vec3 target = position + glm::vec3(0.0f, 0.7f, 0.0f);
        camera.lookAt(eye, target, glm::vec3(0.0f, 1.0f, 0.0f));
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
    const float ground = static_cast<float>(track_.heightAt(chaseEye_.x, chaseEye_.z)) + 0.5f;
    chaseEye_.y = std::max(chaseEye_.y, ground);
    camera.lookAt(chaseEye_, chaseTarget_, glm::vec3(0.0f, 1.0f, 0.0f));
}

void DriveScene::drawHud() {
    const VehicleModel& car = *car_;
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

    const double rpm = car.engineSpeed() * 60.0 / kTwoPi;
    const double maxRpm = car.setup().engine.config.revLimit * 60.0 / kTwoPi;
    char rpmText[32];
    std::snprintf(rpmText, sizeof(rpmText), "%.0f rpm", rpm);
    ImGui::ProgressBar(static_cast<float>(rpm / maxRpm), ImVec2(-1.0f, 0.0f), rpmText);
    ImGui::Text("Throttle");
    ImGui::SameLine(80.0f);
    ImGui::ProgressBar(static_cast<float>(car.throttle()), ImVec2(-1.0f, 0.0f), "");
    ImGui::Text("Brake");
    ImGui::SameLine(80.0f);
    ImGui::ProgressBar(static_cast<float>(car.brake()), ImVec2(-1.0f, 0.0f), car.input().handbrake ? "handbrake" : "");
    ImGui::Text("Steer %.1f deg   Surface: %s", car.steerAngle() * 57.29578, surfaceName(lastSurface_));

    // Driver aids: the ElectronicControlUnit attached to the car, sampled every
    // fixed step. The switches live in the main menu / Vehicle panel; this is
    // the live state (green when the aid is intervening right now).
    const auto aidStatus = [](bool enabled, bool active, const char* name) {
        const char* state = !enabled ? "off" : (active ? "active" : "on");
        const ImVec4 color = !enabled ? ImVec4(0.55f, 0.55f, 0.55f, 1.0f)
                             : active ? ImVec4(0.40f, 0.90f, 0.45f, 1.0f)
                                      : ImVec4(0.75f, 0.75f, 0.75f, 1.0f);
        ImGui::TextColored(color, "%s %s", name, state);
    };
    aidStatus(absEnabled_, aids_.absActive(), "ABS");
    ImGui::SameLine();
    aidStatus(tcsEnabled_, aids_.tcsActive(), "TCS");
    ImGui::SameLine();
    aidStatus(escEnabled_, aids_.escActive(), "ESC");
    ImGui::SameLine();
    ImGui::TextDisabled("throttle x%.2f", aids_.throttleScale());

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
            for (std::size_t i = 0; i < car.corners().size(); ++i) {
                ImGui::TableSetColumnIndex(static_cast<int>(i + 1));
                cell(car.corners()[i]);
            }
        };
        row("Load N", [](const VehicleCorner& c) { ImGui::Text("%.0f", c.load); });
        row("Susp mm", [](const VehicleCorner& c) { ImGui::Text("%+.0f", c.compression * 1000.0); });
        row("Slip", [](const VehicleCorner& c) { ImGui::Text("%+.3f", c.slipRatio); });
        row("Slip deg", [](const VehicleCorner& c) { ImGui::Text("%+.1f", c.slipAngle * 57.29578); });
        row("Fx N", [](const VehicleCorner& c) { ImGui::Text("%+.0f", c.longitudinalForce); });
        row("Fy N", [](const VehicleCorner& c) { ImGui::Text("%+.0f", c.lateralForce); });
        row("Drive Nm", [](const VehicleCorner& c) { ImGui::Text("%+.0f", c.driveTorque); });
        row("Brake Nm", [](const VehicleCorner& c) { ImGui::Text("%+.0f", c.brakeTorque); });
        {
            // The aids' brake-torque adjustment is indexed by the same wheel
            // order as the corners, so it needs the index, not the corner.
            const auto& modulation = car.brakeModulation().torqueDelta;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Aids Nm");
            for (std::size_t i = 0; i < car.corners().size(); ++i) {
                ImGui::TableSetColumnIndex(static_cast<int>(i + 1));
                ImGui::Text("%+.0f", modulation[i]);
            }
        }
        row("Surface", [this](const VehicleCorner& c) {
            ImGui::TextUnformatted(c.contact ? surfaceName(track_.sample(c.contactPoint.x, c.contactPoint.z).surface)
                                             : "air");
        });
        ImGui::EndTable();
    }

    ImGui::Separator();
    const char* cameraNames[] = {"chase", "cockpit", "orbit", "side"};
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

    for (const VehicleCorner& wheel : car_->corners()) {
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
