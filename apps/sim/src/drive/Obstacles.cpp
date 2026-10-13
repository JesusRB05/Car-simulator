// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/Obstacles.hpp"

#include <algorithm>
#include <cmath>

namespace sim::app::drive {
namespace {

// Straight (non-premultiplied) RGBA packed little-endian, matching the colour
// constants the scene unpacks for the shader.
constexpr std::uint32_t packRgba(int r, int g, int b, int a) {
    return static_cast<std::uint32_t>(r) | (static_cast<std::uint32_t>(g) << 8) |
           (static_cast<std::uint32_t>(b) << 16) | (static_cast<std::uint32_t>(a) << 24);
}

constexpr std::uint32_t kBarrierRed = packRgba(200, 30, 26, 255);
constexpr std::uint32_t kBarrierWhite = packRgba(236, 236, 240, 255);
constexpr std::uint32_t kConcrete = packRgba(150, 150, 146, 255);
constexpr std::uint32_t kTyre = packRgba(28, 28, 30, 255);
constexpr std::uint32_t kCone = packRgba(240, 118, 20, 255);

// Yaw about +Y that turns a body's local -Z onto the given horizontal tangent.
physics::Quat yawToTangent(const Vec3& tangent) {
    const Real yaw = std::atan2(-tangent.x, -tangent.z);
    return physics::quaternionFromAxisAngle(Vec3(0.0, 1.0, 0.0), yaw);
}

Real groundY(Real x, Real z, Real halfHeight) {
    return Track::terrainHeight(x, z) + halfHeight;
}

} // namespace

std::vector<Obstacle> buildObstacleLayout(const Track& track) {
    std::vector<Obstacle> obstacles;
    const std::vector<TrackSample>& samples = track.samples();
    if (samples.empty()) {
        return obstacles;
    }

    // --- Barrier walls lining both sides of the circuit --------------------
    // Roughly one block every four metres; tighter corners get taller concrete
    // blocks, straights get the thin red/white armco.
    const Real spacing = track.length() / static_cast<Real>(samples.size());
    const int stride = std::max(1, static_cast<int>(std::lround(4.0 / spacing)));
    int blockIndex = 0;
    for (std::size_t i = 0; i < samples.size(); i += static_cast<std::size_t>(stride)) {
        const TrackSample& sample = samples[i];
        const bool corner = std::abs(sample.curvature) > 0.01;
        const Real lateral = track.halfWidth() + track.kerbWidth() + (corner ? 3.0 : 2.0);
        const physics::Quat orientation = yawToTangent(sample.tangent);
        for (const Real side : {-1.0, 1.0}) {
            const Vec3 base = sample.position + sample.right * (side * lateral);
            Obstacle obstacle;
            obstacle.orientation = orientation;
            if (corner) {
                obstacle.kind = "block";
                obstacle.halfExtents = Vec3(0.35, 0.55, 0.9);
                obstacle.rgba = kConcrete;
            } else {
                obstacle.kind = "barrier";
                obstacle.halfExtents = Vec3(0.12, 0.45, 1.8);
                obstacle.rgba = (blockIndex % 2 == 0) ? kBarrierRed : kBarrierWhite;
            }
            obstacle.position = Vec3(base.x, groundY(base.x, base.z, obstacle.halfExtents.y), base.z);
            obstacles.push_back(obstacle);
            ++blockIndex;
        }
    }

    // --- Run-off collision test yard beside the start straight ------------
    // Anchor it well clear of the centreline (and any other part of the loop),
    // on the grass to the driver's right of the start line.
    const TrackSample& start = samples[track.startIndex()];
    const Vec3 right = start.right;
    const Vec3 forward = start.tangent;
    Vec3 anchor = start.position + right * 22.0;
    for (Real offset = 22.0; offset <= 50.0; offset += 6.0) {
        const Vec3 candidate = start.position + right * offset;
        const Track::Projection projection = track.project(candidate.x, candidate.z);
        const bool farFromTrack = !projection.found || std::abs(projection.lateral) > 16.0;
        if (farFromTrack && track.sample(candidate.x, candidate.z).surface == Surface::Grass) {
            anchor = candidate;
            break;
        }
    }
    const physics::Quat yardYaw = yawToTangent(forward);
    const auto at = [&](Real lateral, Real along) { return anchor + right * lateral + forward * along; };

    // A concrete wall with a gap to drive through.
    for (const Real lateral : {-4.0, -2.0, 2.0, 4.0}) {
        Obstacle block;
        block.kind = "block";
        block.halfExtents = Vec3(0.6, 0.6, 0.6);
        block.rgba = kConcrete;
        block.orientation = yardYaw;
        const Vec3 p = at(lateral, -5.0);
        block.position = Vec3(p.x, groundY(p.x, p.z, block.halfExtents.y), p.z);
        obstacles.push_back(block);
    }

    // Two tall barrier posts framing the approach.
    for (const Real lateral : {-5.0, 5.0}) {
        Obstacle post;
        post.kind = "barrier";
        post.halfExtents = Vec3(0.3, 0.9, 0.3);
        post.rgba = kConcrete;
        post.orientation = yardYaw;
        const Vec3 p = at(lateral, -9.0);
        post.position = Vec3(p.x, groundY(p.x, p.z, post.halfExtents.y), p.z);
        obstacles.push_back(post);
    }

    // Tyre stacks: single and double height.
    {
        const Real stackLateral[4] = {-5.0, -4.2, 4.2, 5.0};
        const Real stackHeight[4] = {0.35, 0.7, 0.7, 0.35};
        for (int i = 0; i < 4; ++i) {
            Obstacle tyre;
            tyre.kind = "tyre";
            tyre.shape = physics::ShapeType::Cylinder;
            tyre.radius = 0.35;
            tyre.halfHeight = stackHeight[i];
            tyre.rgba = kTyre;
            const Vec3 p = at(stackLateral[i], -13.0);
            tyre.position = Vec3(p.x, groundY(p.x, p.z, tyre.halfHeight), p.z);
            obstacles.push_back(tyre);
        }
    }

    // A cone slalom down the middle.
    for (int i = 0; i < 5; ++i) {
        Obstacle cone;
        cone.kind = "cone";
        cone.shape = physics::ShapeType::Cylinder;
        cone.radius = 0.15;
        cone.halfHeight = 0.28;
        cone.rgba = kCone;
        const Vec3 p = at((i % 2 == 0) ? -1.0 : 1.0, 2.0 - 3.0 * static_cast<Real>(i));
        cone.position = Vec3(p.x, groundY(p.x, p.z, cone.halfHeight), p.z);
        obstacles.push_back(cone);
    }

    // A tilted ramp (pitch about the yard's lateral axis).
    {
        Obstacle ramp;
        ramp.kind = "ramp";
        ramp.halfExtents = Vec3(1.6, 0.12, 1.2);
        ramp.rgba = kConcrete;
        ramp.orientation =
            physics::quaternionMultiply(yardYaw, physics::quaternionFromAxisAngle(Vec3(1.0, 0.0, 0.0), 0.2));
        const Vec3 p = at(8.0, -6.0);
        ramp.position = Vec3(p.x, Track::terrainHeight(p.x, p.z) + 0.4, p.z);
        obstacles.push_back(ramp);
    }

    // A yaw-rotated deflector barrier.
    {
        Obstacle deflector;
        deflector.kind = "barrier";
        deflector.halfExtents = Vec3(0.25, 0.5, 1.5);
        deflector.rgba = kBarrierRed;
        deflector.orientation =
            physics::quaternionMultiply(yardYaw, physics::quaternionFromAxisAngle(Vec3(0.0, 1.0, 0.0), 0.6));
        const Vec3 p = at(7.0, -19.0);
        deflector.position = Vec3(p.x, groundY(p.x, p.z, deflector.halfExtents.y), p.z);
        obstacles.push_back(deflector);
    }

    return obstacles;
}

void addObstaclesToWorld(physics::PhysicsWorld& world, const std::vector<Obstacle>& obstacles) {
    for (const Obstacle& obstacle : obstacles) {
        physics::RigidBody& body = world.createBody();
        body.name = obstacle.kind.empty() ? "obstacle" : obstacle.kind;
        body.setType(physics::BodyType::Static);
        body.collisionEnabled = true;
        if (obstacle.shape == physics::ShapeType::Cylinder) {
            body.shape = physics::Shape::cylinder(obstacle.radius, obstacle.halfHeight);
        } else {
            body.shape = physics::Shape::box(obstacle.halfExtents);
        }
        body.position = obstacle.position;
        body.orientation = obstacle.orientation;
        body.setMass(0.0);
        body.updateCache();
    }
}

} // namespace sim::app::drive
