// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "PhysicsScenes.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "Scenes.hpp"
#include "physics/ForceElements.hpp"
#include "physics/Joints.hpp"
#include "physics/Math.hpp"
#include "physics/PhysicsWorld.hpp"
#include "render/DebugDraw.hpp"
#include "render/Mesh.hpp"
#include "render/Scene.hpp"

namespace sim::app {
namespace {

// Every physics scene is captured after this many simulated seconds.
constexpr double kSettleTime = 1.5;

const glm::vec4 kGroundColor{0.30f, 0.33f, 0.28f, 1.0f};
const glm::vec4 kStaticColor{0.55f, 0.57f, 0.60f, 1.0f};
const glm::vec4 kTrajectoryColor{0.35f, 0.85f, 0.40f, 1.0f};
const glm::vec4 kRodColor{0.75f, 0.76f, 0.80f, 1.0f};
const glm::vec4 kSpringColor{0.95f, 0.80f, 0.35f, 1.0f};
const glm::vec4 kAxisX{0.85f, 0.25f, 0.25f, 1.0f};
const glm::vec4 kAxisY{0.30f, 0.85f, 0.35f, 1.0f};
const glm::vec4 kAxisZ{0.35f, 0.55f, 0.95f, 1.0f};

glm::vec4 hueColor(float t)
{
    const float h = t - std::floor(t);
    const float x = 1.0f - std::abs(std::fmod(h * 6.0f, 2.0f) - 1.0f);
    if (h < 1.0f / 6.0f)
    {
        return {1.0f, x, 0.0f, 1.0f};
    }
    if (h < 2.0f / 6.0f)
    {
        return {x, 1.0f, 0.0f, 1.0f};
    }
    if (h < 3.0f / 6.0f)
    {
        return {0.0f, 1.0f, x, 1.0f};
    }
    if (h < 4.0f / 6.0f)
    {
        return {0.0f, x, 1.0f, 1.0f};
    }
    if (h < 5.0f / 6.0f)
    {
        return {x, 0.0f, 1.0f, 1.0f};
    }
    return {1.0f, 0.0f, x, 1.0f};
}

physics::RigidBody& addStatic(physics::PhysicsWorld& world, const physics::Vec3& position, float halfSize)
{
    physics::RigidBody& body = world.createBody();
    body.name = "static";
    body.setType(physics::BodyType::Static);
    body.shape = physics::Shape::box({halfSize, halfSize, halfSize});
    body.position = position;
    return body;
}

physics::RigidBody& addDynamic(physics::PhysicsWorld& world, const std::string& name, float mass, physics::Shape shape,
                               const physics::Vec3& position)
{
    physics::RigidBody& body = world.createBody();
    body.name = name;
    body.setType(physics::BodyType::Dynamic);
    body.shape = shape;
    body.setMass(mass);
    body.setShapeInertia();
    body.position = position;
    return body;
}

// A coil-shaped spring between two world points, drawn with instantiated lines.
void addSpring(render::DebugDraw& draw, const glm::vec3& a, const glm::vec3& b, int coils, float radius, const glm::vec4& color)
{
    const glm::vec3 axis = b - a;
    const float length = glm::length(axis);
    if (length < 1e-4f)
    {
        return;
    }
    const glm::vec3 dir = axis / length;
    const glm::vec3 perpendicular = glm::normalize(std::abs(dir.y) < 0.9f
                                                       ? glm::cross(dir, glm::vec3(0.0f, 1.0f, 0.0f))
                                                       : glm::cross(dir, glm::vec3(1.0f, 0.0f, 0.0f)));
    const int n = coils * 6;
    glm::vec3 previous = a;
    for (int i = 1; i <= n; ++i)
    {
        const float t = 0.10f + 0.80f * static_cast<float>(i) / static_cast<float>(n);
        const glm::vec3 point = a + dir * (length * t)
                                + perpendicular * (radius * static_cast<float>(i % 2 == 0 ? 1 : -1));
        draw.addLine(previous, point, color);
        previous = point;
    }
    draw.addLine(previous, b, color);
}

void buildProjectileScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    physics::RigidBody& body = addDynamic(world, "projectile", 1.0f, physics::Shape::box({0.25, 0.25, 0.25}),
                                          physics::Vec3(0.0, 2.0, 0.0));
    body.linearVelocity = physics::Vec3(5.0, 8.0, 0.0);
}

void buildOscillatorScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    physics::RigidBody& anchor = addStatic(world, physics::Vec3(0.0, 4.0, 0.0), 0.12f);
    physics::RigidBody& bob = addDynamic(world, "bob", 1.0f, physics::Shape::sphere(0.25), physics::Vec3(0.0, 2.3, 0.0));
    physics::PointSpringDamper& spring = world.createForceElement<physics::PointSpringDamper>(&bob, &anchor);
    spring.restLength = 2.0;
    spring.stiffness = 100.0;
}

void buildPendulumScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    const double length = 2.0;
    const double angle = 0.6;
    physics::RigidBody& pivot = addStatic(world, physics::Vec3(0.0, 4.0, 0.0), 0.12f);
    physics::RigidBody& bob = addDynamic(world, "bob", 1.0f, physics::Shape::sphere(0.18),
                                         physics::Vec3(length * std::sin(angle), 4.0 - length * std::cos(angle), 0.0));
    bob.linearVelocity = physics::Vec3(0.4, 0.0, 0.0);
    physics::DistanceConstraint& rod = world.createConstraint<physics::DistanceConstraint>(pivot, bob);
    rod.distance = length;
}

void buildDoublePendulumScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    const double a1 = 0.5;
    const double a2 = -0.6;
    const double length1 = 1.4;
    const double length2 = 1.2;

    physics::RigidBody& pivot = addStatic(world, physics::Vec3(0.0, 5.0, 0.0), 0.12f);
    const physics::Vec3 p1(length1 * std::sin(a1), 5.0 - length1 * std::cos(a1), 0.0);
    physics::RigidBody& link1 = addDynamic(world, "link1", 1.0f, physics::Shape::sphere(0.14), p1);
    const physics::Vec3 p2 = p1 + physics::Vec3(length2 * std::sin(a1 + a2), -length2 * std::cos(a1 + a2), 0.0);
    physics::RigidBody& link2 = addDynamic(world, "link2", 0.7f, physics::Shape::sphere(0.11), p2);

    world.createConstraint<physics::SphericalConstraint>(pivot, link1);
    world.createConstraint<physics::SphericalConstraint>(link1, link2);
}

void buildChainScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    constexpr int kLinks = 20;
    constexpr double kHalf = 0.12;
    const physics::Vec3 localTop(0.0, kHalf, 0.0);
    const physics::Vec3 localBottom(0.0, -kHalf, 0.0);

    physics::RigidBody& anchor = addStatic(world, physics::Vec3(0.0, 6.0, 0.0), 0.15f);

    std::vector<physics::RigidBody*> links;
    for (int i = 0; i < kLinks; ++i)
    {
        const double y = 6.0 - (static_cast<double>(i) + 0.5) * 2.0 * kHalf;
        physics::RigidBody& link = addDynamic(world, "chain", 1.0f, physics::Shape::box({0.05, kHalf, 0.05}),
                                              physics::Vec3(0.03 * (kLinks - i), y, 0.0));
        link.linearVelocity = physics::Vec3(0.25 * (kLinks - i) / kLinks, 0.0, 0.0);
        links.push_back(&link);
    }

    world.createConstraint<physics::SphericalConstraint>(anchor, *links[0]).localAnchorB = localTop;
    for (int i = 0; i < kLinks - 1; ++i)
    {
        physics::SphericalConstraint& joint = world.createConstraint<physics::SphericalConstraint>(*links[i], *links[i + 1]);
        joint.localAnchorA = localBottom;
        joint.localAnchorB = localTop;
    }
}

void buildDzhanibekovScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0);
    physics::RigidBody& body = addDynamic(world, "tumbler", 1.0f, physics::Shape::box({0.60, 0.45, 0.30}),
                                          physics::Vec3(0.0, 0.6, 0.0));
    body.setInertiaPrincipal({1.0, 2.0, 3.0});
    body.angularVelocity = physics::Vec3(0.05, 6.0, 0.0);
    body.updateCache();
}

void buildGyroScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    const double distance = 0.1;
    const double spinRate = 3000.0 * 6.2831853 / 60.0; // 3000 rpm

    physics::RigidBody& pivot = addStatic(world, physics::Vec3(0.0), 0.12f);
    physics::RigidBody& wheel = addDynamic(world, "wheel", 1.0f, physics::Shape::cylinder(0.50, 0.06),
                                           physics::Vec3(0.0, -distance, 0.0));
    wheel.orientation = physics::quaternionFromAxisAngle(physics::Vec3(0.0, 0.0, 1.0), 0.15);
    wheel.position = -physics::rotateVector(wheel.orientation, physics::Vec3(0.0, distance, 0.0));
    wheel.angularVelocity = physics::rotateVector(wheel.orientation, physics::Vec3(0.0, spinRate, 0.0));
    wheel.updateCache();

    physics::SphericalConstraint& joint = world.createConstraint<physics::SphericalConstraint>(pivot, wheel);
    joint.localAnchorB = physics::Vec3(0.0, distance, 0.0);
}

void buildMassesScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);

    // Heavy column (1000 kg, stiff spring) and light column (1 kg, soft
    // spring): identical geometry, ~100x difference in oscillator frequency, so
    // the 1:1000 mass ratio is legible as slow/fast bounce.
    physics::RigidBody& heavyAnchor = addStatic(world, physics::Vec3(-2.0, 5.0, 0.0), 0.12f);
    physics::RigidBody& heavy = addDynamic(world, "heavy", 1000.0f, physics::Shape::sphere(0.55), physics::Vec3(-2.0, 2.52, 0.0));
    physics::PointSpringDamper& heavySpring = world.createForceElement<physics::PointSpringDamper>(&heavy, &heavyAnchor);
    heavySpring.restLength = 2.0;
    heavySpring.stiffness = 10000.0;

    physics::RigidBody& lightAnchor = addStatic(world, physics::Vec3(2.0, 5.0, 0.0), 0.12f);
    physics::RigidBody& light = addDynamic(world, "light", 1.0f, physics::Shape::sphere(0.22), physics::Vec3(2.0, 3.4, 0.0));
    physics::PointSpringDamper& lightSpring = world.createForceElement<physics::PointSpringDamper>(&light, &lightAnchor);
    lightSpring.restLength = 2.0;
    lightSpring.stiffness = 100.0;
}

void setPhysicsCamera(render::Scene& scene, float aspect)
{
    scene.camera().setPerspective(60.0f, aspect, 0.1f, 1000.0f);
}

} // namespace

const std::vector<std::string>& physicsSceneNames()
{
    static const std::vector<std::string> names{"drop", "oscillator", "pendulum",         "double_pendulum",
                                                "chain", "dzhanibekov", "gyro",            "masses"};
    return names;
}

bool isPhysicsScene(const std::string& name)
{
    const std::vector<std::string>& names = physicsSceneNames();
    return std::find(names.begin(), names.end(), name) != names.end();
}

double buildPhysicsWorld(const std::string& name, physics::PhysicsWorld& world)
{
    if (name == "drop")
    {
        buildProjectileScene(world);
    }
    else if (name == "oscillator")
    {
        buildOscillatorScene(world);
    }
    else if (name == "pendulum")
    {
        buildPendulumScene(world);
    }
    else if (name == "double_pendulum")
    {
        buildDoublePendulumScene(world);
    }
    else if (name == "chain")
    {
        buildChainScene(world);
    }
    else if (name == "dzhanibekov")
    {
        buildDzhanibekovScene(world);
    }
    else if (name == "gyro")
    {
        buildGyroScene(world);
    }
    else if (name == "masses")
    {
        buildMassesScene(world);
    }
    return kSettleTime;
}

void physicsSceneCamera(const std::string& name, render::Scene& scene, float aspect)
{
    if (name == "drop")
    {
        setPhysicsCamera(scene, aspect);
        scene.camera().lookAt({10.0f, 6.0f, 10.0f}, {3.5f, 3.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    }
    else if (name == "oscillator")
    {
        setPhysicsCamera(scene, aspect);
        scene.camera().lookAt({5.0f, 2.5f, 6.0f}, {0.0f, 2.2f, 0.0f}, {0.0f, 1.0f, 0.0f});
    }
    else if (name == "pendulum")
    {
        setPhysicsCamera(scene, aspect);
        scene.camera().lookAt({4.5f, 3.2f, 6.5f}, {0.3f, 3.1f, 0.0f}, {0.0f, 1.0f, 0.0f});
    }
    else if (name == "double_pendulum")
    {
        setPhysicsCamera(scene, aspect);
        scene.camera().lookAt({7.0f, 4.2f, 9.0f}, {0.6f, 3.2f, 0.0f}, {0.0f, 1.0f, 0.0f});
    }
    else if (name == "chain")
    {
        setPhysicsCamera(scene, aspect);
        scene.camera().lookAt({6.0f, 3.0f, 7.0f}, {0.0f, 3.6f, 0.0f}, {0.0f, 1.0f, 0.0f});
    }
    else if (name == "dzhanibekov")
    {
        setPhysicsCamera(scene, aspect);
        scene.camera().lookAt({4.0f, 2.0f, 5.0f}, {0.0f, 0.6f, 0.0f}, {0.0f, 1.0f, 0.0f});
    }
    else if (name == "gyro")
    {
        setPhysicsCamera(scene, aspect);
        scene.camera().lookAt({3.0f, 2.0f, 4.5f}, {0.0f, -0.3f, 0.0f}, {0.0f, 1.0f, 0.0f});
    }
    else if (name == "masses")
    {
        setPhysicsCamera(scene, aspect);
        scene.camera().lookAt({0.0f, 3.4f, 9.0f}, {0.0f, 2.4f, 0.0f}, {0.0f, 1.0f, 0.0f});
    }
}

void buildSceneFromWorld(const physics::PhysicsWorld& world, render::Scene& scene, const std::shared_ptr<render::Mesh>& cube,
                         const std::shared_ptr<render::Mesh>& sphere, const std::shared_ptr<render::Mesh>& cylinder,
                         const std::shared_ptr<render::Mesh>& ground)
{
    scene.clear();
    setDefaultLight(scene);
    scene.add(ground, glm::mat4(1.0f), kGroundColor);

    for (std::size_t i = 0; i < world.bodyCount(); ++i)
    {
        const physics::RigidBody& body = world.body(i);
        const physics::Shape& shape = body.shape;
        glm::vec3 scale{1.0f};
        std::shared_ptr<render::Mesh> mesh;
        if (shape.type == physics::ShapeType::Sphere)
        {
            mesh = sphere;
            scale = glm::vec3(static_cast<float>(shape.radius));
        }
        else if (shape.type == physics::ShapeType::Cylinder)
        {
            mesh = cylinder;
            scale = glm::vec3(static_cast<float>(shape.radius), static_cast<float>(2.0 * shape.halfHeight),
                              static_cast<float>(shape.radius));
        }
        else
        {
            mesh = cube;
            scale = glm::vec3(static_cast<float>(shape.halfExtents.x * 2.0), static_cast<float>(shape.halfExtents.y * 2.0),
                              static_cast<float>(shape.halfExtents.z * 2.0));
        }

        const glm::vec3 position(static_cast<float>(body.position.x), static_cast<float>(body.position.y),
                                 static_cast<float>(body.position.z));
        const glm::mat4 model = glm::translate(glm::mat4(1.0f), position)
                                * glm::mat4_cast(glm::quat(body.orientation)) * glm::scale(glm::mat4(1.0f), scale);
        const glm::vec4 color = body.isStatic() ? kStaticColor : hueColor(0.55f + 0.06f * static_cast<float>(i));
        scene.add(mesh, model, color);
    }
}

void drawPhysicsSceneDebug(const std::string& name, const physics::PhysicsWorld& world, render::DebugDraw& draw)
{
    const auto position = [&](std::size_t index) {
        const physics::Vec3 p = world.body(index).position;
        return glm::vec3(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    };

    if (name == "drop")
    {
        // Analytic parabola the numeric trajectory is expected to follow.
        constexpr int kSegments = 24;
        const glm::vec3 start(0.0f, 2.0f, 0.0f);
        const glm::vec3 velocity(5.0f, 8.0f, 0.0f);
        constexpr float kGravity = 9.80665f;
        const float duration = static_cast<float>(kSettleTime);
        glm::vec3 previous = start;
        for (int i = 1; i <= kSegments; ++i)
        {
            const float t = duration * static_cast<float>(i) / static_cast<float>(kSegments);
            const glm::vec3 point = start + velocity * t + glm::vec3(0.0f, -0.5f * kGravity * t * t, 0.0f);
            draw.addLine(previous, point, kTrajectoryColor);
            previous = point;
        }
    }
    else if (name == "oscillator")
    {
        addSpring(draw, position(0), position(1), 8, 0.25f, kSpringColor);
    }
    else if (name == "pendulum")
    {
        draw.addLine(position(0), position(1), kRodColor);
    }
    else if (name == "double_pendulum")
    {
        draw.addLine(position(0), position(1), kRodColor);
        draw.addLine(position(1), position(2), kRodColor);
    }
    else if (name == "dzhanibekov")
    {
        // Body-frame axes, so the intermediate-axis flip is read directly.
        const physics::RigidBody& body = world.body(0);
        const glm::vec3 origin(static_cast<float>(body.position.x), static_cast<float>(body.position.y),
                               static_cast<float>(body.position.z));
        const glm::quat q = glm::quat(body.orientation);
        draw.addLine(origin, origin + q * glm::vec3(0.0f, 0.0f, 1.0f) * 1.2f, kAxisZ);
        draw.addLine(origin, origin + q * glm::vec3(0.0f, 1.0f, 0.0f) * 1.2f, kAxisY);
        draw.addLine(origin, origin + q * glm::vec3(1.0f, 0.0f, 0.0f) * 1.2f, kAxisX);
    }
    else if (name == "gyro")
    {
        // Offset arm from the pivot to the wheel, then the spin axis.
        const physics::RigidBody& wheel = world.body(1);
        const glm::vec3 center = position(1);
        const glm::vec3 arm = center - position(0);
        draw.addLine(position(0), center, kRodColor);
        const glm::quat q = glm::quat(wheel.orientation);
        draw.addLine(center - q * glm::vec3(0.0f, 1.0f, 0.0f) * 0.9f, center + q * glm::vec3(0.0f, 1.0f, 0.0f) * 1.1f, kAxisY);
        draw.addLine(center, center + glm::normalize(arm) * 0.3f, kSpringColor);
    }
    else if (name == "masses")
    {
        addSpring(draw, position(0), position(2), 8, 0.22f, kSpringColor);
        addSpring(draw, position(1), position(3), 6, 0.14f, kSpringColor);
    }
}

} // namespace sim::app