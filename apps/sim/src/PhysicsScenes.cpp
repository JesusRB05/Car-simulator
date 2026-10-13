// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "PhysicsScenes.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
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

// Spring parameters shared by the scene builders and physicsSceneEnergy().
constexpr double kOscillatorRest = 2.0;
constexpr double kOscillatorStiffness = 100.0;

struct PhysicsSceneEntry
{
    const char* name;
    const char* description;
};

// Display order and descriptions of the physics scenes.
constexpr PhysicsSceneEntry kPhysicsScenes[] = {
    {"drop", "Projectile launched under gravity, drawn against its analytic parabola."},
    {"oscillator", "Mass on a linear spring hanging from a fixed anchor."},
    {"pendulum", "Simple pendulum on a rigid distance constraint."},
    {"double_pendulum", "Chaotic double pendulum built from two spherical joints."},
    {"chain", "Twenty box links joined by spherical joints, swinging from an anchor."},
    {"dzhanibekov", "Free box spinning near its intermediate axis (tennis-racket flip)."},
    {"gyro", "Fast-spinning wheel on an offset pivot: gyroscopic precession."},
    {"masses", "1000 kg and 1 kg bodies on springs: a 1:1000 mass-ratio check."},
};

const glm::vec4 kGroundColor{0.30f, 0.33f, 0.28f, 1.0f};
const glm::vec4 kStaticColor{0.55f, 0.57f, 0.60f, 1.0f};
const glm::vec4 kTrajectoryColor{0.35f, 0.85f, 0.40f, 1.0f};
const glm::vec4 kRodColor{0.75f, 0.76f, 0.80f, 1.0f};
const glm::vec4 kSpringColor{0.95f, 0.80f, 0.35f, 1.0f};
const glm::vec4 kAxisX{0.85f, 0.25f, 0.25f, 1.0f};
const glm::vec4 kAxisY{0.30f, 0.85f, 0.35f, 1.0f};
const glm::vec4 kAxisZ{0.35f, 0.55f, 0.95f, 1.0f};
const glm::vec4 kVelocityColor{0.30f, 0.95f, 0.95f, 1.0f};
constexpr float kVelocityArrowSeconds = 0.25f;
constexpr float kVelocityArrowMax = 4.0f;

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

// Free fall (body 1, released from rest) next to a projectile (body 0); both
// analytic trajectories are drawn by drawPhysicsSceneDebug.
const glm::vec3 kProjectileStart(0.0f, 2.0f, 0.0f);
const glm::vec3 kProjectileVelocity(5.0f, 8.0f, 0.0f);
const glm::vec3 kFreeFallStart(-2.0f, 6.0f, 0.0f);

void buildProjectileScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    physics::RigidBody& body = addDynamic(world, "projectile", 1.0f, physics::Shape::box({0.25, 0.25, 0.25}),
                                          physics::Vec3(kProjectileStart));
    body.linearVelocity = physics::Vec3(kProjectileVelocity);
    addDynamic(world, "free fall", 1.0f, physics::Shape::sphere(0.25), physics::Vec3(kFreeFallStart));
}

void buildOscillatorScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    physics::RigidBody& anchor = addStatic(world, physics::Vec3(0.0, 4.0, 0.0), 0.12f);
    physics::RigidBody& bob = addDynamic(world, "bob", 1.0f, physics::Shape::sphere(0.25), physics::Vec3(0.0, 2.3, 0.0));
    physics::PointSpringDamper& spring = world.createForceElement<physics::PointSpringDamper>(&bob, &anchor);
    spring.restLength = kOscillatorRest;
    spring.stiffness = kOscillatorStiffness;
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

    // Massless rods between the body centres (a ball joint with centred
    // anchors would pin each bob onto the previous one).
    world.createConstraint<physics::DistanceConstraint>(pivot, link1).distance = length1;
    world.createConstraint<physics::DistanceConstraint>(link1, link2).distance = length2;
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

    // The pivot sits 1.5 m above the ground so the hanging wheel is clear of it.
    const physics::Vec3 pivotPosition(0.0, 1.5, 0.0);
    physics::RigidBody& pivot = addStatic(world, pivotPosition, 0.12f);
    physics::RigidBody& wheel = addDynamic(world, "wheel", 1.0f, physics::Shape::cylinder(0.50, 0.06),
                                           pivotPosition - physics::Vec3(0.0, distance, 0.0));
    wheel.orientation = physics::quaternionFromAxisAngle(physics::Vec3(0.0, 0.0, 1.0), 0.15);
    wheel.position = pivotPosition - physics::rotateVector(wheel.orientation, physics::Vec3(0.0, distance, 0.0));
    wheel.angularVelocity = physics::rotateVector(wheel.orientation, physics::Vec3(0.0, spinRate, 0.0));
    wheel.updateCache();

    physics::SphericalConstraint& joint = world.createConstraint<physics::SphericalConstraint>(pivot, wheel);
    joint.localAnchorB = physics::Vec3(0.0, distance, 0.0);
}

// Two hanging two-link chains with a 1:1000 mass ratio, as in the physics
// tests: left, static anchor -> 1 kg -> 1000 kg (the hard case: the light link
// carries the heavy one); right, anchor -> 1000 kg -> 1 kg (the easy reference).
// Both start with a lateral push. Bodies: 0 anchor, 1 light, 2 heavy, 3 anchor,
// 4 heavy, 5 light.
void buildMassesScene(physics::PhysicsWorld& world)
{
    world.gravity = physics::Vec3(0.0, -9.80665, 0.0);
    const double link = 1.0;
    const auto column = [&](double x, bool heavyBelow) {
        physics::RigidBody& anchor = addStatic(world, physics::Vec3(x, 4.5, 0.0), 0.12f);
        const float upperMass = heavyBelow ? 1.0f : 1000.0f;
        const float lowerMass = heavyBelow ? 1000.0f : 1.0f;
        const auto shapeFor = [](float mass) { return physics::Shape::sphere(mass > 10.0f ? 0.40 : 0.15); };
        physics::RigidBody& upper = addDynamic(world, upperMass > 10.0f ? "heavy" : "light", upperMass, shapeFor(upperMass),
                                               physics::Vec3(x, 4.5 - link, 0.0));
        physics::RigidBody& lower = addDynamic(world, lowerMass > 10.0f ? "heavy" : "light", lowerMass, shapeFor(lowerMass),
                                               physics::Vec3(x, 4.5 - 2.0 * link, 0.0));
        lower.linearVelocity = physics::Vec3(1.0, 0.0, 0.0);
        // Rigid joints (exact island solve + position projection), as in the
        // 1:1000 physics tests; the default soft 60 Hz joints stretch ~0.17 m here.
        physics::DistanceConstraint& top = world.createConstraint<physics::DistanceConstraint>(anchor, upper);
        top.distance = link;
        top.soft = physics::SoftParams::makeRigid();
        physics::DistanceConstraint& bottom = world.createConstraint<physics::DistanceConstraint>(upper, lower);
        bottom.distance = link;
        bottom.soft = physics::SoftParams::makeRigid();
    };
    column(-1.5, true);
    column(1.5, false);
}

// Radius of a sphere that encloses the body's shape, for framing.
double boundingRadius(const physics::Shape& shape)
{
    switch (shape.type)
    {
    case physics::ShapeType::Sphere:
        return shape.radius;
    case physics::ShapeType::Cylinder:
        return std::sqrt(shape.radius * shape.radius + shape.halfHeight * shape.halfHeight);
    default:
        return glm::length(shape.halfExtents);
    }
}

// Viewing direction (from the target towards the camera) per scene. Most demos
// move in the XY plane, so the camera sits on +Z, a little to the right and
// above, looking towards -Z.
glm::vec3 viewDirection(const std::string& name)
{
    if (name == "gyro" || name == "dzhanibekov")
    {
        return glm::normalize(glm::vec3(0.8f, 0.55f, 1.0f));
    }
    return glm::normalize(glm::vec3(0.35f, 0.25f, 1.0f));
}

} // namespace

const std::vector<std::string>& physicsSceneNames()
{
    static const std::vector<std::string> names = [] {
        std::vector<std::string> list;
        for (const PhysicsSceneEntry& entry : kPhysicsScenes)
        {
            list.emplace_back(entry.name);
        }
        return list;
    }();
    return names;
}

const char* physicsSceneDescription(const std::string& name)
{
    for (const PhysicsSceneEntry& entry : kPhysicsScenes)
    {
        if (name == entry.name)
        {
            return entry.description;
        }
    }
    return "";
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
    // Automatic framing: run a scratch copy of the scene for a while (it is
    // deterministic, so it sweeps exactly what the real run will show), take
    // the bounding box of every body, and fit its bounding sphere in the view
    // frustum along the scene's viewing direction.
    physics::PhysicsWorld world;
    const double settle = buildPhysicsWorld(name, world);
    const double horizon = settle;
    const physics::Real dt = 1.0 / 1000.0;
    const int steps = static_cast<int>(horizon / dt);

    glm::vec3 lower(std::numeric_limits<float>::max());
    glm::vec3 upper(-std::numeric_limits<float>::max());
    const auto accumulate = [&]() {
        for (std::size_t i = 0; i < world.bodyCount(); ++i)
        {
            const physics::RigidBody& body = world.body(i);
            const glm::vec3 centre(body.position);
            const float radius = static_cast<float>(boundingRadius(body.shape));
            lower = glm::min(lower, centre - glm::vec3(radius));
            upper = glm::max(upper, centre + glm::vec3(radius));
        }
    };
    accumulate();
    for (int step = 1; step <= steps; ++step)
    {
        world.step(dt);
        if (step % 20 == 0)
        {
            accumulate();
        }
    }

    constexpr float kFovY = 50.0f;
    scene.camera().setPerspective(kFovY, aspect, 0.05f, 1000.0f);
    // Bodies fall through the ground (no collisions yet); never frame below it.
    lower.y = std::max(lower.y, 0.0f);
    upper.y = std::max(upper.y, lower.y + 0.5f);
    const glm::vec3 target = 0.5f * (lower + upper);
    const float radius = std::max(0.5f * glm::length(upper - lower), 0.75f);
    const float halfFovY = glm::radians(kFovY) * 0.5f;
    const float halfFovX = std::atan(std::tan(halfFovY) * aspect);
    const float distance = 1.15f * radius / std::sin(std::min(halfFovY, halfFovX));
    scene.camera().lookAt(target + viewDirection(name) * distance, target, {0.0f, 1.0f, 0.0f});
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

double physicsSceneEnergy(const std::string& name, const physics::PhysicsWorld& world)
{
    double energy = 0.0;
    for (std::size_t i = 0; i < world.bodyCount(); ++i)
    {
        const physics::RigidBody& body = world.body(i);
        if (!body.isDynamic())
        {
            continue;
        }
        energy += body.kineticEnergy();
        energy -= body.mass * glm::dot(world.gravity, body.position);
    }

    const auto elastic = [&](std::size_t a, std::size_t b, double rest, double stiffness) {
        const double extension = glm::length(world.body(a).position - world.body(b).position) - rest;
        return 0.5 * stiffness * extension * extension;
    };
    // Rigid rods (masses, pendulums, chain) store no energy; only the
    // oscillator's spring does.
    if (name == "oscillator" && world.bodyCount() >= 2)
    {
        energy += elastic(0, 1, kOscillatorRest, kOscillatorStiffness);
    }
    return energy;
}

void drawPhysicsSceneDebug(const std::string& name, const physics::PhysicsWorld& world, render::DebugDraw& draw)
{
    // Headless captures show every layer, like the windowed defaults.
    drawPhysicsSceneDebug(name, world, draw, PhysicsDebugOptions{});
}

void drawPhysicsSceneDebug(const std::string& name, const physics::PhysicsWorld& world, render::DebugDraw& draw,
                           const PhysicsDebugOptions& options)
{
    const auto position = [&](std::size_t index) {
        const physics::Vec3 p = world.body(index).position;
        return glm::vec3(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    };

    if (name == "drop")
    {
        if (options.reference)
        {
            // Analytic trajectories the numeric ones are expected to follow:
            // the projectile's parabola and the free-falling body's vertical
            // line, both drawn down to the ground.
            constexpr int kSegments = 32;
            constexpr float kGravity = 9.80665f;
            const auto analytic = [&](const glm::vec3& start, const glm::vec3& velocity) {
                const float landing =
                    (velocity.y + std::sqrt(velocity.y * velocity.y + 2.0f * kGravity * start.y)) / kGravity;
                glm::vec3 previous = start;
                for (int i = 1; i <= kSegments; ++i)
                {
                    const float t = landing * static_cast<float>(i) / static_cast<float>(kSegments);
                    const glm::vec3 point = start + velocity * t + glm::vec3(0.0f, -0.5f * kGravity * t * t, 0.0f);
                    draw.addLine(previous, point, kTrajectoryColor);
                    previous = point;
                }
            };
            analytic(kProjectileStart, kProjectileVelocity);
            analytic(kFreeFallStart, glm::vec3(0.0f));
        }
    }
    else if (name == "oscillator")
    {
        if (options.springs)
        {
            addSpring(draw, position(0), position(1), 8, 0.25f, kSpringColor);
        }
    }
    else if (name == "pendulum")
    {
        if (options.joints)
        {
            draw.addLine(position(0), position(1), kRodColor);
        }
    }
    else if (name == "double_pendulum")
    {
        if (options.joints)
        {
            draw.addLine(position(0), position(1), kRodColor);
            draw.addLine(position(1), position(2), kRodColor);
        }
    }
    else if (name == "chain")
    {
        if (options.joints && options.genericOverlays)
        {
            // Polyline through the anchor and every link centre: the chain's
            // spine, which passes through each spherical joint.
            for (std::size_t i = 1; i < world.bodyCount(); ++i)
            {
                draw.addLine(position(i - 1), position(i), kRodColor);
            }
        }
    }
    else if (name == "dzhanibekov")
    {
        if (options.bodyAxes)
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
    }
    else if (name == "gyro")
    {
        // Offset arm from the pivot to the wheel, then the spin axis.
        const physics::RigidBody& wheel = world.body(1);
        const glm::vec3 center = position(1);
        const glm::vec3 arm = center - position(0);
        if (options.joints)
        {
            draw.addLine(position(0), center, kRodColor);
        }
        if (options.bodyAxes)
        {
            const glm::quat q = glm::quat(wheel.orientation);
            draw.addLine(center - q * glm::vec3(0.0f, 1.0f, 0.0f) * 0.9f, center + q * glm::vec3(0.0f, 1.0f, 0.0f) * 1.1f,
                         kAxisY);
        }
        if (options.joints)
        {
            draw.addLine(center, center + glm::normalize(arm) * 0.3f, kSpringColor);
        }
    }
    else if (name == "masses")
    {
        if (options.joints)
        {
            // Rigid rods: anchor -> upper -> lower for both columns.
            draw.addLine(position(0), position(1), kRodColor);
            draw.addLine(position(1), position(2), kRodColor);
            draw.addLine(position(3), position(4), kRodColor);
            draw.addLine(position(4), position(5), kRodColor);
        }
    }

    // Generic per-body layers. The scenes that already draw their own body
    // axes are skipped so the overlay never doubles up.
    const bool sceneDrawsAxes = name == "dzhanibekov" || name == "gyro";
    for (std::size_t i = 0; i < world.bodyCount(); ++i)
    {
        const physics::RigidBody& body = world.body(i);
        if (!body.isDynamic())
        {
            continue;
        }
        const glm::vec3 origin = position(i);
        if (options.bodyAxes && options.genericOverlays && !sceneDrawsAxes)
        {
            const physics::Shape& shape = body.shape;
            double extent = glm::length(shape.halfExtents);
            if (shape.type == physics::ShapeType::Sphere)
            {
                extent = shape.radius;
            }
            else if (shape.type == physics::ShapeType::Cylinder)
            {
                extent = std::max(shape.radius, shape.halfHeight);
            }
            const float axisLength = std::max(0.25f, static_cast<float>(extent) * 1.6f);
            const glm::quat q = glm::quat(body.orientation);
            draw.addLine(origin, origin + q * glm::vec3(1.0f, 0.0f, 0.0f) * axisLength, kAxisX);
            draw.addLine(origin, origin + q * glm::vec3(0.0f, 1.0f, 0.0f) * axisLength, kAxisY);
            draw.addLine(origin, origin + q * glm::vec3(0.0f, 0.0f, 1.0f) * axisLength, kAxisZ);
        }
        if (options.velocities)
        {
            // Velocity over a quarter of a second, capped so fast bodies stay
            // on screen, with a small arrow head.
            const glm::vec3 velocity(static_cast<float>(body.linearVelocity.x), static_cast<float>(body.linearVelocity.y),
                                     static_cast<float>(body.linearVelocity.z));
            glm::vec3 arrow = velocity * kVelocityArrowSeconds;
            const float arrowLength = glm::length(arrow);
            if (arrowLength < 1e-4f)
            {
                continue;
            }
            if (arrowLength > kVelocityArrowMax)
            {
                arrow *= kVelocityArrowMax / arrowLength;
            }
            const glm::vec3 tip = origin + arrow;
            const glm::vec3 dir = glm::normalize(arrow);
            const glm::vec3 side = glm::normalize(std::abs(dir.y) < 0.9f ? glm::cross(dir, glm::vec3(0.0f, 1.0f, 0.0f))
                                                                         : glm::cross(dir, glm::vec3(1.0f, 0.0f, 0.0f)));
            const float head = std::min(0.15f, glm::length(arrow) * 0.3f);
            draw.addLine(origin, tip, kVelocityColor);
            draw.addLine(tip, tip - dir * head + side * (head * 0.6f), kVelocityColor);
            draw.addLine(tip, tip - dir * head - side * (head * 0.6f), kVelocityColor);
        }
    }
}

} // namespace sim::app