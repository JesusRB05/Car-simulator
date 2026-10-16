// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "physics/Collision.hpp"
#include "physics/Convex.hpp"
#include "physics/Heightfield.hpp"
#include "physics/Math.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/RigidBody.hpp"
#include "physics/TriangleMesh.hpp"
#include "physics/TriangleMeshBvh.hpp"

using namespace sim::physics;

namespace {

RigidBody makeSphere(Real radius, const Vec3& position) {
    RigidBody body;
    body.shape = Shape::sphere(radius);
    body.position = position;
    body.updateCache();
    return body;
}

RigidBody makeBox(const Vec3& halfExtents, const Vec3& position, const Quat& orientation = Quat(1.0, 0.0, 0.0, 0.0)) {
    RigidBody body;
    body.shape = Shape::box(halfExtents);
    body.position = position;
    body.orientation = orientation;
    body.updateCache();
    return body;
}

RigidBody makeCylinder(Real radius, Real halfHeight, const Vec3& position) {
    RigidBody body;
    body.shape = Shape::cylinder(radius, halfHeight);
    body.position = position;
    body.updateCache();
    return body;
}

} // namespace

TEST_CASE("AABB overlap and separation") {
    const Aabb a{Vec3(-0.5), Vec3(0.5)};
    const Aabb b{Vec3(0.4, 0.0, 0.0), Vec3(1.4, 1.0, 1.0)};
    const Aabb c{Vec3(0.6, 0.0, 0.0), Vec3(1.6, 1.0, 1.0)};
    CHECK(a.overlaps(b));
    CHECK(b.overlaps(a));
    CHECK_FALSE(a.overlaps(c));
    CHECK(a.center() == Vec3(0.0));
    CHECK(a.extent() == Vec3(0.5));
}

TEST_CASE("computeAabb keeps a rotated box inside its bounds") {
    const RigidBody axisAligned = makeBox(Vec3(0.5), Vec3(0.0));
    const RigidBody rotated =
        makeBox(Vec3(0.5), Vec3(0.0), quaternionFromAxisAngle(Vec3(0.0, 0.0, 1.0), kPiPhysics / 4.0));

    const Aabb alignedBox = computeAabb(axisAligned);
    const Aabb rotatedBox = computeAabb(rotated);
    CHECK(alignedBox.min.x == doctest::Approx(-0.5));
    CHECK(alignedBox.max.y == doctest::Approx(0.5));
    // A 45-degree rotation grows the horizontal projection by sqrt(2).
    CHECK(rotatedBox.max.x == doctest::Approx(0.5 * std::sqrt(2.0)));
    CHECK(rotatedBox.max.y == doctest::Approx(0.5 * std::sqrt(2.0)));

    // Every corner of the rotated box must lie inside its computed AABB.
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            const Vec3 corner = rotated.localToWorldPoint(Vec3(0.5 * sx, 0.5 * sy, 0.5));
            CHECK(corner.x <= rotatedBox.max.x + 1e-12);
            CHECK(corner.x >= rotatedBox.min.x - 1e-12);
            CHECK(corner.y <= rotatedBox.max.y + 1e-12);
            CHECK(corner.y >= rotatedBox.min.y - 1e-12);
        }
    }
}

TEST_CASE("sphere-sphere contact is analytic and points from A to B") {
    const RigidBody a = makeSphere(0.5, Vec3(0.0));
    const RigidBody b = makeSphere(0.5, Vec3(0.8, 0.0, 0.0));

    ContactPoint points[kMaxContactPoints];
    const int count = collide(a, b, points, kMaxContactPoints);
    REQUIRE(count == 1);
    CHECK(points[0].normal.x == doctest::Approx(1.0));
    CHECK(points[0].normal.y == doctest::Approx(0.0));
    CHECK(points[0].penetration == doctest::Approx(0.2));
    CHECK(points[0].position.x == doctest::Approx(0.4));
    CHECK(glm::dot(points[0].normal, b.position - a.position) > 0.0);

    const RigidBody far = makeSphere(0.5, Vec3(2.0, 0.0, 0.0));
    CHECK(collide(a, far, points, kMaxContactPoints) == 0);
}

TEST_CASE("sphere-box contact on the ground and normal orientation") {
    const RigidBody sphere = makeSphere(0.5, Vec3(0.0, 0.8, 0.0));
    const RigidBody box = makeBox(Vec3(0.5), Vec3(0.0));

    ContactPoint points[kMaxContactPoints];
    REQUIRE(collide(sphere, box, points, kMaxContactPoints) == 1);
    // Normal points from the sphere (A) down into the box (B).
    CHECK(points[0].normal.y == doctest::Approx(-1.0));
    CHECK(points[0].penetration == doctest::Approx(0.2));
    CHECK(points[0].position.y == doctest::Approx(0.5));

    // The reversed pair flips the normal.
    REQUIRE(collide(box, sphere, points, kMaxContactPoints) == 1);
    CHECK(points[0].normal.y == doctest::Approx(1.0));

    // A sphere clear of the box reports no contact.
    const RigidBody high = makeSphere(0.5, Vec3(0.0, 1.2, 0.0));
    CHECK(collide(high, box, points, kMaxContactPoints) == 0);
}

TEST_CASE("box-box face contact produces four points") {
    const RigidBody a = makeBox(Vec3(0.5), Vec3(0.0));
    const RigidBody b = makeBox(Vec3(0.5), Vec3(0.0, 0.8, 0.0));

    ContactPoint points[kMaxContactPoints];
    const int count = collide(a, b, points, kMaxContactPoints);
    REQUIRE(count == 4);
    for (int i = 0; i < count; ++i) {
        CHECK(points[i].normal.y == doctest::Approx(1.0));
        CHECK(points[i].penetration == doctest::Approx(0.2));
        // Contact points lie on B's incident face (its bottom, y = 0.3).
        CHECK(points[i].position.y == doctest::Approx(0.3));
        CHECK(std::abs(points[i].position.x) == doctest::Approx(0.5));
    }

    const RigidBody separated = makeBox(Vec3(0.5), Vec3(0.0, 2.0, 0.0));
    CHECK(collide(a, separated, points, kMaxContactPoints) == 0);
}

TEST_CASE("box-box handles rotation and keeps the normal unit length") {
    const RigidBody a = makeBox(Vec3(0.5), Vec3(0.0));
    const RigidBody b =
        makeBox(Vec3(0.5), Vec3(0.7, 0.0, 0.0), quaternionFromAxisAngle(Vec3(0.0, 0.0, 1.0), kPiPhysics / 4.0));

    ContactPoint points[kMaxContactPoints];
    const int count = collide(a, b, points, kMaxContactPoints);
    REQUIRE(count >= 1);
    for (int i = 0; i < count; ++i) {
        CHECK(glm::length(points[i].normal) == doctest::Approx(1.0));
        CHECK(points[i].penetration >= 0.0);
        CHECK(glm::dot(points[i].normal, b.position - a.position) > 0.0);
    }
}

TEST_CASE("sphere-cylinder side and cap contacts") {
    const RigidBody cylinder = makeCylinder(0.5, 1.0, Vec3(0.0));
    const RigidBody side = makeSphere(0.5, Vec3(0.8, 0.0, 0.0));
    const RigidBody cap = makeSphere(0.5, Vec3(0.0, 1.3, 0.0));

    ContactPoint points[kMaxContactPoints];
    REQUIRE(collide(side, cylinder, points, kMaxContactPoints) == 1);
    CHECK(points[0].normal.x == doctest::Approx(-1.0));
    CHECK(points[0].penetration == doctest::Approx(0.2));
    CHECK(points[0].position.x == doctest::Approx(0.5));

    REQUIRE(collide(cap, cylinder, points, kMaxContactPoints) == 1);
    CHECK(points[0].normal.y == doctest::Approx(-1.0));
    CHECK(points[0].penetration == doctest::Approx(0.2));
    CHECK(points[0].position.y == doctest::Approx(1.0));
}

TEST_CASE("box and cylinder combinations use the generic convex path") {
    const RigidBody box = makeBox(Vec3(0.5), Vec3(0.0));
    const RigidBody cylinder = makeCylinder(0.5, 0.5, Vec3(0.6, 0.0, 0.0));

    ContactPoint points[kMaxContactPoints];
    // Box-cylinder and cylinder-cylinder are no longer skipped: they go through
    // GJK/EPA and report one witness point with the normal pointing from A to B.
    const int count = collide(box, cylinder, points, kMaxContactPoints);
    REQUIRE(count == 1);
    CHECK(glm::length(points[0].normal) == doctest::Approx(1.0));
    CHECK(points[0].normal.x > 0.9);
    CHECK(points[0].penetration == doctest::Approx(0.4).epsilon(0.05));

    // Swapping the operands flips the normal so it still points from A to B.
    ContactPoint reversed[kMaxContactPoints];
    REQUIRE(collide(cylinder, box, reversed, kMaxContactPoints) == 1);
    CHECK(reversed[0].normal.x == doctest::Approx(-points[0].normal.x));

    const RigidBody cylinderA = makeCylinder(0.5, 0.5, Vec3(0.0));
    const RigidBody cylinderB = makeCylinder(0.5, 0.5, Vec3(0.6, 0.0, 0.0));
    REQUIRE(collide(cylinderA, cylinderB, points, kMaxContactPoints) == 1);
    CHECK(points[0].normal.x > 0.9);

    // A heightfield pair is still skipped: supportWorld only approximates a
    // heightfield as a unit box, so it must not reach the convex path.
    RigidBody terrain;
    terrain.shape.type = ShapeType::Heightfield;
    terrain.updateCache();
    CHECK(collide(terrain, cylinderA, points, kMaxContactPoints) == 0);

    // Zero-capacity output is a no-op rather than an overflow.
    const RigidBody sphere = makeSphere(0.5, Vec3(0.0));
    CHECK(collide(sphere, box, points, 0) == 0);
}

TEST_CASE("AABB-tree broadphase matches the brute-force reference") {
    PhysicsWorld world;

    // A static ground slab (top face at y = 0) and a cluster of dynamic boxes,
    // some overlapping and one far away.
    RigidBody& ground = world.createBody();
    ground.shape = Shape::box(Vec3(10.0, 0.5, 10.0));
    ground.setType(BodyType::Static);
    ground.setMass(0.0);
    ground.collisionEnabled = true;
    ground.position = Vec3(0.0, -0.5, 0.0);
    ground.updateCache();

    const Vec3 positions[] = {Vec3(0.0, 0.5, 0.0), Vec3(0.9, 0.5, 0.0), Vec3(5.0, 0.5, 0.0), Vec3(0.0, 0.5, 0.9),
                              Vec3(20.0, 0.5, 0.0)};
    for (const Vec3& position : positions) {
        RigidBody& box = world.createBody();
        box.shape = Shape::box(Vec3(0.5));
        box.setMass(1.0);
        box.collisionEnabled = true;
        box.position = position;
        box.updateCache();
    }

    // A collidable box with collisions disabled on the body is never a candidate.
    RigidBody& ghost = world.createBody();
    ghost.shape = Shape::box(Vec3(0.5));
    ghost.setMass(1.0);
    ghost.collisionEnabled = false;
    ghost.position = Vec3(0.0, 0.5, 0.0);
    ghost.updateCache();

    // Brute-force reference over the same rules (the pre-tree behaviour).
    std::vector<BodyPair> expected;
    const std::size_t count = world.bodyCount();
    for (std::size_t i = 0; i < count; ++i) {
        if (!world.body(i).collisionEnabled) {
            continue;
        }
        for (std::size_t j = i + 1; j < count; ++j) {
            if (!world.body(j).collisionEnabled) {
                continue;
            }
            if (!world.body(i).isDynamic() && !world.body(j).isDynamic()) {
                continue;
            }
            if (computeAabb(world.body(i)).overlaps(computeAabb(world.body(j)))) {
                expected.push_back(BodyPair{i, j});
            }
        }
    }

    const std::vector<BodyPair> actual = broadphase(world);
    REQUIRE(actual.size() == expected.size());
    for (std::size_t k = 0; k < actual.size(); ++k) {
        CHECK(actual[k].bodyA == expected[k].bodyA);
        CHECK(actual[k].bodyB == expected[k].bodyB);
    }
    // The cluster really does produce overlaps, so the comparison is meaningful.
    CHECK(expected.size() >= 4);
}

TEST_CASE("broadphase is ordered and ignores non-dynamic pairs") {
    PhysicsWorld world;
    RigidBody& ground = world.createBody();
    ground.shape = Shape::box(Vec3(5.0, 0.5, 5.0));
    ground.setType(BodyType::Static);
    ground.setMass(0.0);
    ground.collisionEnabled = true;
    ground.updateCache();

    RigidBody& nearSphere = world.createBody();
    nearSphere.shape = Shape::sphere(0.5);
    nearSphere.setMass(1.0);
    nearSphere.position = Vec3(0.0, 0.6, 0.0);
    nearSphere.collisionEnabled = true;
    nearSphere.updateCache();

    RigidBody& farSphere = world.createBody();
    farSphere.shape = Shape::sphere(0.5);
    farSphere.setMass(1.0);
    farSphere.position = Vec3(100.0, 0.6, 0.0);
    farSphere.collisionEnabled = true;
    farSphere.updateCache();

    const std::vector<BodyPair> pairs = broadphase(world);
    REQUIRE(pairs.size() == 1);
    CHECK(pairs[0].bodyA == 0);
    CHECK(pairs[0].bodyB == 1);
}

TEST_CASE("the cached static broadphase still tracks moving dynamics") {
    PhysicsWorld world;

    RigidBody& ground = world.createBody();
    ground.shape = Shape::box(Vec3(5.0, 0.5, 5.0));
    ground.setType(BodyType::Static);
    ground.setMass(0.0);
    ground.position = Vec3(0.0, -0.5, 0.0);
    ground.collisionEnabled = true;
    ground.updateCache();

    RigidBody& box = world.createBody();
    box.shape = Shape::box(Vec3(0.5));
    box.setMass(1.0);
    box.collisionEnabled = true;
    box.position = Vec3(0.0, 3.0, 0.0);
    box.updateCache();

    // Well above the ground: no overlapping pair, and the tree built here is
    // reused (the static set does not change) by the calls below.
    CHECK(broadphase(world).empty());

    // Lowering the dynamic box overlaps the cached ground tree without a rebuild.
    box.position = Vec3(0.0, 0.5, 0.0);
    box.updateCache();
    std::vector<BodyPair> pairs = broadphase(world);
    REQUIRE(pairs.size() == 1);
    CHECK(pairs[0].bodyA == 0);
    CHECK(pairs[0].bodyB == 1);

    // Raising it again drops the pair, still reusing the cached tree.
    box.position = Vec3(0.0, 3.0, 0.0);
    box.updateCache();
    CHECK(broadphase(world).empty());
}

TEST_CASE("moving a static body needs an explicit broadphase invalidation") {
    PhysicsWorld world;

    RigidBody& wall = world.createBody();
    wall.shape = Shape::box(Vec3(0.5));
    wall.setType(BodyType::Static);
    wall.setMass(0.0);
    wall.collisionEnabled = true;
    wall.position = Vec3(50.0, 0.0, 0.0);
    wall.updateCache();

    RigidBody& box = world.createBody();
    box.shape = Shape::box(Vec3(0.5));
    box.setMass(1.0);
    box.collisionEnabled = true;
    box.position = Vec3(0.0);
    box.updateCache();

    CHECK(broadphase(world).empty());

    // Static bodies are assumed not to move; after an out-of-band move the caller
    // must invalidate the cache, then the pair appears.
    wall.position = Vec3(0.0);
    wall.updateCache();
    world.invalidateBroadphase();
    const std::vector<BodyPair> pairs = broadphase(world);
    REQUIRE(pairs.size() == 1);
    CHECK(pairs[0].bodyA == 0);
    CHECK(pairs[0].bodyB == 1);
}

TEST_CASE("sweep-and-prune matches a brute-force broadphase") {
    PhysicsWorld world;
    // A deterministic scatter of overlapping and separated bodies.
    for (int i = 0; i < 12; ++i) {
        RigidBody& body = world.createBody();
        const Real x = 0.4 * static_cast<Real>((i * 7) % 5);
        const Real y = 0.5 + 0.35 * static_cast<Real>(i % 3);
        const Real z = 0.4 * static_cast<Real>((i * 3) % 4);
        if (i % 2 == 0) {
            body.shape = Shape::sphere(0.5);
        } else {
            body.shape = Shape::box(Vec3(0.4));
        }
        body.setMass(1.0);
        body.position = Vec3(x, y, z);
        body.collisionEnabled = true;
        body.updateCache();
    }

    std::vector<BodyPair> reference;
    for (std::size_t i = 0; i < world.bodyCount(); ++i) {
        for (std::size_t j = i + 1; j < world.bodyCount(); ++j) {
            if (!world.body(i).collisionEnabled || !world.body(j).collisionEnabled) {
                continue;
            }
            if (!world.body(i).isDynamic() && !world.body(j).isDynamic()) {
                continue;
            }
            if (computeAabb(world.body(i)).overlaps(computeAabb(world.body(j)))) {
                reference.push_back(BodyPair{i, j});
            }
        }
    }

    const std::vector<BodyPair> pairs = broadphase(world);
    REQUIRE(pairs.size() == reference.size());
    for (std::size_t k = 0; k < pairs.size(); ++k) {
        CHECK(pairs[k].bodyA == reference[k].bodyA);
        CHECK(pairs[k].bodyB == reference[k].bodyB);
    }
}

TEST_CASE("sub-stepping stops a fast sphere tunnelling a thin wall") {
    PhysicsWorld world;
    world.gravity = Vec3(0.0);
    world.enableSubStepping = true;

    RigidBody& wall = world.createBody();
    wall.shape = Shape::box(Vec3(0.01, 1.0, 1.0));
    wall.setType(BodyType::Static);
    wall.setMass(0.0);
    wall.collisionEnabled = true;
    wall.updateCache();

    RigidBody& ball = world.createBody();
    ball.shape = Shape::sphere(0.05);
    ball.setMass(1.0);
    ball.position = Vec3(-0.3, 0.0, 0.0);
    ball.linearVelocity = Vec3(500.0, 0.0, 0.0);
    ball.restitution = 0.0;
    ball.collisionEnabled = true;
    ball.updateCache();

    // One step moves the ball 0.5 m, far past the 2 cm wall: without
    // sub-stepping it would jump straight through.
    for (int i = 0; i < 5; ++i) {
        world.step(1.0 / 1000.0);
    }

    CHECK(ball.position.x < 0.06); // never crossed the wall
    CHECK(std::abs(ball.linearVelocity.x) < 1.0);
}

TEST_CASE("a dynamic box comes to rest on static ground without tunnelling") {
    PhysicsWorld world;

    RigidBody& ground = world.createBody();
    ground.shape = Shape::box(Vec3(5.0, 0.5, 5.0));
    ground.setType(BodyType::Static);
    ground.setMass(0.0);
    ground.position = Vec3(0.0, -0.5, 0.0); // top face at y = 0
    ground.collisionEnabled = true;
    ground.updateCache();

    RigidBody& box = world.createBody();
    box.shape = Shape::box(Vec3(0.5));
    box.setMass(1.0);
    box.setShapeInertia();
    box.position = Vec3(0.0, 2.0, 0.0);
    box.collisionEnabled = true;
    box.updateCache();

    for (int i = 0; i < 2000; ++i) {
        world.step(1.0 / 1000.0);
    }

    // Resting height is the half-extent plus the allowed penetration slop.
    CHECK(box.position.y > 0.49);
    CHECK(box.position.y < 0.52);
    CHECK(std::abs(box.linearVelocity.y) < 0.05);
}

TEST_CASE("collision response is opt-in: a disabled body falls through") {
    PhysicsWorld world;

    RigidBody& ground = world.createBody();
    ground.shape = Shape::box(Vec3(5.0, 0.5, 5.0));
    ground.setType(BodyType::Static);
    ground.setMass(0.0);
    ground.position = Vec3(0.0, -0.5, 0.0);
    ground.updateCache();

    RigidBody& box = world.createBody();
    box.shape = Shape::box(Vec3(0.5));
    box.setMass(1.0);
    box.setShapeInertia();
    box.position = Vec3(0.0, 2.0, 0.0);
    box.updateCache();

    for (int i = 0; i < 2000; ++i) {
        world.step(1.0 / 1000.0);
    }

    CHECK(box.position.y < -10.0);
}

TEST_CASE("contact response is deterministic across repeated runs and snapshots") {
    const auto build = [](PhysicsWorld& world) {
        RigidBody& ground = world.createBody();
        ground.shape = Shape::box(Vec3(10.0, 0.5, 10.0));
        ground.setType(BodyType::Static);
        ground.setMass(0.0);
        ground.position = Vec3(0.0, -0.5, 0.0);
        ground.collisionEnabled = true;
        ground.updateCache();

        RigidBody& box = world.createBody();
        box.shape = Shape::box(Vec3(0.5));
        box.setMass(1.0);
        box.setShapeInertia();
        box.position = Vec3(0.0, 3.0, 0.0);
        box.collisionEnabled = true;
        box.updateCache();

        RigidBody& sphere = world.createBody();
        sphere.shape = Shape::sphere(0.5);
        sphere.setMass(2.0);
        sphere.setShapeInertia();
        sphere.position = Vec3(0.2, 4.0, 0.1);
        sphere.collisionEnabled = true;
        sphere.updateCache();
    };

    PhysicsWorld first;
    build(first);
    for (int i = 0; i < 2000; ++i) {
        first.step(1.0 / 1000.0);
    }

    PhysicsWorld second;
    build(second);
    for (int i = 0; i < 2000; ++i) {
        second.step(1.0 / 1000.0);
    }
    CHECK(first.stateHash() == second.stateHash());

    PhysicsWorld restored;
    build(restored);
    const WorldSnapshot snapshot = restored.saveSnapshot();
    for (int i = 0; i < 1000; ++i) {
        restored.step(1.0 / 1000.0);
    }
    restored.restoreSnapshot(snapshot);
    for (int i = 0; i < 2000; ++i) {
        restored.step(1.0 / 1000.0);
    }
    CHECK(restored.stateHash() == first.stateHash());
}

TEST_CASE("Coulomb friction rolls a sliding sphere to 5/7 of its speed") {
    const auto finalSpeed = [](Real friction) {
        PhysicsWorld world;
        RigidBody& ground = world.createBody();
        ground.shape = Shape::box(Vec3(10.0, 0.5, 10.0));
        ground.setType(BodyType::Static);
        ground.setMass(0.0);
        ground.position = Vec3(0.0, -0.5, 0.0);
        ground.friction = friction;
        ground.collisionEnabled = true;
        ground.updateCache();

        RigidBody& sphere = world.createBody();
        sphere.shape = Shape::sphere(0.5);
        sphere.setMass(1.0);
        sphere.setShapeInertia();
        sphere.position = Vec3(0.0, 0.5, 0.0);
        sphere.linearVelocity = Vec3(2.0, 0.0, 0.0);
        sphere.friction = friction;
        sphere.collisionEnabled = true;
        sphere.updateCache();

        for (int i = 0; i < 500; ++i) {
            world.step(1.0 / 1000.0);
        }
        return world.body(1).linearVelocity.x;
    };

    const Real sliding = finalSpeed(0.0);
    const Real rolling = finalSpeed(0.6);
    MESSAGE("sphere sliding speed with friction = " << rolling << " (analytic 5/7 * 2 = " << 10.0 / 7.0
                                                    << "), without friction = " << sliding);
    // A solid sphere that starts sliding and rolls without slipping: v -> 5/7 v0.
    CHECK(rolling == doctest::Approx(10.0 / 7.0).epsilon(0.08));
    // With no friction it keeps sliding at its original speed.
    CHECK(sliding == doctest::Approx(2.0).epsilon(0.02));
}

TEST_CASE("restitution makes a dropped sphere bounce") {
    const auto apexAfterBounce = [](Real restitution) {
        PhysicsWorld world;
        RigidBody& ground = world.createBody();
        ground.shape = Shape::box(Vec3(10.0, 0.5, 10.0));
        ground.setType(BodyType::Static);
        ground.setMass(0.0);
        ground.position = Vec3(0.0, -0.5, 0.0);
        ground.restitution = restitution;
        ground.collisionEnabled = true;
        ground.updateCache();

        RigidBody& sphere = world.createBody();
        sphere.shape = Shape::sphere(0.5);
        sphere.setMass(1.0);
        sphere.setShapeInertia();
        sphere.position = Vec3(0.0, 2.0, 0.0);
        sphere.collisionEnabled = true;
        sphere.updateCache();

        Real apex = 0.0;
        bool touched = false;
        for (int i = 0; i < 3000; ++i) {
            world.step(1.0 / 1000.0);
            if (!touched && world.body(1).position.y < 0.55) {
                touched = true;
            }
            if (touched) {
                apex = std::max(apex, world.body(1).position.y);
            }
        }
        return apex;
    };

    const Real drop = 2.0 - 0.5; // centre travel from the start down to contact
    const Real bouncing = apexAfterBounce(0.8);
    const Real dead = apexAfterBounce(0.0);
    MESSAGE("sphere apex after bounce: e=0.8 -> " << bouncing << " (analytic " << 0.5 + 0.64 * drop << "), e=0 -> "
                                                  << dead);
    // e = 0.8 rebounds to e^2 = 0.64 of the drop height above the contact point.
    CHECK(bouncing == doctest::Approx(0.5 + 0.64 * drop).epsilon(0.15));
    // e = 0 does not rebound above the resting height (plus the contact slop).
    CHECK(dead < 0.6);
}

TEST_CASE("a box stack stays stable for 60 s") {
    PhysicsWorld world;
    RigidBody& ground = world.createBody();
    ground.shape = Shape::box(Vec3(5.0, 0.5, 5.0));
    ground.setType(BodyType::Static);
    ground.setMass(0.0);
    ground.position = Vec3(0.0, -0.5, 0.0);
    ground.friction = 0.8;
    ground.collisionEnabled = true;
    ground.updateCache();

    constexpr int kBoxes = 5;
    std::vector<RigidBody*> boxes;
    for (int i = 0; i < kBoxes; ++i) {
        RigidBody& box = world.createBody();
        box.shape = Shape::box(Vec3(0.5));
        box.setMass(1.0);
        box.setShapeInertia();
        box.position = Vec3(0.0, 0.5 + static_cast<Real>(i), 0.0);
        box.friction = 0.8;
        box.collisionEnabled = true;
        box.updateCache();
        boxes.push_back(&box);
    }

    const int steps = 60000; // 60 s at dt = 1 ms
    for (int step = 0; step < steps; ++step) {
        world.step(1.0 / 1000.0);
    }

    Real maxDrift = 0.0;
    Real maxHeightError = 0.0;
    Real maxSpeed = 0.0;
    for (int i = 0; i < kBoxes; ++i) {
        const Vec3 position = boxes[static_cast<std::size_t>(i)]->position;
        maxDrift = std::max(maxDrift, std::sqrt(position.x * position.x + position.z * position.z));
        maxHeightError = std::max(maxHeightError, std::abs(position.y - (0.5 + static_cast<Real>(i))));
        maxSpeed = std::max(maxSpeed, glm::length(boxes[static_cast<std::size_t>(i)]->linearVelocity));
    }
    MESSAGE("box stack over 60 s: max horizontal drift = " << maxDrift << " m, max height error = " << maxHeightError
                                                           << " m, max speed = " << maxSpeed << " m/s");
    CHECK(maxDrift < 0.05);
    CHECK(maxHeightError < 0.05);
    CHECK(maxSpeed < 0.05);
}

TEST_CASE("a resting stack sleeps, stays frozen and is woken by a projectile") {
    PhysicsWorld world;
    RigidBody& ground = world.createBody();
    ground.shape = Shape::box(Vec3(5.0, 0.5, 5.0));
    ground.setType(BodyType::Static);
    ground.setMass(0.0);
    ground.position = Vec3(0.0, -0.5, 0.0);
    ground.friction = 0.8;
    ground.collisionEnabled = true;
    ground.updateCache();

    constexpr int kBoxes = 3;
    std::vector<RigidBody*> boxes;
    for (int i = 0; i < kBoxes; ++i) {
        RigidBody& box = world.createBody();
        box.shape = Shape::box(Vec3(0.5));
        box.setMass(1.0);
        box.setShapeInertia();
        box.position = Vec3(0.0, 0.5 + static_cast<Real>(i), 0.0);
        box.friction = 0.8;
        box.collisionEnabled = true;
        box.updateCache();
        boxes.push_back(&box);
    }

    for (int i = 0; i < 3000; ++i) {
        world.step(1.0 / 1000.0);
    }
    for (RigidBody* box : boxes) {
        CHECK(box->sleeping);
    }

    // A sleeping body is not integrated: its pose and timer are frozen.
    std::vector<Vec3> frozen;
    for (RigidBody* box : boxes) {
        frozen.push_back(box->position);
    }
    for (int i = 0; i < 1000; ++i) {
        world.step(1.0 / 1000.0);
    }
    for (int i = 0; i < kBoxes; ++i) {
        CHECK(boxes[static_cast<std::size_t>(i)]->position == frozen[static_cast<std::size_t>(i)]);
        CHECK(boxes[static_cast<std::size_t>(i)]->sleeping);
    }

    // A projectile dropped onto the stack wakes it.
    RigidBody& ball = world.createBody();
    ball.shape = Shape::sphere(0.4);
    ball.setMass(5.0);
    ball.setShapeInertia();
    ball.position = Vec3(0.0, 6.0, 0.0);
    ball.restitution = 0.2;
    ball.collisionEnabled = true;
    ball.updateCache();

    bool anyAwake = false;
    for (int i = 0; i < 2000 && !anyAwake; ++i) {
        world.step(1.0 / 1000.0);
        for (RigidBody* box : boxes) {
            anyAwake = anyAwake || !box->sleeping;
        }
    }
    MESSAGE("stack woken by projectile: top box sleeping = " << boxes.back()->sleeping << ", awake = " << anyAwake);
    CHECK(anyAwake);
    // The impact pushed the top of the stack down (the boxes moved).
    CHECK(boxes.back()->position.y != frozen.back().y);
}

TEST_CASE("a block on a ramp holds while tan(theta) <= mu and slides otherwise") {
    const Real mu = 0.5;
    const auto slides = [&](Real slopeTangent) {
        PhysicsWorld world;
        const Real theta = std::atan(slopeTangent);
        const Quat rampRotation = quaternionFromAxisAngle(Vec3(0.0, 0.0, 1.0), theta);
        const Vec3 normal = rotateVector(rampRotation, Vec3(0.0, 1.0, 0.0));

        RigidBody& ramp = world.createBody();
        ramp.shape = Shape::box(Vec3(5.0, 0.5, 5.0));
        ramp.setType(BodyType::Static);
        ramp.setMass(0.0);
        ramp.orientation = rampRotation;
        ramp.position = -normal * 0.5; // the ramp's top face passes through the origin
        ramp.friction = mu;
        ramp.collisionEnabled = true;
        ramp.updateCache();

        RigidBody& block = world.createBody();
        block.shape = Shape::box(Vec3(0.5));
        block.setMass(1.0);
        block.setShapeInertia();
        block.orientation = rampRotation;
        block.position = normal * 0.5;
        block.friction = mu;
        block.collisionEnabled = true;
        block.updateCache();
        const Vec3 start = block.position;

        for (int i = 0; i < 2000; ++i) {
            world.step(1.0 / 1000.0);
        }
        MESSAGE("  slope " << slopeTangent << ": displacement = " << glm::length(block.position - start) << " m, |v| = "
                           << glm::length(block.linearVelocity) << " m/s, sleeping = " << block.sleeping);
        return glm::length(block.position - start);
    };

    const Real holding = slides(0.4);  // tan(theta) = 0.4 < mu = 0.5
    const Real slipping = slides(0.7); // tan(theta) = 0.7 > mu = 0.5
    MESSAGE("block on ramp: tan=0.4 -> drift " << holding << " m, tan=0.7 -> drift " << slipping << " m");
    CHECK(holding < 0.01);
    CHECK(slipping > 0.1);
}

TEST_CASE("an elastic collision chain conserves momentum (Newton's cradle)") {
    PhysicsWorld world;
    world.gravity = Vec3(0.0); // free space: no external force along the row

    constexpr Real kRadius = 0.5;
    constexpr Real kGap = 0.01;
    constexpr int kBalls = 3;
    constexpr Real kSpeed = 2.0;

    std::vector<RigidBody*> balls;
    for (int i = 0; i < kBalls; ++i) {
        RigidBody& ball = world.createBody();
        ball.shape = Shape::sphere(kRadius);
        ball.setMass(1.0);
        ball.setShapeInertia();
        ball.position = Vec3(static_cast<Real>(i) * (2.0 * kRadius + kGap), 0.0, 0.0);
        ball.restitution = 1.0; // elastic
        ball.friction = 0.0;
        ball.collisionEnabled = true;
        ball.updateCache();
        balls.push_back(&ball);
    }
    balls.front()->linearVelocity = Vec3(kSpeed, 0.0, 0.0);

    const auto momentum = [&]() {
        Real p = 0.0;
        for (RigidBody* ball : balls) {
            p += ball->mass * ball->linearVelocity.x;
        }
        return p;
    };
    const Real before = momentum();

    for (int i = 0; i < 4000; ++i) {
        world.step(1.0 / 1000.0);
    }

    const Real after = momentum();
    MESSAGE("cradle momentum before = " << before << ", after = " << after << "; vx = " << balls[0]->linearVelocity.x
                                        << ", " << balls[1]->linearVelocity.x << ", " << balls[2]->linearVelocity.x);
    CHECK(std::abs(after - before) < 1e-6);
    // The impulse travels down the row: the struck-end ball stops, the far one leaves.
    CHECK(std::abs(balls[0]->linearVelocity.x) < 0.05);
    CHECK(std::abs(balls[1]->linearVelocity.x) < 0.05);
    CHECK(balls[2]->linearVelocity.x > 0.9 * kSpeed);
}

TEST_CASE("detectCollisions returns deterministic ordered manifolds") {
    PhysicsWorld world;
    RigidBody& ground = world.createBody();
    ground.shape = Shape::box(Vec3(5.0, 0.5, 5.0));
    ground.setType(BodyType::Static);
    ground.setMass(0.0);
    ground.position = Vec3(0.0, -0.5, 0.0);
    ground.collisionEnabled = true;
    ground.updateCache();

    RigidBody& box = world.createBody();
    box.shape = Shape::box(Vec3(0.5));
    box.setMass(1.0);
    box.position = Vec3(0.0, 0.3, 0.0);
    box.collisionEnabled = true;
    box.updateCache();

    const std::vector<ContactManifold> first = detectCollisions(world);
    REQUIRE(first.size() == 1);
    CHECK(first[0].bodyA == 0);
    CHECK(first[0].bodyB == 1);
    CHECK(first[0].count >= 1);
    // Ground top is y = 0, the box bottom is y = -0.2: penetration 0.2.
    CHECK(first[0].points[0].penetration == doctest::Approx(0.2));
    CHECK(first[0].points[0].normal.y == doctest::Approx(1.0));

    const std::vector<ContactManifold> second = detectCollisions(world);
    REQUIRE(second.size() == first.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        CHECK(second[i].bodyA == first[i].bodyA);
        CHECK(second[i].bodyB == first[i].bodyB);
        CHECK(second[i].count == first[i].count);
    }
}

namespace {

// Independent triangle scan used to validate the indexed lookup in
// Heightfield::sampleHeight.
Real bruteForceHeight(const Heightfield& field, Real x, Real z) {
    const Real fx = (x - field.origin.x) / field.spacing;
    const Real fz = (z - field.origin.z) / field.spacing;
    const int ix = static_cast<int>(std::floor(fx));
    const int iz = static_cast<int>(std::floor(fz));
    const Real u = fx - static_cast<Real>(ix);
    const Real v = fz - static_cast<Real>(iz);
    const Real h00 = field.heightAt(ix, iz);
    const Real h10 = field.heightAt(ix + 1, iz);
    const Real h01 = field.heightAt(ix, iz + 1);
    const Real h11 = field.heightAt(ix + 1, iz + 1);
    if (u + v <= 1.0) {
        return h00 + u * (h10 - h00) + v * (h11 - h10);
    }
    return h00 + u * (h11 - h01) + v * (h01 - h00);
}

Heightfield makeTestField(int width, int depth, Real spacing) {
    Heightfield field;
    field.width = width;
    field.depth = depth;
    field.spacing = spacing;
    field.origin = Vec3(0.0);
    field.heights.resize(static_cast<std::size_t>(width) * depth);
    for (int iz = 0; iz < depth; ++iz) {
        for (int ix = 0; ix < width; ++ix) {
            // A deterministic, non-planar but smooth height field.
            field.heights[static_cast<std::size_t>(iz) * width + ix] =
                std::sin(0.3 * static_cast<Real>(ix)) + 0.5 * std::cos(0.4 * static_cast<Real>(iz));
        }
    }
    return field;
}

} // namespace

TEST_CASE("heightfield sampling matches a brute-force triangle scan") {
    const Heightfield field = makeTestField(12, 9, 0.5);
    REQUIRE(field.valid());

    // Exact at the lattice nodes.
    for (int iz = 0; iz < field.depth; ++iz) {
        for (int ix = 0; ix < field.width; ++ix) {
            const Real x = field.origin.x + ix * field.spacing;
            const Real z = field.origin.z + iz * field.spacing;
            CHECK(field.sampleHeight(x, z) == doctest::Approx(field.heightAt(ix, iz)));
        }
    }

    // Interior points agree with the independent scan to machine precision.
    for (int i = 0; i < 200; ++i) {
        const Real x = 0.37 + 0.11 * static_cast<Real>(i);
        const Real z = 0.23 + 0.07 * static_cast<Real>(i);
        if (!field.contains(x, z) || x >= field.origin.x + (field.width - 1) * field.spacing ||
            z >= field.origin.z + (field.depth - 1) * field.spacing) {
            continue;
        }
        CHECK(field.sampleHeight(x, z) == doctest::Approx(bruteForceHeight(field, x, z)).epsilon(1e-12));
    }
}

TEST_CASE("heightfield normals are flat on a plane and tilted on a ramp") {
    Heightfield flat;
    flat.width = 4;
    flat.depth = 4;
    flat.spacing = 1.0;
    flat.heights.assign(16, 3.0);
    REQUIRE(flat.valid());
    CHECK(flat.sampleNormal(1.3, 2.7).y == doctest::Approx(1.0));
    CHECK(std::abs(flat.sampleNormal(1.3, 2.7).x) < 1e-12);
    CHECK(std::abs(flat.sampleNormal(1.3, 2.7).z) < 1e-12);

    // A uniform slope dh/dx = a gives the normal normalize(-a, 1, 0).
    const Real slope = 0.25;
    Heightfield ramp;
    ramp.width = 5;
    ramp.depth = 3;
    ramp.spacing = 1.0;
    ramp.heights.resize(15);
    for (int iz = 0; iz < ramp.depth; ++iz) {
        for (int ix = 0; ix < ramp.width; ++ix) {
            ramp.heights[static_cast<std::size_t>(iz) * ramp.width + ix] = slope * static_cast<Real>(ix);
        }
    }
    REQUIRE(ramp.valid());
    const Vec3 expected = glm::normalize(Vec3(-slope, 1.0, 0.0));
    const Vec3 normal = ramp.sampleNormal(2.4, 1.1);
    CHECK(normal.x == doctest::Approx(expected.x));
    CHECK(normal.y == doctest::Approx(expected.y));
    CHECK(normal.z == doctest::Approx(expected.z));
}

namespace {

std::shared_ptr<const Heightfield> flatField(int size, Real spacing, Real height) {
    auto field = std::make_shared<Heightfield>();
    field->width = size;
    field->depth = size;
    field->spacing = spacing;
    field->origin = Vec3(0.0);
    field->heights.assign(static_cast<std::size_t>(size) * size, height);
    return field;
}

} // namespace

TEST_CASE("heightfield AABB covers the grid footprint and height range") {
    const std::shared_ptr<const Heightfield> field = flatField(6, 2.0, 3.0);
    RigidBody terrain;
    terrain.shape = Shape::terrain(field);
    terrain.setType(BodyType::Static);
    terrain.updateCache();
    const Aabb box = computeAabb(terrain);
    CHECK(box.min == Vec3(0.0, 3.0, 0.0));
    CHECK(box.max == Vec3(10.0, 3.0, 10.0));
}

TEST_CASE("a sphere contacts a flat heightfield") {
    const std::shared_ptr<const Heightfield> field = flatField(5, 1.0, 0.0);
    RigidBody terrain;
    terrain.shape = Shape::terrain(field);
    terrain.setType(BodyType::Static);
    terrain.updateCache();

    RigidBody sphere = makeSphere(0.5, Vec3(2.0, 0.4, 2.0));

    ContactPoint points[kMaxContactPoints];
    const int count = collide(terrain, sphere, points, kMaxContactPoints);
    REQUIRE(count == 1);
    CHECK(points[0].penetration == doctest::Approx(0.1));
    CHECK(points[0].normal == Vec3(0.0, 1.0, 0.0));

    // Flipped order keeps the normal pointing from A to B.
    ContactPoint flipped[kMaxContactPoints];
    const int flippedCount = collide(sphere, terrain, flipped, kMaxContactPoints);
    REQUIRE(flippedCount == 1);
    CHECK(flipped[0].normal == Vec3(0.0, -1.0, 0.0));
}

TEST_CASE("a box rests on a flat heightfield with four contacts") {
    const std::shared_ptr<const Heightfield> field = flatField(6, 1.0, 0.0);
    RigidBody terrain;
    terrain.shape = Shape::terrain(field);
    terrain.setType(BodyType::Static);
    terrain.updateCache();

    RigidBody box = makeBox(Vec3(0.5), Vec3(2.5, 0.4, 2.5));

    ContactPoint points[kMaxContactPoints];
    const int count = collide(terrain, box, points, kMaxContactPoints);
    REQUIRE(count == 4);
    for (int i = 0; i < count; ++i) {
        CHECK(points[i].penetration == doctest::Approx(0.1));
        CHECK(points[i].normal == Vec3(0.0, 1.0, 0.0));
    }
}

TEST_CASE("heightfield contacts follow a slope normal") {
    auto field = std::make_shared<Heightfield>();
    field->width = 6;
    field->depth = 6;
    field->spacing = 1.0;
    field->origin = Vec3(0.0);
    field->heights.resize(36);
    const Real slope = 0.3;
    for (int iz = 0; iz < 6; ++iz) {
        for (int ix = 0; ix < 6; ++ix) {
            field->heights[static_cast<std::size_t>(iz) * 6 + ix] = slope * static_cast<Real>(ix);
        }
    }
    RigidBody terrain;
    terrain.shape = Shape::terrain(field);
    terrain.setType(BodyType::Static);
    terrain.updateCache();

    // A sphere just touching the ramp: the normal is the surface normal.
    const Real surface = field->sampleHeight(3.0, 3.0);
    RigidBody sphere = makeSphere(0.5, Vec3(3.0, surface + 0.4, 3.0));
    ContactPoint points[kMaxContactPoints];
    const int count = collide(terrain, sphere, points, kMaxContactPoints);
    REQUIRE(count == 1);
    const Vec3 expected = glm::normalize(Vec3(-slope, 1.0, 0.0));
    CHECK(points[0].normal.x == doctest::Approx(expected.x));
    CHECK(points[0].normal.y == doctest::Approx(expected.y));
    CHECK(points[0].normal.z == doctest::Approx(expected.z));
}

TEST_CASE("a sphere falls onto terrain and comes to rest") {
    auto field = std::make_shared<Heightfield>();
    field->width = 8;
    field->depth = 8;
    field->spacing = 1.0;
    field->origin = Vec3(0.0);
    field->heights.assign(64, 0.0);

    PhysicsWorld world;
    RigidBody& terrain = world.createBody();
    terrain.shape = Shape::terrain(field);
    terrain.setType(BodyType::Static);
    terrain.setMass(0.0);
    terrain.collisionEnabled = true;
    terrain.updateCache();

    RigidBody& sphere = world.createBody();
    sphere.shape = Shape::sphere(0.5);
    sphere.setType(BodyType::Dynamic);
    sphere.setMass(1.0);
    sphere.setShapeInertia();
    sphere.position = Vec3(3.5, 2.0, 3.5);
    sphere.collisionEnabled = true;
    sphere.updateCache();

    for (int i = 0; i < 4000; ++i) {
        world.step(1.0 / 1000.0);
    }

    MESSAGE("sphere resting y = " << sphere.position.y << " (contact slop at r = 0.5)");
    CHECK(sphere.position.y == doctest::Approx(0.5).epsilon(0.05));
    CHECK(std::abs(sphere.linearVelocity.y) < 0.05);
}

TEST_CASE("supportWorld returns the extreme point of each shape") {
    const RigidBody box = makeBox(Vec3(1.0, 2.0, 3.0), Vec3(1.0, 0.0, 0.0));
    CHECK(supportWorld(box, Vec3(1.0, 1.0, 1.0)) == Vec3(2.0, 2.0, 3.0));
    CHECK(supportWorld(box, Vec3(-1.0, -1.0, -1.0)) == Vec3(0.0, -2.0, -3.0));

    const RigidBody sphere = makeSphere(0.5, Vec3(0.0, 0.0, 0.0));
    CHECK(supportWorld(sphere, Vec3(0.0, 5.0, 0.0)) == Vec3(0.0, 0.5, 0.0));

    const RigidBody cylinder = makeCylinder(0.5, 1.0, Vec3(0.0));
    const Vec3 top = supportWorld(cylinder, Vec3(0.0, 1.0, 0.0));
    CHECK(top.y == doctest::Approx(1.0));

    const RigidBody hull = [&] {
        RigidBody body;
        body.shape = Shape::convex({{1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}});
        body.updateCache();
        return body;
    }();
    CHECK(supportWorld(hull, Vec3(0.0, 0.0, 2.0)) == Vec3(0.0, 0.0, 1.0));
    CHECK(supportWorld(hull, Vec3(-3.0, 0.0, 0.0)) == Vec3(-1.0, 0.0, 0.0));
}

TEST_CASE("GJK separates and joins convex shapes") {
    const RigidBody a = makeSphere(0.5, Vec3(0.0));
    const RigidBody near = makeSphere(0.5, Vec3(0.8, 0.0, 0.0));
    const RigidBody far = makeSphere(0.5, Vec3(2.0, 0.0, 0.0));
    CHECK(convexOverlap(a, near));
    CHECK_FALSE(convexOverlap(a, far));

    const RigidBody boxA = makeBox(Vec3(0.5), Vec3(0.0));
    const RigidBody boxNear = makeBox(Vec3(0.5), Vec3(0.6, 0.0, 0.0));
    const RigidBody boxFar = makeBox(Vec3(0.5), Vec3(2.0, 0.0, 0.0));
    CHECK(convexOverlap(boxA, boxNear));
    CHECK_FALSE(convexOverlap(boxA, boxFar));

    const RigidBody cylinderA = makeCylinder(0.5, 0.5, Vec3(0.0));
    const RigidBody cylinderNear = makeCylinder(0.5, 0.5, Vec3(0.6, 0.0, 0.0));
    CHECK(convexOverlap(cylinderA, cylinderNear));
    CHECK_FALSE(convexOverlap(cylinderA, makeCylinder(0.5, 0.5, Vec3(1.5, 0.0, 0.0))));
}

TEST_CASE("EPA reports the penetration depth and normal of convex shapes") {
    const RigidBody sphereA = makeSphere(0.5, Vec3(0.0));
    const RigidBody sphereB = makeSphere(0.5, Vec3(0.8, 0.0, 0.0));
    {
        const ConvexContact contact = convexContact(sphereA, sphereB);
        REQUIRE(contact.intersecting);
        CHECK(contact.depth == doctest::Approx(0.2).epsilon(1e-3));
        CHECK(contact.normal.x == doctest::Approx(1.0).epsilon(1e-3));
    }
    const RigidBody boxA = makeBox(Vec3(0.5), Vec3(0.0));
    const RigidBody boxB = makeBox(Vec3(0.5), Vec3(0.6, 0.0, 0.0));
    {
        const ConvexContact contact = convexContact(boxA, boxB);
        REQUIRE(contact.intersecting);
        CHECK(contact.depth == doctest::Approx(0.4).epsilon(1e-6));
        CHECK(contact.normal.x == doctest::Approx(1.0).epsilon(1e-6));
        // Witness points straddle the overlap: pointA - pointB = depth * normal.
        CHECK((contact.pointA - contact.pointB).x == doctest::Approx(contact.depth).epsilon(1e-6));
    }
    const RigidBody cylinderA = makeCylinder(0.5, 0.5, Vec3(0.0));
    const RigidBody cylinderB = makeCylinder(0.5, 0.5, Vec3(0.6, 0.0, 0.0));
    {
        const ConvexContact contact = convexContact(cylinderA, cylinderB);
        REQUIRE(contact.intersecting);
        CHECK(contact.depth == doctest::Approx(0.4).epsilon(1e-3));
        CHECK(contact.normal.x == doctest::Approx(1.0).epsilon(1e-3));
    }
    CHECK_FALSE(convexContact(sphereA, makeSphere(0.5, Vec3(2.0, 0.0, 0.0))).intersecting);
}

TEST_CASE("a convex hull behaves like the analytic box") {
    const RigidBody hull = [&] {
        RigidBody body;
        body.shape = Shape::convex({{0.5, 0.5, 0.5},
                                    {-0.5, 0.5, 0.5},
                                    {0.5, -0.5, 0.5},
                                    {-0.5, -0.5, 0.5},
                                    {0.5, 0.5, -0.5},
                                    {-0.5, 0.5, -0.5},
                                    {0.5, -0.5, -0.5},
                                    {-0.5, -0.5, -0.5}});
        body.updateCache();
        return body;
    }();
    const RigidBody box = makeBox(Vec3(0.5), Vec3(0.6, 0.0, 0.0));
    const ConvexContact contact = convexContact(hull, box);
    REQUIRE(contact.intersecting);
    CHECK(contact.depth == doctest::Approx(0.4).epsilon(1e-6));
    CHECK(contact.normal.x == doctest::Approx(1.0).epsilon(1e-6));

    // collide() routes hull pairs through the same generic path.
    ContactPoint points[kMaxContactPoints];
    const int count = collide(hull, box, points, kMaxContactPoints);
    REQUIRE(count == 1);
    CHECK(points[0].penetration == doctest::Approx(0.4).epsilon(1e-6));
    CHECK(points[0].normal.x == doctest::Approx(1.0).epsilon(1e-6));

    // A separated hull produces no contact.
    CHECK(collide(hull, makeBox(Vec3(0.5), Vec3(3.0, 0.0, 0.0)), points, kMaxContactPoints) == 0);
}

namespace {

std::shared_ptr<const TriangleMesh> unitQuad() {
    auto mesh = std::make_shared<TriangleMesh>();
    mesh->vertices = {Vec3(-1.0, 0.0, -1.0), Vec3(1.0, 0.0, -1.0), Vec3(1.0, 0.0, 1.0), Vec3(-1.0, 0.0, 1.0)};
    // Wound so the outward normal is +Y.
    mesh->indices = {0, 2, 1, 0, 3, 2};
    return mesh;
}

} // namespace

TEST_CASE("a sphere and a box rest on a triangle-mesh plane") {
    RigidBody ground;
    ground.shape = Shape::triangleMesh(unitQuad());
    ground.setType(BodyType::Static);
    ground.updateCache();

    // AABB matches the quad.
    const Aabb box = computeAabb(ground);
    CHECK(box.min == Vec3(-1.0, 0.0, -1.0));
    CHECK(box.max == Vec3(1.0, 0.0, 1.0));

    RigidBody sphere = makeSphere(0.5, Vec3(0.0, 0.4, 0.0));
    ContactPoint points[kMaxContactPoints];
    int count = collide(ground, sphere, points, kMaxContactPoints);
    REQUIRE(count == 1);
    CHECK(points[0].penetration == doctest::Approx(0.1));
    CHECK(points[0].normal == Vec3(0.0, 1.0, 0.0));

    // Flipped order flips the normal.
    CHECK(collide(sphere, ground, points, kMaxContactPoints) == 1);
    CHECK(points[0].normal == Vec3(0.0, -1.0, 0.0));

    // A box above the plane rests on its four bottom corners.
    RigidBody resting = makeBox(Vec3(0.5), Vec3(0.0, 0.4, 0.0));
    const int boxCount = collide(ground, resting, points, kMaxContactPoints);
    REQUIRE(boxCount == 4);
    for (int i = 0; i < boxCount; ++i) {
        CHECK(points[i].penetration == doctest::Approx(0.1));
        CHECK(points[i].normal == Vec3(0.0, 1.0, 0.0));
    }
}

TEST_CASE("triangle-mesh contacts reject separated shapes") {
    RigidBody ground;
    ground.shape = Shape::triangleMesh(unitQuad());
    ground.setType(BodyType::Static);
    ground.updateCache();

    ContactPoint points[kMaxContactPoints];
    CHECK(collide(ground, makeSphere(0.5, Vec3(0.0, 2.0, 0.0)), points, kMaxContactPoints) == 0);
    CHECK(collide(ground, makeBox(Vec3(0.5), Vec3(0.0, 3.0, 0.0)), points, kMaxContactPoints) == 0);
}

TEST_CASE("heightfield footprint and bounds follow the grid") {
    const Heightfield field = makeTestField(6, 4, 2.0);
    REQUIRE(field.valid());
    CHECK(field.contains(0.0, 0.0));
    CHECK(field.contains(10.0, 6.0));
    CHECK_FALSE(field.contains(-0.1, 0.0));
    CHECK_FALSE(field.contains(10.1, 6.0));

    Real lowest = field.heightAt(0, 0);
    Real highest = field.heightAt(0, 0);
    for (Real h : field.heights) {
        lowest = std::min(lowest, h);
        highest = std::max(highest, h);
    }
    CHECK(field.minCorner() == Vec3(0.0, lowest, 0.0));
    CHECK(field.maxCorner() == Vec3(10.0, highest, 6.0));
}

TEST_CASE("speculative contacts report the open gap within the margin") {
    ContactPoint points[kMaxContactPoints];

    // Sphere-sphere: 0.9 m apart, so the gap is 0.9 for radii of 0.5.
    const RigidBody sphereA = makeSphere(0.5, Vec3(0.0));
    const RigidBody sphereB = makeSphere(0.5, Vec3(1.9, 0.0, 0.0));
    CHECK(collide(sphereA, sphereB, points, kMaxContactPoints, 0.0) == 0);
    REQUIRE(collide(sphereA, sphereB, points, kMaxContactPoints, 1.0) == 1);
    CHECK(points[0].penetration == doctest::Approx(-0.9));
    CHECK(points[0].normal == Vec3(1.0, 0.0, 0.0));
    CHECK(collide(sphereA, sphereB, points, kMaxContactPoints, 0.5) == 0);

    // Sphere-box: the gap is 0.2 (centre 1.2, half extent 0.5, radius 0.5).
    const RigidBody box = makeBox(Vec3(0.5), Vec3(0.0));
    const RigidBody sphere = makeSphere(0.5, Vec3(-1.2, 0.0, 0.0));
    CHECK(collide(sphere, box, points, kMaxContactPoints, 0.1) == 0);
    REQUIRE(collide(sphere, box, points, kMaxContactPoints, 0.3) == 1);
    CHECK(points[0].penetration == doctest::Approx(-0.2));
    CHECK(points[0].normal == Vec3(1.0, 0.0, 0.0));

    // Box-box: a 1.0 m face gap along X becomes a single speculative contact.
    const RigidBody boxB = makeBox(Vec3(0.5), Vec3(2.0, 0.0, 0.0));
    CHECK(collide(box, boxB, points, kMaxContactPoints, 0.5) == 0);
    REQUIRE(collide(box, boxB, points, kMaxContactPoints, 1.2) == 1);
    CHECK(points[0].penetration == doctest::Approx(-1.0));
    CHECK(points[0].normal == Vec3(1.0, 0.0, 0.0));

    // A real overlap keeps the multi-point manifold and positive penetration.
    const RigidBody boxOverlap = makeBox(Vec3(0.5), Vec3(0.8, 0.0, 0.0));
    CHECK(collide(box, boxOverlap, points, kMaxContactPoints, 0.5) == 4);
}

TEST_CASE("speculative contacts reach boxes and spheres near a heightfield") {
    const std::shared_ptr<const Heightfield> field = flatField(6, 1.0, 0.0);
    RigidBody terrain;
    terrain.shape = Shape::terrain(field);
    terrain.setType(BodyType::Static);
    terrain.updateCache();

    ContactPoint points[kMaxContactPoints];

    // A sphere whose surface is 0.1 above the ground (centre at 0.6).
    const RigidBody sphere = makeSphere(0.5, Vec3(2.5, 0.6, 2.5));
    CHECK(collide(terrain, sphere, points, kMaxContactPoints, 0.05) == 0);
    REQUIRE(collide(terrain, sphere, points, kMaxContactPoints, 0.2) == 1);
    CHECK(points[0].penetration == doctest::Approx(-0.1));
    CHECK(points[0].normal == Vec3(0.0, 1.0, 0.0));

    // A box whose bottom face is 0.2 above the ground (centre at 0.7).
    const RigidBody box = makeBox(Vec3(0.5), Vec3(2.5, 0.7, 2.5));
    CHECK(collide(terrain, box, points, kMaxContactPoints, 0.1) == 0);
    REQUIRE(collide(terrain, box, points, kMaxContactPoints, 0.3) == 4);
    CHECK(points[0].penetration == doctest::Approx(-0.2));
}

TEST_CASE("a fast sphere cannot tunnel through a thin static wall") {
    const auto run = [](bool speculative) {
        PhysicsWorld world;
        world.gravity = Vec3(0.0);
        world.enableSpeculativeContacts = speculative;
        world.enableSleeping = false;

        RigidBody& wall = world.createBody();
        wall.shape = Shape::box(Vec3(0.05, 2.0, 2.0));
        wall.setType(BodyType::Static);
        wall.setMass(0.0);
        wall.position = Vec3(0.0);
        wall.collisionEnabled = true;
        wall.updateCache();

        RigidBody& sphere = world.createBody();
        sphere.shape = Shape::sphere(0.1);
        sphere.setMass(1.0);
        sphere.setShapeInertia();
        sphere.position = Vec3(-0.2, 0.0, 0.0);
        sphere.linearVelocity = Vec3(400.0, 0.0, 0.0);
        sphere.collisionEnabled = true;
        sphere.updateCache();

        for (int i = 0; i < 30; ++i) {
            world.step(1.0 / 1000.0);
        }
        return sphere.position.x;
    };

    // 400 m/s over a 1 ms step is 0.4 m, far more than the 0.1 m wall, so
    // without speculative contacts the sphere passes straight through.
    CHECK(run(false) > 0.05);
    // With speculative contacts it is stopped on the near side of the wall.
    const Real stopped = run(true);
    CHECK(stopped < 0.0);
    CHECK(stopped > -0.2);
}

TEST_CASE("speculative contacts do not perturb a resting pair") {
    auto field = std::make_shared<Heightfield>();
    field->width = 6;
    field->depth = 6;
    field->spacing = 1.0;
    field->heights.assign(36, 0.0);

    PhysicsWorld world;
    RigidBody& terrain = world.createBody();
    terrain.shape = Shape::terrain(field);
    terrain.setType(BodyType::Static);
    terrain.setMass(0.0);
    terrain.collisionEnabled = true;
    terrain.updateCache();

    RigidBody& box = world.createBody();
    box.shape = Shape::box(Vec3(0.5));
    box.setMass(1.0);
    box.setShapeInertia();
    box.position = Vec3(2.5, 0.5, 2.5);
    box.collisionEnabled = true;
    box.updateCache();

    for (int i = 0; i < 500; ++i) {
        world.step(1.0 / 1000.0);
    }
    REQUIRE(std::abs(box.linearVelocity.x) < 1e-6);
    REQUIRE(std::abs(box.linearVelocity.y) < 1e-6);

    // A pair at rest has ~zero relative speed, so the velocity margin adds
    // nothing and the manifold set is identical to the plain detection.
    const std::vector<ContactManifold> plain = detectCollisions(world);
    const std::vector<ContactManifold> speculative = detectCollisions(world, 0.0, 1.0 / 1000.0);
    REQUIRE(plain.size() == speculative.size());
    for (std::size_t i = 0; i < plain.size(); ++i) {
        CHECK(plain[i].bodyA == speculative[i].bodyA);
        CHECK(plain[i].bodyB == speculative[i].bodyB);
        CHECK(plain[i].count == speculative[i].count);
    }
}

namespace {

// A deterministic pseudo-random source, so the brute-force cross-check below is
// reproducible across platforms.
struct Lcg {
    std::uint64_t state = 0x9e3779b97f4a7c15ull;

    Real next() {
        state = state * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<Real>((state >> 11) & ((1ull << 53) - 1)) / static_cast<Real>(1ull << 53);
    }
};

// A wavy grid mesh, the shape a track surface has: smooth, but built from many
// small triangles.
TriangleMesh makeWaveMesh(int cells) {
    TriangleMesh mesh;
    const Real half = 20.0;
    const Real step = 2.0 * half / static_cast<Real>(cells);
    for (int j = 0; j <= cells; ++j) {
        for (int i = 0; i <= cells; ++i) {
            const Real x = -half + step * static_cast<Real>(i);
            const Real z = -half + step * static_cast<Real>(j);
            const Real y = 0.5 * std::sin(0.3 * x) * std::cos(0.2 * z);
            mesh.vertices.push_back(Vec3(x, y, z));
        }
    }
    for (int j = 0; j < cells; ++j) {
        for (int i = 0; i < cells; ++i) {
            const auto a = static_cast<std::uint32_t>(j * (cells + 1) + i);
            const auto b = static_cast<std::uint32_t>(a + 1);
            const auto c = static_cast<std::uint32_t>(a + (cells + 1) + 1);
            const auto d = static_cast<std::uint32_t>(a + (cells + 1));
            mesh.indices.insert(mesh.indices.end(), {a, b, c, a, c, d});
        }
    }
    return mesh;
}

// The reference Moller-Trumbore test, independent of the BVH implementation.
bool bruteTriangle(const Vec3& origin, const Vec3& direction, const Vec3& a, const Vec3& b, const Vec3& c, Real& t) {
    const Vec3 edge1 = b - a;
    const Vec3 edge2 = c - a;
    const Vec3 pvec = glm::cross(direction, edge2);
    const Real determinant = glm::dot(edge1, pvec);
    if (std::abs(determinant) < 1e-12) {
        return false;
    }
    const Real inverse = 1.0 / determinant;
    const Vec3 tvec = origin - a;
    const Real u = glm::dot(tvec, pvec) * inverse;
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    const Vec3 qvec = glm::cross(tvec, edge1);
    const Real v = glm::dot(direction, qvec) * inverse;
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    t = glm::dot(edge2, qvec) * inverse;
    return t >= 0.0;
}

} // namespace

TEST_CASE("triangle-mesh BVH: a ray returns the closest triangle") {
    // Two stacked quads, at y = 0 and y = 1: a downward ray must hit the top one.
    const Real h = 2.0;
    TriangleMesh mesh;
    mesh.vertices = {Vec3(-h, 0.0, -h), Vec3(h, 0.0, -h), Vec3(h, 0.0, h), Vec3(-h, 0.0, h),
                     Vec3(-h, 1.0, -h), Vec3(h, 1.0, -h), Vec3(h, 1.0, h), Vec3(-h, 1.0, h)};
    mesh.indices = {0, 2, 1, 0, 3, 2, 4, 6, 5, 4, 7, 6};

    TriangleMeshBvh bvh;
    bvh.build(mesh);
    REQUIRE_FALSE(bvh.empty());
    CHECK(bvh.triangleCount() == 4);
    CHECK(bvh.nodeCount() >= 4);

    const MeshRayHit hit = bvh.raycast(Vec3(0.0, 5.0, 0.0), Vec3(0.0, -1.0, 0.0), 100.0);
    REQUIRE(hit.hit);
    CHECK(hit.point.y == doctest::Approx(1.0));
    CHECK(hit.distance == doctest::Approx(4.0));
    CHECK(hit.triangle >= 2); // one of the upper quads
    CHECK(hit.normal.y == doctest::Approx(1.0));

    // From below the same stack, the closest hit is the y = 0 quad and the
    // normal faces the incoming ray.
    const MeshRayHit under = bvh.raycast(Vec3(0.0, -5.0, 0.0), Vec3(0.0, 1.0, 0.0), 100.0);
    REQUIRE(under.hit);
    CHECK(under.point.y == doctest::Approx(0.0));
    CHECK(under.normal.y == doctest::Approx(-1.0));
}

TEST_CASE("triangle-mesh BVH: misses, bounds and degenerate input") {
    const Real h = 2.0;
    TriangleMesh mesh;
    mesh.vertices = {Vec3(-h, 0.0, -h), Vec3(h, 0.0, -h), Vec3(h, 0.0, h), Vec3(-h, 0.0, h)};
    mesh.indices = {0, 2, 1, 0, 3, 2};
    TriangleMeshBvh bvh;
    bvh.build(mesh);

    // A ray that stops short of the surface misses.
    CHECK_FALSE(bvh.raycast(Vec3(0.0, 5.0, 0.0), Vec3(0.0, -1.0, 0.0), 2.0).hit);
    // A ray outside the mesh's footprint misses.
    CHECK_FALSE(bvh.raycast(Vec3(100.0, 5.0, 100.0), Vec3(0.0, -1.0, 0.0), 100.0).hit);
    // A zero-length direction misses.
    CHECK_FALSE(bvh.raycast(Vec3(0.0, 5.0, 0.0), Vec3(0.0), 100.0).hit);

    // An empty (or invalid) mesh has no tree and never hits.
    TriangleMeshBvh empty;
    empty.build(TriangleMesh{});
    CHECK(empty.empty());
    CHECK(empty.nodeCount() == 0);
    CHECK_FALSE(empty.raycast(Vec3(0.0, 5.0, 0.0), Vec3(0.0, -1.0, 0.0), 100.0).hit);

    TriangleMesh broken;
    broken.vertices = mesh.vertices;
    broken.indices = {0, 1}; // not a multiple of three
    TriangleMeshBvh invalid;
    invalid.build(broken);
    CHECK(invalid.empty());
}

TEST_CASE("triangle-mesh BVH: the tree matches a brute-force raycast") {
    const TriangleMesh mesh = makeWaveMesh(24);
    TriangleMeshBvh bvh;
    bvh.build(mesh);
    REQUIRE(bvh.triangleCount() == mesh.triangleCount());

    Lcg rng;
    int hits = 0;
    for (int sample = 0; sample < 300; ++sample) {
        const Vec3 origin(-15.0 + 30.0 * rng.next(), 8.0, -15.0 + 30.0 * rng.next());
        const Vec3 direction(rng.next() - 0.5, -1.0, rng.next() - 0.5);

        const MeshRayHit viaBvh = bvh.raycast(origin, direction, 200.0);

        // Brute force over every triangle.
        MeshRayHit brute;
        for (std::size_t t = 0; t < mesh.triangleCount(); ++t) {
            Vec3 a, b, c;
            mesh.triangle(t, a, b, c);
            Real distance = 0.0;
            if (!bruteTriangle(origin, direction, a, b, c, distance)) {
                continue;
            }
            if (brute.hit && distance >= brute.distance) {
                continue;
            }
            brute.hit = true;
            brute.distance = distance;
            brute.triangle = t;
        }

        CHECK(viaBvh.hit == brute.hit);
        if (viaBvh.hit && brute.hit) {
            ++hits;
            CHECK(viaBvh.distance == doctest::Approx(brute.distance).epsilon(1e-12));
            const Vec3 expectedPoint = origin + direction * brute.distance;
            CHECK(viaBvh.point.y == doctest::Approx(expectedPoint.y).epsilon(1e-12));
            CHECK(viaBvh.point.x == doctest::Approx(expectedPoint.x).epsilon(1e-12));
            // Ties are essentially impossible with random rays, so the index
            // must agree.
            CHECK(viaBvh.triangle == brute.triangle);
        }
    }
    CHECK(hits > 250); // the rays mostly hit
}

TEST_CASE("triangle-mesh BVH: the build is deterministic") {
    const TriangleMesh mesh = makeWaveMesh(16);
    TriangleMeshBvh first;
    first.build(mesh);
    TriangleMeshBvh second;
    second.build(mesh);

    CHECK(first.nodeCount() == second.nodeCount());
    CHECK(first.maxDepth() == second.maxDepth());

    Lcg rng;
    for (int sample = 0; sample < 50; ++sample) {
        const Vec3 origin(-10.0 + 20.0 * rng.next(), 8.0, -10.0 + 20.0 * rng.next());
        const Vec3 direction(rng.next() - 0.5, -1.0, rng.next() - 0.5);
        const MeshRayHit a = first.raycast(origin, direction, 200.0);
        const MeshRayHit b = second.raycast(origin, direction, 200.0);
        CHECK(a.hit == b.hit);
        CHECK(a.triangle == b.triangle);
        CHECK(a.distance == b.distance);
    }
}
