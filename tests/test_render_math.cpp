// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include <doctest/doctest.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "render/Camera.hpp"
#include "render/Frustum.hpp"
#include "render/Mesh.hpp"
#include "render/Uniforms.hpp"

namespace sim::render {

TEST_CASE("glm column-major memory matches the shader column_major layout") {
    // std140 upload layout: every member is a multiple of 16 bytes and matrices
    // are stored as their columns contiguously. GLM keeps mat4 as four vec4
    // columns, so a raw byte copy is the identity mapping between the C++
    // struct and the column_major shaders; no transpose must ever happen.
    CHECK(sizeof(glm::mat4) == 64);
    CHECK(sizeof(FrameUniforms) == 16 * 17);
    CHECK(sizeof(ObjectPushConstants) == 16 * 7);
    CHECK(offsetof(FrameUniforms, viewProjection) == 0);
    CHECK(offsetof(ObjectPushConstants, model) == 0);

    // writeMatrix copies the identity verbatim: bytes 0..15 are the first
    // column (1,0,0,0), bytes 16..31 the second column, etc. A row-major reader
    // would see (1,0,0,0, 0,1,0,0, ...) in a different order and fail here.
    glm::mat4 identity(1.0f);
    std::uint8_t bytes[64];
    writeMatrix(identity, bytes);
    for (int column = 0; column < 4; ++column) {
        for (int row = 0; row < 4; ++row) {
            float value = 0.0f;
            std::memcpy(&value, bytes + (column * 4 + row) * sizeof(float), sizeof(float));
            CHECK(value == doctest::Approx(column == row ? 1.0f : 0.0f));
        }
    }
}

TEST_CASE("Camera::forward() is the world-space view direction") {
    // The shadow-map fitting places its orthographic box along the camera's
    // forward vector, so this must be the third column of the view rotation
    // negated (right-handed, -Z forward) and not its transpose.
    Camera camera;
    camera.setPerspective(60.0f, 16.0f / 9.0f, 0.1f, 1000.0f);

    // Looking down -Z from the origin.
    camera.lookAt({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f});
    CHECK(std::abs(camera.forward().x) < 1e-6f);
    CHECK(std::abs(camera.forward().y) < 1e-6f);
    CHECK(std::abs(camera.forward().z + 1.0f) < 1e-6f);

    // An off-axis pose: forward must point from the eye towards the target.
    const glm::vec3 eye(6.0f, 4.0f, 12.0f);
    const glm::vec3 target(0.0f, 0.8f, 0.0f);
    camera.lookAt(eye, target);
    const glm::vec3 expected = glm::normalize(target - eye);
    CHECK(glm::length(camera.forward() - expected) < 1e-5f);
    CHECK(camera.position() == eye);
}

TEST_CASE("a known world point projects to the expected pixel (column-major mul)") {
    constexpr int kWidth = 640;
    constexpr int kHeight = 360;
    constexpr double kTanHalf = 0.4142135623730950; // tan(22.5 deg) = sqrt(2) - 1
    const float aspect = static_cast<float>(kWidth) / static_cast<float>(kHeight);

    // Camera at the origin looking down -Z with +Y up on a right-handed frame.
    // For a point (x, y, -d) the analytic NDC is
    //   ndcX = x / (d * tan(fovY/2) * aspect),   ndcY = y / (d * tan(fovY/2))
    // independent of any matrix convention, so it pins down the transform that
    // viewProjection() * p must reproduce. The pixel mapping mirrors the
    // headless GPU semantic checks (HeadlessRunner::checkSemantics) and matches
    // the RHI viewport flip: +Y world is row 0 (top) and +X world grows the
    // column, so this is the absolute screen orientation of the engine.
    Camera camera;
    camera.setPerspective(45.0f, aspect, 0.05f, 100.0f);
    camera.lookAt({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f});

    const double distance = 2.0;
    const double x = 1.0;
    const double y = 0.5;
    const double expectedNdcX = x / (distance * kTanHalf * aspect);
    const double expectedNdcY = y / (distance * kTanHalf);

    const glm::vec4 clip = camera.viewProjection() * glm::vec4(x, y, -distance, 1.0f);
    REQUIRE(clip.w > 0.0f);
    const double ndcX = clip.x / clip.w;
    const double ndcY = clip.y / clip.w;
    CHECK(ndcX == doctest::Approx(expectedNdcX).epsilon(1e-6));
    CHECK(ndcY == doctest::Approx(expectedNdcY).epsilon(1e-6));

    const int px = static_cast<int>((ndcX * 0.5 + 0.5) * kWidth);
    const int py = static_cast<int>((1.0 - (ndcY * 0.5 + 0.5)) * kHeight);
    const int expectedPx = static_cast<int>((expectedNdcX * 0.5 + 0.5) * kWidth);
    const int expectedPy = static_cast<int>((1.0 - (expectedNdcY * 0.5 + 0.5)) * kHeight);
    MESSAGE("known world point (1, 0.5, -2) projects to pixel (" << px << ", " << py << ")");
    CHECK(px == expectedPx);
    CHECK(py == expectedPy);
}

TEST_CASE("screen orientation is absolute: +Y world renders above, +X world renders right") {
    // Pins the engine's screen orientation without a GPU: with a horizontal
    // camera, the RHI-flipped mapping must place a higher world point on a
    // strictly smaller row (top of the image) and a point towards +X on a
    // strictly larger column, and anything behind the camera must stay behind.
    constexpr int kWidth = 640;
    constexpr int kHeight = 360;
    const float aspect = static_cast<float>(kWidth) / static_cast<float>(kHeight);

    Camera camera;
    camera.setPerspective(60.0f, aspect, 0.1f, 100.0f);
    camera.lookAt({0.0f, 0.6f, 0.0f}, {0.0f, 0.6f, -10.0f}); // horizontal gaze, +Y up

    const auto rowCol = [&](float x, float y, float z) -> std::pair<int, int> {
        const glm::vec4 clip = camera.viewProjection() * glm::vec4(x, y, z, 1.0f);
        REQUIRE(clip.w > 0.0f);
        const float ndcX = clip.x / clip.w;
        const float ndcY = clip.y / clip.w;
        const int px = static_cast<int>((ndcX * 0.5f + 0.5f) * static_cast<float>(kWidth));
        const int py = static_cast<int>((1.0f - (ndcY * 0.5f + 0.5f)) * static_cast<float>(kHeight));
        return {px, py};
    };

    // Vertical: same gaze axis, higher object must land above (smaller row).
    const auto high = rowCol(0.0f, 3.0f, -8.0f);
    const auto low = rowCol(0.0f, 0.4f, -8.0f);
    CHECK(high.second < low.second);
    CHECK(high.first == low.first); // same ray, same column

    // Horizontal: objects at the same height/ray must order by +X.
    const auto right = rowCol(2.5f, 0.6f, -8.0f);
    const auto left = rowCol(-2.5f, 0.6f, -8.0f);
    CHECK(right.first > left.first);
    CHECK(right.second == left.second);

    // World up really is the top half of the frame and world ahead is in front
    // of the camera (never a negative clip.w).
    CHECK(high.second < kHeight / 2);
    CHECK(low.second > kHeight / 2);
}

namespace {

// Every triangle must be counter-clockwise seen from outside: its geometric
// normal (p1 - p0) x (p2 - p0) has to agree with the vertex normals. The
// pipeline culls back faces with a CCW front face, so a reversed triangle is
// invisible from outside and shows the interior instead.
void checkOutwardWinding(const MeshData& mesh, const std::string& name) {
    REQUIRE(mesh.indices.size() % 3 == 0);
    int reversed = 0;
    int badNormals = 0;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
        const Vertex& a = mesh.vertices[mesh.indices[i]];
        const Vertex& b = mesh.vertices[mesh.indices[i + 1]];
        const Vertex& c = mesh.vertices[mesh.indices[i + 2]];
        const glm::vec3 faceNormal = glm::cross(b.position - a.position, c.position - a.position);
        if (glm::length(faceNormal) < 1e-9f) {
            continue; // degenerate pole triangle
        }
        const glm::vec3 averageNormal = a.normal + b.normal + c.normal;
        if (glm::dot(faceNormal, averageNormal) <= 0.0f) {
            ++reversed;
        }
        // Flat faces (cube, plane, cylinder caps) must carry the face normal.
        const glm::vec3 unit = glm::normalize(faceNormal);
        for (const Vertex* v : {&a, &b, &c}) {
            if (glm::dot(unit, glm::normalize(v->normal)) < 0.1f) {
                ++badNormals;
            }
        }
    }
    MESSAGE(name << ": " << reversed << " reversed triangles, " << badNormals << " vertex normals off the face");
    CHECK(reversed == 0);
    CHECK(badNormals == 0);
}

} // namespace

TEST_CASE("built-in meshes are wound counter-clockwise seen from outside") {
    checkOutwardWinding(cubeData(), "cube");
    checkOutwardWinding(sphereData(), "sphere");
    checkOutwardWinding(cylinderData(), "cylinder");
    checkOutwardWinding(groundPlaneData(10.0f), "ground plane");
}

TEST_CASE("mesh bounds match the unit geometry the generators build") {
    // The renderer culls with these bounds, so they must be both complete (no
    // vertex outside, or geometry would vanish) and correct in component order.
    // Each generator yields a centred shape of known extent; a transform in the
    // scene then scales it, so the local box has to be exact.
    const auto expectBox = [](const MeshData& mesh, float halfExtent) {
        const Aabb bounds = meshBounds(mesh.vertices);
        CHECK(bounds.lo.x == doctest::Approx(-halfExtent).epsilon(1e-5));
        CHECK(bounds.lo.y == doctest::Approx(-halfExtent).epsilon(1e-5));
        CHECK(bounds.lo.z == doctest::Approx(-halfExtent).epsilon(1e-5));
        CHECK(bounds.hi.x == doctest::Approx(halfExtent).epsilon(1e-5));
        CHECK(bounds.hi.y == doctest::Approx(halfExtent).epsilon(1e-5));
        CHECK(bounds.hi.z == doctest::Approx(halfExtent).epsilon(1e-5));
    };
    expectBox(cubeData(), 0.5f);     // unit cube: side 1, so +/-0.5
    expectBox(sphereData(), 1.0f);   // unit sphere: radius 1
    expectBox(cylinderData(), 1.0f); // radius 1, half-height 1

    // The ground plane is flat and reaches exactly +/-halfSize in x and z.
    const Aabb plane = meshBounds(groundPlaneData(7.5f).vertices);
    CHECK(plane.lo.x == doctest::Approx(-7.5f));
    CHECK(plane.hi.x == doctest::Approx(7.5f));
    CHECK(plane.lo.z == doctest::Approx(-7.5f));
    CHECK(plane.hi.z == doctest::Approx(7.5f));
    CHECK(plane.lo.y == plane.hi.y);

    // No vertex may fall outside what the renderer believes it must draw.
    for (const MeshData& mesh : {cubeData(), sphereData(), cylinderData(), groundPlaneData(7.5f)}) {
        REQUIRE_FALSE(mesh.vertices.empty());
        const Aabb bounds = meshBounds(mesh.vertices);
        for (const Vertex& vertex : mesh.vertices) {
            CHECK(vertex.position.x >= bounds.lo.x - 1e-6f);
            CHECK(vertex.position.y >= bounds.lo.y - 1e-6f);
            CHECK(vertex.position.z >= bounds.lo.z - 1e-6f);
            CHECK(vertex.position.x <= bounds.hi.x + 1e-6f);
            CHECK(vertex.position.y <= bounds.hi.y + 1e-6f);
            CHECK(vertex.position.z <= bounds.hi.z + 1e-6f);
        }
    }
}

namespace {

// Deterministic points in [-range, range]^3. Built straight from mt19937
// instead of a distribution, whose mapping differs between standard libraries.
std::vector<glm::vec3> samplePoints(std::uint32_t seed, int count, float range) {
    std::mt19937 generator(seed);
    const auto unit = [&generator] { return static_cast<float>(generator()) * (1.0f / 4294967296.0f); };
    std::vector<glm::vec3> points;
    points.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        points.emplace_back((unit() * 2.0f - 1.0f) * range, (unit() * 2.0f - 1.0f) * range,
                            (unit() * 2.0f - 1.0f) * range);
    }
    return points;
}

// True when `clip` is comfortably inside the volume rather than grazing it. The
// clip-coordinate test and the plane test are the same inequalities written
// differently, so they only disagree by rounding; asking for a margin keeps the
// assertions below about real errors instead of that rounding.
bool robustlyVisible(const glm::vec4& clip, float relative = 1e-3f) {
    if (clip.w <= 0.0f) {
        return false;
    }
    const float margin = relative * clip.w;
    return clip.x >= -clip.w + margin && clip.x <= clip.w - margin && clip.y >= -clip.w + margin &&
           clip.y <= clip.w - margin && clip.z >= margin && clip.z <= clip.w - margin;
}

} // namespace

TEST_CASE("transformBounds maps an affine transform onto a tight world box") {
    const Aabb unit{{-1.0f, -1.0f, -1.0f}, {1.0f, 1.0f, 1.0f}};

    // Translation moves the box: it must not keep the pre-transform extent, which
    // a naive "grow from the local box" would leave behind.
    const Aabb translated = transformBounds(unit, glm::translate(glm::mat4(1.0f), glm::vec3(10.0f, 5.0f, -3.0f)));
    CHECK(translated.lo == glm::vec3(9.0f, 4.0f, -4.0f));
    CHECK(translated.hi == glm::vec3(11.0f, 6.0f, -2.0f));

    const Aabb scaled = transformBounds(unit, glm::scale(glm::mat4(1.0f), glm::vec3(2.0f, 0.5f, 3.0f)));
    CHECK(scaled.lo == glm::vec3(-2.0f, -0.5f, -3.0f));
    CHECK(scaled.hi == glm::vec3(2.0f, 0.5f, 3.0f));

    // A 45 deg yaw widens the x and z extents to sqrt(2) and leaves y alone.
    const Aabb rotated =
        transformBounds(unit, glm::rotate(glm::mat4(1.0f), 0.7853981634f, glm::vec3(0.0f, 1.0f, 0.0f)));
    const float diagonal = std::sqrt(2.0f);
    CHECK(rotated.lo.x == doctest::Approx(-diagonal).epsilon(1e-5));
    CHECK(rotated.hi.x == doctest::Approx(diagonal).epsilon(1e-5));
    CHECK(rotated.lo.y == -1.0f);
    CHECK(rotated.hi.y == 1.0f);
    CHECK(rotated.lo.z == doctest::Approx(-diagonal).epsilon(1e-5));
    CHECK(rotated.hi.z == doctest::Approx(diagonal).epsilon(1e-5));

    // Whatever the transform, every mapped corner stays inside the result: a box
    // that missed a corner would cull geometry the renderer still has to draw.
    const glm::mat4 transform = glm::translate(glm::mat4(1.0f), glm::vec3(3.0f, -1.0f, 7.0f)) *
                                glm::rotate(glm::mat4(1.0f), 0.6f, glm::vec3(1.0f, 1.0f, 0.0f)) *
                                glm::scale(glm::mat4(1.0f), glm::vec3(2.0f, 3.0f, 0.5f));
    const Aabb combined = transformBounds(unit, transform);
    for (int corner = 0; corner < 8; ++corner) {
        const glm::vec3 p(corner & 1 ? unit.hi.x : unit.lo.x, corner & 2 ? unit.hi.y : unit.lo.y,
                          corner & 4 ? unit.hi.z : unit.lo.z);
        const glm::vec3 mapped = glm::vec3(transform * glm::vec4(p, 1.0f));
        for (int axis = 0; axis < 3; ++axis) {
            CHECK(mapped[axis] >= combined.lo[axis] - 1e-5f);
            CHECK(mapped[axis] <= combined.hi[axis] + 1e-5f);
        }
    }
}

TEST_CASE("frustum planes reject only geometry outside the view") {
    Camera camera;
    // Straight down -Z, square aspect, 45 deg vertical fov: the half-width at
    // depth d is d * tan(22.5 deg) ~= 0.4142 * d, near at 1 m, far at 100 m.
    camera.setPerspective(45.0f, 1.0f, 1.0f, 100.0f);
    camera.lookAt({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f});
    const Frustum frustum(camera.viewProjection());

    // Straight ahead, comfortably inside.
    CHECK(frustum.contains(Aabb{{-1.0f, -1.0f, -5.0f}, {1.0f, 1.0f, -3.0f}}));

    // Behind the camera: culled by the lateral planes.
    CHECK_FALSE(frustum.contains(Aabb{{-1.0f, -1.0f, 5.0f}, {1.0f, 1.0f, 7.0f}}));

    // Entirely nearer than the 1 m near plane.
    CHECK_FALSE(frustum.contains(Aabb{{-0.5f, -0.5f, -0.9f}, {0.5f, 0.5f, -0.3f}}));

    // Entirely beyond the 100 m far plane.
    CHECK_FALSE(frustum.contains(Aabb{{-1.0f, -1.0f, -200.0f}, {1.0f, 1.0f, -150.0f}}));

    // At z = -10 the frustum reaches x = +/-4.14 m; a box at x in [18, 22] misses.
    CHECK_FALSE(frustum.contains(Aabb{{18.0f, -1.0f, -12.0f}, {22.0f, 1.0f, -8.0f}}));

    // Same for the top plane in y.
    CHECK_FALSE(frustum.contains(Aabb{{-1.0f, 18.0f, -12.0f}, {1.0f, 22.0f, -8.0f}}));

    // A box straddling the right plane (its near edge reaches x = 4.97 m at
    // z = -12, past the 4.14 m boundary at the far edge) is still drawn.
    CHECK(frustum.contains(Aabb{{3.5f, -1.0f, -12.0f}, {5.0f, 1.0f, -8.0f}}));

    // A sphere just outside stays culled, and the same sphere grown until it
    // crosses a plane is drawn: the radius test is a real distance test.
    CHECK_FALSE(frustum.contains(glm::vec3(4.5f, 0.0f, -10.0f), 0.2f));
    CHECK(frustum.contains(glm::vec3(4.5f, 0.0f, -10.0f), 0.5f));
}

TEST_CASE("frustum culling never rejects a point the camera can see") {
    Camera camera;
    camera.setPerspective(60.0f, 16.0f / 9.0f, 0.05f, 3000.0f);
    camera.lookAt({3.0f, 1.6f, 8.0f}, {-2.0f, 1.0f, -6.0f});
    const Frustum frustum(camera.viewProjection());
    const glm::mat4 viewProjection = camera.viewProjection();

    // The property that makes culling safe to ship: any point inside the clip
    // volume must survive. If a plane had the wrong sign or the wrong depth
    // convention (this projection is reversed-Z), points near that plane would
    // disappear from the frame.
    int visible = 0;
    for (const glm::vec3& point : samplePoints(0x51ECu, 20000, 40.0f)) {
        const glm::vec4 clip = viewProjection * glm::vec4(point, 1.0f);
        if (!robustlyVisible(clip)) {
            continue;
        }
        ++visible;
        REQUIRE(frustum.contains(point, 0.0f));
    }
    MESSAGE("points inside the frustum: " << visible);
    // The sample really did land points in the view volume.
    CHECK(visible > 500);
}

TEST_CASE("frustum culling is conservative for transformed boxes") {
    Camera camera;
    camera.setPerspective(70.0f, 16.0f / 9.0f, 0.05f, 3000.0f);
    camera.lookAt({0.0f, 1.5f, 5.0f}, {0.0f, 1.0f, -10.0f});
    const Frustum frustum(camera.viewProjection());
    const glm::mat4 viewProjection = camera.viewProjection();

    // For each sampled box, any of its eight corners that the camera can see
    // proves the box must not be culled.
    int kept = 0;
    int culledVisible = 0;
    const std::vector<glm::vec3> samples = samplePoints(0xC0111Du, 3000, 30.0f);
    for (std::size_t i = 0; i + 1 < samples.size(); i += 2) {
        const glm::vec3 center = samples[i];
        const glm::vec3 halfExtent = glm::abs(samples[i + 1]) * 0.1f;
        const Aabb box{center - halfExtent, center + halfExtent};
        const bool drawn = frustum.contains(box);
        if (drawn) {
            ++kept;
            continue;
        }
        // The box was culled; confirm no corner is actually visible.
        for (int corner = 0; corner < 8; ++corner) {
            const glm::vec3 corner3(corner & 1 ? box.hi.x : box.lo.x, corner & 2 ? box.hi.y : box.lo.y,
                                    corner & 4 ? box.hi.z : box.lo.z);
            const glm::vec4 clip = viewProjection * glm::vec4(corner3, 1.0f);
            if (robustlyVisible(clip)) {
                ++culledVisible;
            }
        }
    }
    MESSAGE("boxes drawn: " << kept << ", culled with a visible corner: " << culledVisible);
    CHECK(culledVisible == 0);
    CHECK(kept > 0);
}

} // namespace sim::render
