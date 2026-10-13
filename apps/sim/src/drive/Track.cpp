// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/Track.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "physics/Math.hpp"

namespace sim::app::drive {
namespace {

struct Point2 {
    Real x = 0.0;
    Real z = 0.0;
};

// Control points of the closed circuit in the direction of travel. The car
// starts on the long straight along x = 0, heading -Z.
constexpr Point2 kControlPoints[] = {
    {0.0, 120.0},   // main straight
    {0.0, 20.0},    //   start/finish line around z = 40
    {0.0, -90.0},   //   end of the straight, braking zone
    {6.0, -165.0},  // hairpin entry (right)
    {28.0, -192.0}, //   hairpin apex
    {52.0, -170.0}, //   hairpin exit
    {55.0, -95.0},  // back straight over the hill
    {105.0, -25.0}, // fast sweeper
    {122.0, 55.0},  //   sweeper exit
    {98.0, 100.0},  // chicane (left)
    {124.0, 128.0}, //   chicane (right)
    {96.0, 160.0},  //   chicane exit
    {48.0, 196.0},  // long left onto the main straight
    {8.0, 168.0},
};
constexpr std::size_t kControlCount = sizeof(kControlPoints) / sizeof(kControlPoints[0]);

// One smooth Gaussian hill gives a gentle elevation change (max grade ~5.5 %).
constexpr Real kHillX = 90.0;
constexpr Real kHillZ = -60.0;
constexpr Real kHillHeight = 5.0;
constexpr Real kHillSigma = 55.0;

constexpr Real kStartZ = 40.0;
constexpr Real kStartBehind = 6.0;
constexpr Real kSampleSpacing = 1.0;

Point2 controlPoint(std::ptrdiff_t index) {
    const std::ptrdiff_t count = static_cast<std::ptrdiff_t>(kControlCount);
    const std::ptrdiff_t wrapped = ((index % count) + count) % count;
    return kControlPoints[static_cast<std::size_t>(wrapped)];
}

Point2 lerp2(const Point2& a, const Point2& b, Real wa, Real wb) {
    return {a.x * wa + b.x * wb, a.z * wa + b.z * wb};
}

// Centripetal Catmull-Rom (alpha = 0.5, Barry-Goldman evaluation): no cusps or
// self-intersections inside a segment even with unevenly spaced points.
Point2 catmullRom(const Point2& p0, const Point2& p1, const Point2& p2, const Point2& p3, Real t) {
    const auto knot = [](const Point2& a, const Point2& b) {
        const Real d = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.z - a.z) * (b.z - a.z));
        return std::max(std::sqrt(d), 1e-6);
    };
    const Real t0 = 0.0;
    const Real t1 = t0 + knot(p0, p1);
    const Real t2 = t1 + knot(p1, p2);
    const Real t3 = t2 + knot(p2, p3);
    const Real u = t1 + (t2 - t1) * t;

    const Point2 a1 = lerp2(p0, p1, (t1 - u) / (t1 - t0), (u - t0) / (t1 - t0));
    const Point2 a2 = lerp2(p1, p2, (t2 - u) / (t2 - t1), (u - t1) / (t2 - t1));
    const Point2 a3 = lerp2(p2, p3, (t3 - u) / (t3 - t2), (u - t2) / (t3 - t2));
    const Point2 b1 = lerp2(a1, a2, (t2 - u) / (t2 - t0), (u - t0) / (t2 - t0));
    const Point2 b2 = lerp2(a2, a3, (t3 - u) / (t3 - t1), (u - t1) / (t3 - t1));
    return lerp2(b1, b2, (t2 - u) / (t2 - t1), (u - t1) / (t2 - t1));
}

Real headingOf(const Vec3& tangent) {
    // Heading 0 faces -Z; a positive heading is a rotation about +Y (to the left).
    return std::atan2(-tangent.x, -tangent.z);
}

Real wrapAngle(Real angle) {
    while (angle > physics::kPiPhysics) {
        angle -= physics::kTwoPiPhysics;
    }
    while (angle < -physics::kPiPhysics) {
        angle += physics::kTwoPiPhysics;
    }
    return angle;
}

} // namespace

const char* surfaceName(Surface surface) {
    switch (surface) {
    case Surface::Asphalt:
        return "asphalt";
    case Surface::Kerb:
        return "kerb";
    case Surface::Grass:
        return "grass";
    }
    return "?";
}

Track::Track() {
    buildCentreline();
    buildGrid();
}

Real Track::terrainHeight(Real x, Real z) {
    const Real dx = x - kHillX;
    const Real dz = z - kHillZ;
    return kHillHeight * std::exp(-(dx * dx + dz * dz) / (2.0 * kHillSigma * kHillSigma));
}

Vec3 Track::terrainNormal(Real x, Real z) {
    const Real dx = x - kHillX;
    const Real dz = z - kHillZ;
    const Real h = kHillHeight * std::exp(-(dx * dx + dz * dz) / (2.0 * kHillSigma * kHillSigma));
    const Real inverseSigma2 = 1.0 / (kHillSigma * kHillSigma);
    const Real gradientX = -h * dx * inverseSigma2;
    const Real gradientZ = -h * dz * inverseSigma2;
    return glm::normalize(Vec3(-gradientX, 1.0, -gradientZ));
}

void Track::buildCentreline() {
    // Dense polyline through the spline, then resampled by arc length.
    constexpr int kSubdivisions = 200;
    std::vector<Point2> dense;
    dense.reserve(kControlCount * kSubdivisions + 1);
    for (std::size_t i = 0; i < kControlCount; ++i) {
        const std::ptrdiff_t index = static_cast<std::ptrdiff_t>(i);
        const Point2 p0 = controlPoint(index - 1);
        const Point2 p1 = controlPoint(index);
        const Point2 p2 = controlPoint(index + 1);
        const Point2 p3 = controlPoint(index + 2);
        for (int k = 0; k < kSubdivisions; ++k) {
            dense.push_back(catmullRom(p0, p1, p2, p3, static_cast<Real>(k) / kSubdivisions));
        }
    }
    dense.push_back(dense.front());

    std::vector<Real> cumulative(dense.size(), 0.0);
    for (std::size_t i = 1; i < dense.size(); ++i) {
        const Real dx = dense[i].x - dense[i - 1].x;
        const Real dz = dense[i].z - dense[i - 1].z;
        cumulative[i] = cumulative[i - 1] + std::sqrt(dx * dx + dz * dz);
    }
    length_ = cumulative.back();

    const std::size_t count = static_cast<std::size_t>(std::max(16.0, std::round(length_ / kSampleSpacing)));
    const Real spacing = length_ / static_cast<Real>(count);
    samples_.resize(count);
    std::size_t cursor = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const Real s = spacing * static_cast<Real>(i);
        while (cursor + 2 < dense.size() && cumulative[cursor + 1] < s) {
            ++cursor;
        }
        const Real segmentLength = std::max(cumulative[cursor + 1] - cumulative[cursor], 1e-12);
        const Real t = std::clamp((s - cumulative[cursor]) / segmentLength, 0.0, 1.0);
        const Real x = dense[cursor].x + (dense[cursor + 1].x - dense[cursor].x) * t;
        const Real z = dense[cursor].z + (dense[cursor + 1].z - dense[cursor].z) * t;
        samples_[i].position = Vec3(x, terrainHeight(x, z), z);
        samples_[i].distance = s;
    }

    for (std::size_t i = 0; i < count; ++i) {
        const Vec3& next = samples_[(i + 1) % count].position;
        const Vec3& previous = samples_[(i + count - 1) % count].position;
        Vec3 tangent(next.x - previous.x, 0.0, next.z - previous.z);
        tangent = physics::safeNormalize(tangent, Vec3(0.0, 0.0, -1.0));
        samples_[i].tangent = tangent;
        samples_[i].right = Vec3(-tangent.z, 0.0, tangent.x);
    }

    // Curvature from the heading change over +-3 samples (smooths the
    // resampling noise), then kerbs wherever the corner is tighter than R160 m.
    constexpr std::size_t kSpan = 3;
    for (std::size_t i = 0; i < count; ++i) {
        const Real a = headingOf(samples_[(i + count - kSpan) % count].tangent);
        const Real b = headingOf(samples_[(i + kSpan) % count].tangent);
        samples_[i].curvature = wrapAngle(b - a) / (2.0 * kSpan * spacing);
    }
    std::vector<bool> tight(count, false);
    for (std::size_t i = 0; i < count; ++i) {
        tight[i] = std::abs(samples_[i].curvature) > 1.0 / 160.0;
    }
    constexpr std::size_t kDilate = 4;
    for (std::size_t i = 0; i < count; ++i) {
        bool kerb = false;
        for (std::size_t k = 0; k <= 2 * kDilate && !kerb; ++k) {
            kerb = tight[(i + count + k - kDilate) % count];
        }
        samples_[i].kerb = kerb;
    }

    // Start/finish line on the main straight.
    Real best = std::numeric_limits<Real>::max();
    for (std::size_t i = 0; i < count; ++i) {
        const Real dx = samples_[i].position.x;
        const Real dz = samples_[i].position.z - kStartZ;
        const Real d = dx * dx + dz * dz;
        if (d < best) {
            best = d;
            startIndex_ = i;
        }
    }
}

void Track::buildGrid() {
    const Real band = halfWidth_ + kerbWidth_ + 2.0;
    Real minX = std::numeric_limits<Real>::max();
    Real minZ = std::numeric_limits<Real>::max();
    Real maxX = std::numeric_limits<Real>::lowest();
    Real maxZ = std::numeric_limits<Real>::lowest();
    for (const TrackSample& sample : samples_) {
        minX = std::min(minX, sample.position.x);
        minZ = std::min(minZ, sample.position.z);
        maxX = std::max(maxX, sample.position.x);
        maxZ = std::max(maxZ, sample.position.z);
    }
    gridMinX_ = minX - band - cellSize_;
    gridMinZ_ = minZ - band - cellSize_;
    gridColumns_ = static_cast<int>(std::ceil((maxX - gridMinX_ + band + cellSize_) / cellSize_));
    gridRows_ = static_cast<int>(std::ceil((maxZ - gridMinZ_ + band + cellSize_) / cellSize_));
    cells_.assign(static_cast<std::size_t>(gridColumns_) * static_cast<std::size_t>(gridRows_), {});

    const std::size_t count = samples_.size();
    for (std::size_t i = 0; i < count; ++i) {
        const Vec3& a = samples_[i].position;
        const Vec3& b = samples_[(i + 1) % count].position;
        const int c0 = static_cast<int>(std::floor((std::min(a.x, b.x) - band - gridMinX_) / cellSize_));
        const int c1 = static_cast<int>(std::floor((std::max(a.x, b.x) + band - gridMinX_) / cellSize_));
        const int r0 = static_cast<int>(std::floor((std::min(a.z, b.z) - band - gridMinZ_) / cellSize_));
        const int r1 = static_cast<int>(std::floor((std::max(a.z, b.z) + band - gridMinZ_) / cellSize_));
        for (int row = std::max(r0, 0); row <= std::min(r1, gridRows_ - 1); ++row) {
            for (int column = std::max(c0, 0); column <= std::min(c1, gridColumns_ - 1); ++column) {
                cells_[static_cast<std::size_t>(row) * static_cast<std::size_t>(gridColumns_) +
                       static_cast<std::size_t>(column)]
                    .push_back(i);
            }
        }
    }
}

Track::Projection Track::project(Real x, Real z) const {
    Projection result;
    const int column = static_cast<int>(std::floor((x - gridMinX_) / cellSize_));
    const int row = static_cast<int>(std::floor((z - gridMinZ_) / cellSize_));
    if (column < 0 || row < 0 || column >= gridColumns_ || row >= gridRows_) {
        return result;
    }

    const std::vector<std::size_t>& cell =
        cells_[static_cast<std::size_t>(row) * static_cast<std::size_t>(gridColumns_) +
               static_cast<std::size_t>(column)];
    const std::size_t count = samples_.size();
    Real best = std::numeric_limits<Real>::max();
    for (const std::size_t i : cell) {
        const Vec3& a = samples_[i].position;
        const Vec3& b = samples_[(i + 1) % count].position;
        const Real ex = b.x - a.x;
        const Real ez = b.z - a.z;
        const Real lengthSquared = std::max(ex * ex + ez * ez, 1e-12);
        const Real t = std::clamp(((x - a.x) * ex + (z - a.z) * ez) / lengthSquared, 0.0, 1.0);
        const Real px = a.x + ex * t;
        const Real pz = a.z + ez * t;
        const Real d = (x - px) * (x - px) + (z - pz) * (z - pz);
        if (d < best) {
            best = d;
            result.found = true;
            result.segment = i;
            result.along = samples_[i].distance + t * std::sqrt(lengthSquared);
            const Vec3& right = samples_[i].right;
            result.lateral = (x - px) * right.x + (z - pz) * right.z;
        }
    }
    return result;
}

GroundSample Track::sample(Real x, Real z) const {
    GroundSample result;
    result.height = terrainHeight(x, z);
    result.normal = terrainNormal(x, z);
    result.surface = Surface::Grass;

    const Projection projection = project(x, z);
    if (projection.found) {
        const Real offset = std::abs(projection.lateral);
        if (offset <= halfWidth_) {
            result.surface = Surface::Asphalt;
        } else if (offset <= halfWidth_ + kerbWidth_ && samples_[projection.segment].kerb) {
            result.surface = Surface::Kerb;
        }
    }

    switch (result.surface) {
    case Surface::Asphalt:
        result.friction = 1.0;
        result.rollingResistance = 0.015;
        break;
    case Surface::Kerb:
        result.friction = 0.9;
        result.rollingResistance = 0.02;
        break;
    case Surface::Grass:
        result.friction = 0.6;
        result.rollingResistance = 0.06;
        break;
    }
    return result;
}

Vec3 Track::startPosition() const {
    const TrackSample& start = samples_[startIndex_];
    const Vec3 p = start.position - start.tangent * kStartBehind;
    return Vec3(p.x, terrainHeight(p.x, p.z), p.z);
}

Real Track::startHeading() const {
    return headingOf(samples_[startIndex_].tangent);
}

} // namespace sim::app::drive
