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

constexpr Real kStartBehind = 6.0;

// Track-width and banking shaping. The description's `nominalHalfWidth` is what
// halfWidth() reports (and what the tests and the obstacle layout assume); the
// per-sample asphalt leans on it.
constexpr Real kWidenMax = 0.8;          // extra half-width at the start/finish box
constexpr Real kWidenSigma = 40.0;       // metres, spread of the widening pulse
constexpr Real kNarrowMax = 0.9;         // most a corner narrows the asphalt
constexpr Real kNarrowScale = 36.0;      // metres per rad of curvature
constexpr Real kBankGain = 2.4;          // radians of bank per rad of heading change / m
constexpr Real kBankMax = 0.061;         // ~3.5 degrees of maximum cross-slope
constexpr Real kMinHalfWidthDelta = 0.8; // most the asphalt narrows below the nominal
constexpr Real kMaxHalfWidthDelta = 1.0; // most the asphalt widens above the nominal
constexpr int kSmoothRadius = 4;
constexpr Real kKerbLift = 0.04; // kerb top above the terrain (metres)

// Height added to the asphalt at a signed lateral offset: the low edge sits on
// the terrain (no ditch) and the high edge is lifted, pivoting about the outside
// kerb. Zero away from the asphalt or on an unbanked sample.
Real bankingLift(const TrackSample& sample, Real lateral) {
    if (sample.banking == 0.0) {
        return 0.0;
    }
    const Real clamped = std::clamp(lateral, -sample.halfWidth, sample.halfWidth);
    const Real direction = sample.banking > 0.0 ? 1.0 : -1.0;
    return (direction * clamped + sample.halfWidth) * std::abs(std::tan(sample.banking));
}

Point2 controlPoint(const std::vector<TrackDescription::ControlPoint>& points, std::ptrdiff_t index) {
    const std::ptrdiff_t count = static_cast<std::ptrdiff_t>(points.size());
    const std::ptrdiff_t wrapped = ((index % count) + count) % count;
    const auto& point = points[static_cast<std::size_t>(wrapped)];
    return {point.x, point.z};
}

Real terrainHeightOf(const std::vector<TrackHill>& hills, Real x, Real z) {
    Real height = 0.0;
    for (const TrackHill& hill : hills) {
        const Real dx = x - hill.x;
        const Real dz = z - hill.z;
        height += hill.height * std::exp(-(dx * dx + dz * dz) / (2.0 * hill.sigma * hill.sigma));
    }
    return height;
}

Vec3 terrainNormalOf(const std::vector<TrackHill>& hills, Real x, Real z) {
    Real gradientX = 0.0;
    Real gradientZ = 0.0;
    for (const TrackHill& hill : hills) {
        const Real dx = x - hill.x;
        const Real dz = z - hill.z;
        const Real h = hill.height * std::exp(-(dx * dx + dz * dz) / (2.0 * hill.sigma * hill.sigma));
        const Real inverseSigma2 = 1.0 / (hill.sigma * hill.sigma);
        gradientX += -h * dx * inverseSigma2;
        gradientZ += -h * dz * inverseSigma2;
    }
    return glm::normalize(Vec3(-gradientX, 1.0, -gradientZ));
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

const TrackDescription& defaultTrackDescription() {
    static const TrackDescription description = [] {
        TrackDescription d;
        // Centreline control points in the direction of travel. The car starts
        // on the long straight along x = 0, heading -Z.
        d.controlPoints = {
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
        // A handful of smooth Gaussian hills give the circuit gentle elevation
        // changes (max grade ~6 % each) without ever becoming a hill climb.
        d.hills = {
            {90.0, -60.0, 5.0, 55.0},  // under the back straight / sweeper
            {30.0, -176.0, 3.2, 40.0}, // around the hairpin
            {112.0, 116.0, 2.8, 44.0}, // under the chicane
            {-40.0, 150.0, 2.2, 55.0}, // beside the main straight
        };
        d.sampleSpacing = 1.0;
        d.nominalHalfWidth = 5.0;
        d.kerbWidth = 1.0;
        d.startZ = 40.0;
        d.loop = true;
        return d;
    }();
    return description;
}

Track::Track() : Track(defaultTrackDescription()) {}

Track::Track(const TrackDescription& description) : description_(description) {
    if (description_.controlPoints.size() < 3) {
        // A degenerate description cannot form a loop: fall back to the default
        // so the track is still usable.
        description_ = defaultTrackDescription();
    }
    halfWidth_ = description_.nominalHalfWidth;
    kerbWidth_ = description_.kerbWidth;
    buildCentreline();
    buildGrid();
}

Real Track::terrainHeight(Real x, Real z) {
    return terrainHeightOf(defaultTrackDescription().hills, x, z);
}

Vec3 Track::terrainNormal(Real x, Real z) {
    return terrainNormalOf(defaultTrackDescription().hills, x, z);
}

Real Track::heightAt(Real x, Real z) const {
    return terrainHeightOf(description_.hills, x, z);
}

Vec3 Track::normalAt(Real x, Real z) const {
    return terrainNormalOf(description_.hills, x, z);
}

void Track::buildCentreline() {
    // Dense polyline through the spline, then resampled by arc length.
    constexpr int kSubdivisions = 200;
    const std::vector<TrackDescription::ControlPoint>& controlPoints = description_.controlPoints;
    std::vector<Point2> dense;
    dense.reserve(controlPoints.size() * kSubdivisions + 1);
    for (std::size_t i = 0; i < controlPoints.size(); ++i) {
        const std::ptrdiff_t index = static_cast<std::ptrdiff_t>(i);
        const Point2 p0 = controlPoint(controlPoints, index - 1);
        const Point2 p1 = controlPoint(controlPoints, index);
        const Point2 p2 = controlPoint(controlPoints, index + 1);
        const Point2 p3 = controlPoint(controlPoints, index + 2);
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

    const std::size_t count =
        static_cast<std::size_t>(std::max(16.0, std::round(length_ / std::max(description_.sampleSpacing, 1e-3))));
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
        samples_[i].position = Vec3(x, heightAt(x, z), z);
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
        const Real dz = samples_[i].position.z - description_.startZ;
        const Real d = dx * dx + dz * dz;
        if (d < best) {
            best = d;
            startIndex_ = i;
        }
    }

    // Per-sample asphalt width and banking, both driven by the curvature. The
    // road widens a little around the start/finish box and narrows into the
    // corners; the banking leans the outside edge up, capped so the car never
    // drives up a wall. Both are box-blurred so the ribbon has no kinks.
    const Real minHalfWidth = std::max(description_.nominalHalfWidth - kMinHalfWidthDelta, 1.0);
    const Real maxHalfWidth = description_.nominalHalfWidth + kMaxHalfWidthDelta;
    std::vector<Real> width(count, description_.nominalHalfWidth);
    std::vector<Real> banking(count, 0.0);
    const auto wrapIndex = [count](std::ptrdiff_t index) {
        const std::ptrdiff_t n = static_cast<std::ptrdiff_t>(count);
        return static_cast<std::size_t>(((index % n) + n) % n);
    };
    for (std::size_t i = 0; i < count; ++i) {
        const std::ptrdiff_t diff = static_cast<std::ptrdiff_t>(i) - static_cast<std::ptrdiff_t>(startIndex_);
        const std::ptrdiff_t wrapped = std::min(std::abs(diff), static_cast<std::ptrdiff_t>(count) - std::abs(diff));
        const Real distance = static_cast<Real>(wrapped) * spacing;
        const Real widen = kWidenMax * std::exp(-(distance * distance) / (2.0 * kWidenSigma * kWidenSigma));
        const Real narrow = std::min(std::abs(samples_[i].curvature) * kNarrowScale, kNarrowMax);
        width[i] = std::clamp(description_.nominalHalfWidth + widen - narrow, minHalfWidth, maxHalfWidth);
        banking[i] = std::clamp(samples_[i].curvature * kBankGain, -kBankMax, kBankMax);
    }
    const auto smooth = [&](std::vector<Real>& values) {
        std::vector<Real> blurred(values.size(), 0.0);
        for (std::size_t i = 0; i < values.size(); ++i) {
            Real sum = 0.0;
            for (int k = -kSmoothRadius; k <= kSmoothRadius; ++k) {
                sum += values[wrapIndex(static_cast<std::ptrdiff_t>(i) + k)];
            }
            blurred[i] = sum / static_cast<Real>(2 * kSmoothRadius + 1);
        }
        values.swap(blurred);
    };
    smooth(width);
    smooth(banking);
    for (std::size_t i = 0; i < count; ++i) {
        samples_[i].halfWidth = width[i];
        samples_[i].banking = banking[i];
    }
}

void Track::buildGrid() {
    Real maxHalfWidth = halfWidth_;
    for (const TrackSample& sample : samples_) {
        maxHalfWidth = std::max(maxHalfWidth, sample.halfWidth);
    }
    const Real band = maxHalfWidth + kerbWidth_ + 2.0;
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
    result.surface = Surface::Grass;

    const Projection projection = project(x, z);
    const TrackSample* surfaceSample = nullptr;
    if (projection.found) {
        const TrackSample& sample = samples_[projection.segment];
        const Real offset = std::abs(projection.lateral);
        if (offset <= sample.halfWidth) {
            result.surface = Surface::Asphalt;
            surfaceSample = &sample;
        } else if (offset <= sample.halfWidth + kerbWidth_ && sample.kerb) {
            result.surface = Surface::Kerb;
        }
    }

    // Compute the height and normal for the surface we actually found. The asphalt
    // is draped over the terrain (its height follows the terrain under each point,
    // so the terrain never pokes through on a lateral slope) and leans with the
    // banking; grass and kerbs use the bare terrain (a kerb is just lifted).
    // Evaluating only the branch we need avoids the discarded terrain samples the
    // old unconditional pre-computation computed on every call.
    if (result.surface == Surface::Asphalt) {
        result.height = heightAt(x, z) + bankingLift(*surfaceSample, projection.lateral);
        result.normal = roadNormal(*surfaceSample, projection.lateral);
    } else {
        result.height = heightAt(x, z);
        result.normal = normalAt(x, z);
        if (result.surface == Surface::Kerb) {
            result.height += kKerbLift;
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

Vec3 Track::roadPoint(const TrackSample& sample, Real lateral, Real raise) const {
    const Real x = sample.position.x + sample.right.x * lateral;
    const Real z = sample.position.z + sample.right.z * lateral;
    return Vec3(x, heightAt(x, z) + bankingLift(sample, lateral) + raise, z);
}

Vec3 Track::roadNormal(const TrackSample& sample, Real lateral) const {
    const Vec3 point = roadPoint(sample, lateral, 0.0);
    const Vec3 terrain = normalAt(point.x, point.z);
    Real gradientX = -terrain.x / terrain.y;
    Real gradientZ = -terrain.z / terrain.y;
    const Real slope = std::tan(sample.banking);
    gradientX += slope * sample.right.x;
    gradientZ += slope * sample.right.z;
    return glm::normalize(Vec3(-gradientX, 1.0, -gradientZ));
}

Vec3 Track::startPosition() const {
    const TrackSample& start = samples_[startIndex_];
    const Vec3 p = start.position - start.tangent * kStartBehind;
    return Vec3(p.x, sample(p.x, p.z).height, p.z);
}

Real Track::startHeading() const {
    return headingOf(samples_[startIndex_].tangent);
}

} // namespace sim::app::drive
