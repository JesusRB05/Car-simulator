// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace sim::app::drive {

using physics::Real;
using physics::Vec3;

enum class Surface {
    Asphalt,
    Kerb,
    Grass,
};

const char* surfaceName(Surface surface);

// What a wheel sees where it touches the ground.
struct GroundSample {
    Real height = 0.0;
    Vec3 normal{0.0, 1.0, 0.0};
    Real friction = 1.0;            // peak tyre/road friction coefficient
    Real rollingResistance = 0.015; // rolling resistance coefficient (force / load)
    Surface surface = Surface::Asphalt;
};

// Analytic ground used by the raycast wheels. There is no general collision
// detection yet: the ground is a smooth height function y = h(x, z), so a ray
// query against it is exact (a few Newton iterations on a smooth function).
class GroundQuery {
public:
    virtual ~GroundQuery() = default;

    virtual Real height(Real x, Real z) const = 0;
    // Unit surface normal (+Y up) at (x, z).
    virtual Vec3 normal(Real x, Real z) const = 0;
    // Height, normal and surface properties at (x, z).
    virtual GroundSample sample(Real x, Real z) const = 0;
};

// An infinite flat asphalt plane at a fixed height (used by the tests).
class FlatGround final : public GroundQuery {
public:
    explicit FlatGround(Real planeHeight = 0.0, Real planeFriction = 1.0)
        : height_(planeHeight), friction_(planeFriction) {}

    Real height(Real, Real) const override { return height_; }
    Vec3 normal(Real, Real) const override { return {0.0, 1.0, 0.0}; }
    GroundSample sample(Real, Real) const override {
        GroundSample result;
        result.height = height_;
        result.friction = friction_;
        return result;
    }

private:
    Real height_;
    Real friction_;
};

// One resampled point of the track centreline (roughly every metre).
struct TrackSample {
    Vec3 position{0.0};           // on the terrain
    Vec3 tangent{0.0, 0.0, -1.0}; // horizontal unit direction of travel
    Vec3 right{1.0, 0.0, 0.0};    // horizontal unit vector to the driver's right
    Real distance = 0.0;          // arc length from sample 0 (metres, horizontal)
    Real curvature = 0.0;         // signed horizontal curvature (1/m, > 0 turns left)
    Real halfWidth = 5.0;         // asphalt half-width here (metres, >= 0)
    Real banking = 0.0;           // cross-slope (radians, > 0 raises the right edge)
    bool kerb = false;            // kerbs are laid on both edges of this sample
};

// The procedural test circuit: a closed centripetal Catmull-Rom spline through a
// fixed set of control points (long straight, hairpin, fast sweeper, chicane)
// laid over a rolling terrain of a few smooth hills. The terrain height is
// analytic, so every query is exact; each centreline sample also carries its own
// asphalt width and banking, so the road widens on the straights, narrows into
// the corners and leans into them. The track changes both the surface (friction)
// and the height (banking) in a band around the centreline.
class Track final : public GroundQuery {
public:
    Track();

    // --- GroundQuery ------------------------------------------------------
    Real height(Real x, Real z) const override { return terrainHeight(x, z); }
    Vec3 normal(Real x, Real z) const override { return terrainNormal(x, z); }
    GroundSample sample(Real x, Real z) const override;

    // The smooth terrain shared by the asphalt and the grass.
    static Real terrainHeight(Real x, Real z);
    static Vec3 terrainNormal(Real x, Real z);

    // --- Geometry ---------------------------------------------------------
    const std::vector<TrackSample>& samples() const { return samples_; }
    Real length() const { return length_; }
    Real halfWidth() const { return halfWidth_; }
    Real kerbWidth() const { return kerbWidth_; }

    // Road cross-section geometry, shared by the physics query and the renderer
    // so the wheels always sit on the asphalt that is actually drawn. `lateral`
    // is signed (positive to the driver's right) and `raise` lifts the point
    // along +Y (for decals like edge lines). Both account for the banking.
    Vec3 roadPoint(const TrackSample& sample, Real lateral, Real raise) const;
    Vec3 roadNormal(const TrackSample& sample, Real lateral) const;

    // Closest point on the centreline polyline to (x, z), searched through a
    // uniform grid. `found` is false when (x, z) is far from the track.
    struct Projection {
        bool found = false;
        std::size_t segment = 0; // segment from sample `segment` to the next one
        Real along = 0.0;        // arc length of the closest point
        Real lateral = 0.0;      // signed offset, > 0 to the right of the direction of travel
    };
    Projection project(Real x, Real z) const;

    // Start/finish line: index of its centreline sample, and the pose a car is
    // reset to (a few metres behind the line, facing the direction of travel).
    std::size_t startIndex() const { return startIndex_; }
    Vec3 startPosition() const;
    // Yaw about +Y; heading 0 faces -Z.
    Real startHeading() const;

private:
    void buildCentreline();
    void buildGrid();

    std::vector<TrackSample> samples_;
    Real length_ = 0.0;
    Real halfWidth_ = 5.0;
    Real kerbWidth_ = 1.0;
    std::size_t startIndex_ = 0;

    // Uniform acceleration grid over the XZ bounding box: each cell lists the
    // segments that pass within the surface band of it.
    Real gridMinX_ = 0.0;
    Real gridMinZ_ = 0.0;
    Real cellSize_ = 16.0;
    int gridColumns_ = 0;
    int gridRows_ = 0;
    std::vector<std::vector<std::size_t>> cells_;
};

} // namespace sim::app::drive
