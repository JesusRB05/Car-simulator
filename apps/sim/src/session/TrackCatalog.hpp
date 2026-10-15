// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace sim::app {

// One selectable circuit. Length and the minimap centreline come from the real
// procedural Track, so the preview matches what is driven. Variants and
// availability are declared here; when a track declares no variants the UI
// shows a single "Full circuit" layout.
struct TrackSpec {
    std::string id;
    std::string name;
    std::string description;
    std::string location;

    double lengthMeters = 0.0;
    double widthMeters = 0.0;

    // Ordered list of layout variants. Always at least one ("Full circuit").
    std::vector<std::string> variants;

    // The centreline, projected to the unit square centred on the origin
    // (component order: X (east), Z (north)), for drawing a 2D minimap.
    std::vector<glm::vec2> minimap;
    // Aspect ratio (x extent / z extent) of the real centreline, so the minimap
    // keeps the circuit's true proportions.
    float aspect = 1.0f;

    bool available = true;
    std::string availabilityNote;
};

// The set of circuits the player can pick. Built lazily from the compiled-in
// tracks; content/base/tracks/*/track.toml is documentation only for now.
class TrackCatalog {
public:
    static const TrackCatalog& instance();

    const std::vector<TrackSpec>& tracks() const { return tracks_; }
    const TrackSpec* find(const std::string& id) const;
    const TrackSpec& defaultTrack() const { return tracks_.front(); }

private:
    TrackCatalog();

    std::vector<TrackSpec> tracks_;
};

} // namespace sim::app
