// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "drive/Track.hpp"
#include "session/ContentOwner.hpp"

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::app {

// One selectable circuit. Length and the minimap centreline come from the real
// procedural Track, so the preview matches what is driven. Variants and
// availability are declared here; when a track declares no variants the UI
// shows a single "Full circuit" layout.
//
// A circuit is identified by its **content reference** `<mod-id>/<content-id>`
// (for example `base.core/sunset_circuit`).
struct TrackSpec {
    std::string modId; // owning mod, e.g. "base.core"
    std::string id;    // content id (folder name), e.g. "sunset_circuit"

    // The namespaced reference used everywhere a circuit is identified.
    std::string ref() const { return modId.empty() ? id : modId + "/" + id; }

    std::string name;
    std::string description;
    std::string location;

    // The description the drive scene rebuilds the real circuit from, so what is
    // driven is exactly the data that was loaded.
    drive::TrackDescription track;

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

// The set of circuits the player can pick. Populated from the VFS: every
// `tracks/<id>/track.toml` in the official content and in the enabled mods is
// loaded (see TrackLoader). There is no compiled-in circuit: the official
// content is data (`base.core/sunset_circuit`).
class TrackCatalog {
public:
    static TrackCatalog& instance();

    // Rescans the VFS. Called once at start-up after the mods are mounted.
    // `owner` resolves which mod provides each descriptor (see ContentOwner).
    void reload(const vfs::VirtualFileSystem& vfs, const ContentOwner& owner = {});

    const std::vector<TrackSpec>& tracks() const { return tracks_; }

    // Looks a circuit up by content reference or, as a fallback for saved
    // selections, by a bare, unambiguous id.
    const TrackSpec* find(const std::string& key) const;
    const TrackSpec* defaultTrack() const { return tracks_.empty() ? nullptr : &tracks_.front(); }

    // One entry per track descriptor that was rejected, with the reason.
    const std::vector<std::string>& errors() const { return errors_; }

private:
    TrackCatalog();

    std::vector<TrackSpec> tracks_;
    std::vector<std::string> errors_;
};

} // namespace sim::app
