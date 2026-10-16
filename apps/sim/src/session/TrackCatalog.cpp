// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/TrackCatalog.hpp"

#include <utility>

#include "core/Log.hpp"
#include "drive/Track.hpp"
#include "session/TrackLoader.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

// The compiled-in fallback circuit. It mirrors
// content/base/tracks/sunset_circuit/track.toml and is only used if no
// descriptor loads (a broken install).
TrackSpec makeSunsetCircuit() {
    const drive::Track track;

    TrackSpec spec;
    spec.id = "sunset_circuit";
    spec.name = "Sunset Circuit";
    spec.location = "Procedural test loop";
    spec.description =
        "The base test circuit: a long straight, a right hairpin, a fast sweeper and a chicane draped over a "
        "rolling terrain of a few smooth hills. Width and banking change along the lap. Defined in "
        "content/base/tracks/sunset_circuit/track.toml.";
    spec.variants = {"Full circuit"};
    fillTrackGeometry(spec, track);
    return spec;
}

} // namespace

TrackCatalog::TrackCatalog() {
    tracks_.push_back(makeSunsetCircuit());
}

TrackCatalog& TrackCatalog::instance() {
    static TrackCatalog catalog;
    return catalog;
}

void TrackCatalog::reload(const vfs::VirtualFileSystem& vfs) {
    TrackLoadResult result = loadTracks(vfs);

    errors_ = std::move(result.errors);
    for (const std::string& error : errors_) {
        core::Log::get("content")->error("track rejected: {}", error);
    }

    if (result.tracks.empty()) {
        core::Log::get("content")->warn("no track descriptor loaded; keeping the compiled-in Sunset Circuit");
        tracks_.clear();
        tracks_.push_back(makeSunsetCircuit());
        return;
    }

    tracks_ = std::move(result.tracks);
    core::Log::get("content")->info("loaded {} track(s) from content", tracks_.size());
}

const TrackSpec* TrackCatalog::find(const std::string& id) const {
    for (const TrackSpec& track : tracks_) {
        if (track.id == id) {
            return &track;
        }
    }
    return nullptr;
}

} // namespace sim::app
