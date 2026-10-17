// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/TrackCatalog.hpp"

#include <utility>

#include "core/Log.hpp"
#include "session/TrackLoader.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {

TrackCatalog::TrackCatalog() = default;

TrackCatalog& TrackCatalog::instance() {
    static TrackCatalog catalog;
    return catalog;
}

void TrackCatalog::reload(const vfs::VirtualFileSystem& vfs, const ContentOwner& owner) {
    TrackLoadResult result = loadTracks(vfs, owner);

    errors_ = std::move(result.errors);
    for (const std::string& error : errors_) {
        core::Log::get("content")->error("track rejected: {}", error);
    }

    tracks_ = std::move(result.tracks);
    if (tracks_.empty()) {
        core::Log::get("content")->error("no track descriptor loaded (is the base content mounted?)");
    } else {
        core::Log::get("content")->info("loaded {} track(s) from content", tracks_.size());
    }
}

const TrackSpec* TrackCatalog::find(const std::string& key) const {
    for (const TrackSpec& track : tracks_) {
        if (track.ref() == key) {
            return &track;
        }
    }
    const TrackSpec* match = nullptr;
    for (const TrackSpec& track : tracks_) {
        if (track.id == key) {
            if (match != nullptr) {
                return nullptr; // ambiguous, refuse to guess
            }
            match = &track;
        }
    }
    return match;
}

} // namespace sim::app
