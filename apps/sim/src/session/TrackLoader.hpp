// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "session/TrackCatalog.hpp"

namespace sim::app::drive {
class Track;
}

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::app {

// Result of scanning the VFS for track descriptors.
struct TrackLoadResult {
    std::vector<TrackSpec> tracks;
    std::vector<std::string> errors; // "<virtual path>: <message>", one per bad file
};

// Fills the physical fields of a spec (length, width, minimap, aspect) from a
// real circuit, so the preview matches what is driven.
void fillTrackGeometry(TrackSpec& spec, const drive::Track& track);

// Parses one track descriptor (the text of a track.toml). Returns std::nullopt
// and fills `error` when the document is malformed or a value is out of range.
// All values are SI: metres and seconds.
std::optional<TrackSpec> parseTrackSpec(std::string_view text, std::string_view sourcePath, std::string& error);

// Loads every `tracks/<id>/track.toml` reachable through the VFS. Tracks are
// returned sorted by id. Invalid tracks are skipped with an error.
TrackLoadResult loadTracks(const vfs::VirtualFileSystem& vfs, std::string_view tracksDirectory = "tracks");

} // namespace sim::app
