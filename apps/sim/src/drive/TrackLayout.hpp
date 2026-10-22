// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// The circuit layout (`track.json`): the centreline with its real elevation, the
// start/finish line, the grid, the sector splits and the pit lane. It is the
// companion of the visual/collision glTF: the glTF says what the circuit looks
// like and what a wheel touches, this says where a lap starts, how the cars line
// up and how long each sector is.
//
// Device-free: the parser is unit-tested against the shipped `sierra_verde`
// layout without a device.

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sim::vfs {
class VirtualFileSystem;
}

namespace sim::app::drive {

struct TrackLayout {
    struct Point {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct GridSlot {
        double x = 0.0;
        double z = 0.0;
        double headingRadians = 0.0;
    };

    struct PitLane {
        double speedLimitKmh = 0.0;
        std::vector<Point> polyline; // entry -> exit, on the ground
    };

    std::string name;
    double lengthMeters = 0.0; // as authored; the centreline is authoritative
    double halfWidthMeters = 6.5;
    std::vector<Point> centreline; // in travel order, every ~4 m

    bool hasStart = false;
    Point start;                    // centre of the start/finish line
    double startHeadingRadians = 0; // heading of `forward` (0 faces -Z)

    std::vector<GridSlot> grid; // slot 0 = pole, in order
    // Arc length from the start line at which each sector begins, plus the lap
    // length at the end. Always starts at 0 and is strictly increasing.
    std::vector<double> sectorDistances;
    PitLane pitLane;
};

// Parses a layout document. Returns nullopt and a message naming the problem on
// a malformed document.
std::optional<TrackLayout> parseTrackLayout(std::string_view json, std::string_view source, std::string& error);

// Reads and parses `path` through the VFS.
std::optional<TrackLayout> loadTrackLayout(const vfs::VirtualFileSystem& vfs, std::string_view path,
                                           std::string& error);

} // namespace sim::app::drive
