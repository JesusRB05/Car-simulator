// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "drive/TrackLayout.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

#include "drive/Json.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app::drive {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;

// Heading of a world-space direction: 0 faces -Z and a positive angle turns to
// the left (about +Y), matching Track::startHeading and the vehicle.
double headingOf(double x, double z) {
    return std::atan2(-x, -z);
}

bool readPoint(const json::Value& value, TrackLayout::Point& out, const std::string& where, std::string& error) {
    if (!value.isArray() || value.items.size() != 3) {
        error = where + " must be an [x, y, z] array of three numbers";
        return false;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        if (!value.items[i].isNumber()) {
            error = where + " must contain only numbers";
            return false;
        }
    }
    out.x = value.items[0].number;
    out.y = value.items[1].number;
    out.z = value.items[2].number;
    if (std::abs(out.x) > 1.0e5 || std::abs(out.y) > 1.0e5 || std::abs(out.z) > 1.0e5) {
        error = where + " is outside +/-100 km";
        return false;
    }
    return true;
}

// Reads an array of numbers, appending to `out`.
bool readNumbers(const json::Value* value, const std::string& where, std::vector<double>& out, std::string& error) {
    if (value == nullptr || !value->isArray()) {
        error = where + " must be an array of numbers";
        return false;
    }
    out.clear();
    for (const json::Value& item : value->items) {
        if (!item.isNumber()) {
            error = where + " must contain only numbers";
            return false;
        }
        out.push_back(item.number);
    }
    return true;
}

} // namespace

std::optional<TrackLayout> parseTrackLayout(std::string_view json, std::string_view source, std::string& error) {
    const std::string prefix = std::string(source) + ": ";
    std::string parseError;
    const std::optional<json::Value> document = json::parse(json, parseError);
    if (!document) {
        error = prefix + parseError;
        return std::nullopt;
    }
    if (!document->isObject()) {
        error = prefix + "the layout must be a JSON object";
        return std::nullopt;
    }

    TrackLayout layout;
    if (const json::Value* name = document->find("name")) {
        if (name->isString()) {
            layout.name = name->text;
        }
    }
    if (const json::Value* length = document->find("length_m")) {
        if (length->isNumber()) {
            layout.lengthMeters = length->number;
        }
    }
    if (const json::Value* width = document->find("half_width_m")) {
        if (!width->isNumber() || width->number <= 0.0) {
            error = prefix + "'half_width_m' must be a positive number";
            return std::nullopt;
        }
        layout.halfWidthMeters = width->number;
    }

    const json::Value* centreline = document->find("centerline");
    if (centreline == nullptr || !centreline->isArray()) {
        error = prefix + "missing required 'centerline' array";
        return std::nullopt;
    }
    if (centreline->items.size() < 3) {
        error = prefix + "'centerline' needs at least three points";
        return std::nullopt;
    }
    layout.centreline.reserve(centreline->items.size());
    for (std::size_t i = 0; i < centreline->items.size(); ++i) {
        TrackLayout::Point point;
        if (!readPoint(centreline->items[i], point, "'centerline[" + std::to_string(i) + "]'", error)) {
            error = prefix + error;
            return std::nullopt;
        }
        layout.centreline.push_back(point);
    }

    if (const json::Value* start = document->find("start_finish")) {
        if (!start->isObject()) {
            error = prefix + "'start_finish' must be an object";
            return std::nullopt;
        }
        const json::Value* position = start->find("position");
        if (position != nullptr) {
            if (!readPoint(*position, layout.start, "'start_finish.position'", error)) {
                error = prefix + error;
                return std::nullopt;
            }
            layout.hasStart = true;
        }
        if (const json::Value* forward = start->find("forward"); forward != nullptr) {
            TrackLayout::Point direction;
            if (!readPoint(*forward, direction, "'start_finish.forward'", error)) {
                error = prefix + error;
                return std::nullopt;
            }
            layout.startHeadingRadians = headingOf(direction.x, direction.z);
        } else if (layout.hasStart) {
            // Face along the centreline where the line sits.
            std::size_t best = 0;
            double bestDistance = 1.0e300;
            for (std::size_t i = 0; i < layout.centreline.size(); ++i) {
                const double dx = layout.centreline[i].x - layout.start.x;
                const double dz = layout.centreline[i].z - layout.start.z;
                const double d = dx * dx + dz * dz;
                if (d < bestDistance) {
                    bestDistance = d;
                    best = i;
                }
            }
            const TrackLayout::Point& next = layout.centreline[(best + 1) % layout.centreline.size()];
            const TrackLayout::Point& here = layout.centreline[best];
            layout.startHeadingRadians = headingOf(next.x - here.x, next.z - here.z);
        }
    }

    if (const json::Value* grid = document->find("grid_slots")) {
        if (!grid->isArray()) {
            error = prefix + "'grid_slots' must be an array";
            return std::nullopt;
        }
        for (std::size_t i = 0; i < grid->items.size(); ++i) {
            const json::Value& item = grid->items[i];
            if (!item.isObject()) {
                error = prefix + "'grid_slots[" + std::to_string(i) + "]' must be an object";
                return std::nullopt;
            }
            TrackLayout::GridSlot slot;
            TrackLayout::Point position;
            const json::Value* node = item.find("position");
            if (node == nullptr ||
                !readPoint(*node, position, "'grid_slots[" + std::to_string(i) + "].position'", error)) {
                error = prefix + (error.empty() ? "'grid_slots[" + std::to_string(i) + "]' needs a 'position'" : error);
                return std::nullopt;
            }
            slot.x = position.x;
            slot.z = position.z;
            if (const json::Value* heading = item.find("heading_deg")) {
                if (!heading->isNumber()) {
                    error = prefix + "'grid_slots[" + std::to_string(i) + "].heading_deg' must be a number";
                    return std::nullopt;
                }
                slot.headingRadians = heading->number * kDegreesToRadians;
            }
            layout.grid.push_back(slot);
        }
    }

    // Sector boundaries are arc lengths from the line. The generated layouts
    // shipped the key as `sectors_s` although the values are metres; both names
    // are accepted so older circuits keep loading.
    const json::Value* sectors = document->find("sectors_m");
    if (sectors == nullptr) {
        sectors = document->find("sectors_s");
    }
    if (sectors != nullptr) {
        std::vector<double> values;
        if (!readNumbers(sectors, "'sectors_m'", values, error)) {
            error = prefix + error;
            return std::nullopt;
        }
        double previous = -1.0;
        for (const double value : values) {
            if (value < 0.0 || value <= previous) {
                error = prefix + "'sectors_m' must be positive and strictly increasing";
                return std::nullopt;
            }
            previous = value;
            layout.sectorDistances.push_back(value);
        }
        if (!layout.sectorDistances.empty() && layout.sectorDistances.front() != 0.0) {
            layout.sectorDistances.insert(layout.sectorDistances.begin(), 0.0);
        }
        if (!layout.sectorDistances.empty() && layout.lengthMeters > 0.0 &&
            layout.sectorDistances.back() < layout.lengthMeters) {
            layout.sectorDistances.push_back(layout.lengthMeters);
        }
    }

    if (const json::Value* pit = document->find("pit_lane")) {
        if (!pit->isObject()) {
            error = prefix + "'pit_lane' must be an object";
            return std::nullopt;
        }
        if (const json::Value* limit = pit->find("speed_limit_kmh")) {
            if (!limit->isNumber() || limit->number < 0.0) {
                error = prefix + "'pit_lane.speed_limit_kmh' must be a non-negative number";
                return std::nullopt;
            }
            layout.pitLane.speedLimitKmh = limit->number;
        }
        if (const json::Value* polyline = pit->find("polyline")) {
            if (!polyline->isArray()) {
                error = prefix + "'pit_lane.polyline' must be an array of [x, y, z] points";
                return std::nullopt;
            }
            for (std::size_t i = 0; i < polyline->items.size(); ++i) {
                TrackLayout::Point point;
                if (!readPoint(polyline->items[i], point, "'pit_lane.polyline[" + std::to_string(i) + "]'", error)) {
                    error = prefix + error;
                    return std::nullopt;
                }
                layout.pitLane.polyline.push_back(point);
            }
        }
    }

    return layout;
}

std::optional<TrackLayout> loadTrackLayout(const vfs::VirtualFileSystem& vfs, std::string_view path,
                                           std::string& error) {
    const auto text = vfs.readText(path);
    if (!text) {
        error = std::string(path) + ": cannot read the layout";
        return std::nullopt;
    }
    return parseTrackLayout(*text, path, error);
}

} // namespace sim::app::drive
