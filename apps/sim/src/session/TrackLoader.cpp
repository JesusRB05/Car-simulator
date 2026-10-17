// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/TrackLoader.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include <toml++/toml.hpp>

#include "drive/Track.hpp"
#include "modding/Composition.hpp"
#include "modding/Package.hpp"
#include "session/ContentToml.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

using content::ParseContext;

// Reads the `control_points = [[x, z], ...]` array of the geometry table.
bool readControlPoints(const toml::table& geometry, std::vector<drive::TrackDescription::ControlPoint>& out,
                       ParseContext& context) {
    const toml::node* node = geometry.get("control_points");
    if (node == nullptr) {
        context.fail("missing required 'control_points'");
        return false;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr || array->size() < 3) {
        context.fail("'control_points' must be an array of at least three [x, z] pairs");
        return false;
    }
    std::vector<drive::TrackDescription::ControlPoint> points;
    points.reserve(array->size());
    for (std::size_t i = 0; i < array->size(); ++i) {
        const toml::array* pair = (*array)[i].as_array();
        if (pair == nullptr || pair->size() != 2) {
            context.fail("'control_points[" + std::to_string(i) + "]' must be an [x, z] pair");
            return false;
        }
        const auto x = (*pair)[0].value<double>();
        const auto z = (*pair)[1].value<double>();
        if (!x || !z) {
            context.fail("'control_points[" + std::to_string(i) + "]' must contain two numbers");
            return false;
        }
        if (std::abs(*x) > 1.0e4 || std::abs(*z) > 1.0e4) {
            context.fail("'control_points[" + std::to_string(i) + "]' is outside +/-10000 m");
            return false;
        }
        points.push_back({*x, *z});
    }
    out = std::move(points);
    return true;
}

// Reads the optional `[[terrain.hills]]` array of Gaussian hills.
bool readHills(const toml::table& root, std::vector<drive::TrackHill>& out, ParseContext& context) {
    const toml::node* node = root.get("terrain");
    if (node == nullptr) {
        return true;
    }
    const toml::table* terrain = node->as_table();
    if (terrain == nullptr) {
        context.fail("'terrain' must be a table");
        return false;
    }
    const toml::node* hillsNode = terrain->get("hills");
    if (hillsNode == nullptr) {
        return true;
    }
    const toml::array* array = hillsNode->as_array();
    if (array == nullptr) {
        context.fail("'terrain.hills' must be an array of tables");
        return false;
    }
    std::vector<drive::TrackHill> hills;
    hills.reserve(array->size());
    for (std::size_t i = 0; i < array->size(); ++i) {
        const toml::table* hill = (*array)[i].as_table();
        if (hill == nullptr) {
            context.fail("'terrain.hills[" + std::to_string(i) + "]' must be a table");
            return false;
        }
        drive::TrackHill entry;
        content::readNumber(*hill, "x", -1.0e4, 1.0e4, entry.x, context);
        content::readNumber(*hill, "z", -1.0e4, 1.0e4, entry.z, context);
        content::readNumber(*hill, "height", -5.0e2, 5.0e2, entry.height, context);
        content::readNumber(*hill, "sigma", 1.0, 1.0e3, entry.sigma, context);
        hills.push_back(entry);
    }
    out = std::move(hills);
    return context.ok();
}

std::optional<TrackSpec> parseTrackSpecImpl(const toml::table& root, ParseContext& context) {
    TrackSpec spec;
    spec.id = content::readString(root, "id", {});
    spec.name = content::readString(root, "name", spec.id);
    spec.description = content::readString(root, "description", "");
    spec.location = content::readString(root, "location", "Data-defined circuit");
    if (const toml::node* idNode = root.get("id")) {
        if (!content::isValidId(spec.id)) {
            context.failAt(*idNode, "id",
                           "must be a lower-case identifier of letters, digits, '_' and '-' (got '" + spec.id + "')");
        }
    } else {
        context.fail("missing required field 'id'");
    }

    drive::TrackDescription description;
    const toml::table* geometry = root["geometry"].as_table();
    if (geometry == nullptr) {
        context.fail("missing required '[geometry]' table");
    } else {
        if (const toml::node* loop = geometry->get("loop")) {
            const auto value = loop->value<bool>();
            if (!value) {
                context.fail("'geometry.loop' must be a boolean");
            } else if (!*value) {
                context.fail("open circuits are not supported yet ('geometry.loop' must be true)");
            }
        }
        content::readNumber(*geometry, "sample_spacing", 0.1, 10.0, description.sampleSpacing, context);
        content::readNumber(*geometry, "half_width", 1.0, 20.0, description.nominalHalfWidth, context);
        content::readNumber(*geometry, "kerb_width", 0.0, 10.0, description.kerbWidth, context);
        content::readNumber(*geometry, "start_z", -1.0e4, 1.0e4, description.startZ, context);
        readControlPoints(*geometry, description.controlPoints, context);
    }
    description.loop = true;
    readHills(root, description.hills, context);

    if (!context.ok()) {
        return std::nullopt;
    }

    // Keep the description so the drive scene rebuilds exactly this circuit.
    spec.track = description;

    // Build the real circuit to measure it and to derive the minimap.
    const drive::Track track(description);
    spec.variants = {"Full circuit"};
    fillTrackGeometry(spec, track);
    return spec;
}

} // namespace

void fillTrackGeometry(TrackSpec& spec, const drive::Track& track) {
    spec.lengthMeters = static_cast<double>(track.length());
    spec.widthMeters = 2.0 * static_cast<double>(track.halfWidth());

    const std::vector<drive::TrackSample>& samples = track.samples();
    if (samples.size() < 2) {
        return;
    }

    double minX = static_cast<double>(samples.front().position.x);
    double maxX = minX;
    double minZ = static_cast<double>(samples.front().position.z);
    double maxZ = minZ;
    for (const drive::TrackSample& sample : samples) {
        minX = std::min(minX, static_cast<double>(sample.position.x));
        maxX = std::max(maxX, static_cast<double>(sample.position.x));
        minZ = std::min(minZ, static_cast<double>(sample.position.z));
        maxZ = std::max(maxZ, static_cast<double>(sample.position.z));
    }
    const double centreX = 0.5 * (minX + maxX);
    const double centreZ = 0.5 * (minZ + maxZ);
    const double halfX = std::max(1e-3, 0.5 * (maxX - minX));
    const double halfZ = std::max(1e-3, 0.5 * (maxZ - minZ));
    spec.aspect = static_cast<float>(halfX / halfZ);

    const std::size_t stride = std::max<std::size_t>(1, samples.size() / 256);
    const double scale = static_cast<double>(std::max(halfX, halfZ));
    spec.minimap.clear();
    spec.minimap.reserve(samples.size() / stride + 1);
    for (std::size_t i = 0; i < samples.size(); i += stride) {
        const drive::TrackSample& sample = samples[i];
        spec.minimap.emplace_back(static_cast<float>((sample.position.x - centreX) / scale),
                                  static_cast<float>((sample.position.z - centreZ) / scale));
    }
    if (!spec.minimap.empty() && spec.minimap.front() != spec.minimap.back()) {
        spec.minimap.push_back(spec.minimap.front()); // close the loop
    }
}

std::optional<TrackSpec> parseTrackSpec(std::string_view text, std::string_view sourcePath, std::string& error) {
    toml::table root;
    try {
        root = toml::parse(text, sourcePath);
    } catch (const toml::parse_error& parseError) {
        error = std::string(sourcePath) + ": " + std::string(parseError.description());
        return std::nullopt;
    }

    ParseContext context{std::string(sourcePath), {}};
    std::optional<TrackSpec> spec = parseTrackSpecImpl(root, context);
    if (!spec) {
        error = context.joined();
    }
    return spec;
}

// Same as parseTrackSpec, on an already-parsed table (so patches can be applied
// to it first).
std::optional<TrackSpec> parseTrackSpecTable(const toml::table& root, std::string_view sourcePath, std::string& error) {
    ParseContext context{std::string(sourcePath), {}};
    std::optional<TrackSpec> spec = parseTrackSpecImpl(root, context);
    if (!spec) {
        error = context.joined();
    }
    return spec;
}

TrackLoadResult loadTracks(const vfs::VirtualFileSystem& vfs, const ContentOwner& owner,
                           const modding::Composition* composition) {
    TrackLoadResult result;
    const std::string root(modding::kTracksDir);

    std::vector<std::string> entries = vfs.list(root);
    std::sort(entries.begin(), entries.end());
    for (const std::string& entry : entries) {
        const std::string directory = root + "/" + entry;
        if (!vfs.isDirectory(directory)) {
            continue;
        }
        const std::string path = directory + "/track.toml";
        if (!vfs.exists(path)) {
            continue;
        }
        const auto text = vfs.readText(path);
        if (!text) {
            result.errors.push_back(path + ": cannot read the descriptor");
            continue;
        }

        const std::string modId = owner ? owner(path) : std::string{};
        const std::string ref = modId.empty() ? entry : modId + "/" + entry;
        const bool patched = composition != nullptr && !composition->patchesFor(ref).empty();

        std::string error;
        std::optional<TrackSpec> spec;
        if (patched) {
            toml::table table;
            try {
                table = toml::parse(*text, path);
            } catch (const toml::parse_error& parseError) {
                result.errors.push_back(path + ": " + std::string(parseError.description()));
                continue;
            }
            modding::applyPatches(composition->patchesFor(ref), table, &result.errors);
            spec = parseTrackSpecTable(table, path, error);
        } else {
            spec = parseTrackSpec(*text, path, error);
        }

        if (spec) {
            spec->modId = modId;
            result.tracks.push_back(std::move(*spec));
        } else {
            result.errors.push_back(std::move(error));
        }
    }

    std::sort(result.tracks.begin(), result.tracks.end(),
              [](const TrackSpec& a, const TrackSpec& b) { return a.ref() < b.ref(); });
    return result;
}

} // namespace sim::app
