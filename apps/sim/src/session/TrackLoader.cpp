// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/TrackLoader.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include <toml++/toml.hpp>

#include "drive/Track.hpp"
#include "drive/TrackLayout.hpp"
#include "modding/Composition.hpp"
#include "modding/Package.hpp"
#include "session/ContentToml.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

using content::ParseContext;

// How many centreline points of a layout become Catmull-Rom control points. The
// layout is authored every few metres; keeping one of every four (about 16 m)
// reproduces the real path to the centimetre while keeping the analytic circuit
// small. The driven surface comes from the collision mesh, not from this spline.
constexpr std::size_t kLayoutControlStride = 4;

// Reads `model` (a glTF) and `layout` (a track.json), both relative to the
// descriptor folder, and resolves them against the VFS.
bool readCircuitFiles(const toml::table& root, std::string_view sourcePath, const vfs::VirtualFileSystem* vfs,
                      std::string& modelPath, std::string& layoutPath, ParseContext& context) {
    const std::string directory = std::string(sourcePath.substr(0, sourcePath.find_last_of('/')));
    const auto resolve = [&](const std::string& relative) { return directory + "/" + relative; };

    if (const toml::node* node = root.get("model")) {
        const auto value = node->value<std::string>();
        if (!value || value->empty()) {
            context.fail("'model' must be a non-empty string");
            return false;
        }
        if (value->size() < 5 ||
            (value->compare(value->size() - 4, 4, ".glb") != 0 && value->compare(value->size() - 5, 5, ".gltf") != 0)) {
            context.fail("'model' must name a .glb or .gltf file (got '" + *value + "')");
            return false;
        }
        modelPath = resolve(*value);
        if (vfs != nullptr && !vfs->exists(modelPath)) {
            context.fail("'model' names a missing file: " + modelPath);
            return false;
        }
    }

    if (const toml::node* node = root.get("layout")) {
        const auto value = node->value<std::string>();
        if (!value || value->empty()) {
            context.fail("'layout' must be a non-empty string");
            return false;
        }
        if (value->size() < 5 || value->compare(value->size() - 5, 5, ".json") != 0) {
            context.fail("'layout' must name a .json file (got '" + *value + "')");
            return false;
        }
        layoutPath = resolve(*value);
        if (vfs != nullptr && !vfs->exists(layoutPath)) {
            context.fail("'layout' names a missing file: " + layoutPath);
            return false;
        }
    }
    return true;
}

// Derives the analytic fallback circuit from a layout: the centreline becomes
// control points and the start line sets `startZ`, so lap timing, the minimap
// and the autopilot all follow the authored path.
void describeFromLayout(const drive::TrackLayout& layout, drive::TrackDescription& description) {
    if (layout.centreline.size() >= 3) {
        description.controlPoints.clear();
        const std::size_t stride = layout.centreline.size() >= 8 ? kLayoutControlStride : 1;
        for (std::size_t i = 0; i < layout.centreline.size(); i += stride) {
            const drive::TrackLayout::Point& point = layout.centreline[i];
            description.controlPoints.push_back({point.x, point.z});
        }
    }
    if (layout.halfWidthMeters > 0.0) {
        description.nominalHalfWidth = layout.halfWidthMeters;
    }
    if (layout.hasStart) {
        description.startZ = layout.start.z;
    }
    // The kerbs are part of the model; the analytic fallback does not need them.
    description.kerbWidth = 0.0;
}

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

std::optional<TrackSpec> parseTrackSpecImpl(const toml::table& root, std::string_view sourcePath,
                                            const vfs::VirtualFileSystem* vfs, ParseContext& context) {
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

    if (!readCircuitFiles(root, sourcePath, vfs, spec.modelPath, spec.layoutPath, context)) {
        return std::nullopt;
    }
    if (!spec.layoutPath.empty() && vfs != nullptr) {
        std::string layoutError;
        std::optional<drive::TrackLayout> layout = drive::loadTrackLayout(*vfs, spec.layoutPath, layoutError);
        if (!layout) {
            context.fail(layoutError);
        } else {
            spec.layout = std::move(*layout);
        }
    }
    const bool hasLayout = spec.layout.has_value();

    drive::TrackDescription description;
    if (hasLayout) {
        describeFromLayout(*spec.layout, description);
    }
    const toml::table* geometry = root["geometry"].as_table();
    if (geometry == nullptr) {
        if (!hasLayout) {
            context.fail("missing required '[geometry]' table (a 'layout' replaces it)");
        }
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
        if (geometry->get("control_points") != nullptr) {
            readControlPoints(*geometry, description.controlPoints, context);
        } else if (!hasLayout) {
            context.fail("missing required 'control_points' (or a 'layout')");
        }
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
    if (spec.layout && spec.layout->lengthMeters > 0.0) {
        // The authored length is authoritative: the spline is an approximation of
        // the same path, and a lap is timed against the layout's sectors.
        spec.lengthMeters = spec.layout->lengthMeters;
    }
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

std::optional<TrackSpec> parseTrackSpec(std::string_view text, std::string_view sourcePath, std::string& error,
                                        const vfs::VirtualFileSystem* vfs) {
    toml::table root;
    try {
        root = toml::parse(text, sourcePath);
    } catch (const toml::parse_error& parseError) {
        error = std::string(sourcePath) + ": " + std::string(parseError.description());
        return std::nullopt;
    }

    ParseContext context{std::string(sourcePath), {}};
    std::optional<TrackSpec> spec = parseTrackSpecImpl(root, sourcePath, vfs, context);
    if (!spec) {
        error = context.joined();
    }
    return spec;
}

// Same as parseTrackSpec, on an already-parsed table (so patches can be applied
// to it first).
std::optional<TrackSpec> parseTrackSpecTable(const toml::table& root, std::string_view sourcePath, std::string& error,
                                             const vfs::VirtualFileSystem* vfs) {
    ParseContext context{std::string(sourcePath), {}};
    std::optional<TrackSpec> spec = parseTrackSpecImpl(root, sourcePath, vfs, context);
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
            spec = parseTrackSpecTable(table, path, error, &vfs);
        } else {
            spec = parseTrackSpec(*text, path, error, &vfs);
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
