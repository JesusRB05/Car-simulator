// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/TrackCatalog.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "drive/Track.hpp"

namespace sim::app {
namespace {

// Samples the real circuit and normalises its centreline into a unit box for
// the 2D minimap. The circuit is built once by the Track constructor.
TrackSpec makeSunsetCircuit() {
    const drive::Track track;

    TrackSpec spec;
    spec.id = "sunset_circuit";
    spec.name = "Sunset Circuit";
    spec.location = "Procedural test loop";
    spec.description =
        "The base test circuit: a long straight, a right hairpin, a fast sweeper and a chicane draped over a "
        "rolling terrain of a few smooth hills. Width and banking change along the lap. Defined in code and "
        "mirrored by content/base/tracks/sunset_circuit/track.toml.";
    spec.lengthMeters = static_cast<double>(track.length());
    spec.widthMeters = 2.0 * static_cast<double>(track.halfWidth());
    spec.variants = {"Full circuit"};

    const std::vector<drive::TrackSample>& samples = track.samples();
    if (samples.size() < 2) {
        return spec;
    }

    double minX = samples.front().position.x;
    double maxX = minX;
    double minZ = samples.front().position.z;
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
    spec.minimap.reserve(samples.size() / stride + 1);
    for (std::size_t i = 0; i < samples.size(); i += stride) {
        const drive::TrackSample& sample = samples[i];
        spec.minimap.emplace_back(static_cast<float>((sample.position.x - centreX) / scale),
                                  static_cast<float>((sample.position.z - centreZ) / scale));
    }
    if (!spec.minimap.empty() && spec.minimap.front() != spec.minimap.back()) {
        spec.minimap.push_back(spec.minimap.front()); // close the loop
    }
    return spec;
}

} // namespace

TrackCatalog::TrackCatalog() {
    tracks_.push_back(makeSunsetCircuit());
}

const TrackCatalog& TrackCatalog::instance() {
    static const TrackCatalog catalog;
    return catalog;
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
