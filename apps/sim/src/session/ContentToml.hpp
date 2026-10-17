// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// Small helpers shared by the content loaders (cars, tracks). They read typed
// values out of a TOML table, validate that each is inside a plausible SI range
// and collect precise error messages, so a modder is told which key is wrong and
// what range is allowed instead of just "invalid file".

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <toml++/toml.hpp>

#include "physics/PhysicsTypes.hpp"

namespace sim::app::content {

inline std::string formatNumber(double value) {
    std::ostringstream out;
    out.precision(6);
    out << value;
    return out.str();
}

// A descriptor id: lower-case letters, digits, underscores and hyphens.
inline bool isValidId(std::string_view id) {
    if (id.empty()) {
        return false;
    }
    return std::all_of(id.begin(), id.end(), [](unsigned char c) {
        return std::islower(c) != 0 || std::isdigit(c) != 0 || c == '_' || c == '-';
    });
}

// Collects the errors of one descriptor.
struct ParseContext {
    std::string source;
    std::vector<std::string> errors;

    void fail(std::string message) { errors.push_back(source + ": " + std::move(message)); }

    // Names the exact place of a value so a modder knows which line to fix:
    // "<file>: line N, key 'a.b': <message>" (value and expectation are in the
    // message). Falls back to just the file when the node has no source region.
    void failAt(const toml::node& node, std::string_view key, std::string message) {
        const toml::source_region region = node.source();
        std::string where = source;
        if (region.begin.line > 0) {
            where += ": line " + std::to_string(region.begin.line);
        }
        if (!key.empty()) {
            where += ", key '" + std::string(key) + "'";
        }
        errors.push_back(where + ": " + std::move(message));
    }

    bool ok() const { return errors.empty(); }

    std::string joined() const {
        std::ostringstream out;
        for (std::size_t i = 0; i < errors.size(); ++i) {
            if (i != 0) {
                out << "; ";
            }
            out << errors[i];
        }
        return out.str();
    }
};

// Reads an optional number, keeping `out` unchanged when the key is absent.
inline bool readNumber(const toml::table& table, std::string_view key, double low, double high, double& out,
                       ParseContext& context) {
    const toml::node* node = table.get(key);
    if (node == nullptr) {
        return true;
    }
    const auto value = node->value<double>();
    if (!value) {
        context.failAt(*node, key, "must be a number");
        return false;
    }
    if (!(low <= *value && *value <= high)) {
        context.failAt(*node, key,
                       "= " + formatNumber(*value) + " is outside the allowed range [" + formatNumber(low) + ", " +
                           formatNumber(high) + "]");
        return false;
    }
    out = *value;
    return true;
}

// Reads an optional [x, y] pair.
inline bool readVec2(const toml::table& table, std::string_view key, double low, double high, glm::dvec2& out,
                     ParseContext& context) {
    const toml::node* node = table.get(key);
    if (node == nullptr) {
        return true;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr || array->size() != 2) {
        context.failAt(*node, key, "must be an array of two numbers");
        return false;
    }
    glm::dvec2 value{};
    for (std::size_t i = 0; i < 2; ++i) {
        const toml::node& componentNode = (*array)[i];
        const std::string elementKey = std::string(key) + "[" + std::to_string(i) + "]";
        const auto component = componentNode.value<double>();
        if (!component) {
            context.failAt(componentNode, elementKey, "must be a number");
            return false;
        }
        if (!(low <= *component && *component <= high)) {
            context.failAt(componentNode, elementKey,
                           "= " + formatNumber(*component) + " is outside the allowed range [" + formatNumber(low) +
                               ", " + formatNumber(high) + "]");
            return false;
        }
        value[static_cast<glm::length_t>(i)] = *component;
    }
    out = value;
    return true;
}

// Reads an optional [x, y, z] triple.
inline bool readVec3(const toml::table& table, std::string_view key, double low, double high, physics::Vec3& out,
                     ParseContext& context) {
    const toml::node* node = table.get(key);
    if (node == nullptr) {
        return true;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr || array->size() != 3) {
        context.failAt(*node, key, "must be an array of three numbers");
        return false;
    }
    physics::Vec3 value{};
    for (std::size_t i = 0; i < 3; ++i) {
        const toml::node& componentNode = (*array)[i];
        const std::string elementKey = std::string(key) + "[" + std::to_string(i) + "]";
        const auto component = componentNode.value<double>();
        if (!component) {
            context.failAt(componentNode, elementKey, "must be a number");
            return false;
        }
        if (!(low <= *component && *component <= high)) {
            context.failAt(componentNode, elementKey,
                           "= " + formatNumber(*component) + " is outside the allowed range [" + formatNumber(low) +
                               ", " + formatNumber(high) + "]");
            return false;
        }
        value[static_cast<glm::length_t>(i)] = *component;
    }
    out = value;
    return true;
}

// Reads an optional non-empty array of numbers.
inline bool readNumberArray(const toml::table& table, std::string_view key, double low, double high,
                            std::vector<double>& out, ParseContext& context) {
    const toml::node* node = table.get(key);
    if (node == nullptr) {
        return true;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr || array->empty()) {
        context.failAt(*node, key, "must be a non-empty array of numbers");
        return false;
    }
    std::vector<double> values;
    values.reserve(array->size());
    for (std::size_t i = 0; i < array->size(); ++i) {
        const toml::node& componentNode = (*array)[i];
        const std::string elementKey = std::string(key) + "[" + std::to_string(i) + "]";
        const auto component = componentNode.value<double>();
        if (!component) {
            context.failAt(componentNode, elementKey, "must be a number");
            return false;
        }
        if (!(low <= *component && *component <= high)) {
            context.failAt(componentNode, elementKey,
                           "= " + formatNumber(*component) + " is outside the allowed range [" + formatNumber(low) +
                               ", " + formatNumber(high) + "]");
            return false;
        }
        values.push_back(*component);
    }
    out = std::move(values);
    return true;
}

inline std::string readString(const toml::table& root, std::string_view key, std::string fallback) {
    if (const toml::node* node = root.get(key)) {
        if (const auto value = node->value<std::string>()) {
            return *value;
        }
    }
    return fallback;
}

inline bool strictlyIncreasing(const std::vector<double>& values) {
    for (std::size_t i = 1; i < values.size(); ++i) {
        if (!(values[i] > values[i - 1])) {
            return false;
        }
    }
    return true;
}

inline bool strictlyDecreasing(const std::vector<double>& values) {
    for (std::size_t i = 1; i < values.size(); ++i) {
        if (!(values[i] < values[i - 1])) {
            return false;
        }
    }
    return true;
}

} // namespace sim::app::content
