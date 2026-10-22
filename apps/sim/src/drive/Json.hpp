// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

// A small, strict JSON reader. The only JSON the game consumes is the circuit
// layout (`track.json`), which is part of the content format and must be
// validated with precise errors instead of being trusted. Keeping the parser
// here (instead of pulling a library) keeps the dependency list short and the
// format testable without a device.

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sim::app::json {

struct Value {
    enum class Type { Null, Bool, Number, String, Array, Object };

    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string text;
    std::vector<Value> items;                           // Array
    std::vector<std::pair<std::string, Value>> members; // Object, in file order

    bool isNull() const { return type == Type::Null; }
    bool isBool() const { return type == Type::Bool; }
    bool isNumber() const { return type == Type::Number; }
    bool isString() const { return type == Type::String; }
    bool isArray() const { return type == Type::Array; }
    bool isObject() const { return type == Type::Object; }

    // A member of an object, or nullptr. Duplicate keys resolve to the first
    // occurrence (the parser rejects duplicates, so this is only defensive).
    const Value* find(std::string_view key) const;
    // Element `index` of an array, or nullptr.
    const Value* at(std::size_t index) const;
    std::size_t size() const { return isArray() ? items.size() : members.size(); }
};

// Parses `text`. On failure returns nullopt and fills `error` with a message
// that names the line and column. The input must be exactly one value plus
// optional whitespace.
std::optional<Value> parse(std::string_view text, std::string& error);

} // namespace sim::app::json
