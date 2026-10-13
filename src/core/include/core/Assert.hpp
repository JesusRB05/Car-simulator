// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstdlib>
#include <sstream>
#include <string>

#include "core/Log.hpp"

namespace sim::core {

// Terminates with diagnostics when a programming invariant is broken. Assertions
// are always active in debug builds and are compiled out in release builds.
[[noreturn]] inline void assertionFailed(const char* expr, const char* file, int line, const std::string& msg) {
    std::string text = std::string("assertion failed: (") + expr + ")\n  at " + file + ":" + std::to_string(line);
    if (!msg.empty()) {
        text += "\n  " + msg;
    }
    Log::get("assert")->critical("{}", text);
    std::abort();
}

template<typename... Args>
[[noreturn]] inline void assertionFailedFmt(const char* expr, const char* file, int line, Args&&... args) {
    std::ostringstream oss;
    ((oss << args), ...);
    assertionFailed(expr, file, line, oss.str());
}

} // namespace sim::core

#if defined(SIM_DEBUG) && SIM_DEBUG
#define SIM_ASSERT(expr, ...)                                                                                          \
    do {                                                                                                               \
        if (!(expr)) {                                                                                                 \
            ::sim::core::assertionFailedFmt(#expr, __FILE__, __LINE__ __VA_OPT__(, ) __VA_ARGS__);                     \
        }                                                                                                              \
    } while (false)
#else
#define SIM_ASSERT(expr, ...) ((void)0)
#endif
