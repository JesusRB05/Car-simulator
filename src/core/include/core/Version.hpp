// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#define SIM_VERSION_MAJOR 0
#define SIM_VERSION_MINOR 1
#define SIM_VERSION_PATCH 0

#define SIM_STRINGIFY_IMPL(x) #x
#define SIM_STRINGIFY(x) SIM_STRINGIFY_IMPL(x)

namespace sim::core {

inline constexpr int kVersionMajor = SIM_VERSION_MAJOR;
inline constexpr int kVersionMinor = SIM_VERSION_MINOR;
inline constexpr int kVersionPatch = SIM_VERSION_PATCH;

inline constexpr const char* kVersionString =
    SIM_STRINGIFY(SIM_VERSION_MAJOR) "." SIM_STRINGIFY(SIM_VERSION_MINOR) "." SIM_STRINGIFY(SIM_VERSION_PATCH);

} // namespace sim::core
