# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_telemetry STATIC
    src/Telemetry.cpp)
add_library(sim::telemetry ALIAS sim_telemetry)

target_include_directories(sim_telemetry PUBLIC include)
target_link_libraries(sim_telemetry
    PUBLIC
        sim::core
        sim::vfs)
sim_configure_target(sim_telemetry)
