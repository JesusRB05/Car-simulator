# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_vehicle STATIC
    src/Vehicle.cpp)
add_library(sim::vehicle ALIAS sim_vehicle)

target_include_directories(sim_vehicle PUBLIC include)
target_link_libraries(sim_vehicle
    PUBLIC
        sim::core
        sim::physics
        glm::glm)
sim_configure_target(sim_vehicle)
sim_enable_deterministic_fp(sim_vehicle)
