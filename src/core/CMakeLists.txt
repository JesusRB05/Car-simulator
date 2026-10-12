# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_core STATIC
    src/Clock.cpp
    src/Config.cpp
    src/JobSystem.cpp
    src/Log.cpp)
add_library(sim::core ALIAS sim_core)

target_include_directories(sim_core PUBLIC include)
target_link_libraries(sim_core
    PUBLIC
        glm::glm
        Eigen3::Eigen
        spdlog::spdlog
        tomlplusplus::tomlplusplus)
sim_configure_target(sim_core)
