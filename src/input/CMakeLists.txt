# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_input STATIC
    include/input/Backend.hpp
    include/input/Calibration.hpp
    include/input/CompatibilityDatabase.hpp
    include/input/ControlMapping.hpp
    include/input/ForceFeedback.hpp
    include/input/Profiles.hpp
    include/input/Types.hpp
    include/input/WheelManager.hpp
    src/Calibration.cpp
    src/CompatibilityDatabase.cpp
    src/ControlMapping.cpp
    src/ForceFeedback.cpp
    src/Profiles.cpp
    src/Types.cpp
    src/WheelManager.cpp)
add_library(sim::input ALIAS sim_input)

target_include_directories(sim_input PUBLIC include)
target_link_libraries(sim_input
    PUBLIC
        sim::core)
sim_configure_target(sim_input)
