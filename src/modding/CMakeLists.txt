# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_modding STATIC
    src/Composition.cpp
    src/DependencyResolver.cpp
    src/Manifest.cpp
    src/ModInstaller.cpp
    src/ModManager.cpp
    src/Package.cpp
    src/Version.cpp)
add_library(sim::modding ALIAS sim_modding)

target_include_directories(sim_modding PUBLIC include)
target_link_libraries(sim_modding
    PUBLIC
        sim::core
        sim::vfs
        tomlplusplus::tomlplusplus)
sim_configure_target(sim_modding)
