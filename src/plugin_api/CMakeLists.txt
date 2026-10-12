# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_plugin_api STATIC
    src/PluginLoader.cpp)
add_library(sim::plugin_api ALIAS sim_plugin_api)

target_include_directories(sim_plugin_api PUBLIC include)
target_link_libraries(sim_plugin_api
    PUBLIC
        sim::core
        sim::vfs
        SDL3::SDL3)
sim_configure_target(sim_plugin_api)
