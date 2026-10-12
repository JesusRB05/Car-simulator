# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_scripting STATIC
    src/LuaSandbox.cpp
    src/ScriptHost.cpp)
add_library(sim::scripting ALIAS sim_scripting)

target_include_directories(sim_scripting PUBLIC include)
target_link_libraries(sim_scripting
    PUBLIC
        sim::core
        sim::vfs
        sim::luajit
        sol2::sol2)
sim_configure_target(sim_scripting)
