# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_net STATIC
    src/HttpClient.cpp
    src/NetworkSystem.cpp)
add_library(sim::net ALIAS sim_net)

target_include_directories(sim_net PUBLIC include)
target_link_libraries(sim_net
    PUBLIC
        sim::core)
sim_configure_target(sim_net)
