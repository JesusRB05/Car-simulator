# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_physics STATIC
    src/Constraint.cpp
    src/Curve.cpp
    src/ForceElements.cpp
    src/Integrator.cpp
    src/Joints.cpp
    src/PhysicsWorld.cpp
    src/RigidBody.cpp)
add_library(sim::physics ALIAS sim_physics)

target_include_directories(sim_physics PUBLIC include)
target_link_libraries(sim_physics
    PUBLIC
        sim::core
        glm::glm)
sim_configure_target(sim_physics)
sim_enable_deterministic_fp(sim_physics)
