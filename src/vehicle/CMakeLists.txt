# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
add_library(sim_vehicle STATIC
    # Header-only abstract interfaces (listed for IDE visibility).
    include/vehicle/IAerodynamics.hpp
    include/vehicle/IBrakeSystem.hpp
    include/vehicle/IDrivetrain.hpp
    include/vehicle/ISuspension.hpp
    include/vehicle/ITireModel.hpp
    include/vehicle/IVehicleElectronics.hpp
    include/vehicle/IVehicleModel.hpp
    # Pure-slip Pacejka Magic Formula tyre model (Phase 2, P2.2 core).
    include/vehicle/MagicFormulaTire.hpp
    src/MagicFormulaTire.cpp
    # Nonlinear vertical spring-damper and effective radius (Phase 2, P2.1).
    include/vehicle/TireVertical.hpp
    src/TireVertical.cpp
    # Bristle (LuGre) friction for standstill and very low speed (Phase 2, P2.4).
    include/vehicle/TireBristle.hpp
    src/TireBristle.cpp
    # Implicit longitudinal wheel-spin integration (Phase 2, P2.1).
    include/vehicle/WheelSpin.hpp
    src/WheelSpin.cpp
    # TEMPORARY kinematic placeholder implementing IVehicleModel.
    include/vehicle/PlaceholderVehicle.hpp
    src/PlaceholderVehicle.cpp)
add_library(sim::vehicle ALIAS sim_vehicle)

target_include_directories(sim_vehicle PUBLIC include)
target_link_libraries(sim_vehicle
    PUBLIC
        sim::core
        sim::physics
        glm::glm)
sim_configure_target(sim_vehicle)
sim_enable_deterministic_fp(sim_vehicle)
