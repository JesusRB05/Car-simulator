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
    # Double-wishbone hardpoint kinematics, the multibody reference (Phase 2, P2.5).
    include/vehicle/DoubleWishbone.hpp
    src/DoubleWishbone.cpp
    # Spring, damper and progressive stops (Phase 2, P2.7).
    include/vehicle/SpringDamper.hpp
    src/SpringDamper.cpp
    # Anti-roll bar (Phase 2, P2.7).
    include/vehicle/AntiRollBar.hpp
    src/AntiRollBar.cpp
    # Rubber bushings, per-axis 6-DOF compliance (Phase 2, P2.7).
    include/vehicle/Bushing.hpp
    src/Bushing.cpp
    # Steering geometry, Ackermann and column torque (Phase 2, P2.8).
    include/vehicle/Steering.hpp
    src/Steering.cpp
    # Mass properties and aerodynamics (Phase 2, P2.9).
    include/vehicle/MassProperties.hpp
    src/MassProperties.cpp
    include/vehicle/Aerodynamics.hpp
    src/Aerodynamics.cpp
    # Engine torque map and friction (Phase 2, P2.10).
    include/vehicle/Engine.hpp
    src/Engine.cpp
    # Friction and centrifugal clutches (Phase 2, P2.10).
    include/vehicle/Clutch.hpp
    src/Clutch.cpp
    # Gearbox ratios and torque converter (Phase 2, P2.10).
    include/vehicle/Gearbox.hpp
    src/Gearbox.cpp
    # Differentials (Phase 2, P2.10).
    include/vehicle/Differential.hpp
    src/Differential.cpp
    # Driveline torsional compliance (Phase 2, P2.10).
    include/vehicle/DrivelineCompliance.hpp
    src/DrivelineCompliance.cpp
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
