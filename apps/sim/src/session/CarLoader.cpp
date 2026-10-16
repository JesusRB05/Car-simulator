// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#include "session/CarLoader.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

#include <toml++/toml.hpp>

#include "session/ContentToml.hpp"
#include "vehicle/VehicleModel.hpp"
#include "vfs/VirtualFileSystem.hpp"

namespace sim::app {
namespace {

namespace vehicle = sim::vehicle;
using content::ParseContext;

bool readColor(const toml::table& root, glm::vec4& out, ParseContext& context) {
    const toml::node* node = root.get("color");
    if (node == nullptr) {
        return true;
    }
    const toml::array* array = node->as_array();
    if (array == nullptr || (array->size() != 3 && array->size() != 4)) {
        context.fail("'color' must be an array of three or four numbers in [0, 1]");
        return false;
    }
    glm::vec4 color = out;
    for (std::size_t i = 0; i < array->size(); ++i) {
        const auto component = (*array)[i].value<double>();
        if (!component || *component < 0.0 || *component > 1.0) {
            context.fail("'color[" + std::to_string(i) + "]' must be a number in [0, 1]");
            return false;
        }
        color[static_cast<glm::length_t>(i)] = static_cast<float>(*component);
    }
    out = color;
    return true;
}

std::optional<CarSpec> parseCarSpecImpl(const toml::table& root, ParseContext& context) {
    CarSpec car;
    car.id = content::readString(root, "id", {});
    car.model = content::readString(root, "name", car.id);
    car.manufacturer = content::readString(root, "author", "Unknown");
    car.category = content::readString(root, "category", "Data-defined car");
    car.description = content::readString(root, "description", "");
    if (!content::isValidId(car.id)) {
        context.fail("'id' must be a lower-case identifier of letters, digits and '_' (got '" + car.id + "')");
    }

    vehicle::VehicleSetup setup = vehicle::makeDefaultVehicleSetup();

    // --- Chassis -----------------------------------------------------------
    if (const toml::table* chassis = root["chassis"].as_table()) {
        content::readNumber(*chassis, "mass", 100.0, 5000.0, setup.mass, context);
        content::readVec3(*chassis, "inertia", 1.0, 5.0e4, setup.principalInertia, context);
        content::readVec3(*chassis, "half_extents", 0.1, 5.0, setup.bodyHalfExtents, context);
    }

    // --- Geometry ----------------------------------------------------------
    if (const toml::table* geometry = root["geometry"].as_table()) {
        content::readNumber(*geometry, "front_axle_z", -5.0, -0.1, setup.frontAxleZ, context);
        content::readNumber(*geometry, "rear_axle_z", 0.1, 5.0, setup.rearAxleZ, context);
        content::readNumber(*geometry, "half_track", 0.2, 2.0, setup.halfTrack, context);
        content::readNumber(*geometry, "mount_y", -2.0, 2.0, setup.mountY, context);
        content::readNumber(*geometry, "rest_length", 0.05, 1.0, setup.restLength, context);
    }
    if (!(setup.frontAxleZ < 0.0 && setup.rearAxleZ > 0.0 && setup.rearAxleZ - setup.frontAxleZ >= 0.5)) {
        context.fail("geometry must put the front axle before the rear axle with a wheelbase of at least 0.5 m");
    }
    setup.ackermann.track = 2.0 * setup.halfTrack;
    setup.ackermann.wheelbase = setup.rearAxleZ - setup.frontAxleZ;

    // --- Suspension --------------------------------------------------------
    vehicle::SpringDamperCoefficients frontSpring = setup.frontSpring;
    if (const toml::table* suspension = root["suspension"].as_table()) {
        content::readNumber(*suspension, "spring_stiffness_front", 1.0e3, 5.0e5, frontSpring.stiffness, context);
        content::readNumber(*suspension, "damper_bump", 0.0, 1.0e5, frontSpring.dampingCompression, context);
        content::readNumber(*suspension, "damper_rebound", 0.0, 1.0e5, frontSpring.dampingRebound, context);
        content::readNumber(*suspension, "bump_stop_length", 0.0, 1.0, frontSpring.bumpStopLength, context);
        content::readNumber(*suspension, "bump_stop_stiffness", 0.0, 5.0e6, frontSpring.bumpStopStiffness, context);
        content::readNumber(*suspension, "bump_stop_damping", 0.0, 1.0e5, frontSpring.bumpStopDamping, context);
        content::readNumber(*suspension, "bump_stop_exponent", 0.1, 5.0, frontSpring.bumpStopExponent, context);
    }
    frontSpring.restLength = setup.restLength;
    frontSpring.hasBumpStop = frontSpring.bumpStopStiffness > 0.0;
    setup.frontSpring = frontSpring;
    setup.rearSpring = frontSpring; // the rear damper and bump stop mirror the front
    if (const toml::table* suspension = root["suspension"].as_table()) {
        content::readNumber(*suspension, "spring_stiffness_rear", 1.0e3, 5.0e5, setup.rearSpring.stiffness, context);
    }

    // --- Wheels and tyres --------------------------------------------------
    if (const toml::table* wheels = root["wheels"].as_table()) {
        content::readNumber(*wheels, "radius", 0.05, 1.0, setup.wheelRadius, context);
        content::readNumber(*wheels, "inertia", 0.01, 100.0, setup.wheelInertia, context);
    }
    if (const toml::table* tyres = root["tyres"].as_table()) {
        content::readNumber(*tyres, "peak_friction_longitudinal", 0.1, 3.0, setup.tyre.pDx1, context);
        content::readNumber(*tyres, "peak_friction_lateral", 0.1, 3.0, setup.tyre.pDy1, context);
        content::readNumber(*tyres, "nominal_load", 5.0e2, 2.0e4, setup.tyre.nominalLoad, context);
    }

    // --- Engine ------------------------------------------------------------
    if (const toml::table* engine = root["engine"].as_table()) {
        content::readNumber(*engine, "idle_speed", 10.0, 500.0, setup.engine.config.idleSpeed, context);
        content::readNumber(*engine, "rev_limit", 100.0, 3000.0, setup.engine.config.revLimit, context);
        content::readNumber(*engine, "friction_constant", 0.0, 1.0e3, setup.engine.friction.constant, context);
        content::readNumber(*engine, "friction_linear", 0.0, 10.0, setup.engine.friction.linear, context);

        const bool hasSpeed = engine->contains("torque_curve_speed");
        const bool hasTorque = engine->contains("torque_curve_torque");
        if (hasSpeed != hasTorque) {
            context.fail("'torque_curve_speed' and 'torque_curve_torque' must be given together");
        } else if (hasSpeed) {
            std::vector<double> speeds;
            std::vector<double> torques;
            content::readNumberArray(*engine, "torque_curve_speed", 1.0, 3.0e3, speeds, context);
            content::readNumberArray(*engine, "torque_curve_torque", 0.0, 5.0e3, torques, context);
            if (context.ok()) {
                if (speeds.size() != torques.size()) {
                    context.fail("'torque_curve_speed' and 'torque_curve_torque' must have the same length");
                } else if (speeds.size() < 2) {
                    context.fail("the torque curve needs at least two points");
                } else if (!content::strictlyIncreasing(speeds)) {
                    context.fail("'torque_curve_speed' must be strictly increasing");
                } else {
                    setup.engine.map.speed.assign(speeds.begin(), speeds.end());
                    setup.engine.map.throttle = {0.0, 1.0};
                    setup.engine.map.torque.resize(speeds.size() * 2);
                    for (std::size_t i = 0; i < speeds.size(); ++i) {
                        setup.engine.map.torque[i * 2 + 0] = 0.0;        // closed throttle
                        setup.engine.map.torque[i * 2 + 1] = torques[i]; // wide-open throttle
                    }
                }
            }
        }
    }
    if (!(setup.engine.config.revLimit > setup.engine.config.idleSpeed)) {
        context.fail("'rev_limit' must be greater than 'idle_speed'");
    }

    // --- Drivetrain --------------------------------------------------------
    if (const toml::table* drivetrain = root["drivetrain"].as_table()) {
        if (drivetrain->contains("gear_ratios")) {
            std::vector<double> ratios;
            if (content::readNumberArray(*drivetrain, "gear_ratios", 0.05, 20.0, ratios, context) && context.ok()) {
                if (!content::strictlyDecreasing(ratios)) {
                    context.fail("'gear_ratios' must be strictly decreasing from first to last gear");
                } else {
                    setup.gearbox.forwardRatios.assign(ratios.begin(), ratios.end());
                }
            }
        }
        content::readNumber(*drivetrain, "reverse_ratio", 0.1, 20.0, setup.gearbox.reverseRatio, context);
        content::readNumber(*drivetrain, "final_drive", 0.5, 20.0, setup.finalDrive, context);
        content::readNumber(*drivetrain, "efficiency", 0.1, 1.0, setup.gearbox.efficiency, context);
        content::readNumber(*drivetrain, "upshift_speed", 50.0, 3.0e3, setup.upshiftSpeed, context);
        content::readNumber(*drivetrain, "downshift_speed", 10.0, 3.0e3, setup.downshiftSpeed, context);
    }
    if (setup.gearbox.forwardRatios.empty()) {
        context.fail("'gear_ratios' must list at least one forward gear");
    }
    if (!(setup.upshiftSpeed > setup.downshiftSpeed)) {
        context.fail("'upshift_speed' must be greater than 'downshift_speed'");
    }

    // --- Brakes ------------------------------------------------------------
    if (const toml::table* brakes = root["brakes"].as_table()) {
        content::readNumber(*brakes, "max_brake_pressure", 1.0e5, 5.0e7, setup.maxBrakePressure, context);
        content::readNumber(*brakes, "brake_bias", 0.0, 1.0, setup.brakeBias.frontFraction, context);
        content::readNumber(*brakes, "handbrake_pressure", 0.0, 2.0e7, setup.handbrakePressure, context);
        content::readNumber(*brakes, "pad_friction", 0.05, 1.5, setup.brake.padFrictionCoefficient, context);
        content::readNumber(*brakes, "piston_area", 1.0e-4, 1.0e-1, setup.brake.pistonArea, context);
        content::readNumber(*brakes, "effective_radius", 0.01, 0.5, setup.brake.effectiveRadius, context);
    }

    // --- Steering ----------------------------------------------------------
    if (const toml::table* steering = root["steering"].as_table()) {
        content::readNumber(*steering, "max_steer_angle", 0.05, 1.5, setup.maxSteerAngle, context);
        content::readNumber(*steering, "min_steer_angle", 0.0, 1.5, setup.minSteerAngle, context);
        content::readNumber(*steering, "steer_speed_scale", 1.0, 100.0, setup.steerSpeedScale, context);
        content::readNumber(*steering, "steer_input_rate", 0.1, 20.0, setup.steerInputRate, context);
        content::readNumber(*steering, "steer_return_rate", 0.1, 20.0, setup.steerReturnRate, context);
    }
    if (!(setup.maxSteerAngle >= setup.minSteerAngle)) {
        context.fail("'max_steer_angle' must be at least 'min_steer_angle'");
    }

    // --- Aerodynamics ------------------------------------------------------
    if (const toml::table* aero = root["aero"].as_table()) {
        content::readNumber(*aero, "drag_coefficient", 0.0, 3.0, setup.dragCoefficient, context);
        content::readNumber(*aero, "lift_coefficient", -5.0, 5.0, setup.liftCoefficient, context);
        content::readNumber(*aero, "frontal_area", 0.1, 20.0, setup.frontalArea, context);
        content::readVec3(*aero, "centre_of_pressure", -5.0, 5.0, setup.centreOfPressure, context);
        content::readNumber(*aero, "air_density", 0.1, 5.0, setup.airDensity, context);
    }

    readColor(root, car.bodyColor, context);

    if (!context.ok()) {
        return std::nullopt;
    }

    car.setup = setup;
    deriveCarSpec(car);
    return car;
}

} // namespace

void deriveCarSpec(CarSpec& car) {
    const vehicle::VehicleSetup& setup = car.setup;

    car.massKg = setup.mass;
    car.gearCount = static_cast<int>(setup.gearbox.forwardRatios.size());
    car.wheelbaseM = setup.rearAxleZ - setup.frontAxleZ;
    car.trackWidthM = 2.0 * setup.halfTrack;
    car.wheelRadiusM = setup.wheelRadius;
    // The model is rear-wheel drive (VehicleModel sets driven = !front).
    car.drivetrain = "Rear-wheel drive";

    const vehicle::EngineTorqueMap& map = setup.engine.map;
    if (map.throttle.empty() || map.speed.empty()) {
        return;
    }
    const std::size_t columns = map.throttle.size();
    if (map.torque.size() < map.speed.size() * columns) {
        return;
    }
    double peakPowerW = 0.0;
    for (std::size_t i = 0; i < map.speed.size(); ++i) {
        // Wide-open-throttle column (throttle is increasing).
        const double torque = static_cast<double>(map.torque[i * columns + (columns - 1)]);
        car.peakTorqueNm = std::max(car.peakTorqueNm, torque);
        const double speed = static_cast<double>(map.speed[i]); // rad/s
        peakPowerW = std::max(peakPowerW, torque * speed);
    }
    car.peakPowerKw = peakPowerW / 1000.0;
}

std::optional<CarSpec> parseCarSpec(std::string_view text, std::string_view sourcePath, std::string& error) {
    toml::table root;
    try {
        root = toml::parse(text, sourcePath);
    } catch (const toml::parse_error& parseError) {
        error = std::string(sourcePath) + ": " + std::string(parseError.description());
        return std::nullopt;
    }

    ParseContext context{std::string(sourcePath), {}};
    std::optional<CarSpec> car = parseCarSpecImpl(root, context);
    if (!car) {
        error = context.joined();
    }
    return car;
}

CarLoadResult loadCars(const vfs::VirtualFileSystem& vfs, std::string_view carsDirectory) {
    CarLoadResult result;
    const std::string root(carsDirectory);

    std::vector<std::string> entries = vfs.list(root);
    std::sort(entries.begin(), entries.end());
    for (const std::string& entry : entries) {
        const std::string directory = root + "/" + entry;
        if (!vfs.isDirectory(directory)) {
            continue;
        }
        const std::string path = directory + "/car.toml";
        if (!vfs.exists(path)) {
            continue;
        }
        const auto text = vfs.readText(path);
        if (!text) {
            result.errors.push_back(path + ": cannot read the descriptor");
            continue;
        }
        std::string error;
        if (auto car = parseCarSpec(*text, path, error)) {
            result.cars.push_back(std::move(*car));
        } else {
            result.errors.push_back(std::move(error));
        }
    }

    std::sort(result.cars.begin(), result.cars.end(), [](const CarSpec& a, const CarSpec& b) { return a.id < b.id; });
    return result;
}

} // namespace sim::app
