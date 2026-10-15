// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
//
// The input module is deliberately free of any operating-system code: the
// backend is an interface, so the whole wheel stack (hot-plug, identification,
// mapping, calibration, force feedback safety) is exercised here through a mock
// backend and a mock force feedback sink, with no hardware attached.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "core/Config.hpp"
#include "input/Calibration.hpp"
#include "input/CompatibilityDatabase.hpp"
#include "input/ControlMapping.hpp"
#include "input/ForceFeedback.hpp"
#include "input/Profiles.hpp"
#include "input/Types.hpp"
#include "input/WheelManager.hpp"

using namespace sim::input;

namespace {

// Records every call so the tests can assert what was sent to the device.
struct MockFfbOutput : IFfbOutput {
    FfbCapabilities caps;
    int stopCalls = 0;
    int constantCalls = 0;
    int rumbleCalls = 0;
    double lastConstant = 0.0;
    double lastRumble = 0.0;
    std::map<FfbCondition, double> conditions;

    FfbCapabilities capabilities() const override { return caps; }
    void setRumble(double low, double, int) override {
        lastRumble = low;
        ++rumbleCalls;
    }
    void setConstantForce(double magnitude) override {
        lastConstant = magnitude;
        ++constantCalls;
    }
    void setCondition(FfbCondition condition, double coefficient) override { conditions[condition] = coefficient; }
    void setLed(double, double, double) override {}
    void stopAll() override { ++stopCalls; }
};

struct MockInputBackend : IInputBackend {
    std::map<std::uint32_t, DeviceDescriptor> descriptors;
    std::map<std::uint32_t, RawDeviceState> states;
    std::map<std::uint32_t, std::shared_ptr<MockFfbOutput>> outputs;
    bool focused = true;

    void poll() override {}

    std::vector<DeviceDescriptor> deviceDescriptors() const override {
        std::vector<DeviceDescriptor> list;
        for (const auto& [id, descriptor] : descriptors) {
            (void)id;
            list.push_back(descriptor);
        }
        return list;
    }

    bool readState(std::uint32_t instanceId, RawDeviceState& out) const override {
        const auto it = states.find(instanceId);
        if (it == states.end()) {
            return false;
        }
        out = it->second;
        return true;
    }

    bool windowFocused() const override { return focused; }

    IFfbOutput* ffbOutput(std::uint32_t instanceId) override {
        const auto it = outputs.find(instanceId);
        return it == outputs.end() ? nullptr : it->second.get();
    }

    void add(std::uint32_t id, DeviceDescriptor descriptor, int axes, int buttons, int hats,
             FfbCapabilities caps = {}) {
        descriptor.instanceId = id;
        descriptor.backendId = std::to_string(id);
        descriptor.caps.axes = axes;
        descriptor.caps.buttons = buttons;
        descriptor.caps.hats = hats;
        descriptors[id] = std::move(descriptor);
        states[id].axes.assign(static_cast<std::size_t>(axes), 0.0);
        states[id].buttons.assign(static_cast<std::size_t>(buttons), false);
        states[id].hats.assign(static_cast<std::size_t>(hats), 0);
        if (caps.any()) {
            auto output = std::make_shared<MockFfbOutput>();
            output->caps = caps;
            outputs[id] = std::move(output);
        }
    }

    void remove(std::uint32_t id) { descriptors.erase(id); }
};

DeviceDescriptor makeDescriptor(const std::string& name, std::uint16_t vid, std::uint16_t pid, bool gamepad = false) {
    DeviceDescriptor descriptor;
    descriptor.name = name;
    descriptor.vendorId = vid;
    descriptor.productId = pid;
    descriptor.isGamepad = gamepad;
    descriptor.bus = DeviceBus::Usb;
    return descriptor;
}

FfbCapabilities fullFfb() {
    FfbCapabilities caps;
    caps.constantForce = true;
    caps.spring = true;
    caps.damper = true;
    caps.friction = true;
    caps.rumble = true;
    return caps;
}

} // namespace

TEST_CASE("Calibration maps raw readings to signed and unit controls") {
    AxisCalibration steer;
    steer.rawMin = -1.0;
    steer.rawMax = 1.0;
    steer.center = 0.0;
    CHECK(toSigned(0.0, steer) == doctest::Approx(0.0));
    CHECK(toSigned(1.0, steer) == doctest::Approx(1.0));
    CHECK(toSigned(-1.0, steer) == doctest::Approx(-1.0));

    AxisCalibration pedal;
    pedal.rawMin = -1.0;
    pedal.rawMax = 1.0;
    pedal.useCenter = false;
    CHECK(toUnit(-1.0, pedal) == doctest::Approx(0.0));
    CHECK(toUnit(1.0, pedal) == doctest::Approx(1.0));
    CHECK(toUnit(0.0, pedal) == doctest::Approx(0.5));

    AxisCalibration inverted = pedal;
    inverted.invert = true;
    CHECK(toUnit(-1.0, inverted) == doctest::Approx(1.0));
    CHECK(toUnit(1.0, inverted) == doctest::Approx(0.0));
}

TEST_CASE("Calibration respects the deadzone and a drifted centre") {
    AxisCalibration steer;
    steer.deadzone = 0.1;
    CHECK(toSigned(0.05, steer) == doctest::Approx(0.0));
    CHECK(toSigned(-0.05, steer) == doctest::Approx(0.0));
    CHECK(toSigned(1.0, steer) == doctest::Approx(1.0));

    // A wheel that rests slightly off-centre still reports zero at rest.
    AxisCalibration drifted;
    drifted.center = 0.08;
    CHECK(toSigned(0.08, drifted) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(toSigned(-1.0, drifted) < 0.0);

    AxisCalibration disabled;
    disabled.active = false;
    CHECK(toSigned(1.0, disabled) == doctest::Approx(0.0));
}

TEST_CASE("shapeUnit removes the deadzone and trims saturation") {
    CHECK(shapeUnit(0.0, 0.2, 0.0) == doctest::Approx(0.0));
    CHECK(shapeUnit(0.1, 0.2, 0.0) == doctest::Approx(0.0));
    CHECK(shapeUnit(1.0, 0.2, 0.0) == doctest::Approx(1.0));
    CHECK(shapeUnit(0.8, 0.0, 0.2) == doctest::Approx(1.0)); // trimmed -> full
}

TEST_CASE("Device key is stable and TOML-safe") {
    const DeviceDescriptor wheel = makeDescriptor("G29", 0x046D, 0xC24F);
    CHECK(deviceKey(wheel) == "046d:c24f");
    CHECK(ProfileStore::sanitizeKey(deviceKey(wheel)) == "046d_c24f");

    DeviceDescriptor diy;
    diy.name = "My DIY Wheel!";
    CHECK(deviceKey(diy) == "My_DIY_Wheel_");
}

TEST_CASE("Compatibility database identifies known wheels exactly") {
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    const DeviceDescriptor g29 = makeDescriptor("G29 Driving Force", 0x046D, 0xC24F);
    const Identification id = database.identify(g29);
    CHECK(id.known());
    CHECK(static_cast<int>(id.kind) == static_cast<int>(DeviceKind::WheelBase));
    CHECK(id.manufacturer == "Logitech");
    CHECK(id.modelName == "G29 Driving Force");
    CHECK(id.declaredFfb.constantForce);
    CHECK(static_cast<int>(id.status) == static_cast<int>(VerificationStatus::CommunityReported));
}

TEST_CASE("Compatibility database matches a vendor family without pinning a PID") {
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    const DeviceDescriptor fanatec = makeDescriptor("ClubSport DD+", 0x0EB7, 0x1234);
    const Identification id = database.identify(fanatec);
    CHECK(id.known());
    CHECK(static_cast<int>(id.kind) == static_cast<int>(DeviceKind::WheelBase));
    CHECK(id.manufacturer == "Fanatec");
}

TEST_CASE("Compatibility database falls back to name heuristics for unknown devices") {
    CompatibilityDatabase& database = CompatibilityDatabase::instance();

    DeviceDescriptor custom = makeDescriptor("Acme Handbrake", 0x1234, 0x0001);
    custom.manufacturer = "Acme";
    Identification id = database.identify(custom);
    CHECK_FALSE(id.known());
    CHECK(static_cast<int>(id.kind) == static_cast<int>(DeviceKind::Handbrake));
    CHECK(id.manufacturer == "Acme");
    CHECK(static_cast<int>(id.status) == static_cast<int>(VerificationStatus::Unknown));

    const DeviceDescriptor pedals = makeDescriptor("USB Pedals", 0x1234, 0x0002);
    CHECK(static_cast<int>(database.identify(pedals).kind) == static_cast<int>(DeviceKind::Pedals));

    const DeviceDescriptor pad = makeDescriptor("Wireless Controller", 0x045E, 0x028E, true);
    CHECK(static_cast<int>(database.identify(pad).kind) == static_cast<int>(DeviceKind::Gamepad));
}

TEST_CASE("Default profiles follow the device kind") {
    DeviceDescriptor wheel = makeDescriptor("Wheel", 0x046D, 0xC24F);
    wheel.caps.axes = 3;
    wheel.caps.buttons = 10;
    const DeviceProfile wheelProfile = defaultProfile(wheel, DeviceKind::WheelBase);
    REQUIRE(wheelProfile.find(ControlRole::Steer) != nullptr);
    CHECK(wheelProfile.find(ControlRole::Steer)->axis.axis == 0);
    CHECK(wheelProfile.find(ControlRole::Throttle)->axis.axis == 1);
    CHECK(wheelProfile.find(ControlRole::Brake)->axis.axis == 2);

    DeviceDescriptor pedals = makeDescriptor("Pedals", 0x1234, 0x0002);
    pedals.caps.axes = 2;
    const DeviceProfile pedalProfile = defaultProfile(pedals, DeviceKind::Pedals);
    CHECK(pedalProfile.find(ControlRole::Throttle)->axis.axis == 0);
    CHECK(pedalProfile.find(ControlRole::Brake)->axis.axis == 1);
    CHECK(pedalProfile.find(ControlRole::Throttle)->axis.calibration.useCenter == false);
}

TEST_CASE("WheelManager connects a wheel and resolves its axes") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(1, makeDescriptor("G29", 0x046D, 0xC24F), 3, 10, 0, fullFfb());
    backend.states[1].axes[0] = 0.5;  // steering
    backend.states[1].axes[1] = -1.0; // throttle fully up (rest)
    backend.states[1].axes[2] = -1.0; // brake fully up (rest)

    WheelManager manager(backend, database, profiles);
    manager.update(0.016);

    REQUIRE(manager.devices().size() == 1);
    CHECK(manager.controls().hasDrivingDevice);
    CHECK(manager.controls().steer == doctest::Approx(0.5));
    // Pedals rest at their minimum raw value, so a pedal fully up reads zero.
    CHECK(manager.controls().throttle == doctest::Approx(0.0));
    CHECK(manager.controls().brake == doctest::Approx(0.0));

    backend.states[1].axes[0] = -1.0;
    backend.states[1].axes[1] = 1.0;
    manager.update(0.016);
    CHECK(manager.controls().steer == doctest::Approx(-1.0));
    CHECK(manager.controls().throttle == doctest::Approx(1.0));
}

TEST_CASE("WheelManager applies per-axis calibration and inversion") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(1, makeDescriptor("G29", 0x046D, 0xC24F), 3, 10, 0);
    backend.states[1].axes[0] = 1.0;

    WheelManager manager(backend, database, profiles);
    manager.update(0.016);

    AxisCalibration inverted;
    inverted.rawMin = -1.0;
    inverted.rawMax = 1.0;
    inverted.center = 0.0;
    inverted.invert = true;
    manager.setCalibration(0, ControlRole::Steer, inverted);
    manager.update(0.016);
    CHECK(manager.controls().steer == doctest::Approx(-1.0));
}

TEST_CASE("WheelManager merges a wheel and a separate pedal set") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    // The wheel only steers; the pedals are their own USB device.
    backend.add(1, makeDescriptor("G29", 0x046D, 0xC24F), 1, 10, 0);
    DeviceDescriptor pedals = makeDescriptor("Load Cell Pedals", 0x1234, 0x0002);
    backend.add(2, pedals, 2, 0, 0);
    backend.states[1].axes[0] = -0.25;
    backend.states[2].axes[0] = 0.0; // throttle at 50%
    backend.states[2].axes[1] = 1.0; // brake at full
    backend.states[2].axes[0] = 0.0;

    WheelManager manager(backend, database, profiles);
    manager.update(0.016);

    CHECK(manager.controls().steer == doctest::Approx(-0.25));
    CHECK(manager.controls().brake == doctest::Approx(1.0));
    // Throttle is the first axis of a dedicated pedal set: -1 rest -> 0.5 here.
    CHECK(manager.controls().throttle == doctest::Approx(0.5));
}

TEST_CASE("WheelManager supports combined pedals on one centred axis") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(1, makeDescriptor("Wireless Controller", 0x045E, 0x028E, true), 2, 16, 0);
    backend.states[1].axes[0] = 0.3;  // steering
    backend.states[1].axes[1] = -0.8; // stick down = throttle
    WheelManager manager(backend, database, profiles);
    manager.update(0.016);
    CHECK(manager.controls().steer == doctest::Approx(0.3));
    CHECK(manager.controls().throttle == doctest::Approx(0.8));
    CHECK(manager.controls().brake == doctest::Approx(0.0));

    backend.states[1].axes[1] = 0.6; // stick up = brake
    manager.update(0.016);
    CHECK(manager.controls().throttle == doctest::Approx(0.0));
    CHECK(manager.controls().brake == doctest::Approx(0.6));
}

TEST_CASE("WheelManager reports unknown devices but still reads them") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(7, makeDescriptor("Mystery Wheel", 0x9999, 0x0001), 1, 4, 0);
    backend.states[7].axes[0] = 0.5;
    WheelManager manager(backend, database, profiles);
    manager.update(0.016);

    CHECK(manager.diagnostics().unknownDevices == 1);
    CHECK(manager.controls().steer == doctest::Approx(0.5));
}

TEST_CASE("WheelManager handles hot-plug during a session") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(1, makeDescriptor("G29", 0x046D, 0xC24F), 3, 10, 0);
    WheelManager manager(backend, database, profiles);
    manager.update(0.016);
    REQUIRE(manager.devices().size() == 1);

    backend.add(2, makeDescriptor("T300", 0x044F, 0xB66D), 3, 12, 0);
    manager.update(0.016);
    CHECK(manager.devices().size() == 2);

    backend.states[2].axes[0] = 0.75;
    manager.update(0.016);
    // The first wheel-like device stays the primary, so device 1 still steers.
    CHECK(manager.controls().steer == doctest::Approx(0.0));

    backend.remove(1);
    manager.update(0.016);
    CHECK(manager.devices().size() == 1);
    CHECK(manager.diagnostics().removedDevices == 1);
    CHECK(manager.controls().steer == doctest::Approx(0.75));
}

TEST_CASE("WheelManager remaps a control live") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(1, makeDescriptor("G29", 0x046D, 0xC24F), 4, 10, 0);
    backend.states[1].axes[3] = 0.8; // analog pedal: -1 rest, +1 fully applied
    WheelManager manager(backend, database, profiles);
    manager.update(0.016);
    CHECK(manager.controls().handbrake == doctest::Approx(0.0));

    manager.bindAxis(0, ControlRole::Handbrake, 3, AxisHalf::Full);
    manager.update(0.016);
    CHECK(manager.controls().handbrake == doctest::Approx(0.9));

    manager.clearBinding(0, ControlRole::Handbrake);
    manager.update(0.016);
    CHECK(manager.controls().handbrake == doctest::Approx(0.0));
}

TEST_CASE("WheelManager reads buttons and hats") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(1, makeDescriptor("G29", 0x046D, 0xC24F), 1, 4, 1);
    WheelManager manager(backend, database, profiles);
    manager.update(0.016); // connect first, so the device exists to be bound
    manager.bindButton(0, ControlRole::ShiftUp, 0);
    manager.bindButton(0, ControlRole::ShiftDown, 1);
    manager.bindHat(0, ControlRole::LookLeft, 0, 8);  // SDL_HAT_LEFT
    manager.bindHat(0, ControlRole::LookRight, 0, 2); // SDL_HAT_RIGHT

    backend.states[1].buttons[0] = true;
    backend.states[1].hats[0] = 8;
    manager.update(0.016);
    CHECK(manager.controls().shiftUp);
    CHECK_FALSE(manager.controls().shiftDown);
    CHECK(manager.controls().lookLeft);
    CHECK_FALSE(manager.controls().lookRight);
}

TEST_CASE("WheelManager keeps a profile across a reconnect") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(1, makeDescriptor("G29", 0x046D, 0xC24F), 4, 10, 0);
    {
        WheelManager manager(backend, database, profiles);
        manager.update(0.016);
        manager.bindAxis(0, ControlRole::Handbrake, 3, AxisHalf::Full);
        manager.storeProfiles();
    }

    backend.remove(1);
    backend.add(9, makeDescriptor("G29", 0x046D, 0xC24F), 4, 10, 0); // new instance id
    backend.states[9].axes[3] = -0.2;                                // analog pedal -> 0.4 once mapped
    WheelManager manager(backend, database, profiles);
    manager.update(0.016);
    CHECK(manager.controls().handbrake == doctest::Approx(0.4));
}

TEST_CASE("ProfileStore round-trips a profile and the FFB settings") {
    sim::core::Config config("does-not-exist.toml");
    ProfileStore store(config);

    DeviceProfile profile;
    profile.key = "046d:c24f";
    profile.modelName = "G29 Driving Force";
    profile.steeringRangeDegrees = 1080.0;
    ControlBinding steer;
    steer.kind = BindingKind::Axis;
    steer.axis.axis = 0;
    steer.axis.half = AxisHalf::Full;
    steer.axis.calibration.deadzone = 0.05;
    steer.axis.calibration.invert = true;
    profile.bindings[ControlRole::Steer] = steer;
    ControlBinding shift;
    shift.kind = BindingKind::Button;
    shift.button = 3;
    profile.bindings[ControlRole::ShiftUp] = shift;

    store.storeProfile(profile);

    const DeviceProfile loaded = store.loadProfile("046d:c24f", DeviceProfile{});
    REQUIRE(loaded.bindings.count(ControlRole::Steer) == 1);
    CHECK(loaded.bindings.at(ControlRole::Steer).axis.axis == 0);
    CHECK(loaded.bindings.at(ControlRole::Steer).axis.calibration.deadzone == doctest::Approx(0.05));
    CHECK(loaded.bindings.at(ControlRole::Steer).axis.calibration.invert);
    CHECK(static_cast<int>(loaded.bindings.at(ControlRole::ShiftUp).kind) == static_cast<int>(BindingKind::Button));
    CHECK(loaded.bindings.at(ControlRole::ShiftUp).button == 3);
    CHECK(loaded.steeringRangeDegrees == doctest::Approx(1080.0));

    FfbSettings settings;
    settings.enabled = false;
    settings.masterGain = 0.42;
    settings.invertDirection = true;
    store.storeFfbSettings(settings);
    const FfbSettings loadedFfb = store.loadFfbSettings();
    CHECK_FALSE(loadedFfb.enabled);
    CHECK(loadedFfb.masterGain == doctest::Approx(0.42));
    CHECK(loadedFfb.invertDirection);
}

TEST_CASE("ProfileStore returns the fallback for an unknown device") {
    sim::core::Config config("does-not-exist.toml");
    ProfileStore store(config);
    DeviceProfile fallback;
    fallback.key = "abcd:0001";
    fallback.bindings[ControlRole::Steer] = ControlBinding{};
    const DeviceProfile loaded = store.loadProfile("abcd:0001", fallback);
    CHECK(loaded.key == "abcd:0001");
    CHECK(loaded.bindings.count(ControlRole::Steer) == 1);
}

TEST_CASE("Force feedback is stopped when it must not run") {
    ForceFeedbackController controller;
    controller.settings().caps = fullFfb();
    FfbInput input;
    input.steeringTorque = 0.5;

    CHECK_FALSE(controller.update(input, true, false, 0.016).active); // focus lost
    CHECK_FALSE(controller.update(input, false, true, 0.016).active); // no device

    controller.settings().enabled = false;
    CHECK_FALSE(controller.update(input, true, true, 0.016).active);

    controller.settings().enabled = true;
    controller.requestStop();
    CHECK_FALSE(controller.update(input, true, true, 0.016).active);
    controller.clearStop();
    CHECK(controller.update(input, true, true, 0.016).active);
}

TEST_CASE("Force feedback scales with the settings and clamps the torque") {
    ForceFeedbackController controller;
    FfbSettings settings;
    settings.caps = fullFfb();
    settings.masterGain = 1.0;
    settings.maxForce = 1.0;
    controller.settings() = settings;

    FfbInput input;
    input.steeringTorque = 0.8;
    input.speed = 1.0;
    const FfbCommand command = controller.update(input, true, true, 0.016);
    CHECK(command.active);
    CHECK(command.constantForce == doctest::Approx(0.8));
    CHECK(command.spring > 0.0);
    CHECK(command.damper > 0.0);

    input.steeringTorque = 2.0; // out of range input is clamped
    CHECK(controller.update(input, true, true, 0.016).constantForce == doctest::Approx(1.0));
}

TEST_CASE("Force feedback self-test is bounded and then stops") {
    ForceFeedbackController controller;
    controller.settings().caps = fullFfb();
    controller.startTest(0.2);
    CHECK(controller.testing());

    FfbInput input;
    double elapsed = 0.0;
    while (controller.testing() && elapsed < 1.0) {
        const FfbCommand command = controller.update(input, true, true, 0.05);
        CHECK(command.active);
        CHECK(std::abs(command.constantForce) <= 0.3);
        elapsed += 0.05;
    }
    CHECK(elapsed <= 0.25);
    CHECK_FALSE(controller.testing());
}

TEST_CASE("WheelManager sends force feedback and stops it on disconnect") {
    MockInputBackend backend;
    CompatibilityDatabase& database = CompatibilityDatabase::instance();
    sim::core::Config config("does-not-exist.toml");
    ProfileStore profiles(config);

    backend.add(1, makeDescriptor("G29", 0x046D, 0xC24F), 3, 10, 0, fullFfb());
    WheelManager manager(backend, database, profiles);
    manager.update(0.016);
    CHECK(manager.diagnostics().ffbAvailable);

    FfbInput input;
    input.steeringTorque = 0.5;
    input.speed = 1.0;
    manager.setFfbInput(input);
    manager.update(0.016);

    MockFfbOutput& output = *backend.outputs.at(1);
    CHECK(output.constantCalls > 0);
    CHECK(output.lastConstant == doctest::Approx(0.5 * manager.ffb().settings().masterGain));

    backend.remove(1);
    manager.update(0.016);
    CHECK(output.stopCalls > 0);
    CHECK_FALSE(manager.controls().hasDrivingDevice);
}
