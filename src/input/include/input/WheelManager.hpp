// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 The Car Simulator contributors
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "input/Backend.hpp"
#include "input/CompatibilityDatabase.hpp"
#include "input/ControlMapping.hpp"
#include "input/ForceFeedback.hpp"
#include "input/Profiles.hpp"

namespace sim::input {

// One connected device with everything the manager knows about it: what the
// backend reported, how the database identified it, its mapping and its last raw
// state (kept for the diagnostics panel).
struct DeviceEntry {
    DeviceDescriptor descriptor;
    Identification identification;
    DeviceProfile profile;
    RawDeviceState state;
};

// Resolved driving controls for the frame. Analog values are clean doubles;
// button-like roles are momentary.
struct Controls {
    double steer = 0.0;     // [-1, 1], + = left
    double throttle = 0.0;  // [0, 1]
    double brake = 0.0;     // [0, 1]
    double clutch = 0.0;    // [0, 1]
    double handbrake = 0.0; // [0, 1]
    bool shiftUp = false;
    bool shiftDown = false;
    bool gearNeutral = false;
    bool lookLeft = false;
    bool lookRight = false;
    bool hasDrivingDevice = false; // a wheel / gamepad / pedal set is connected
};

struct InputDiagnostics {
    int connectedDevices = 0;
    int removedDevices = 0; // since start-up, for the diagnostics panel
    int unknownDevices = 0; // not found in the compatibility database
    bool ffbActive = false;
    bool ffbAvailable = false;
};

// Ties the backend, the compatibility database and the profiles together and
// produces the frame's controls and force feedback command. Hot-plug, unknown
// devices, several devices at once and combined/split pedals are all handled
// here; the class itself never touches the operating system, so it is exercised
// by the tests through a mock backend.
class WheelManager {
public:
    WheelManager(IInputBackend& backend, CompatibilityDatabase& database, ProfileStore& profiles);
    ~WheelManager();

    WheelManager(const WheelManager&) = delete;
    WheelManager& operator=(const WheelManager&) = delete;

    // Polls the backend, reconciles the device list, resolves the controls and
    // updates force feedback. Call once per frame.
    void update(double dt);

    const std::vector<DeviceEntry>& devices() const { return devices_; }
    const Controls& controls() const { return controls_; }
    const InputDiagnostics& diagnostics() const { return diagnostics_; }

    ForceFeedbackController& ffb() { return ffb_; }
    const ForceFeedbackController& ffb() const { return ffb_; }

    // Physics-derived force feedback demand for this frame.
    void setFfbInput(const FfbInput& input) { ffbInput_ = input; }

    // Which device drives the car. -1 means "choose automatically" (the first
    // wheel-like device, else the first device).
    void selectDevice(int index) { selected_ = index; }
    int selectedDevice() const { return selected_; }
    int primaryDeviceIndex() const;

    // --- Remapping (applied live and persisted on the next storeProfiles) ---
    void bindAxis(std::size_t deviceIndex, ControlRole role, int axis, AxisHalf half);
    void bindButton(std::size_t deviceIndex, ControlRole role, int button);
    void bindHat(std::size_t deviceIndex, ControlRole role, int hat, int direction);
    void clearBinding(std::size_t deviceIndex, ControlRole role);
    void setCalibration(std::size_t deviceIndex, ControlRole role, const AxisCalibration& calibration);
    // Physical lock-to-lock rotation of the steering axis, in degrees.
    void setSteeringRange(std::size_t deviceIndex, double degrees);

    // Writes every device profile and the FFB settings into the store.
    void storeProfiles();

    // Stops every running effect; call on shutdown and on focus loss.
    void stopForceFeedback();

private:
    void reconcileDevices();
    void resolveControls();
    void updateFfb(double dt);

    // Evaluates one binding against a device's raw state. `bound` is false when
    // the binding is absent or points at a missing axis / button / hat.
    static double evaluate(const DeviceEntry& device, ControlRole role, const ControlBinding& binding, bool& bound);

    int rolePriority(std::size_t deviceIndex, ControlRole role) const;

    IInputBackend& backend_;
    CompatibilityDatabase& database_;
    ProfileStore& profiles_;
    std::vector<DeviceEntry> devices_;
    Controls controls_;
    ForceFeedbackController ffb_;
    FfbInput ffbInput_{};
    InputDiagnostics diagnostics_{};
    int selected_ = -1;
    int removedCount_ = 0;
};

} // namespace sim::input
