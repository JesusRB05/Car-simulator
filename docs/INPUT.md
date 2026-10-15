<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
<!-- Copyright (C) 2026 The Car Simulator contributors -->

# Wheel, pedal and gamepad input

The `input` module turns whatever USB HID device is plugged in - a steering
wheel base, a standalone pedal set, an H-pattern shifter, a handbrake or a game
pad - into the abstract driving controls the simulator consumes (steering,
throttle, brake, clutch, handbrake, shifters and look-around). Driving games
live or die by this layer, so it is deliberately its own module: it never
touches the operating system and is fully unit-tested.

## Design

```
  devices (SDL3)            sim::input (OS-free)                 app
  ---------------    ------------------------------------      --------
  IInputBackend  --> WheelManager  --> WheelManager::controls() --> VehicleInput
  (platform)         |  CompatibilityDatabase                    --> DriveKeys
                     |  ProfileStore (core::Config TOML)
                     +-> ForceFeedbackController --> IFfbOutput --> device
```

- `src/input/` (`namespace sim::input`, target `sim::input`) is **SDL-free**. The
  only OS work is behind `IInputBackend` and `IFfbOutput`; the tests provide
  mocks, so the whole manager runs without hardware.
- `src/platform/` implements `IInputBackend` on top of SDL3 (`platform::InputManager`)
  and exposes an `IFfbOutput` per device (`platform::SdlFfbOutput`). SDL3 is the
  single backend on Windows, Linux and macOS; it already sits on top of
  DirectInput / HIDAPI / evdev, so no redundant per-OS code is added.

## Files

| File | Responsibility |
| --- | --- |
| `input/Types.hpp` | `DeviceKind`, `DeviceBus`, `VerificationStatus`, `FfbCapabilities`, `DeviceCapabilities`, `DeviceDescriptor`, `deviceKey()` |
| `input/CompatibilityDatabase.{hpp,cpp}` | VID:PID table, vendor families and name heuristics; declares capabilities and a verification tier per model |
| `input/Calibration.{hpp,cpp}` | `AxisCalibration` and `toSigned()` / `toUnit()` / `shapeUnit()` |
| `input/ControlMapping.{hpp,cpp}` | `ControlRole`, bindings and `defaultProfile()` |
| `input/Backend.hpp` | `IInputBackend`, `IFfbOutput`, `RawDeviceState`, `FfbCondition` |
| `input/ForceFeedback.{hpp,cpp}` | `FfbSettings`, `FfbInput`, `FfbCommand`, `ForceFeedbackController` |
| `input/Profiles.{hpp,cpp}` | `ProfileStore`: device profiles and FFB settings persisted in the user's `config.toml` |
| `input/WheelManager.{hpp,cpp}` | Hot-plug reconciliation, multi-device merge, remapping, force feedback, diagnostics |

## Detection and identity

Every frame the manager polls the backend, which reports the connected devices
with their USB metadata (`vendorId`, `productId`, `productVersion`, serial,
path, bus and how many axes / buttons / hats they expose). Each device is
identified:

1. **Exact** match on `VID:PID` in the compatibility database,
2. a **vendor-family** match (for example every Fanatec base shares a range),
3. a **name heuristic** (the SDL name contains "wheel", "pedal", ...).

The result is a `DeviceKind` and a **verification tier**
(`Unknown`, `CommunityReported`, `ManufacturerDeclared`, `VerifiedDocumentation`,
`VerifiedHardware`). The tier is never promoted without evidence: a model only
reaches `VerifiedHardware` after a maintainer exercises it on real hardware.

Profiles are keyed by `deviceKey()` - a stable `"vid:pid"` string, or a
sanitised product name for virtual boards that report no USB ids. The key
survives a reconnect, so a device keeps its calibration and mapping across
plug/unplug cycles even though SDL reuses instance ids.

## Calibration

Raw readings arrive normalised to `[-1, 1]`, but real hardware drifts. The
calibration records the measured extremes and turns any reading into a clean
value:

- `toSigned()` for steering (0 at the drifted centre, deadzone and saturation),
- `toUnit()` for pedals (rest at `rawMin`, mapped to `0..1`),
- `shapeUnit()` for the halves of a centred "combined pedals" axis.

Deadzone, saturation, inversion and an `active` switch are all per axis.

## Mapping

`ControlRole` is the abstract control (`Steer`, `Throttle`, `Brake`, `Clutch`,
`Handbrake`, `ShiftUp`, `ShiftDown`, `GearNeutral`, `LookLeft`, `LookRight`).
Each device carries a `DeviceProfile` mapping roles to an axis (optionally one
half of a centred axis), a button or a hat direction. `defaultProfile()` builds
a sensible starting layout from the device kind and axis/button counts; the user
can then remap everything live from the **Input** panel.

When several devices are connected (a wheel plus a separate pedal set plus a
shifter), controls are merged: each role is resolved from the highest-priority
device that binds it - the selected primary wheel first, then a device whose
kind matches the role, then the rest.

## Force feedback

`ForceFeedbackController` turns a physics-derived demand (`steeringTorque`,
`understeer`, `oversteer`, `kerbRumble`, `impact`, `speed`) into concrete effect
values, scaled by the user gains and gated by the device's live
`FfbCapabilities`. It enforces the safety rules that force everything to stop:
force feedback disabled, no device, window not focused, or an explicit stop
request.

`SdlFfbOutput` implements what SDL3 exposes portably: whole-body and trigger
**rumble** and the RGB/mono **LED**. Directional constant force and the
spring / damper / friction conditions require the Windows-only DirectInput
effect API (`SDL_SendJoystickEffect` with HID descriptors), so they are
currently **no-ops in the backend** - the database still declares them, but the
manager never sends an effect the live backend cannot honour. Wiring the full
DirectInput effects is tracked as a follow-up.

## Configuration

Profiles and force feedback live in the user's `config.toml` under `input.*`:

```toml
[input.ffb]
enabled = true
master_gain = 0.7
invert_direction = false
# ...

[input.profiles."046d:c29b"]
version = 1
model = "Logitech G29"
# roles.<n>.<field> holds the binding and its calibration
```

The store operates in memory and is written once on shutdown (`config_->save()`).

## Using it

The Input panel (**View > Input**) lists the connected devices, lets you pick
the primary one (or leave it on *Auto-select*), remap every role, edit each
axis' calibration, tune force feedback and press **Test** for a bounded,
gentle self-test. **Save profiles** persists immediately.

## Adding a device to the database

Add a `DeviceModel` row to the table in `CompatibilityDatabase.cpp` with the
exact `VID:PID`, a human name, the `DeviceKind`, the declared `FfbCapabilities`
and - honestly - the `VerificationStatus` you can justify. Use a vendor family
entry only when a whole range shares the same shape. Never promote a tier
without evidence.

## Testing

`tests/test_input.cpp` drives `WheelManager` through `MockInputBackend` and
`MockFfbOutput`: identification (exact family and heuristic), default profiles,
calibration (including centre drift and combined pedals), multi-device merge,
unknown devices, hot-plug, live remapping, profile persistence across a
reconnect, and the force-feedback safety and scaling.

```bat
build\windows-msvc-release\bin\sim_tests.exe -tc="*WheelManager*,*Calibration*,*Compatibility*,*ProfileStore*,*Force feedback*"
```

## Open items

- Full DirectInput constant-force / condition effects (Windows only).
- Per-model default profiles for the most common wheels, verified on hardware.
- Kernel-level features that SDL3 does not expose (high-resolution
  torque, telemetry from the wheel base) need vendor SDKs and are out of scope
  for now.
