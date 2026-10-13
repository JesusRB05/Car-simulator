# Architecture

This document describes how the simulator is organised, the conventions all
the code follows and how the different layers communicate.

## Principles

1. **No third-party engines** for physics or rendering. Utility libraries are
   allowed (Vulkan headers, VMA, volk, glm, SDL3, ...), but the physics engine
   and the renderer are our own.
2. **SI units** throughout the engine (metres, kilograms, seconds, radians).
   There are no hidden "game" units.
3. **Controlled precision**: world positions are stored as `double`
   (`glm::dvec3`); local data and whatever goes to the GPU use `float`.
4. **Determinism**: the same set of inputs ⇒ the same sequence of states,
   regardless of how fast the machine is.
5. **Coordinate system**: right-handed, **+Y up**, **-Z forward**.
6. **Strict layering**: each module depends only on the ones below it. Physics
   knows nothing about rendering; the RHI knows nothing about SDL.

## Layers and modules

From bottom to top:

| Module | Responsibility | Depends on |
|---|---|---|
| `core` | Fixed-step clock, jobs, TOML config, logging, math, assertions | — |
| `platform` | SDL3: context, window, input, Vulkan surface | `core` |
| `vfs` | Virtual file system (PhysicsFS) | `core` |
| `modding` | Manifests, versions, dependencies, mod manager | `core`, `vfs` |
| `scripting` | Lua sandbox and script host | `core`, `vfs` |
| `plugin_api` | Native plugin ABI + loader | `core`, `vfs` |
| `rhi` | Instance/device, swapchain, resources, pipelines | `core` |
| `render` | Scene, camera, meshes, shaders, render graph | `core`, `rhi`, `vfs` |
| `physics` | Rigid bodies, force elements and joints (PGS solver) | `core` |
| `vehicle` | Vehicle model | `core`, `physics` |
| `telemetry` | Frame metrics and CSV export | `core`, `vfs` |
| `audio` | miniaudio | `core`, `vfs` |
| `net` | Network skeleton | `core` |
| `apps/sim` | Application: main loop, ImGui, physics on its own thread | all |

Each module is a static library `sim::<name>` with its public headers in
`include/<name>/` and its sources in `src/`.

## Main loop

The program (`apps/sim`) follows this order:

```
main()
  └─ Application::run()
       ├─ initialise()
       │    ├─ SdlContext, Window, InputManager
       │    ├─ Config (user config.toml)
       │    ├─ VirtualFileSystem
       │    ├─ ModManager::reload()      (mounts content and mods)
       │    ├─ mount the executable directory (shaders) underneath
       │    ├─ LuaSandbox + ScriptHost, PluginLoader
       │    ├─ Vulkan surface + Device + Swapchain + Frame
       │    ├─ Renderer, ImGuiLayer, Telemetry, AudioEngine, NetworkSystem
       │    ├─ initial scene (ground + cube) and vehicle
       │    └─ startPhysicsThread()
       └─ loop:
            ├─ pumpInput()              (SDL → InputManager/ImGui, keys → atomics)
            ├─ gatherSnapshots()        (drains the ring, interpolates)
            ├─ frame_->begin()          (waits for a slot, acquires an image)
            ├─ imgui_->beginFrame()
            ├─ updateScene()            (fills the Scene from the snapshot)
            ├─ renderer_->renderScene()
            ├─ drawUi()                 (telemetry/input panels)
            ├─ imgui_->endFrame()
            ├─ frame_->submitAndPresent()
            └─ resize/close handling
```

## Physics thread and snapshots

The simulation runs on a **dedicated thread** with a fixed step (`1/1000 s` by
default) so that the result does not depend on the frame rate. The render
thread interpolates between the last two states so that motion looks smooth.

```
Physics thread                         Render thread
─────────────────                      ─────────────────
reads input atomics                    writes input atomics
fixed-step loop                        every frame:
  ├─ FixedStepClock.advance(dt)          ├─ pop() all new snapshots
  ├─ vehicle.step(dt)                    ├─ previous_ = second to last
  ├─ world.step(dt)                      ├─ current_  = last
  └─ push(SimSnapshot)  ──── ring ────►  └─ alpha = clamp((now - current.wallTime)/dt)
                                              interpolate position/heading/wheels
```

- The input channel is a set of `std::atomic` (`throttle`, `brake`, `steer`,
  `handbrake`).
- The output channel is `SnapshotRing`: a **lock-free SPSC** ring of 128
  `SimSnapshot`, with atomic write/read indices.
- `SimSnapshot` carries: simulation time, wall-clock time, step count, vehicle
  position (`dvec3`), heading, speed, rpm, steering angle and the spin of the 4
  wheels.
- The render clock uses `FrameClock` (based on `std::chrono::steady_clock`).

## Determinism

- Fixed step in physics; no variable `dt` inside the simulation.
- MSVC: `/fp:precise`. GCC/Clang: `-ffp-contract=off`. Never `-ffast-math`.
- Physics is always evaluated in the same order (no parallelism inside a
  step). The `JobSystem` is reserved for work that does **not** affect the
  simulation (asset loading, mod scanning).

## Configuration and VFS

- The **user configuration** is a TOML file read/written by `core::Config`
  (dotted keys, e.g. `mods.active`). This file is **not** part of the VFS.
- All **game content** (textures, meshes, scripts, sounds) is read through
  `vfs::VirtualFileSystem`. Official content and mods are mounted in the same
  tree; later mounts take precedence.
- The **log** and the configuration file are the only two files touched
  outside the VFS, and they are managed by the application layer.

## Errors

- Programming errors use `SIM_ASSERT` (`core/Assert.hpp`).
- Recoverable errors are returned as `bool`, `std::optional` or result
  structures (for example `ModReport`). An invalid mod **never** aborts the
  process: it is disabled with a clear message.
- Exceptions (`std::runtime_error`) are reserved for unrecoverable start-up
  failures (the Vulkan device cannot be created, ...).
