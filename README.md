# Car Simulator

Open-source, community-maintained driving simulator with its **own physics
engine** and its **own Vulkan renderer**. No third-party physics or rendering
engines are used (Jolt, Bullet, PhysX, bgfx, Diligent, Filament, ...): physics
and rendering are written from scratch.

> Current version: **0.1.0** (initial milestone). The goal of this milestone is
> a solid, well-documented foundation, not a finished game.

## Features of this foundation

- **Core**: fixed-step clock, job system, typed TOML configuration, spdlog
  logging, math utilities, assertions.
- **Platform**: SDL3 window, keyboard input, wheels/gamepads/joysticks with hot
  plugging, Vulkan surface creation (the RHI never sees SDL).
- **VFS**: virtual file system on top of PhysicsFS. All content is read through
  it; the official content and mods live in the same virtual tree.
- **Modding**: `mod.toml` manifests, dependency resolution with (semver)
  version ranges, deterministic topological load order, VFS mounting.
- **Scripting**: LuaJIT + sol2 sandbox with an instruction budget.
- **Native plugins**: plain C ABI, versioned, with struct evolution controlled
  by `struct_size`.
- **RHI**: thin layer over Vulkan (volk + VMA): instance/device, swapchain,
  buffers, images, pipelines, command lists, frames in flight.
- **Render**: perspective camera, flat scene, linear `RenderGraph`, Slang
  shaders compiled to SPIR-V at build time.
- **Physics**: rigid bodies, AABB broadphase, sphere/sphere, sphere/plane,
  box/plane and sphere/box narrowphase, sequential-impulse solver,
  semi-implicit Euler integration.
- **Vehicle**: deterministic kinematic bicycle model with wheels.
- **Telemetry**: history of frame/physics/render/UI times and CSV export.
- **Audio**: minimal wrapper over miniaudio.
- **Network**: skeleton of the future multiplayer layer (no sockets yet).

## Requirements

- **CMake** >= 3.25
- **C++20 compiler**:
  - **MSVC / Visual Studio 2022** with the **v143** toolset (stable) or
    later; recent VS builds have also been tested. Use the *x64 Native Tools
    Command Prompt* or run `vcvars64.bat` before configuring.
  - **GCC 13+** or **Clang 16+**.
- **vcpkg** (manifest mode)
- **Ninja** (recommended; the presets use it)
- **Slang** (`slangc`): downloaded automatically via `FetchContent` unless a
  local copy is given
- **Vulkan SDK**: **not** needed to build (only `vulkan-headers` and `volk` are
  used); a **Vulkan driver** is needed at run time

## Quick build (Windows / MSVC)

```powershell
# 1) Clone and bootstrap vcpkg once, then set VCPKG_ROOT
$env:VCPKG_ROOT = "C:\path\to\vcpkg"

# 2) Configure (the first run downloads and installs the dependencies)
cmake --preset windows-msvc-debug

# 3) Build
cmake --build --preset windows-msvc-debug

# 4) Run the tests
ctest --test-dir build/windows-msvc-debug --output-on-failure
```

Generated binaries:

```
build/windows-msvc-debug/bin/sim.exe
build/windows-msvc-debug/bin/sim_tests.exe
```

For Release:

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --test-dir build/windows-msvc-release --output-on-failure
```

To use a local copy of Slang instead of the downloaded one:

```powershell
cmake --preset windows-msvc-debug -DSIM_SLANGC=C:\path\to\slang\bin\slangc.exe
```

On Linux, the `linux-gcc-*` and `linux-clang-*` presets are provided.

Detailed instructions in [`docs/BUILDING.md`](docs/BUILDING.md).

## GPU tests without a GPU (headless)

The simulator can render **off screen**, without a window or swapchain, using a
software Vulkan driver. This makes it possible to run the tests labelled `gpu`
(golden image comparison, pixel checks, etc.) on a machine without a GPU and in
CI.

First download the software driver (lavapipe), **pinned and SHA-256
verified**:

```powershell
# Windows
powershell -ExecutionPolicy Bypass -File tools/setup_swiftshader.ps1
```

```bash
# Linux (uses the mesa-vulkan-drivers package, or installs it with apt)
./tools/setup_swiftshader.sh
```

The script prints the ICD path. Configure and run the tests:

```powershell
cmake --preset windows-msvc-release -DSIM_ENABLE_GPU_TESTS=ON `
      -DSIM_VULKAN_ICD="$PWD\tools\mesa\x64\lvp_icd.x86_64.json"
cmake --build --preset windows-msvc-release
ctest --test-dir build/windows-msvc-release -L gpu --output-on-failure
```

> On Windows, `VK_ICD_FILENAMES` **must** use backslashes; the loader rejects
> forward slashes. Also add the ICD directory to `PATH`.

Running the headless renderer manually:

```powershell
./build/windows-msvc-release/bin/sim.exe --headless --frames 60 `
    --scene default --screenshot capture.png
```

The headless runner also supports `--verify-golden`/`--update-golden` (with
`--golden-dir`), `--semantic`, `--contact-sheet` and `--debug-draw`. The `gpu`
tests include one reference per scene plus one for the *debug draw*
(`tests/golden/<driver>/grid_debug.png`); references are only enabled after
approval, and until then those checks are reported as skipped (see
`tests/golden/README.md`).

## Running

`sim.exe` looks for the project root (the directory that contains
`content/base`) starting from the working directory and from the executable's
directory. From there it mounts the content, reads the configuration and
starts.

```powershell
./build/windows-msvc-debug/bin/sim.exe
```

### Default controls

| Action | Key |
|---|---|
| Throttle | `W` / `↑` |
| Brake | `S` / `↓` |
| Steer | `A` / `D` / `←` / `→` |
| Handbrake | `Space` |
| Quit | `Esc` |

Camera and debugging: `C` cycles the **Chase / Orbit / Free** modes (drag with
the right mouse button to look, wheel to zoom in Orbit; `WASD`/`Q`/`E` to fly
in Free), and `G` toggles the instanced debug grid.

Gamepads, wheels and pedals detected by SDL3 also work.

## Repository layout

```
apps/sim/            Executable application (main loop, ImGui, interpolation)
cmake/               CMake modules (helpers, dependencies, Slang, ImGui, FSR)
content/base/        Official content (CC BY-SA 4.0)
mods/                Example mods
plugins/example_native/  Example native plugin
src/
  core/              Clock, jobs, configuration, logging, math
  platform/          SDL3: context, window, input
  vfs/               Virtual file system (PhysicsFS)
  modding/           Manifests, versions, dependencies, mod manager
  scripting/         Lua sandbox (LuaJIT + sol2) and script host
  plugin_api/        Native plugin ABI + loader
  rhi/               In-house Vulkan render layer
  render/            Scene, camera, meshes, shaders (Slang)
  physics/           Rigid bodies, collisions, solver
  vehicle/           Vehicle model
  telemetry/         Frame metrics + CSV export
  audio/             miniaudio
  net/               Network skeleton
tests/               doctest suite
docs/                Documentation
```

## Documentation

- [Roadmap](docs/ROADMAP.md)
- [Vehicle physics plan](docs/PHYSICS_PLAN.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Building](docs/BUILDING.md)
- [Physics](docs/FISICA.md)
- [Rendering](docs/RENDERIZADO.md)
- [Mod system](docs/MODS.md)
- [Scripting (Lua)](docs/SCRIPTS.md)
- [Native plugins](docs/PLUGINS.md)
- [Content and licensing](docs/CONTENT.md)
- [Design decisions and limitations](docs/DECISIONS.md)
- [Conventions](docs/CONVENTIONS.md)
- [Contributing](docs/CONTRIBUTING.md)

## License

- Engine code: **GNU GPL v3.0 or later**, with a **linking exception** (see
  [`LICENSE`](LICENSE)).
- Official content (`content/base`): **CC BY-SA 4.0**
  (see [`content/base/LICENSE.txt`](content/base/LICENSE.txt)).

Real brands, cars and tracks are deliberately not used.
