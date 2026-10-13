# Design decisions

Record of the most important technical decisions and why they were made.

## Own engine: no third-party physics or rendering engines

The project implements its own physics engine and its own Vulkan renderer.
Using engines such as Jolt, Bullet or PhysX for physics, or bgfx, Diligent or
Filament for rendering, is **forbidden**. The goal is for the project to be
fully understandable and modifiable by the community. Utility libraries are
used (volk, VMA, glm, ImGui, ...).

## Units and conventions

- All internal computation uses **SI units** (metres, kilograms, seconds,
  radians).
- **Right-handed** system, **+Y up**, **-Z forward**.
- World positions are stored as **`double`**; local physics and whatever is
  sent to the renderer use **`float`**. This avoids precision loss on large
  maps without paying for `double` in all the geometry.

## Deterministic physics on its own thread

- **Fixed step** (`1/1000` s by default). Physics does **not** use the frame
  `dt`.
- It runs on its **own thread**. The renderer **interpolates** between two
  snapshots.
- Communication goes through a **lock-free ring** (`SnapshotRing`, SPSC with
  128 slots): there is no mutex on the hot path.
- Strict determinism: MSVC with `/fp:precise`, GCC/Clang with
  `-ffp-contract=off`. **Never** `-ffast-math`, which would break the
  reproducibility needed for replays and debugging.

## RHI isolated from the platform

The RHI (`src/rhi`) **does not know about SDL**. The platform provides a
factory that creates the `VkSurfaceKHR`, and the `Device` requests it through
`DeviceConfig`. This keeps rendering independent of the window, so it could be
reused with another platform.

## volk + VMA instead of the Vulkan loader

**volk** is used to load Vulkan functions dynamically, so **the Vulkan SDK is
not needed** and the loader is not linked. VMA is compiled with
`VMA_IMPLEMENTATION` in a single translation unit (`src/rhi/src/Vma.cpp`) and
uses dynamic functions (`vkGetInstanceProcAddr`/`vkGetDeviceProcAddr`).

## Dynamic rendering (no render passes)

Passes use `vkCmdBeginRendering` instead of `VkRenderPass` + `VkFramebuffer`.
It is simpler, avoids render pass compatibility juggling and is the modern
Vulkan 1.3 path.

## VFS with PhysicsFS

Content access always goes through a virtual file system (PhysicsFS). The base
content and every mod are **all mounted at `/`**; the mount priority decides
which one wins. This lets mods override files without any special logic in the
engine.

## Sandboxed scripting (LuaJIT + sol2)

Scripts run in a **sandbox** (no `io`, `os`, `package`, `debug`, `ffi`; no
`load`/`require`/`dofile`; text only, never bytecode) with an **instruction
limit**. A misbehaving script or one with an infinite loop cannot bring the
simulator down.

## C ABI for plugins

The plugin interface is **plain C** and versioned (`struct_size` + version).
Structures only grow at the end, so a plugin compiled against one version can
keep working. This allows plugins in any language that can produce a C ABI.

## License: GPLv3 with a linking exception

The engine is **GPL-3.0-or-later** with a **linking exception**, so that
plugins and mods do not have to be GPL just because they link against the
engine. The official content is **CC BY-SA 4.0**. No real brands or cars.

## Modern CMake with presets and vcpkg

Configuration with **CMake ≥ 3.25**, **vcpkg manifest** and **CMakePresets**.
This makes it easy to reproduce the environment on Windows (MSVC) and Linux
(GCC/Clang) without manual steps. Slang is not in vcpkg, so it is downloaded as
a prebuilt binary (FetchContent) and can be overridden with `SIM_SLANGC`.

## Slang for shaders

Shaders are written in **Slang** and compiled to SPIR-V at build time. Slang
is a modern language with good ergonomics for passing structures between CPU
and GPU.
