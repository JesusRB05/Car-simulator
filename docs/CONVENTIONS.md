# Conventions

Rules that every contribution must follow. The essentials are in this file;
the details of each module live in their own documents ("Documentation" in
[`README.md`](../README.md)).

## Coordinate system

- **Right-handed** world coordinates: `+X` right, `+Y` up, **`-Z` forward**
  across the whole project, **with no exceptions** (rendering, physics scenes
  and vehicle).
- Angles in **radians**, time in **seconds**, distances in **metres** (SI
  units, never any other unit).
- Vehicle: heading `h` is a yaw about `+Y`; heading 0 faces `-Z`, so
  `forward = (-sin h, 0, -cos h)` and `right = (cos h, 0, -sin h)`. Positive
  heading, yaw rate and steer turn **left** (counter-clockwise seen from `+Y`).
  The car's model matrix is just `translate(p) * rotate(h, +Y)`, no axis
  conversion. See [`PHYSICS.md`](PHYSICS.md).

## Matrix convention (rendering)

There is a single convention and it applies to all rendering code:

- **CPU (C++ / GLM):** **column-major** matrices. `glm::mat4` is copied to the
  GPU **verbatim**; GLM stores columns in memory (the first 16 bytes are
  column 0).
- **GPU (Slang):** every matrix is declared **`column_major float4x4`** and
  multiplied as **`mul(M, v)`** (column vector), exactly like `M * v` in GLM.
- Uploads go through `writeMatrix()` (`render/Uniforms.hpp`): a `memcpy` of a
  `glm::mat4` into the uniform block. Transposing, reordering rows/columns or
  negating coordinates at the upload site is **forbidden**.
- Uniform structures: `FrameUniforms` and `ObjectPushConstants` in
  `render/Uniforms.hpp`. The GPU layout is std140; every member starts 16-byte
  aligned (a matrix = 4 contiguous 16-byte columns).
- Screen orientation is defined **exactly once**, in the RHI:
  `CommandList::setViewport` emits a negative-height viewport, so world `+Y`
  points up in the image and world `+X` points right (standard projection, no
  other flip anywhere: not in the projection, the shaders or the matrix upload).
  `tests/test_render_math.cpp` and `checkSemantics` use the standard
  NDC-to-pixel mapping; `checkOrientation` (headless `orientation` scene)
  verifies sky-up/ground-down, placement and culling from pixels alone,
  without the projection matrix.
- Winding: every mesh is **counter-clockwise seen from outside** and the
  pipeline uses `VK_FRONT_FACE_COUNTER_CLOCKWISE` with back-face culling
  (`tests/test_render_math.cpp` checks every built-in mesh).

## Depth (reversed-Z)

- **Right-handed perspective projection with reversed depth** (reversed-Z):
  near → 1, far → 0. See `Camera::setPerspective` and
  [`RENDERING.md`](RENDERING.md).
- The pipeline compares with `GreaterEqual` and clears depth to `0.0f`.

## Physics

- **Double precision** for positions and world state; fixed step (`1/1000 s`
  by default) and a single thread running the step (determinism).
- Compilation: `-ffast-math` is forbidden, FMA contraction is disabled, MSVC
  uses `/fp:precise`.
- See [`PHYSICS.md`](PHYSICS.md) for the integrator, joints and test suite.

## Code

- C++17 or later, `clang-format` style (`.clang-format` at the root).
- Headers use `#pragma once`; **SPDX GPL-3.0-or-later** license on the first
  line of every file.
- Everything in English: documentation, comments, identifiers, names and
  messages.
- Slang shaders live in `src/render/shaders/`; they are compiled to SPIR-V
  with `-emit-spirv-via-glsl`.
