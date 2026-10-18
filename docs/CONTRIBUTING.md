# Contributing

Thanks for wanting to help. This is a community project developed in the open.

## Principles

- **No third-party engines**: physics and rendering are our own. Jolt, Bullet,
  PhysX, bgfx, Diligent and Filament are not accepted.
- **No real brands**: no real cars, tracks or brands, neither in code nor in
  content.
- **Determinism**: do not introduce floating-point optimisations that break
  reproducibility (`-ffast-math`, FMA contraction, etc.).
- **Everything in English**: code, comments, documentation and commit
  messages.

## Setting up the environment

Follow [`BUILDING.md`](BUILDING.md). In short:

```powershell
cmake --preset windows-msvc-debug
cmake --build --preset windows-msvc-debug
ctest --test-dir build/windows-msvc-debug --output-on-failure
```

Before submitting changes, check **Debug and Release** and that the tests pass
in both:

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --test-dir build/windows-msvc-release --output-on-failure
```

### Building with warnings as errors

Warnings are enabled by default (`SIM_WARNINGS=ON`). To treat them as errors:

```powershell
cmake --preset windows-msvc-debug -DSIM_WARNINGS_AS_ERRORS=ON
```

## Code style

- **C++20**. Follow the style of the existing code.
- Format with the repository's `.clang-format`.
- **No unnecessary comments**: code should explain itself; only comment what
  is not obvious (the "why", not the "what").
- **SPDX header** required at the top of every source file:

  ```cpp
  // SPDX-License-Identifier: GPL-3.0-or-later
  // Copyright (C) 2026 The Car Simulator contributors
  ```

  In CMake files and others that use `#` for comments, the same header with
  `#`. In plain C (plugins) with `/* ... */`.
- Respect module isolation: dependencies between modules are already defined
  in the `CMakeLists.txt` files. Do not add new dependencies without a reason.
- Namespaces: each module lives in `sim::<module>`.

## Tests

The suite uses **doctest** and lives in `tests/`. If you add functionality, add
its tests too. To register a case:

```cpp
#include <doctest/doctest.h>

TEST_CASE("description")
{
    CHECK(1 + 1 == 2);
}
```

## Adding a module or changing dependencies

- External dependencies are declared in `vcpkg.json`. When adding one, also
  update `cmake/SimDependencies.cmake`.
- New modules are added with `add_subdirectory` and their options in the root
  `CMakeLists.txt`.
- Presets and options live in `CMakePresets.json` and `CMakeLists.txt`
  (`SIM_BUILD_TESTS`, `SIM_WARNINGS`, `SIM_WARNINGS_AS_ERRORS`,
  `SIM_ENABLE_PROFILING`, `SIM_ENABLE_SLANG`, `SIM_ENABLE_OPENXR`,
  `SIM_ENABLE_FSR`).

## Workflow

1. Fork the repository and create a descriptive branch.
2. Make small, focused changes; commits must be reviewable.
3. Add or update the documentation in `docs/` if the change affects it.
4. Make sure Debug, Release and the tests pass.
5. Open a *pull request* explaining **what** and **why**.

Continuous integration (`.github/workflows/ci.yml`) builds and runs the tests
on Linux (GCC and Clang); it must pass before merging.

### Signing off commits (DCO)

Every commit must include the developer sign-off (Developer Certificate of
Origin):

```text
Signed-off-by: Full Name <email>
```

With Git, add `-s` when committing (`git commit -s`); the sign-off is built
from `user.name`/`user.email`. By signing off you state that you have the right
to publish the change under the project's license. *Pull requests* whose
commits lack `Signed-off-by` are not accepted.

## Adding content

New content normally goes into a **mod** (`mods/`), not into the base tree. To
contribute to the official content (`content/base`), respect the
**CC BY-SA 4.0** license and do not use real brands. More details in
[`MODS.md`](MODS.md) and [`CONTENT.md`](CONTENT.md).

### Large files use Git LFS

Models (`*.glb`, `*.gltf`) and the textures in `content/` (`*.png`, `*.jpg`,
`*.jpeg`, `*.bin`) are stored with **Git LFS**; the rules are in
[`.gitattributes`](../.gitattributes).

- One-time setup, per machine: install Git LFS (`git lfs install`). Git for
  Windows ships it; on Debian/Ubuntu it is the `git-lfs` package.
- Cloning then pulls the real files automatically. If a clone was made without
  LFS (or an LFS download was skipped), fetch the content with `git lfs pull`.
- A file committed without the filter (a "pointer" of a few lines) means LFS was
  not installed when it was added: run `git lfs install` and re-add the file.
- The hygiene check fails when a tracked `*.glb` or a `content/` image over
  1 MiB is not tracked by LFS, so this cannot slip through CI.

## License of contributions

By submitting code changes you agree that they are distributed under
**GPL-3.0-or-later** with the project's linking exception. Content you submit
to `content/base` is distributed under **CC BY-SA 4.0**. Each mod chooses its
own license.
