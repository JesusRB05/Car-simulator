# Golden images

Regression references per **Vulkan driver**. Each subfolder holds the
**approved** golden images for one specific driver, because rasterization
differs slightly between drivers:

- `lavapipe/` — Mesa lavapipe (CI on Linux and local development on Windows).

No golden images are kept in the repository until a person approves them:
**candidates** are first generated in `screenshots/golden_candidates/` and only
move into a folder here after human review.

## Usage

`sim_tests` does not use them; they are consumed by the GPU tests
(`ctest -L gpu`), which read the folder of the driver selected at configure
time:

```bat
cmake --preset windows-msvc-release -DSIM_ENABLE_GPU_TESTS=ON -DSIM_GOLDEN_DRIVER=lavapipe
```

## Approval cycle

1. **Generate candidates** after an intentional change (never to paper over
   regressions):
   `cmake --build <build> --target update_golden`. Candidates are written to
   `screenshots/golden_candidates/`, never directly here.
2. **Review** the candidates and, if approved, copy them to
   `tests/golden/<driver>/<scene>.png`.
3. While a reference is not approved, its GPU test returns **77** and ctest
   reports it as **SKIPPED** (neither passed nor failed); as soon as the PNG
   exists in this folder, it is verified normally.
4. In CI, the `gpu-linux` job generates the candidates if they are missing and
   uploads them as the `golden-candidates` artifact; it does not mark any test
   as failed for lacking an approved reference.

The comparison uses realistic tolerances (at most 1 % differing pixels,
PSNR ≥ 30 dB). See `apps/sim/src/HeadlessRunner.cpp`.

A `scene_debug.png` name is the variant of that scene with the debug-draw
overlay (`--debug-draw`).
