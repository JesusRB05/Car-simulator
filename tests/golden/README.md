# Dorados (golden images)

Referencias de regresión por **driver Vulkan**. Cada subcarpeta reúne los
dorados generados por un driver concreto, porque la rasterización difiere
levemente entre ellos:

- `lavapipe/` — Mesa lavapipe (CI en Linux y desarrollo local en Windows).
- `swiftshader/` — SwiftShader (solo si se usa como ICD local; no se mantiene
  en el repositorio salvo que alguien lo genere).

## Uso

`sim_tests` no los usa; los consumen los tests GPU (`ctest -L gpu`), que leen la
carpeta del driver seleccionado en configuración:

```bat
cmake --preset windows-msvc-release -DSIM_ENABLE_GPU_TESTS=ON -DSIM_GOLDEN_DRIVER=lavapipe
```

## Regeneración y política

- Regenerar **solo** tras un cambio intencionado (nada de maquillar regresiones):
  `cmake --build <build> --target update_golden`.
- En CI, la tarea `gpu-linux` genera los dorados de `lavapipe` **si faltan** y los
  sube como artefacto `golden-lavapipe`.
- La comparación usa tolerancias realistas (máx. 1 % de píxeles distintos,
  PSNR ≥ 30 dB). Ver `apps/sim/src/HeadlessRunner.cpp`.

Un nombre `scene_debug.png` es la variante con el overlay de debug draw
(`--debug-draw`) de esa escena.