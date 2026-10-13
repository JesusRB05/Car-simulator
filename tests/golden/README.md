# Dorados (golden images)

Referencias de regresión por **driver Vulkan**. Cada subcarpeta reúne los
dorados **aprobados** por un driver concreto, porque la rasterización difiere
levemente entre ellos:

- `lavapipe/` — Mesa lavapipe (CI en Linux y desarrollo local en Windows).

No se mantienen dorados en el repositorio hasta que una persona los aprueba:
primero se generan **candidatas** en `screenshots/golden_candidates/` y solo
pasan a una carpeta de aquí tras la revisión humana.

## Uso

`sim_tests` no los usa; los consumen los tests GPU (`ctest -L gpu`), que leen la
carpeta del driver seleccionado en configuración:

```bat
cmake --preset windows-msvc-release -DSIM_ENABLE_GPU_TESTS=ON -DSIM_GOLDEN_DRIVER=lavapipe
```

## Ciclo de aprobación

1. **Generar candidatas** tras un cambio intencionado (nada de maquillar
   regresiones):
   `cmake --build <build> --target update_golden`. Las candidatas se escriben en
   `screenshots/golden_candidates/`, nunca directamente aquí.
2. **Revisar** las candidatas y, si se aprueban, copiarlas:
   `tests/golden/<driver>/<escena>.png`.
3. Mientras una referencia no está aprobada, su test GPU devuelve **77** y ctest
   lo reporta como **SKIPPED** (ni pasa ni falla); en cuanto existe el PNG en
   esta carpeta, pasa a verificarse normalmente.
4. En CI, la tarea `gpu-linux` genera las candidatas si faltan y las sube como
   artefacto `golden-candidates`; no marca ningún test como fallido por no tener
   referencia aprobada.

La comparación usa tolerancias realistas (máx. 1 % de píxeles distintos,
PSNR ≥ 30 dB). Ver `apps/sim/src/HeadlessRunner.cpp`.

Un nombre `scene_debug.png` es la variante con el overlay de debug draw
(`--debug-draw`) de esa escena.