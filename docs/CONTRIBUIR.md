# Contribuir

Gracias por querer echar una mano. Este proyecto es de la comunidad y se
mantiene en abierto.

## Principios

- **Sin motores de terceros**: la física y el render son propios. No se aceptan
  Jolt, Bullet, PhysX, bgfx, Diligent ni Filament.
- **Sin marcas reales**: nada de coches, circuitos ni marcas reales, ni en
  código ni en contenido.
- **Determinismo**: no introduzcas optimizaciones de punto flotante que rompan
  la reproducibilidad (`-ffast-math`, contracción FMA, etc.).
- **Código en inglés, documentación en español**.

## Poner en marcha el entorno

Sigue [`CONSTRUCCION.md`](CONSTRUCCION.md). Resumen:

```powershell
cmake --preset windows-msvc-debug
cmake --build --preset windows-msvc-debug
ctest --test-dir build/windows-msvc-debug --output-on-failure
```

Antes de enviar cambios, comprueba **Debug y Release** y que los tests pasan en
ambos:

```powershell
cmake --preset windows-msvc-release
cmake --build --preset windows-msvc-release
ctest --test-dir build/windows-msvc-release --output-on-failure
```

### Compilación con avisos como errores

Los avisos están activos por defecto (`SIM_WARNINGS=ON`). Para tratarlos como
errores:

```powershell
cmake --preset windows-msvc-debug -DSIM_WARNINGS_AS_ERRORS=ON
```

## Estilo de código

- **C++20**. Sigue el estilo del código que ya existe.
- Formato con el `.clang-format` del repositorio.
- **Sin comentarios innecesarios**: el código debe explicarse solo; comenta solo
  lo que no es obvio (el "por qué", no el "qué").
- **Cabecera SPDX** obligatoria al principio de cada fichero fuente:

  ```cpp
  // SPDX-License-Identifier: GPL-3.0-or-later
  // Copyright (C) 2026 The Car Simulator contributors
  ```

  En ficheros de CMake y otros que usan `#` como comentario, la misma cabecera
  con `#`. En C puro (plugins) con `/* ... */`.
- Respeta el aislamiento de módulos: la dependencia entre módulos ya está
  definida en los `CMakeLists.txt`. No añadas dependencias nuevas sin
  justificarlo.
- Espacios de nombres: cada módulo vive en `sim::<modulo>`.

## Tests

La suite usa **doctest** y vive en `tests/`. Si añades funcionalidad, añade
también sus tests. Para registrar un caso:

```cpp
#include <doctest/doctest.h>

TEST_CASE("descripción")
{
    CHECK(1 + 1 == 2);
}
```

## Añadir un módulo o cambiar las dependencias

- Las dependencias externas se declaran en `vcpkg.json`. Al añadir una, actualiza
  también `cmake/SimDependencies.cmake`.
- Los módulos nuevos se añaden con `add_subdirectory` y sus opciones en el
  `CMakeLists.txt` raíz.
- Los presets y opciones viven en `CMakePresets.json` y `CMakeLists.txt`
  (`SIM_BUILD_TESTS`, `SIM_WARNINGS`, `SIM_WARNINGS_AS_ERRORS`,
  `SIM_ENABLE_PROFILING`, `SIM_ENABLE_SLANG`, `SIM_ENABLE_OPENXR`,
  `SIM_ENABLE_FSR`).

## Flujo de trabajo

1. Haz un *fork* y crea una rama descriptiva.
2. Haz cambios pequeños y enfocados; los commits deben poder revisarse.
3. Añade o actualiza la documentación de `docs/` si el cambio la afecta.
4. Asegúrate de que Debug, Release y los tests pasan.
5. Abre una *pull request* explicando **qué** y **por qué**.

La integración continua (`.github/workflows/ci.yml`) compila y ejecuta los tests
en Linux (GCC y Clang); el trabajo debe pasar antes de fusionarse.

### Firma de los commits (DCO)

Todo commit debe incluir la firma de desarrollador (Developer Certificate of
Origin):

```text
Signed-off-by: Nombre Apellidos <email>
```

Con Git basta añadir `-s` al confirmar (`git commit -s`); la firma se añade a
partir de `user.name`/`user.email`. Al firmar afirmas que tienes derecho a
publicar el cambio bajo la licencia del proyecto. No se aceptan *pull requests*
cuyos commits carezcan de `Signed-off-by`.

## Añadir contenido

El contenido nuevo normalmente va en un **mod** (`mods/`), no en el árbol base.
Para contribuir al contenido oficial (`content/base`), respeta la licencia
**CC BY-SA 4.0** y no uses marcas reales. Más detalles en
[`MODS.md`](MODS.md) y [`CONTENIDO.md`](CONTENIDO.md).

## Licencia de las contribuciones

Al enviar cambios de código aceptas que se distribuyan bajo **GPL-3.0-or-later**
con la excepción de enlazado del proyecto. El contenido que envíes a
`content/base` se distribuye bajo **CC BY-SA 4.0**. Cada mod decide su licencia.
