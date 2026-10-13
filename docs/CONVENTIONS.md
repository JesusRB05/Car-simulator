# Convenciones

Normas que deben cumplir todas las contribuciones. Lo esencial está en este
archivo; los detalles de cada módulo viven en sus propios documentos
(«Documentación» en [`README.md`](../README.md)).

## Sistema de coordenadas

- Mundo en coordenadas **diestras**: `+X` a la derecha, `+Y` arriba, **`-Z`
  adelante** (render y escenas de física).
- Ángulos en **radianes**, tiempo en **segundos**, distancias en **metros**
  (unidades SI, nunca otra unidad).
- Excepción deliberada: el vehículo usa **forward = (sin h, 0, cos h)**, es decir
  `+Z` a rumbo 0. Al construir la matriz de modelo del coche hay que convertir a
  la base general (`-Z` adelante). Ver [`FISICA.md`](FISICA.md).

## Convención de matrices (render)

Hay una sola convención y se aplica en todo el render:

- **CPU (C++ / GLM):** matrices **column-major**. `glm::mat4` se copia a la GPU
  **verbatim**; GLM guarda las columnas en memoria (los 16 bytes `[0..15]` son la
  columna 0).
- **GPU (Slang):** toda matriz se declara **`column_major float4x4`** y se
  multiplica como **`mul(M, v)`** (vector columna), exactamente igual que
  `M * v` en GLM.
- La subida se centraliza en `writeMatrix()` (`render/Uniforms.hpp`): es un
  `memcpy` de `glm::mat4` al bloque de uniformes. **Prohibido** transponer,
  cambiar el orden de filas/columnas o negar coordenadas en el sitio de subida.
- Estructuras de uniformes: `FrameUniforms` y `ObjectPushConstants` en
  `render/Uniforms.hpp`. El layout GPU es std140; cada miembro empieza alineado
  a 16 bytes (matriz = 4 columnas de 16 bytes contiguas).
- La transformación CPU→píxel que comprueban los tests (incluida la inversión de
  `+Y` al pasar a coordenadas de pantalla) está en
  `apps/sim/src/HeadlessRunner.cpp` (`checkSemantics`) y en
  `tests/test_render_math.cpp`.

## Profundidad (reversed-Z)

- Proyección en perspectiva **diestra con profundidad inversa** (reversed-Z):
  cerca → 1, lejos → 0. Ver `Camera::setPerspective` y
  [`RENDERIZADO.md`](RENDERIZADO.md).
- El pipeline compara con `GreaterEqual` y limpia la profundidad a `0.0f`.

## Física

- **Doble precisión** para posiciones y estado del mundo; paso fijo
  (`1/1000 s` por defecto) y un solo hilo ejecutando el paso (determinismo).
- Compilación: `-ffast-math` prohibido, FMA contractivo desactivado, MSVC en
  `/fp:precise`.
- Ver [`FISICA.md`](FISICA.md) para integrador, articulaciones y suite de tests.

## Código

- C++17 o superior, estilo `clang-format` (`.clang-format` en la raíz).
- Cabeceras con `#pragma once`, licencia **SPDX GPL-3.0-or-later** en la primera
  línea de cada archivo.
- Documentación en español; identificadores, nombres y mensajes en inglés.
- Shaders en Slang dentro de `src/render/shaders/`; se compilan a SPIR-V con
  `-emit-spirv-via-glsl`.