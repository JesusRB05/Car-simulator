# Convenciones

Normas que deben cumplir todas las contribuciones. Lo esencial está en este
archivo; los detalles de cada módulo viven en sus propios documentos
(«Documentación» en [`README.md`](../README.md)).

## Sistema de coordenadas

- Mundo en coordenadas **diestras**: `+X` a la derecha, `+Y` arriba, **`-Z`
  adelante** en todo el proyecto, **sin excepciones** (render, escenas de física
  y vehículo).
- Ángulos en **radianes**, tiempo en **segundos**, distancias en **metros**
  (unidades SI, nunca otra unidad).
- Vehicle: heading `h` is a yaw about `+Y`; heading 0 faces `-Z`, so
  `forward = (-sin h, 0, -cos h)` and `right = (cos h, 0, -sin h)`. Positive
  heading, yaw rate and steer turn **left** (counter-clockwise seen from `+Y`).
  The car's model matrix is just `translate(p) * rotate(h, +Y)`, no axis
  conversion. See [`FISICA.md`](FISICA.md).

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