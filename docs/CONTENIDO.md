# Contenido

## Dónde va el contenido

```
content/
  base/                 <- contenido oficial (CC BY-SA 4.0)
    mod.toml
    README.txt
    LICENSE.txt
    scripts/main.lua
mods/                   <- mods del usuario (se escanean al arrancar)
  mi_mod/
    mod.toml
    ...
```

El motor monta `content/base` en la **raíz virtual** (`/`) y, por encima, los
mods activos. Por eso las rutas son siempre relativas a la raíz del VFS, sin
importar de dónde vengan físicamente:

```lua
sim.read("/README.txt")   -- lee content/base/README.txt
```

## Contenido base

`content/base` es, a efectos del sistema, el mod `base`: siempre presente y con
la prioridad más baja. Cualquier mod puede **sobrescribir** sus ficheros
montándose por encima.

Contenido actual:

| Fichero | Descripción |
|---|---|
| `mod.toml` | Manifiesto del contenido base (id `base`, versión `0.1.0`) |
| `README.txt` | Descripción del árbol de contenido |
| `LICENSE.txt` | Aviso CC BY-SA 4.0 y referencia a la licencia raíz |
| `scripts/main.lua` | Script de entrada; solo registra un mensaje de carga |

Los *shaders* compilados (`shaders/*.spv`) **no** están aquí: el build genera el
SPIR-V junto al ejecutable y la aplicación monta ese directorio después de
cargar los mods, para que no lo puedan tapar.

## Licencia del contenido

Todo lo que hay en `content/base` se distribuye bajo **Creative Commons
Attribution-ShareAlike 4.0** (`CC BY-SA 4.0`), mientras que el **código** del
motor está bajo GPL-3.0-or-later con excepción de enlazado. Ver `LICENSE.txt`
en `content/base` para el detalle. Cada mod decide su propia licencia y la
declara en su `mod.toml`.

## Sin marcas reales

A propósito **no** se usan marcas, coches ni circuitos reales en ninguna parte
del contenido. Cualquier contribución con nombres o modelos reales será
rechazada.

## Añadir contenido

### Como mod (recomendado)

Crea `mods/mi_mod/mod.toml` (ver [MODS.md](MODS.md)) y coloca tus ficheros
dentro. Se montarán sobre el contenido base y podrán sustituir sus ficheros.

### Al contenido base

Si quieres contribuir al contenido oficial, añade los ficheros dentro de
`content/base/` respetando la licencia CC BY-SA 4.0 y actualiza su `mod.toml`
si procede. Consulta [CONTRIBUIR.md](CONTRIBUIR.md).

No hay un formato de malla ni de textura cerrado todavía: mientras el
renderizador crece, el contenido puede ir en formato intermedio (`cgltf` y `ktx`
ya están disponibles para glTF y KTX2).
