# Sistema de mods

El motor está pensado para ser modificable. Los mods **no cambian el binario**:
se montan sobre el sistema de ficheros virtual (VFS) y, opcionalmente, añaden
scripts Lua o plugins nativos.

## Dónde vive un mod

```
mods/
  mi_mod/
    mod.toml            <- manifiesto obligatorio
    scripts/main.lua    <- opcional (entry_script)
    ...                 <- cualquier otro contenido (texturas, mallas, datos)
```

Un mod puede ser un **directorio** o un **`.zip`**. El gestor escanea
`mods/`, y cada entrada es candidata a mod. El contenido oficial
(`content/base`) es, a efectos del sistema, otro mod más (el "base"), siempre
presente y de menor prioridad.

`ModManager`:

1. Descubre candidatos (contenido base primero, luego los mods ordenados por
   nombre).
2. Lee y valida cada `mod.toml`.
3. Resuelve dependencias y obtiene el orden de carga.
4. Desmonta todo el VFS y monta el contenido base y, encima, los mods activos.
5. Persiste la lista de mods activos y genera un informe.

Un mod inválido **nunca** aborta el arranque: se marca como `Invalid` con un
mensaje claro y el resto sigue cargando.

## `mod.toml`

```toml
id = "mi_mod"                 # obligatorio, único
name = "Mi Mod"               # obligatorio
version = "1.0.0"             # obligatorio, semver (x.y.z)
author = "Tu nombre"          # opcional
license = "CC BY-SA 4.0"      # obligatorio
description = "Qué hace"      # opcional
min_game_version = "0.1.0"    # opcional; semver o "*"
entry_script = "scripts/main.lua"  # opcional
priority = 10                 # opcional; mayor = se monta más arriba
dependencies = [
    "otro_mod",
    "biblioteca >=1.2.0",
    "^otra_lib 1.4.0",
]
```

### Campos

| Campo | Obligatorio | Notas |
|---|---|---|
| `id` | Sí | minúsculas, dígitos, `-` y `_`, empezando por letra |
| `name` | Sí | Nombre para mostrar |
| `version` | Sí | Semver válido (`major.minor.patch[-prerelease]`) |
| `license` | Sí | Cadena SPDX o descriptiva |
| `author` | No | |
| `description` | No | |
| `min_game_version` | No | Semver o `"*"`. Si no es parseable, el mod es inválido |
| `entry_script` | No | Ruta dentro del mod a un script Lua de entrada |
| `priority` | No | Entero; **mayor prioridad ⇒ se monta más arriba** (sobrescribe) |
| `dependencies` | No | Lista de `"id"` o `"id <rango de versión>"` |

### Dependencias

Son **cadenas**, no tablas. Cada entrada es un `id` opcionalmente seguido de un
espacio y un rango de versión:

```toml
dependencies = ["base", "ruedas >=2.0.0 <3.0.0"]
```

`resolveMods` (en `DependencyResolver`) construye un orden de carga **topológico
y estable**: un mod aparece siempre después de sus dependencias. Las
dependencias de los mods pedidos se activan automáticamente. Se detectan y
reportan:

- dependencias que faltan,
- rangos de versión no satisfechos,
- ciclos,
- ids duplicados entre dos mods (conflicto).

Los mods afectados por un error quedan deshabilitados; el resto carga igual.

## Rangos de versión

Sintaxis soportada por `satisfies()` (todos los comparadores se combinan con
AND; las alternativas se separan con `||`):

| Rango | Significado |
|---|---|
| `*` | Cualquier versión |
| `1.2.3`, `=1.2.3` | Coincidencia exacta |
| `1.2`, `1` | Coincidencia por prefijo |
| `>=1.2.0`, `>1.2.0` | Comparación |
| `<=1.2.0`, `<1.2.0` | Comparación |
| `^1.2.3` | `>=1.2.3` y `<2.0.0` (respetando la regla de `0.x`) |
| `~1.2.3`, `~1.2` | `>=1.2.3` y `<1.3.0` |
| `>=1.0.0 <2.0.0` | Varios comparadores |

El *build metadata* de semver (`+algo`) se ignora en las comparaciones.

## Prioridad y sobrescritura

Todos los mods (y el contenido base) se montan en el **mismo punto virtual**
(`/`). La prioridad de montaje determina quién gana cuando dos mods tienen la
misma ruta:

```
base (siempre, la más baja)
  → mods activos con priority = 0 ...
  → mods activos con priority mayor (por encima)
```

Como un mod de mayor prioridad se monta **después**, sus ficheros sobrescriben
los de prioridad menor. Un mod puede reemplazar, por ejemplo, una textura del
contenido base sin tocar nada del motor.

## Activar y desactivar mods

Los mods activos se guardan en la configuración del usuario, en la clave
`mods.active` (una lista de ids). El contenido base se pide siempre, tanto si
está en la lista como si no.

`ModManager::setModActive(id, activo)` actualiza la lista y recarga. La
configuración se guarda automáticamente tras cada `reload()`.

## Informe

`ModReport` (accesible por `ModManager::report()`) describe cada mod descubierto
con su estado (`Active`, `Disabled`, `Invalid`), dependencias y errores, más una
lista de mensajes (conflictos, ciclos, ...). `toString()` devuelve un resumen
legible que el motor escribe en el log.
