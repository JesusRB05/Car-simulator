# Scripting (Lua)

Los mods pueden incluir scripts Lua. El motor usa **LuaJIT** con el enlace de
**sol2**, pero dentro de un **sandbox** deliberadamente restrictivo.

## API disponible para los scripts

El host instala una tabla global `sim`:

| Función / valor | Descripción |
|---|---|
| `sim.log(mensaje)` | Escribe en el log con el nombre del mod como etiqueta |
| `sim.read(ruta)` | Lee un fichero **a través del VFS**; devuelve el texto o `nil` |
| `sim.register_on_load(funcion)` | Registra la función que se llamará al cargar el mod |
| `sim.register_on_update(funcion)` | Registra la función que se llamará cada frame, con `dt` |
| `sim.game_version` | Cadena de versión del juego (p. ej. `"0.1.0"`) |

Ejemplo (`content/base/scripts/main.lua`):

```lua
local loaded = false

sim.register_on_load(function()
    sim.log("base content loaded (game " .. sim.game_version .. ")")
    loaded = true
end)

sim.register_on_update(function(dt)
    -- dt es el tiempo de frame en segundos
end)
```

## Sandbox

`LuaSandbox` abre **solo** las bibliotecas seguras: `base`, `coroutine`,
`math`, `string` y `table`. Las bibliotecas peligrosas (`io`, `os`, `package`,
`debug`, `ffi`) **no** se abren; además se eliminan de `base` las funciones que
permitirían escapar del sandbox o cargar bytecode:

- `load`, `loadstring`, `loadfile`, `dofile`, `require`, `collectgarbage`
- `print` (usa `sim.log` en su lugar)
- `string.dump`

Los *chunks* se cargan siempre en **modo texto** (`"t"`), de modo que se
rechaza bytecode precompilado: no se puede colar código compilado malicioso.

### Presupuesto de instrucciones

`LuaSandbox` instala un *hook* de depuración con `LUA_MASKCOUNT` cada
`kHookCount` (1000) instrucciones. Cuando un script supera el presupuesto
(5 000 000 de instrucciones por defecto), el hook lanza un error de Lua que
aborta **ese** script sin congelar el simulador. Así un `while true do end` no
bloquea el juego. El presupuesto se ajusta con `setInstructionBudget()`.

## Carga

`ScriptHost::loadScript(modId, ruta)` lee el script del VFS y lo ejecuta. Las
funciones registradas durante la ejecución se guardan asociadas a ese mod. El
host expone:

- `callOnLoad()`: ejecuta todos los `on_load` (en orden de carga).
- `callOnUpdate(dt)`: ejecuta todos los `on_update` con el `dt` del frame.
- `clear()`: libera todas las funciones Lua; **hay que llamarlo antes** de
  destruir el sandbox.

El log usa `core::Log::forMod(modId)`, de forma que los mensajes de cada mod
quedan etiquetados con su id.

## El `entry_script`

Un mod declara su script de entrada en `mod.toml`:

```toml
entry_script = "scripts/main.lua"
```

La aplicación carga ese script (a través de `ScriptHost`) después de montar los
mods. Si un `entry_script` falla, se registra el error y se continúa.
