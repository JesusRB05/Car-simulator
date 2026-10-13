# Scripting (Lua)

Mods can include Lua scripts. The engine uses **LuaJIT** with the **sol2**
binding, but inside a deliberately restrictive **sandbox**.

## API available to scripts

The host installs a global `sim` table:

| Function / value | Description |
|---|---|
| `sim.log(message)` | Writes to the log, tagged with the mod name |
| `sim.read(path)` | Reads a file **through the VFS**; returns the text or `nil` |
| `sim.register_on_load(function)` | Registers the function called when the mod loads |
| `sim.register_on_update(function)` | Registers the function called every frame, with `dt` |
| `sim.game_version` | Game version string (e.g. `"0.1.0"`) |

Example (`content/base/scripts/main.lua`):

```lua
local loaded = false

sim.register_on_load(function()
    sim.log("base content loaded (game " .. sim.game_version .. ")")
    loaded = true
end)

sim.register_on_update(function(dt)
    -- dt is the frame time in seconds
end)
```

## Sandbox

`LuaSandbox` opens **only** the safe libraries: `base`, `coroutine`, `math`,
`string` and `table`. The dangerous libraries (`io`, `os`, `package`, `debug`,
`ffi`) are **not** opened; in addition, the functions that would allow escaping
the sandbox or loading bytecode are removed from `base`:

- `load`, `loadstring`, `loadfile`, `dofile`, `require`, `collectgarbage`
- `print` (use `sim.log` instead)
- `string.dump`

*Chunks* are always loaded in **text mode** (`"t"`), so precompiled bytecode is
rejected: malicious compiled code cannot be slipped in.

### Instruction budget

`LuaSandbox` installs a debug *hook* with `LUA_MASKCOUNT` every `kHookCount`
(1000) instructions. When a script exceeds the budget (5,000,000 instructions
by default), the hook raises a Lua error that aborts **that** script without
freezing the simulator. This way a `while true do end` does not block the game.
The budget is adjusted with `setInstructionBudget()`.

### JIT engine disabled

LuaJIT does not call debug hooks while executing JIT-compiled traces, so once a
hot loop such as `while true do end` is compiled the instruction-count hook
never fires again and the game would hang. `LuaSandbox` therefore calls
`luaJIT_setmode(L, 0, LUAJIT_MODE_ENGINE | LUAJIT_MODE_OFF)` right after
creating the state, before any mod script is loaded: all mod code runs in the
LuaJIT interpreter, where the budget is always enforced. The `jit` and `ffi`
libraries are never opened and their globals are cleared, so scripts cannot
turn the engine back on. Mod scripts are small gameplay glue, so the
interpreter's speed is more than enough; safety wins over raw throughput.

## Loading

`ScriptHost::loadScript(modId, path)` reads the script from the VFS and runs
it. Functions registered during execution are stored associated with that mod.
The host exposes:

- `callOnLoad()`: runs every `on_load` (in load order).
- `callOnUpdate(dt)`: runs every `on_update` with the frame `dt`.
- `clear()`: releases all Lua functions; **it must be called before**
  destroying the sandbox.

Logging uses `core::Log::forMod(modId)`, so each mod's messages are tagged with
its id.

## The `entry_script`

A mod declares its entry script in `mod.toml`:

```toml
entry_script = "scripts/main.lua"
```

The application loads that script (through `ScriptHost`) after mounting the
mods. If an `entry_script` fails, the error is logged and loading continues.
