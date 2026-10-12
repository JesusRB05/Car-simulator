-- SPDX-License-Identifier: GPL-3.0-or-later
-- Car Simulator base entry script. Runs inside the sandboxed Lua state.

sim.log("base script loaded (sim.read sees the VFS only)")
sim.register_on_load(function()
    sim.log("base on_load fired")
end)
sim.register_on_update(function(dt)
    -- Intentionally silent: a per-frame sim.log would spam the log file.
end)