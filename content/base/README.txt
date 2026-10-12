Car Simulator - Base Content (CC BY-SA 4.0)

This directory is the official, community-maintained base content tree. It is
mounted by the engine at the virtual root. Files in a mod that share a path
override these (later mounts win).

Layout so far:
  mod.toml          base mod manifest (id "base")
  scripts/main.lua  base entry script, loaded by the application on startup
  shaders/          built SPIR-V lives next to the executable, not here

No real car brands, cars or tracks are used anywhere on purpose.