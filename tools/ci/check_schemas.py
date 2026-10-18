#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 The Car Simulator contributors
"""Keep the editor JSON schemas in docs/schemas/ in step with the content.

Dependency-free (standard library only). It checks two things and fails with a
non-zero exit code if either is violated:

1. Every schema is well-formed JSON with a ``$schema`` and a ``properties``
   object.
2. Every key used by the TOML descriptors in ``content/`` and ``mods/`` is
   declared by the schema for that file, so a schema cannot silently drift from
   the format reference in docs/MOD_FORMAT.md.

Run from the repository root::

    python tools/ci/check_schemas.py
"""

from __future__ import annotations

import json
import re
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCHEMA_DIR = ROOT / "docs" / "schemas"
CONTENT_DIRS = [ROOT / "content", ROOT / "mods"]


def schema_for(path: Path) -> str | None:
    """Which schema describes this descriptor, by its name and folder."""
    name = path.name
    parts = path.parts
    if name == "mod.toml":
        return "mod.schema.json"
    if name == "car.toml":
        return "car.schema.json"
    if name == "track.toml":
        return "track.schema.json"
    if name in ("livery.toml", "skin.toml"):
        return "livery.schema.json"
    if name == "collection.toml":
        return "collection.schema.json"
    if "parts" in parts:
        return "part.schema.json"
    if "setups" in parts:
        return "setup.schema.json"
    if "patches" in parts:
        return "patch.schema.json"
    return None


def read_toml(path: Path) -> dict:
    # utf-8-sig tolerates a byte-order mark, as the loader does.
    return tomllib.loads(path.read_text(encoding="utf-8-sig"))


def key_is_declared(key: str, schema: dict) -> bool:
    if key in schema.get("properties", {}):
        return True
    for pattern in schema.get("patternProperties", {}):
        if re.search(pattern, key):
            return True
    additional = schema.get("additionalProperties", True)
    if additional is False or additional is None:
        return False
    return True


def check_object(where: str, value: dict, schema: dict, failures: list[str]) -> None:
    for key, child in value.items():
        if not key_is_declared(key, schema):
            failures.append(f"{where}: key '{key}' is not declared by the schema")
            continue
        child_schema = schema.get("properties", {}).get(key)
        if isinstance(child, dict) and isinstance(child_schema, dict):
            # A declared section with its own properties is checked one level
            # deep too (that is where most drift happens).
            if "properties" in child_schema:
                check_object(f"{where}.{key}", child, child_schema, failures)


def main() -> int:
    failures: list[str] = []
    schemas: dict[str, dict] = {}

    for path in sorted(SCHEMA_DIR.glob("*.schema.json")):
        try:
            schema = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            failures.append(f"{path.name}: not valid JSON ({error})")
            continue
        if "$schema" not in schema:
            failures.append(f"{path.name}: missing $schema")
        if "properties" not in schema:
            failures.append(f"{path.name}: missing properties")
        schemas[path.name] = schema

    checked = 0
    for directory in CONTENT_DIRS:
        if not directory.is_dir():
            continue
        for path in sorted(directory.rglob("*.toml")):
            name = schema_for(path)
            if name is None:
                continue
            schema = schemas.get(name)
            if schema is None:
                failures.append(f"{path}: no schema named {name}")
                continue
            try:
                document = read_toml(path)
            except (OSError, tomllib.TOMLDecodeError) as error:
                failures.append(f"{path}: cannot parse ({error})")
                continue
            where = str(path.relative_to(ROOT)).replace("\\", "/")
            check_object(where, document, schema, failures)
            checked += 1

    for line in failures:
        print(f"error: {line}", file=sys.stderr)
    if failures:
        print(f"schemas: FAILED ({len(failures)} problem(s))", file=sys.stderr)
        return 1
    print(f"schemas: OK ({len(schemas)} schema(s), {checked} descriptor(s) checked)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
