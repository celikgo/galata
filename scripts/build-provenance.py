#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Identify source bytes and effective compile configuration before compilation."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import re
import sys

from provenance_support import canonical, digest, inventory, inventory_digest, read_cache, write_if_changed


def file_identity(path):
    source = Path(path)
    return {"path": str(source), "resolved_path": str(source.resolve()),
            "sha256": digest(source)} if source.is_file() else {"path": str(source), "status": "unavailable"}


def build_configuration(build):
    cache = read_cache(build / "CMakeCache.txt")
    commands_file = build / "compile_commands.json"
    if not commands_file.is_file():
        raise ValueError("compile_commands.json is required for effective build provenance (use Ninja)")
    commands = json.loads(commands_file.read_text(encoding="utf-8"))
    if not isinstance(commands, list) or not commands:
        raise ValueError("compile_commands.json must contain compilation commands")
    commands = sorted(commands, key=lambda item: (item["file"], item.get("output", ""), canonical(item)))
    encoded_commands = canonical(commands)
    write_if_changed(build / "galata-compile-commands.json", encoded_commands)
    # Only configuration fields are retained; arbitrary cache entries can carry
    # credentials or unrelated application data. Effective per-target flags are
    # also captured in the compilation database, not inferred from defaults.
    exact = {"CMAKE_BUILD_TYPE", "CMAKE_GENERATOR", "CMAKE_C_COMPILER", "CMAKE_CXX_COMPILER",
             "CMAKE_C_STANDARD", "CMAKE_CXX_STANDARD", "CMAKE_TOOLCHAIN_FILE", "CMAKE_SYSROOT",
             "CMAKE_OSX_SYSROOT", "CMAKE_OSX_ARCHITECTURES", "CMAKE_OSX_DEPLOYMENT_TARGET",
             "CMAKE_MAKE_PROGRAM", "BUILD_SHARED_LIBS", "BUILD_TESTING",
             "VCPKG_TARGET_TRIPLET", "VCPKG_HOST_TRIPLET"}
    flags = re.compile(r"CMAKE_(?:C|CXX|EXE_LINKER|SHARED_LINKER|MODULE_LINKER|STATIC_LINKER)_FLAGS(?:_\w+)?$")
    selected = {key: value for key, value in cache.items()
                if key in exact or flags.fullmatch(key) or key.startswith("GALATA_")}
    tools = {key: file_identity(cache[key]) for key in
             ("CMAKE_C_COMPILER", "CMAKE_CXX_COMPILER", "CMAKE_MAKE_PROGRAM", "CMAKE_TOOLCHAIN_FILE")
             if cache.get(key)}
    return {"schema": "galata.build-configuration.v1", "cache": selected,
            "compile_commands_sha256": hashlib.sha256(encoded_commands.encode()).hexdigest(),
            "compile_command_count": len(commands), "tools": tools,
            "cmake_version": ".".join(cache.get("CMAKE_CACHE_" + part + "_VERSION", "unknown")
                                       for part in ("MAJOR", "MINOR", "PATCH")),
            "host": {"system": platform.system(), "release": platform.release(),
                     "version": platform.version(), "machine": platform.machine()},
            "scope": "Digest of effective compilation commands (retained in galata-compile-commands.json), "
                     "selected linker/configuration flags and tool files; not a complete sysroot "
                     "or reproducible-build attestation."}


def generate(source, build):
    records = inventory(source, excluded=(build,))
    configuration = build_configuration(build)
    encoded = canonical(configuration)
    identity = hashlib.sha256(encoded.encode()).hexdigest()
    write_if_changed(build / "galata-build-configuration.json", encoded)
    source_identity = inventory_digest(records)
    write_if_changed(build / "galata-source-inventory.json", canonical({
        "source_tree_sha256": source_identity, "files": records}) + "\n")
    print(source_identity + ";" + identity)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("build", type=Path)
    args = parser.parse_args()
    try:
        generate(args.source.resolve(), args.build.resolve())
    except (OSError, ValueError, KeyError, TypeError) as error:
        sys.exit(f"build provenance failed: {error}")
