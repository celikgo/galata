#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Collect actual installed vcpkg notices and version/hash evidence for an archive."""
import hashlib
import json
from pathlib import Path
import sys


def collect(build, destination):
    cache = {}
    for line in (build / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
        if line and not line.startswith(("#", "//")) and "=" in line:
            key, value = line.split("=", 1)
            cache[key.split(":", 1)[0]] = value
    installed = Path(cache["VCPKG_INSTALLED_DIR"])
    triplet = cache["VCPKG_TARGET_TRIPLET"]
    records = []
    for block in (installed / "vcpkg/status").read_text(encoding="utf-8").split("\n\n"):
        record = dict(line.split(": ", 1) for line in block.splitlines()
                      if ": " in line and not line.startswith(" "))
        if (record.get("Status") == "install ok installed"
                and record.get("Architecture") == triplet and "Feature" not in record):
            records.append(record)
    if not {"eigen3", "yaml-cpp"}.issubset({item["Package"] for item in records}):
        raise ValueError("runtime dependency metadata is incomplete")
    destination.mkdir(parents=True, exist_ok=False)
    inventory = []
    for record in sorted(records, key=lambda item: item["Package"]):
        name = record["Package"]
        data = (installed / triplet / "share" / name / "copyright").read_bytes()
        if not data.strip():
            raise ValueError(f"empty notice for {name}")
        filename = name + ".txt"
        (destination / filename).write_bytes(data)
        inventory.append({"name": name, "version": record["Version"],
                          "port_version": record.get("Port-Version", "0"),
                          "architecture": triplet, "notice": filename,
                          "notice_sha256": hashlib.sha256(data).hexdigest(),
                          "role": "runtime" if name in {"eigen3", "yaml-cpp"} else "build/test"})
    (destination / "dependencies.json").write_text(
        json.dumps(inventory, indent=2) + "\n", encoding="utf-8")
    print(f"Collected {len(inventory)} dependency notices in {destination}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: collect-dependency-notices.py <build-directory> <new-output-directory>")
    try:
        collect(Path(sys.argv[1]), Path(sys.argv[2]))
    except (OSError, KeyError, ValueError) as error:
        sys.exit(f"cannot package dependency notices: {error}")
