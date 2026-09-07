# SPDX-License-Identifier: Apache-2.0
"""Canonical source inventories shared by build stamping and release packaging."""
import hashlib
import json
import os
from pathlib import Path
import subprocess


EXCLUDED_PARTS = {".git", "build", "out", "_deps", "vcpkg_installed", "__pycache__",
                  ".venv", "node_modules", "target", ".idea", ".vscode", ".cache"}
# These are generated output locations declared in .gitignore. Some legacy
# reports remain tracked, so Git's untracked-file exclusions alone are not
# sufficient. Do not replace this list with a file-extension allowlist: example
# code, model tables and study scripts are source inputs too.
GENERATED_EXAMPLE_FILES = {
    "examples/nt33a-lateral-modes/lateral-modes.md",
    "examples/nt33a-longitudinal-modes/longitudinal-modes.md",
    "examples/nt33a-trim-and-linearise/trim-and-modes.md",
    "examples/nt33a-bank-loop-margins/bank-loop-margins.md",
    "examples/nt33a-lateral-mimo/lateral-mimo.md",
    "examples/nt33a-control-design/control-design.md",
    "examples/nt33a-control-design/control-design.html",
    "examples/nt33a-control-design/linear-response.csv",
    "examples/nt33a-control-design/nonlinear-response.csv",
}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def digest(path):
    result = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def source_file(path, tracked=True):
    if any(part in EXCLUDED_PARTS or part.startswith("cmake-build-") for part in path.parts):
        return False
    if path.parts[0] == "examples":
        if path.as_posix() in GENERATED_EXAMPLE_FILES:
            return False
        if not tracked and (path.name.endswith(".md.out")
                            or (path.name.startswith("run-") and path.suffix == ".json")):
            return False
    return True


def inventory(root, excluded=()):
    """Hash current bytes, including nonignored new files; never follow symlinks."""
    root = Path(root).resolve()
    excluded = tuple(Path(path).resolve() for path in excluded)
    command = ["git", "-C", str(root), "ls-files", "--cached", "-z"]
    try:
        top = subprocess.run(["git", "-C", str(root), "rev-parse", "--show-toplevel"],
                             check=True, capture_output=True, text=True).stdout.strip()
        if Path(top).resolve() != root:
            raise subprocess.CalledProcessError(1, command)
        cached = subprocess.run(command, check=True, capture_output=True).stdout
        others = subprocess.run(["git", "-C", str(root), "ls-files", "--others",
                                 "--exclude-standard", "-z"], check=True, capture_output=True).stdout
        tracked = {os.fsdecode(value) for value in cached.split(b"\0") if value}
        names = sorted(tracked | {os.fsdecode(value) for value in others.split(b"\0") if value})
    except (FileNotFoundError, subprocess.CalledProcessError):
        # Downloaded source archives have no Git directory. Their declared
        # source inventory identifies source files without adding bin/PACKAGE.
        snapshot = root / "SOURCE-SNAPSHOT.json"
        if not snapshot.is_file():
            raise ValueError("source provenance requires Git or SOURCE-SNAPSHOT.json") from None
        names = sorted(item["path"] for item in json.loads(snapshot.read_text(encoding="utf-8"))["files"])
        if len(names) != len(set(names)):
            raise ValueError("duplicate source snapshot paths")
        tracked = set(names)  # Preserve every explicitly declared snapshot input.
    records = []
    for name in names:
        relative = Path(name)
        original = root / relative
        if relative.is_absolute() or ".." in relative.parts or not relative.parts:
            raise ValueError(f"invalid source path: {name}")
        # Reject a symlink in any component, including an internal directory.
        if any((root / Path(*relative.parts[:index])).is_symlink()
               for index in range(1, len(relative.parts) + 1)):
            raise ValueError(f"source inventory refuses symlinks: {name}")
        if not source_file(relative, name in tracked) or any(original.resolve().is_relative_to(path) for path in excluded):
            continue
        if not original.exists():  # An unstaged deletion changes the inventory.
            continue
        if not original.is_file():
            raise ValueError(f"source inventory contains a non-file: {name}")
        records.append({"path": relative.as_posix(), "sha256": digest(original),
                        "executable": bool(original.stat().st_mode & 0o111)})
    if not records:
        raise ValueError("refusing an empty source inventory")
    return records


def inventory_digest(records):
    return hashlib.sha256(canonical(records).encode()).hexdigest()


def read_cache(path):
    cache = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line and not line.startswith(("#", "//")) and "=" in line:
            key, value = line.split("=", 1)
            cache[key.split(":", 1)[0]] = value
    return cache


def write_if_changed(path, text):
    path = Path(path)
    if not path.exists() or path.read_text(encoding="utf-8") != text:
        path.write_text(text, encoding="utf-8")
