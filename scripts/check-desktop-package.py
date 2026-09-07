#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify a local desktop candidate, then run its worker after safe relocation.

Checksums establish content identity, not publisher identity or CI approval.
Metadata-only mode never executes packaged code and does not establish runtime
or Gatekeeper acceptance. The runtime check requires macOS command-line tools.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import plistlib
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import unicodedata
import zipfile

from provenance_support import digest, inventory_digest


APP = "Galata Preview.app"
UI = APP + "/Contents/MacOS/Galata Preview"
WORKER = APP + "/Contents/MacOS/galata"
RESOURCES = APP + "/Contents/Resources"
MAX_FILES = 20000
MAX_BYTES = 2 * 1024 ** 3
SHA256 = re.compile(r"[0-9a-f]{64}")


def relative_name(name):
    if (not isinstance(name, str) or not name or "\\" in name or ":" in name
            or any(ord(character) < 32 for character in name)
            or name.startswith("/") or any(part in {"", ".", ".."} for part in name.split("/"))):
        raise ValueError(f"unsafe package path: {name!r}")
    return PurePosixPath(name)


def path_key(name):
    # macOS commonly extracts onto case-insensitive, Unicode-normalizing disks.
    return unicodedata.normalize("NFD", str(name)).casefold()


def file_inventory(stage):
    records = []
    for path in sorted(stage.rglob("*")):
        name = path.relative_to(stage).as_posix()
        relative_name(name)
        if path.is_symlink():
            raise ValueError(f"package refuses symlinks: {name}")
        if path.is_dir():
            continue
        if not path.is_file():
            raise ValueError(f"package refuses special files: {name}")
        if name != "PACKAGE.json":
            records.append({"path": name, "sha256": digest(path),
                            "executable": bool(path.stat().st_mode & 0o111)})
    validate_records(records)
    return records


def validate_records(records):
    if not isinstance(records, list) or not 1 <= len(records) <= MAX_FILES:
        raise ValueError("missing or oversized file inventory")
    seen = set()
    for record in records:
        if (not isinstance(record, dict) or set(record) != {"path", "sha256", "executable"}
                or type(record["executable"]) is not bool
                or not isinstance(record["sha256"], str)
                or not SHA256.fullmatch(record["sha256"])):
            raise ValueError("invalid file inventory record")
        key = path_key(relative_name(record["path"]))
        if key in seen:
            raise ValueError("duplicate or case-colliding file inventory path")
        seen.add(key)
    if records != sorted(records, key=lambda item: item["path"]):
        raise ValueError("file inventory must have canonical path order")


def extract_zip(archive, destination):
    """Validate the entire directory first; extract regular files without links."""
    if destination.exists():
        raise ValueError("extraction destination must not exist")
    with zipfile.ZipFile(archive) as handle:
        members = handle.infolist()
        if not 1 <= len(members) <= MAX_FILES or sum(item.file_size for item in members) > MAX_BYTES:
            raise ValueError("archive exceeds file or byte limit")
        names, roots = set(), set()
        for item in members:
            path = relative_name(item.filename)
            mode = item.external_attr >> 16
            if (item.is_dir() or stat.S_IFMT(mode) != stat.S_IFREG
                    or stat.S_IMODE(mode) not in {0o644, 0o755}
                    or item.flag_bits & 1):
                raise ValueError("archive may contain only ordinary unencrypted files with safe modes")
            key = path_key(path)
            if key in names or any(parent in names for parent in
                                   (path_key(parent) for parent in path.parents if str(parent) != ".")):
                raise ValueError("duplicate or conflicting archive path")
            if any(existing.startswith(key + "/") for existing in names):
                raise ValueError("conflicting archive file and directory paths")
            if len(path.parts) < 2:
                raise ValueError("archive must contain one installation directory")
            names.add(key)
            roots.add(path.parts[0])
        if len(roots) != 1:
            raise ValueError("archive must contain one installation directory")
        destination.mkdir(parents=True)
        for item in members:
            target = destination / item.filename
            target.parent.mkdir(parents=True, exist_ok=True)
            with handle.open(item) as incoming, target.open("xb") as outgoing:
                shutil.copyfileobj(incoming, outgoing)
            target.chmod(stat.S_IMODE(item.external_attr >> 16))
    return destination / next(iter(roots))


def check_source_archive(stage, metadata, snapshot):
    reference = metadata["source_archive"]
    source = stage / relative_name(reference["file"])
    if digest(source) != reference["sha256"]:
        raise ValueError("source archive hash mismatch")
    records = {record["path"]: record for record in snapshot["files"]}
    seen, roots, total = set(), set(), 0
    with tarfile.open(source, mode="r:gz") as archive:
        for member in archive:
            path = relative_name(member.name)
            if not member.isfile() or member.mode not in {0o644, 0o755} or len(path.parts) < 2:
                raise ValueError("source archive contains an unsafe member")
            roots.add(path.parts[0])
            name = PurePosixPath(*path.parts[1:]).as_posix()
            total += member.size
            if name in seen or len(seen) >= MAX_FILES or total > MAX_BYTES:
                raise ValueError("duplicate or oversized source archive")
            seen.add(name)
            with archive.extractfile(member) as content:
                if name == "SOURCE-SNAPSHOT.json":
                    if member.size > 16 * 1024 ** 2 or json.load(content) != snapshot:
                        raise ValueError("source archive snapshot differs from package snapshot")
                else:
                    record = records.get(name)
                    actual = hashlib.sha256()
                    for chunk in iter(lambda: content.read(1024 * 1024), b""):
                        actual.update(chunk)
                    if (record is None or actual.hexdigest() != record["sha256"]
                            or bool(member.mode & 0o111) != record["executable"]):
                        raise ValueError(f"source archive file differs from inventory: {name}")
    if len(roots) != 1 or seen != set(records) | {"SOURCE-SNAPSHOT.json"}:
        raise ValueError("source archive is missing declared files")


def check_build_stamp(stamp, source_sha256, configuration_sha256, ui_sha256, worker_sha256):
    expected = {"schema": "galata.desktop-build-stamp.v1", "source_tree_sha256": source_sha256,
                "configuration_sha256": configuration_sha256, "ui_sha256": ui_sha256,
                "worker_sha256": worker_sha256}
    if stamp != expected or not all(SHA256.fullmatch(value) for key, value in expected.items()
                                    if key != "schema"):
        raise ValueError("desktop build stamp is stale or differs from source, configuration, UI or worker")


def check_stage(stage):
    package_path = stage / "PACKAGE.json"
    if package_path.is_symlink() or package_path.stat().st_size > 16 * 1024 ** 2:
        raise ValueError("invalid package metadata file")
    metadata = json.loads(package_path.read_text(encoding="utf-8"))
    if (metadata["schema"] != "galata.desktop-package.v1"
            or metadata["channel"] != "local-candidate"
            or metadata["distribution"] != {"signing": "ad-hoc", "notarization": "not-submitted",
                                            "ci_approval": "not-claimed", "stable_release": False}):
        raise ValueError("package must truthfully identify a local, ad-hoc candidate")
    validate_records(metadata["files"])
    if metadata["files"] != file_inventory(stage):
        raise ValueError("package file inventory mismatch: missing, extra, changed file or executable mode")
    if inventory_digest(metadata["files"]) != metadata["files_sha256"]:
        raise ValueError("package inventory digest mismatch")
    records = {item["path"]: item for item in metadata["files"]}
    for name, key in ((UI, "ui_sha256"), (WORKER, "worker_sha256")):
        if not records.get(name, {}).get("executable") or records[name]["sha256"] != metadata[key]:
            raise ValueError("desktop executable identity or mode mismatch")
    if metadata["worker_sha256"] != metadata["built_cli_sha256"]:
        raise ValueError("desktop worker differs from the built CLI")
    plist = plistlib.loads((stage / APP / "Contents/Info.plist").read_bytes())
    if (plist["CFBundleExecutable"] != "Galata Preview"
            or plist["CFBundleIdentifier"] != "org.galata.desktop.preview"
            or plist["CFBundleShortVersionString"] != metadata["version"]
            or plist["CFBundleVersion"] != metadata["version"]
            or plist["LSMinimumSystemVersion"] != metadata["minimum_macos"]
            or not re.fullmatch(r"[0-9]+(?:\.[0-9]+)*", metadata["minimum_macos"])):
        raise ValueError("desktop bundle identification mismatch")
    if metadata["build_type"] not in {"Release", "RelWithDebInfo"}:
        raise ValueError("desktop candidate requires a Release or RelWithDebInfo build")
    if metadata["platform"] not in {"macos-arm64", "macos-x86_64"}:
        raise ValueError("unsupported desktop package platform")
    snapshot = json.loads((stage / "SOURCE-SNAPSHOT.json").read_text(encoding="utf-8"))
    validate_records(snapshot["files"])
    identity = inventory_digest(snapshot["files"])
    built = metadata["build_source"]
    if (identity != snapshot["source_files_sha256"] or identity != built["source_tree_sha256"]
            or snapshot["commit"] != built["commit"] or snapshot["status"] != built["status"]):
        raise ValueError("built source identity differs from packaged source")
    check_source_archive(stage, metadata, snapshot)
    configuration_reference = metadata["build_configuration"]
    configuration_path = stage / relative_name(configuration_reference["file"])
    if digest(configuration_path) != configuration_reference["sha256"]:
        raise ValueError("build configuration digest mismatch")
    stamp = json.loads((stage / "build-evidence/galata-desktop-build.json").read_text(encoding="utf-8"))
    check_build_stamp(stamp, built["source_tree_sha256"], configuration_reference["sha256"],
                      metadata["built_ui_sha256"], metadata["built_cli_sha256"])
    configuration = json.loads(configuration_path.read_text(encoding="utf-8"))
    commands_path = stage / "build-evidence/galata-compile-commands.json"
    if digest(commands_path) != configuration["compile_commands_sha256"]:
        raise ValueError("effective compilation commands digest mismatch")
    commands = json.loads(commands_path.read_text(encoding="utf-8"))
    if (configuration["cache"]["GALATA_BUILD_DESKTOP"] != "ON"
            or configuration["cache"]["CMAKE_BUILD_TYPE"] != metadata["build_type"]
            or not any(item["file"].endswith("/src/desktop/main.mm") for item in commands)):
        raise ValueError("build evidence does not contain the packaged desktop configuration")
    for filename in ("LICENSE", "NOTICE", "THIRD_PARTY_LICENSES.md", "START_HERE.txt"):
        if not (stage / RESOURCES / filename).read_bytes().strip():
            raise ValueError(f"missing desktop distribution notice or help: {filename}")
    notices = stage / RESOURCES / "licenses"
    dependencies = json.loads((notices / "dependencies.json").read_text(encoding="utf-8"))
    if not {"eigen3", "yaml-cpp"}.issubset({item["name"] for item in dependencies}):
        raise ValueError("missing runtime dependency attribution")
    for item in dependencies:
        notice = relative_name(item["notice"])
        if len(notice.parts) != 1 or digest(notices / notice) != item["notice_sha256"]:
            raise ValueError("dependency notice hash mismatch or unsafe path")
    return metadata


def check_loader_output(libraries, load_commands):
    dependencies = []
    for line in libraries.splitlines()[1:]:
        name = line.strip().split(" (compatibility version", 1)[0]
        if not name.startswith(("/System/Library/", "/usr/lib/")) or ".." in PurePosixPath(name).parts:
            raise ValueError(f"non-system runtime dependency is not bundled: {name}")
        dependencies.append(name)
    if not dependencies:
        raise ValueError("desktop executable has no inspected system dependencies")
    lines = load_commands.splitlines()
    for index, line in enumerate(lines):
        if line.strip() == "cmd LC_RPATH":
            path_lines = [value.strip() for value in lines[index + 1:index + 5]
                          if value.strip().startswith("path ")]
            if len(path_lines) != 1:
                raise ValueError("malformed loader search path")
            name = path_lines[0][5:].split(" (offset", 1)[0]
            if not name.startswith(("/System/Library/", "/usr/lib/")):
                raise ValueError(f"non-system runtime search path: {name}")
    return dependencies


def check_minimum_os(load_commands, declared):
    versions = re.findall(r"^\s*minos ([0-9]+(?:\.[0-9]+)*)\s*$", load_commands, re.MULTILINE)
    if not versions:
        versions = re.findall(r"cmd LC_VERSION_MIN_MACOSX\s+cmdsize [0-9]+\s+version "
                              r"([0-9]+(?:\.[0-9]+)*)", load_commands)

    def components(value):
        parts = tuple(map(int, value.split(".")))
        return parts + (0,) * max(0, 3 - len(parts))

    if len(versions) != 1 or components(declared) < components(versions[0]):
        raise ValueError("declared minimum macOS understates executable deployment requirement")


def run(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, capture_output=True,
                          text=True, timeout=120, **kwargs)


def check_runtime(stage, metadata, scratch):
    if sys.platform != "darwin":
        raise ValueError("desktop runtime verification requires macOS; use --metadata-only elsewhere")
    environment = {"PATH": "/usr/bin:/bin:/usr/sbin:/sbin", "HOME": str(scratch),
                   "TMPDIR": str(scratch), "LANG": "en_US.UTF-8"}
    worker = stage / WORKER
    dependencies = {}
    for name in (UI, WORKER):
        binary = stage / name
        load_commands = run(["/usr/bin/otool", "-l", binary]).stdout
        dependencies[name] = check_loader_output(run(["/usr/bin/otool", "-L", binary]).stdout,
                                                 load_commands)
        check_minimum_os(load_commands, metadata["minimum_macos"])
        architecture = run(["/usr/bin/lipo", "-archs", binary]).stdout.strip()
        if architecture != metadata["platform"].removeprefix("macos-"):
            raise ValueError("desktop executable architecture differs from package platform")
        run(["/usr/bin/codesign", "--verify", "--strict", binary])
    run(["/usr/bin/codesign", "--verify", "--deep", "--strict", stage / APP])
    signing = run(["/usr/bin/codesign", "-dv", "--verbose=4", stage / APP]).stderr
    if "Signature=adhoc" not in signing or "TeamIdentifier=not set" not in signing:
        raise ValueError("bundle signature does not match declared ad-hoc distribution")
    version = run([worker, "--version"], cwd=scratch, env=environment).stdout.strip()
    if version != metadata["binary_version"]:
        raise ValueError("relocated worker version differs from package metadata")
    project = scratch / "project with spaces.galata"

    def command(*arguments):
        return json.loads(run([worker, "project", *arguments], cwd=scratch, env=environment).stdout)

    created = command("create", project)
    if command("inspect", project) != created:
        raise ValueError("relocated worker failed project create/open round trip")
    draft = {"schema": "galata.project-draft.v1",
             **{key: created[key] for key in ("model", "presentation", "simulation")}}
    first = draft["model"]["blocks"][0]["id"]
    draft["presentation"]["positions"][first] = {"x": 96, "y": 112}
    draft_path = scratch / "edited draft.json"
    draft_path.write_text(json.dumps(draft), encoding="utf-8")
    saved = command("save", project, draft_path, "--expected-revision", created["revision"])
    if saved["revision"] == created["revision"] or saved != command("inspect", project):
        raise ValueError("relocated worker failed save/reopen")
    result = command("run", project)
    if result["status"] != "completed":
        raise ValueError("relocated worker did not complete the saved project")
    reopened = command("inspect", project)
    retained = next((item for item in reopened["runs"] if item["id"] == result["id"]), None)
    if retained is None or retained["status"] != "completed":
        raise ValueError("relocated worker failed retained run verification")
    manifest_path = Path(result["manifest_path"])
    if not manifest_path.is_absolute():
        manifest_path = project / manifest_path
    if not manifest_path.resolve().is_relative_to(project / "runs" / result["id"]):
        raise ValueError("worker manifest path escapes the owned run")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    built = metadata["build_source"]
    if (manifest_path.name != "run-" + digest(manifest_path) + ".json"
            or manifest["executable"]["sha256"] != metadata["worker_sha256"]
            or manifest["build"]["source_commit"] != built["commit"]
            or manifest["build"]["source_status"] != built["status"]
            or manifest["build"]["source_tree_sha256"] != built["source_tree_sha256"]
            or manifest["build"]["configuration_sha256"] != metadata["build_configuration"]["sha256"]
            or manifest["build"]["dependency_manifest_sha256"] != built["dependency_manifest_sha256"]):
        raise ValueError("worker runtime provenance differs from packaged build/source identity")
    relocated = scratch / "moved project.galata"
    project.rename(relocated)
    moved = command("inspect", relocated)
    if moved["revision"] != saved["revision"] or not any(
            item["id"] == result["id"] and item["status"] == "completed" for item in moved["runs"]):
        raise ValueError("saved project or completed run failed relocation verification")
    return {"worker_smoke": "passed", "bundle_signature": "ad-hoc verified",
            "dependencies": dependencies, "revision": saved["revision"], "run": result["id"]}


def check(archive, metadata_only=False):
    with tempfile.TemporaryDirectory(prefix="galata-desktop-check-") as directory:
        scratch = Path(directory).resolve()
        stage = extract_zip(archive, scratch / "extracted with spaces")
        relocated = scratch / "relocated installation"
        stage.rename(relocated)
        metadata = check_stage(relocated)
        evidence = {"schema": "galata.desktop-package-check.v1", "archive_sha256": digest(archive),
                    "inventory": "passed", "runtime": "not-run"}
        if not metadata_only:
            evidence["runtime"] = check_runtime(relocated, metadata, scratch)
            check_stage(relocated)  # Running the worker must not mutate the app or package.
        return evidence


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--metadata-only", action="store_true", help="verify bytes without executing code")
    args = parser.parse_args()
    try:
        print(json.dumps(check(args.archive.resolve(), args.metadata_only), indent=2, sort_keys=True))
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError,
            zipfile.BadZipFile, tarfile.TarError) as error:
        sys.exit(f"desktop package verification failed: {error}")
