#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Rebuild a configured CLI and package the current source snapshot; never publish."""
import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import zipfile

from ci_evidence import check_evidence
from provenance_support import inventory, inventory_digest


ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def git(root, *arguments):
    return run(["git", "-C", root, *arguments], capture_output=True).stdout


def snapshot(root, destination, excluded=()):
    """Use Git's source inventory, reading working-tree bytes (including new files)."""
    destination.mkdir(parents=True)
    records = inventory(root, excluded)
    for record in records:
        relative = Path(record["path"])
        original = root / relative
        copied = destination / relative
        copied.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original, copied)
        if digest(copied) != record["sha256"] or bool(copied.stat().st_mode & 0o111) != record["executable"]:
            raise ValueError(f"source changed while snapshotting: {relative}")
    try:
        git_root = Path(os.fsdecode(git(root, "rev-parse", "--show-toplevel")).strip()).resolve()
    except (FileNotFoundError, subprocess.CalledProcessError):
        git_root = None
    commit, status_name, status = "unknown", "unknown", []
    if git_root == root.resolve():
        try:
            commit = git(root, "rev-parse", "HEAD").decode().strip()
        except subprocess.CalledProcessError:  # A new repository has no HEAD yet.
            pass
        status = git(root, "status", "--porcelain=v1", "--untracked-files=all").decode().splitlines()
        status_name = "dirty" if status else "clean"
    identity = inventory_digest(records)
    metadata = {"commit": commit, "status": status_name,
                "working_tree_changes": status, "source_files_sha256": identity,
                "files": records,
                "scope": "Current tracked and nonignored untracked working-tree files; "
                         "builds, caches, Git internals and generated example outputs excluded."}
    write_json(destination / "SOURCE-SNAPSHOT.json", metadata)
    return metadata


def archive_tree(stage, destination):
    """Stable order, ownership and timestamps; archive exact staged file bytes."""
    if destination.suffix == ".zip":
        with zipfile.ZipFile(destination, "w", zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(stage.rglob("*")):
                if not path.is_file():
                    continue
                name = (Path(stage.name) / path.relative_to(stage)).as_posix()
                info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
                mode = 0o755 if path.stat().st_mode & 0o111 else 0o644
                info.external_attr = (0o100000 | mode) << 16
                info.compress_type = zipfile.ZIP_DEFLATED
                archive.writestr(info, path.read_bytes())
    else:
        with destination.open("wb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive:
                    for path in sorted(stage.rglob("*")):
                        if not path.is_file():
                            continue
                        name = (Path(stage.name) / path.relative_to(stage)).as_posix()
                        data = path.read_bytes()
                        info = tarfile.TarInfo(name)
                        info.size = len(data)
                        info.mode = 0o755 if path.stat().st_mode & 0o111 else 0o644
                        archive.addfile(info, io.BytesIO(data))


def read_defines(path):
    return {name: json.loads('"' + value + '"') for name, value in
            re.findall(r'^#define\s+(GALATA_\w+)\s+"((?:\\.|[^"\\\n])*)"',
                       path.read_text(encoding="utf-8"), re.MULTILINE)}


def check_archive_configuration(cache):
    if cache.get("BUILD_SHARED_LIBS", "OFF").upper() in {"1", "ON", "YES", "TRUE", "Y"}:
        raise ValueError("release archives require static Galata libraries (BUILD_SHARED_LIBS=OFF); "
                         "shared-library installation remains available through CMake")


def package(build, output, release_sha=None, ci_evidence=None):
    if (release_sha is None) != (ci_evidence is None):
        raise ValueError("release mode requires both --release-sha and --ci-evidence")
    if release_sha is not None:
        check_evidence(ci_evidence, release_sha)
    cache = {}
    for line in (build / "CMakeCache.txt").read_text(encoding="utf-8").splitlines():
        if line and not line.startswith(("#", "//")) and "=" in line:
            key, value = line.split("=", 1)
            cache[key.split(":", 1)[0]] = value
    check_archive_configuration(cache)
    root = Path(cache["CMAKE_HOME_DIRECTORY"]).resolve()
    if root != ROOT:
        raise ValueError("build directory belongs to a different source checkout")
    if output == root or root.is_relative_to(output) or output == build:
        raise ValueError("output must be a separate new or empty directory")
    if output.exists() and any(output.iterdir()):
        raise ValueError("output directory must be empty; existing artifacts are never overwritten")
    # A dirty Git flag cannot establish which working-tree bytes a binary used.
    # Let CMake rebuild all stale compilation inputs before pairing the binary
    # with a snapshot. This also reruns configuration when VERSION/CMake changes.
    run(["cmake", "--build", build, "--config", cache.get("CMAKE_BUILD_TYPE") or "Release"])
    config = read_defines(build / "generated/include/galata/build_config.hpp")
    version = (root / "VERSION").read_text(encoding="utf-8").strip()
    if version != config["GALATA_VERSION_STRING"] or not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
        raise ValueError("configured and source versions disagree or have an invalid format")
    system = config["GALATA_BUILD_SYSTEM"]
    systems = {"Darwin": "macos", "Linux": "linux", "Windows": "windows"}
    processor = config["GALATA_BUILD_PROCESSOR"].lower()
    processor = {"amd64": "x86_64", "x64": "x86_64", "aarch64": "arm64"}.get(processor, processor)
    if system not in systems or processor not in {"arm64", "x86_64"}:
        raise ValueError(f"unsupported archive platform {system}/{processor}")
    platform_name = systems[system] + "-" + processor
    suffix = ".exe" if system == "Windows" else ""
    binary = build / "src/cli" / ("galata" + suffix)
    binary_version = run([binary, "--version"], capture_output=True, text=True).stdout.strip()
    expected_version = (f"galata {version} ({config['GALATA_BUILD_COMPILER_ID']} "
                        f"{config['GALATA_BUILD_COMPILER_VERSION']}, {config['GALATA_BUILD_TYPE']}, "
                        f"{system}/{config['GALATA_BUILD_PROCESSOR']})")
    if binary_version != expected_version:
        raise ValueError("built CLI identification differs from the configured/source metadata")
    provenance_path = build / "src/pipeline/galata_pipeline_provenance.hpp"
    if not provenance_path.exists():
        raise ValueError(f"missing built provenance header: {provenance_path}")
    provenance = read_defines(provenance_path)
    if provenance["GALATA_DEPENDENCY_MANIFEST_SHA256"] != digest(root / "vcpkg.json"):
        raise ValueError("dependency manifest changed since the CLI was built")
    configuration_path = build / "galata-build-configuration.json"
    configuration = json.loads(configuration_path.read_text(encoding="utf-8"))
    if digest(configuration_path) != provenance["GALATA_BUILD_CONFIGURATION_SHA256"]:
        raise ValueError("build configuration differs from the compiled provenance")
    commands_path = build / "galata-compile-commands.json"
    if digest(commands_path) != configuration["compile_commands_sha256"]:
        raise ValueError("effective compilation commands differ from the built configuration")
    if release_sha is not None and (provenance["GALATA_SOURCE_COMMIT"] != release_sha
                                    or provenance["GALATA_SOURCE_STATUS"] != "clean"):
        raise ValueError("release binary must come from the clean CI-verified commit")
    output.parent.mkdir(parents=True, exist_ok=True)
    # Staging inside the checkout would itself make a clean release source dirty.
    with tempfile.TemporaryDirectory(prefix="galata-package-") as directory:
        scratch = Path(directory).resolve()
        assets = scratch / "assets"
        assets.mkdir()
        stem = f"galata-v{version}"
        source = scratch / f"{stem}-source-{platform_name}"
        source_metadata = snapshot(root, source, excluded=(output, build, scratch))
        if source_metadata["source_files_sha256"] != provenance["GALATA_SOURCE_TREE_SHA256"]:
            raise ValueError("source contents changed between the build and the package snapshot")
        if release_sha is not None and (source_metadata["commit"] != release_sha
                                        or source_metadata["status"] != "clean"):
            raise ValueError("release source snapshot must be the clean CI-verified commit")
        source_archive = assets / (source.name + ".tar.gz")
        archive_tree(source, source_archive)
        stage = scratch / f"{stem}-{platform_name}"
        # Documentation links are part of the downloaded product: retain the
        # complete snapshot so links to APIs, tests, scripts and governance work.
        shutil.copytree(source, stage)
        (stage / "build-evidence").mkdir()
        shutil.copy2(configuration_path, stage / "build-evidence" / configuration_path.name)
        shutil.copy2(commands_path, stage / "build-evidence" / commands_path.name)
        (stage / "bin").mkdir(exist_ok=True)
        shutil.copy2(binary, stage / "bin" / binary.name)
        if suffix:
            installed = Path(cache["VCPKG_INSTALLED_DIR"]) / cache["VCPKG_TARGET_TRIPLET"]
            dependency_bin = installed / ("debug/bin" if config["GALATA_BUILD_TYPE"] == "Debug" else "bin")
            dlls = {path.name: path for path in dependency_bin.glob("*.dll")}
            dlls.update({path.name: path for path in binary.parent.glob("*.dll")})
            for path in dlls.values():
                shutil.copy2(path, stage / "bin" / path.name)
        # Keep source-reference notices byte-identical to SOURCE-SNAPSHOT.json.
        # Build-resolved notices live separately and are the binary's inventory.
        run([sys.executable, root / "scripts/collect-dependency-notices.py", build,
             stage / "third_party/build-licenses"])
        metadata = {"schema_version": 1, "version": version, "platform": platform_name,
                    "binary_version": binary_version,
                    "compiler": {"id": config["GALATA_BUILD_COMPILER_ID"],
                                 "version": config["GALATA_BUILD_COMPILER_VERSION"]},
                    "build_type": config["GALATA_BUILD_TYPE"],
                    "build_source": {"commit": provenance["GALATA_SOURCE_COMMIT"],
                                     "status": provenance["GALATA_SOURCE_STATUS"],
                                     "source_tree_sha256": provenance["GALATA_SOURCE_TREE_SHA256"],
                                     "dependency_manifest_sha256": provenance["GALATA_DEPENDENCY_MANIFEST_SHA256"]},
                    "build_configuration": {"file": "build-evidence/galata-build-configuration.json",
                                            "sha256": provenance["GALATA_BUILD_CONFIGURATION_SHA256"]},
                    "packaged_source": {key: source_metadata[key] for key in
                                        ("commit", "status", "source_files_sha256")},
                    "source_archive": {"file": source_archive.name, "sha256": digest(source_archive)},
                    "dependency_inventory": "third_party/build-licenses/dependencies.json",
                    "runtime_files": [{"path": path.relative_to(stage).as_posix(), "sha256": digest(path)}
                                      for path in sorted((stage / "bin").iterdir())],
                    "build_verification": "CMake build completed successfully immediately before snapshot/staging; "
                                          "no source edits are permitted during packaging.",
                    "validation_scope": "Packaging rebuilds stale inputs and runs the shipped continuous-model, trim/linearization "
                                        "and control-design studies. It does not run the full test suite or "
                                        "qualify the tool. The source archive records exact working-tree bytes; "
                                        "this is not an independent reproducible-build attestation."}
        if ci_evidence is not None:
            metadata["ci_evidence"] = ci_evidence
        write_json(stage / "PACKAGE.json", metadata)
        (stage / "RUNNING.txt").write_text(
            f"galata {version} - {platform_name} ({metadata['build_type']})\n\n"
            f"bin/galata{suffix} --version\n"
            f"bin/galata{suffix} capabilities\n"
            f"bin/galata{suffix} run examples/nt33a-control-design/study.yaml --output-dir results\n"
            f"bin/galata{suffix} run examples/continuous-feedback/study.yaml --output-dir model-results\n\n"
            "Run from this directory; keep examples and models together. Existing reports require --overwrite.\n"
            "PACKAGE.json records this binary, its source snapshot and compiler.\n"
            "SOURCE-SNAPSHOT.json identifies the complete source tree included alongside bin.\n"
            "third_party/build-licenses/dependencies.json records actual dependency versions and notice hashes.\n"
            "See docs/VERIFICATION.md for the checked and unvalidated capability boundaries.\n"
            "This is an engineering workbench, not a qualified certification tool or flight computer.\n"
            "Apache-2.0; see LICENSE, NOTICE and THIRD_PARTY_LICENSES.md.\n", encoding="utf-8")
        archive = assets / (stage.name + (".zip" if suffix else ".tar.gz"))
        archive_tree(stage, archive)
        run([sys.executable, root / "scripts/check-release-archive.py", archive])
        metadata["cli_archive"] = {"file": archive.name, "sha256": digest(archive)}
        metadata["archive_smoke"] = "passed"
        write_json(assets / (stage.name + ".package.json"), metadata)
        for path in (archive, source_archive):
            (assets / (path.name + ".sha256")).write_text(
                f"{digest(path)}  {path.name}\n", encoding="utf-8")
        if not output.exists():
            shutil.move(assets, output)
        else:
            for path in assets.iterdir():
                shutil.move(path, output / path.name)
    for path in sorted(output.iterdir()):
        print(path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path, help="configured vcpkg CMake directory (rebuilt before packaging)")
    parser.add_argument("--output-dir", type=Path, required=True, help="new or empty local artifact directory")
    parser.add_argument("--release-sha", help="require a clean source snapshot at this CI-verified commit")
    parser.add_argument("--ci-evidence", type=Path, help="complete CI evidence from this workflow run")
    args = parser.parse_args()
    try:
        evidence = json.loads(args.ci_evidence.read_text(encoding="utf-8")) if args.ci_evidence else None
        package(args.build.resolve(), args.output_dir.resolve(), args.release_sha, evidence)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        sys.exit(f"release packaging failed: {error}")
