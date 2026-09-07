#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Package an already-built macOS desktop as a local candidate; never publish.

Build and test the frozen source first. This command deliberately does not
rebuild after those checks. It refuses stale source/configuration provenance,
stages outside the checkout, seals the copied bundle with an ad-hoc signature,
and verifies the extracted archive before copying any output into place.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import plistlib
import re
import shutil
import subprocess
import sys
import tempfile

from provenance_support import digest, inventory, inventory_digest, read_cache


ROOT = Path(__file__).resolve().parents[1]


def script_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RELEASE = script_module("desktop_release_helpers", "package-release.py")
CHECK = script_module("desktop_package_checker", "check-desktop-package.py")


def run(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, capture_output=True,
                          text=True, timeout=120, **kwargs)


def copy_file(source, destination):
    if source.is_symlink() or not source.is_file():
        raise ValueError(f"missing or linked package input: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    if digest(source) != digest(destination):
        raise ValueError(f"package input changed while copying: {source}")


def stamp_build(build, app, destination):
    """Called by the desktop POST_BUILD only after the worker copy completes."""
    cache = read_cache(build / "CMakeCache.txt")
    if Path(cache["CMAKE_HOME_DIRECTORY"]).resolve() != ROOT:
        raise ValueError("desktop stamp build belongs to another source checkout")
    expected_app = build / "src/desktop" / CHECK.APP
    if app != expected_app or destination != build / "src/desktop/galata-desktop-build.json":
        raise ValueError("desktop build stamp requires the configured app and adjacent stamp path")
    provenance = RELEASE.read_defines(build / "src/pipeline/galata_pipeline_provenance.hpp")
    source_sha256 = inventory_digest(inventory(ROOT, excluded=(build,)))
    configuration_sha256 = digest(build / "galata-build-configuration.json")
    if (source_sha256 != provenance["GALATA_SOURCE_TREE_SHA256"]
            or configuration_sha256 != provenance["GALATA_BUILD_CONFIGURATION_SHA256"]):
        raise ValueError("source or configuration changed during desktop build; rebuild the frozen source")
    CHECK.file_inventory(app)
    worker_sha256 = digest(app / "Contents/MacOS/galata")
    if worker_sha256 != digest(build / "src/cli/galata"):
        raise ValueError("cannot stamp a desktop containing a different worker")
    stamp = {"schema": "galata.desktop-build-stamp.v1", "source_tree_sha256": source_sha256,
             "configuration_sha256": configuration_sha256,
             "ui_sha256": digest(app / "Contents/MacOS/Galata Preview"), "worker_sha256": worker_sha256}
    RELEASE.write_json(destination, stamp)
    print(f"Stamped desktop build: {destination}")


def check_build(build, output):
    cache = read_cache(build / "CMakeCache.txt")
    RELEASE.check_archive_configuration(cache)
    if Path(cache["CMAKE_HOME_DIRECTORY"]).resolve() != ROOT:
        raise ValueError("build directory belongs to a different source checkout")
    if (cache.get("GALATA_BUILD_DESKTOP") != "ON"
            or cache.get("CMAKE_BUILD_TYPE") not in {"Release", "RelWithDebInfo"}):
        raise ValueError("configure the desktop with Release or RelWithDebInfo before packaging")
    if any(cache.get(key, "OFF").upper() not in {"OFF", "0", "NO", "FALSE", "N", ""}
           for key in ("GALATA_ENABLE_ASAN", "GALATA_ENABLE_UBSAN")):
        raise ValueError("instrumented builds are test artifacts, not desktop candidates")
    if output == ROOT or ROOT.is_relative_to(output) or output == build or build.is_relative_to(output):
        raise ValueError("output must be a separate new or empty artifact directory")
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError("output directory must be empty; existing artifacts are never overwritten")
    config = RELEASE.read_defines(build / "generated/include/galata/build_config.hpp")
    provenance = RELEASE.read_defines(build / "src/pipeline/galata_pipeline_provenance.hpp")
    version = (ROOT / "VERSION").read_text(encoding="utf-8").strip()
    if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version) or config["GALATA_VERSION_STRING"] != version:
        raise ValueError("source and configured desktop versions differ")
    processor = config["GALATA_BUILD_PROCESSOR"].lower()
    processor = {"aarch64": "arm64", "amd64": "x86_64"}.get(processor, processor)
    if (config["GALATA_BUILD_SYSTEM"] != "Darwin" or processor not in {"arm64", "x86_64"}
            or config["GALATA_BUILD_TYPE"] != cache["CMAKE_BUILD_TYPE"]):
        raise ValueError("desktop packaging requires a matching single-architecture macOS build")
    if provenance["GALATA_SOURCE_TREE_SHA256"] != inventory_digest(inventory(ROOT, excluded=(build, output))):
        raise ValueError("source changed since the worker was built; rebuild and rerun checks before packaging")
    if provenance["GALATA_DEPENDENCY_MANIFEST_SHA256"] != digest(ROOT / "vcpkg.json"):
        raise ValueError("dependency manifest changed after the build")
    configuration_path = build / "galata-build-configuration.json"
    configuration = json.loads(configuration_path.read_text(encoding="utf-8"))
    if digest(configuration_path) != provenance["GALATA_BUILD_CONFIGURATION_SHA256"]:
        raise ValueError("build configuration differs from compiled provenance")
    if digest(build / "galata-compile-commands.json") != configuration["compile_commands_sha256"]:
        raise ValueError("effective compilation commands differ from built configuration")
    cli = build / "src/cli/galata"
    app = build / "src/desktop" / CHECK.APP
    CHECK.file_inventory(app)
    if digest(app / "Contents/MacOS/galata") != digest(cli):
        raise ValueError("desktop bundle contains a stale worker; rebuild the desktop target")
    stamp = json.loads((build / "src/desktop/galata-desktop-build.json").read_text(encoding="utf-8"))
    CHECK.check_build_stamp(stamp, provenance["GALATA_SOURCE_TREE_SHA256"],
                            provenance["GALATA_BUILD_CONFIGURATION_SHA256"],
                            digest(app / "Contents/MacOS/Galata Preview"), digest(cli))
    binary_version = run([cli, "--version"]).stdout.strip()
    expected = (f"galata {version} ({config['GALATA_BUILD_COMPILER_ID']} "
                f"{config['GALATA_BUILD_COMPILER_VERSION']}, {config['GALATA_BUILD_TYPE']}, "
                f"Darwin/{config['GALATA_BUILD_PROCESSOR']})")
    if binary_version != expected:
        raise ValueError("CLI identity differs from configured build metadata")
    return cache, config, provenance, binary_version, "macos-" + processor


def start_here(version, platform_name, minimum):
    return f"""Galata Preview {version} — local desktop candidate ({platform_name})

Requires macOS {minimum} or later on the packaged architecture. This is the
build's deployment requirement, not a completed supported-OS test matrix.

Extract the ZIP, then double-click Galata Preview.app. You may copy the app
to Applications or another folder. Its numerical worker and notices are inside
the app; no Python, compiler, package manager or network service is needed to
create, open, edit, save and run a project.

This local candidate has an ad-hoc integrity signature, no Developer ID, and
no notarization. Gatekeeper may refuse a downloaded copy. Do not disable system
security to install it. Public distribution signing remains a release task.

In the app, use New Project or Open Project. Save keeps a retained revision;
Run Saved executes that saved revision, and the run list checks retained
artifacts when reopened. Keep the entire .galata project directory together.
The app does not upgrade a project merely by opening it. Back up projects
before trying preview builds. Presentation v2 routes need a compatible worker.

Diagram controls: Fit All and Focus Diagram adjust the view. Drag block bodies
to place them. Snap On uses a 16-point grid and edge/center alignment; hold
Option or turn Snap Off for free movement. Click a line to select it. Round
middle handles reshape segments; square endpoint handles reconnect ports.
Reset Route restores automatic routing. Edit gestures support Undo/Redo.

The examples folder contains scalar feedback and NT-33A graph-study inputs.
Keep examples and models together. Example values illustrate workflows and
do not establish engineering acceptance of a design.

For a terminal workflow, the same engine is at:
  "Galata Preview.app/Contents/MacOS/galata" --help
  "Galata Preview.app/Contents/MacOS/galata" project create ~/example.galata
  "Galata Preview.app/Contents/MacOS/galata" project run ~/example.galata

PACKAGE.json inventories all delivered bytes and executable modes except its
own bytes. The ZIP's sibling SHA256 file identifies the whole archive. The
source archive and build-evidence directory retain exact source and effective
build inputs. These checksums identify content; they are not signed authorship,
CI approval, a reproducible-build proof, or a qualification claim.

This is a local candidate, not a stable public release. Native workflow checks,
release test evidence, signed/notarized distribution, clean-machine acceptance
and accessibility review determine readiness separately.

Apache-2.0. See the app's Contents/Resources directory for LICENSE, NOTICE,
THIRD_PARTY_LICENSES.md and build-resolved dependency notices under licenses.
"""


def package(build, output):
    if sys.platform != "darwin":
        raise ValueError("desktop candidate packaging requires macOS")
    _, config, provenance, binary_version, platform_name = check_build(build, output)
    version = config["GALATA_VERSION_STRING"]
    app = build / "src/desktop" / CHECK.APP
    plist = plistlib.loads((app / "Contents/Info.plist").read_bytes())
    minimum = plist["LSMinimumSystemVersion"]
    with tempfile.TemporaryDirectory(prefix="galata-desktop-package-") as directory:
        scratch = Path(directory).resolve()
        assets = scratch / "assets"
        assets.mkdir()
        name = f"galata-desktop-v{version}-candidate-{platform_name}"
        stage = scratch / name
        stage.mkdir()
        source = scratch / f"galata-v{version}-source"
        snapshot = RELEASE.snapshot(ROOT, source, excluded=(build, output, scratch))
        if (snapshot["source_files_sha256"] != provenance["GALATA_SOURCE_TREE_SHA256"]
                or snapshot["commit"] != provenance["GALATA_SOURCE_COMMIT"]
                or snapshot["status"] != provenance["GALATA_SOURCE_STATUS"]):
            raise ValueError("source snapshot differs from built identity")
        source_archive = stage / "source" / (source.name + ".tar.gz")
        source_archive.parent.mkdir()
        RELEASE.archive_tree(source, source_archive)
        copy_file(source / "SOURCE-SNAPSHOT.json", stage / "SOURCE-SNAPSHOT.json")
        shutil.copytree(app, stage / CHECK.APP, symlinks=True)
        resources = stage / CHECK.RESOURCES
        resources.mkdir(exist_ok=True)
        for filename in ("LICENSE", "NOTICE", "THIRD_PARTY_LICENSES.md"):
            copy_file(source / filename, resources / filename)
        notices = resources / "licenses"
        # A build tree may already contain installed notices; replace only this
        # private staged copy with the actual resolved dependency inventory.
        if notices.exists():
            shutil.rmtree(notices)
        run([sys.executable, ROOT / "scripts/collect-dependency-notices.py", build, notices])
        for filename in ("galata-build-configuration.json", "galata-compile-commands.json"):
            copy_file(build / filename, stage / "build-evidence" / filename)
        copy_file(build / "src/desktop/galata-desktop-build.json",
                  stage / "build-evidence/galata-desktop-build.json")
        for filename in ("examples/continuous-feedback/model.yaml", "examples/continuous-feedback/study.yaml",
                         "examples/nt33a-graph-design/study.yaml", "models/nt33a/nt33a-fc1.yaml",
                         "models/nt33a/PROVENANCE.md"):
            copy_file(source / filename, stage / filename)
        help_text = start_here(version, platform_name, minimum)
        (stage / "START_HERE.txt").write_text(help_text, encoding="utf-8")
        (resources / "START_HERE.txt").write_text(help_text, encoding="utf-8")
        build_source = {"commit": provenance["GALATA_SOURCE_COMMIT"],
                        "status": provenance["GALATA_SOURCE_STATUS"],
                        "source_tree_sha256": provenance["GALATA_SOURCE_TREE_SHA256"],
                        "dependency_manifest_sha256": provenance["GALATA_DEPENDENCY_MANIFEST_SHA256"]}
        RELEASE.write_json(resources / "BUILD_INFO.json", {
            "schema": "galata.desktop-build.v1", "channel": "local-candidate",
            "version": version, "platform": platform_name, "minimum_macos": minimum,
            "source": build_source,
            "configuration_sha256": provenance["GALATA_BUILD_CONFIGURATION_SHA256"],
            "worker_sha256": digest(build / "src/cli/galata")})
        CHECK.file_inventory(stage)  # Refuse links/special files before sealing or archiving.
        run(["/usr/bin/codesign", "--force", "--sign", "-", "--timestamp=none", stage / CHECK.APP])
        metadata = {
            "schema": "galata.desktop-package.v1", "channel": "local-candidate",
            "version": version, "platform": platform_name, "minimum_macos": minimum,
            "build_type": config["GALATA_BUILD_TYPE"], "binary_version": binary_version,
            "distribution": {"signing": "ad-hoc", "notarization": "not-submitted",
                             "ci_approval": "not-claimed", "stable_release": False},
            "ui_sha256": digest(stage / CHECK.UI),
            "built_ui_sha256": digest(app / "Contents/MacOS/Galata Preview"),
            "worker_sha256": digest(stage / CHECK.WORKER),
            "built_cli_sha256": digest(build / "src/cli/galata"),
            "build_source": build_source,
            "build_configuration": {"file": "build-evidence/galata-build-configuration.json",
                                    "sha256": provenance["GALATA_BUILD_CONFIGURATION_SHA256"]},
            "source_archive": {"file": source_archive.relative_to(stage).as_posix(),
                               "sha256": digest(source_archive)},
            "build_verification": "Already-built inputs; no rebuild during packaging. Current source inventory "
                                  "must match compiled worker provenance and the desktop POST_BUILD stamp. "
                                  "UI bytes are inventoried before and after ad-hoc bundle sealing.",
            "validation_scope": "Safe extraction, complete file/source inventories, bundle seal, system-only "
                                "loader paths and relocated worker create/open/save/run/inspect. Native UI "
                                "acceptance and the complete release gate are separate evidence.",
            "files": CHECK.file_inventory(stage)}
        metadata["files_sha256"] = inventory_digest(metadata["files"])
        RELEASE.write_json(stage / "PACKAGE.json", metadata)
        CHECK.check_stage(stage)
        archive = assets / (name + ".zip")
        RELEASE.archive_tree(stage, archive)
        verification = CHECK.check(archive)
        if inventory_digest(inventory(ROOT, excluded=(build, output, scratch))) != snapshot["source_files_sha256"]:
            raise ValueError("source changed during packaging; discard candidate and repeat build/checks")
        if (digest(build / "src/cli/galata") != metadata["built_cli_sha256"]
                or digest(app / "Contents/MacOS/Galata Preview") != metadata["built_ui_sha256"]):
            raise ValueError("build executables changed during packaging")
        RELEASE.write_json(assets / (name + ".package.json"), {
            **metadata, "archive": {"file": archive.name, "sha256": digest(archive)},
            "archive_verification": verification})
        (assets / (archive.name + ".sha256")).write_text(
            f"{digest(archive)}  {archive.name}\n", encoding="utf-8")
        if output.exists() and any(output.iterdir()):
            raise ValueError("output directory changed during packaging; refusing to overwrite it")
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.exists():
            for path in assets.iterdir():
                shutil.move(path, output / path.name)
        else:
            shutil.move(assets, output)
    for path in sorted(output.iterdir()):
        print(path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("build", type=Path, help="configured and built Release/RelWithDebInfo directory")
    parser.add_argument("--output-dir", type=Path, help="new or empty artifact directory")
    parser.add_argument("--stamp-app", type=Path, help="internal CMake POST_BUILD bundle to stamp")
    parser.add_argument("--stamp-output", type=Path, help="internal CMake POST_BUILD stamp path")
    args = parser.parse_args()
    try:
        if args.stamp_app is not None and args.stamp_output is not None and args.output_dir is None:
            stamp_build(args.build.resolve(), args.stamp_app.resolve(), args.stamp_output.resolve())
        elif args.output_dir is not None and args.stamp_app is None and args.stamp_output is None:
            package(args.build.resolve(), args.output_dir.resolve())
        else:
            parser.error("provide --output-dir or both internal --stamp-app/--stamp-output options")
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        sys.exit(f"desktop packaging failed: {error}")
