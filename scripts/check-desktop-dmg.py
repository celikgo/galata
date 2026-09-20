#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify a local Galata macOS DMG container and its mounted file inventory."""
import argparse
import importlib.util
import json
from pathlib import Path
import plistlib
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def module_from(filename, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CHECK = module_from("check-desktop-package.py", "desktop_package_checker_for_dmg_check")
PROVENANCE = module_from("provenance_support.py", "desktop_dmg_provenance_check")


def run(command):
    return subprocess.run(list(map(str, command)), check=True, capture_output=True,
                          text=True, timeout=120)


def check(dmg, metadata_path):
    if sys.platform != "darwin":
        raise ValueError("DMG verification requires macOS")
    dmg = dmg.resolve()
    metadata_path = metadata_path.resolve()
    if dmg.is_symlink() or not dmg.is_file() or metadata_path.is_symlink():
        raise ValueError("DMG and metadata must be regular files")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if (metadata.get("schema") != "galata.desktop-dmg.v1"
            or metadata.get("channel") != "local-candidate"
            or metadata.get("distribution") != {
                "signing": "ad-hoc",
                "notarization": "not-submitted",
                "ci_approval": "not-claimed",
                "stable_release": False,
            }):
        raise ValueError("metadata does not identify a local ad-hoc candidate")
    if metadata["dmg"]["file"] != dmg.name:
        raise ValueError("metadata does not identify the supplied DMG")
    if metadata["dmg"]["sha256"] != PROVENANCE.digest(dmg):
        raise ValueError("DMG digest does not match metadata")
    run(["/usr/bin/hdiutil", "verify", "-quiet", dmg])

    with tempfile.TemporaryDirectory(prefix="galata-desktop-dmg-check-") as directory:
        mount = Path(directory) / "mounted volume"
        mount.mkdir()
        attached = run(["/usr/bin/hdiutil", "attach", "-plist", "-readonly", "-nobrowse",
                        "-mountpoint", mount, dmg])
        try:
            attachment = plistlib.loads(attached.stdout.encode("utf-8"))
            entities = attachment.get("system-entities", [])
            expected_mount = mount.resolve()
            if not any(entity.get("mount-point")
                       and Path(entity["mount-point"]).resolve() == expected_mount
                       for entity in entities):
                raise ValueError("hdiutil did not report the requested mount point")
            records = CHECK.file_inventory(mount)
            if records != metadata["volume"]["files"]:
                raise ValueError("mounted DMG inventory differs from metadata")
            if CHECK.inventory_digest(records) != metadata["volume"]["files_sha256"]:
                raise ValueError("mounted DMG inventory digest differs from metadata")
            app = mount / CHECK.APP
            run(["/usr/bin/codesign", "--verify", "--deep", "--strict", app])
        finally:
            run(["/usr/bin/hdiutil", "detach", "-quiet", mount])

    return {
        "schema": "galata.desktop-dmg-check.v1",
        "status": "verified",
        "dmg_sha256": metadata["dmg"]["sha256"],
        "volume_inventory": "passed",
        "bundle_signature": "ad-hoc verified",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dmg", type=Path)
    parser.add_argument("metadata", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(check(args.dmg, args.metadata), indent=2, sort_keys=True))
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError,
            plistlib.InvalidFileException) as error:
        sys.exit(f"desktop DMG verification failed: {error}")
