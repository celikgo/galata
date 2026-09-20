#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Create and verify a local macOS DMG from a verified desktop ZIP candidate.

The DMG is a delivery container, not a signing or notarization step. The
source ZIP remains the authoritative review package; this command verifies it,
copies only regular files into a fresh volume, creates a compressed read-only
image, and verifies the image with hdiutil before publishing metadata.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def module_from(filename, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CHECK = module_from("check-desktop-package.py", "desktop_package_checker_for_dmg")
PROVENANCE = module_from("provenance_support.py", "desktop_dmg_provenance")


def digest_bytes(value):
    return hashlib.sha256(value).hexdigest()


def run(command):
    return subprocess.run(list(map(str, command)), check=True, capture_output=True,
                          text=True, timeout=120)


def copy_regular_tree(source, destination):
    if source.is_symlink() or not source.is_dir():
        raise ValueError(f"missing or linked directory: {source}")
    shutil.copytree(source, destination, symlinks=False)


def package(archive, output):
    if sys.platform != "darwin":
        raise ValueError("DMG packaging requires macOS")
    archive = archive.resolve()
    output = output.resolve()
    if archive.is_symlink() or not archive.is_file():
        raise ValueError("desktop ZIP must be a regular file")
    if output == ROOT or output == archive.parent or output.exists() and not output.is_dir():
        raise ValueError("DMG output must be a separate directory")
    if output.exists() and any(output.iterdir()):
        raise ValueError("DMG output directory must be empty")
    output.mkdir(parents=True, exist_ok=True)

    # This executes the same relocated worker and bundle checks as the ZIP
    # release gate before the installer container is created.
    zip_evidence = CHECK.check(archive, metadata_only=False)
    with tempfile.TemporaryDirectory(prefix="galata-desktop-dmg-") as directory:
        scratch = Path(directory).resolve()
        extracted = CHECK.extract_zip(archive, scratch / "zip extraction")
        volume = scratch / "Galata Preview volume"
        volume.mkdir()
        copy_regular_tree(extracted / CHECK.APP, volume / CHECK.APP)
        start_here = extracted / "START_HERE.txt"
        if start_here.is_symlink() or not start_here.is_file():
            raise ValueError("desktop ZIP is missing a regular START_HERE.txt")
        shutil.copy2(start_here, volume / start_here.name)
        inventory = CHECK.file_inventory(volume)
        if any(path.is_symlink() for path in volume.rglob("*")):
            raise ValueError("DMG source volume contains a symlink")

        dmg_name = archive.stem + ".dmg"
        dmg = output / dmg_name
        run(["/usr/bin/hdiutil", "create", "-quiet", "-format", "UDZO",
             "-volname", "Galata Preview", "-srcfolder", volume, dmg])
        run(["/usr/bin/hdiutil", "verify", "-quiet", dmg])
        image_info = run(["/usr/bin/hdiutil", "imageinfo", "-plist", dmg]).stdout

    metadata = {
        "schema": "galata.desktop-dmg.v1",
        "channel": "local-candidate",
        "distribution": {
            "signing": "ad-hoc",
            "notarization": "not-submitted",
            "ci_approval": "not-claimed",
            "stable_release": False,
        },
        "source_zip": {
            "file": archive.name,
            "sha256": PROVENANCE.digest(archive),
        },
        "dmg": {
            "file": dmg.name,
            "sha256": PROVENANCE.digest(dmg),
        },
        "volume": {
            "name": "Galata Preview",
            "files": inventory,
            "files_sha256": PROVENANCE.inventory_digest(inventory),
        },
        "zip_verification": zip_evidence,
        "hdiutil_imageinfo_plist_sha256": digest_bytes(image_info.encode()),
    }
    metadata_path = output / (dmg_name + ".json")
    metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
    return dmg, metadata_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path, help="verified desktop candidate ZIP")
    parser.add_argument("--output-dir", type=Path, required=True,
                        help="new or empty directory for the DMG and metadata")
    args = parser.parse_args()
    try:
        dmg, metadata = package(args.archive, args.output_dir)
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError,
            SystemExit) as error:
        sys.exit(f"desktop DMG packaging failed: {error}")
    print(dmg)
    print(metadata)
