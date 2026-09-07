#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run the downloadable CLI after extraction, including its models and notices."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import tempfile
import zipfile
from urllib.parse import unquote, urlsplit

from ci_evidence import check_evidence


def check_document_links(stage):
    checked = 0
    for document in sorted(stage.rglob("*.md")):
        for target in re.findall(r'\]\(([^)\s]+)(?:\s+"[^"]*")?\)',
                                 document.read_text(encoding="utf-8")):
            url = urlsplit(target.strip("<>"))
            if url.scheme or url.netloc or not url.path:
                continue
            resolved = (document.parent / unquote(url.path)).resolve()
            if not resolved.is_relative_to(stage) or not resolved.exists():
                raise ValueError(f"broken extracted documentation link: {document.relative_to(stage)} -> {target}")
            checked += 1
    if not checked:
        raise ValueError("archive has no local documentation links to check")
    return checked


def check_source_snapshot(stage, metadata):
    snapshot = json.loads((stage / "SOURCE-SNAPSHOT.json").read_text(encoding="utf-8"))
    records = snapshot["files"]
    identity = hashlib.sha256(json.dumps(records, sort_keys=True,
                                         separators=(",", ":")).encode()).hexdigest()
    if (not records or identity != snapshot["source_files_sha256"]
            or identity != metadata["packaged_source"]["source_files_sha256"]):
        raise ValueError("packaged source snapshot identity does not match its inventory")
    names = set()
    for item in records:
        source = (stage / item["path"]).resolve()
        if (item["path"] in names or not source.is_relative_to(stage)
                or not source.is_file() or digest(source) != item["sha256"]):
            raise ValueError(f"source snapshot file missing, duplicate or changed: {item['path']}")
        names.add(item["path"])


def check(archive):
    with tempfile.TemporaryDirectory(prefix="galata-archive-") as directory:
        scratch = Path(directory).resolve()

        def safe(name):
            if not (scratch / name).resolve().is_relative_to(scratch):
                raise ValueError(f"archive member escapes extraction directory: {name}")

        if zipfile.is_zipfile(archive):
            with zipfile.ZipFile(archive) as handle:
                for member in handle.infolist():
                    safe(member.filename)
                    if (member.external_attr >> 16) & 0o170000 == 0o120000:
                        raise ValueError("archive may not contain symlinks")
                handle.extractall(scratch)
        else:
            with tarfile.open(archive) as handle:
                for member in handle.getmembers():
                    safe(member.name)
                    if not (member.isfile() or member.isdir()):
                        raise ValueError("archive may contain only files and directories")
                handle.extractall(scratch, filter="data")
        roots = list(scratch.iterdir())
        if len(roots) != 1 or not roots[0].is_dir():
            raise ValueError("archive must contain exactly one installation directory")
        stage = roots[0]
        metadata = json.loads((stage / "PACKAGE.json").read_text(encoding="utf-8"))
        check_source_snapshot(stage, metadata)
        built = metadata["build_source"]
        if built["source_tree_sha256"] != metadata["packaged_source"]["source_files_sha256"]:
            raise ValueError("built source contents differ from the packaged snapshot")
        if "ci_evidence" in metadata:
            check_evidence(metadata["ci_evidence"], built["commit"])
            if built["status"] != "clean" or metadata["packaged_source"]["status"] != "clean":
                raise ValueError("release evidence cannot endorse dirty source")
        configuration_path = (stage / metadata["build_configuration"]["file"]).resolve()
        if (not configuration_path.is_relative_to(stage)
                or digest(configuration_path) != metadata["build_configuration"]["sha256"]):
            raise ValueError("build configuration hash or path mismatch")
        configuration = json.loads(configuration_path.read_text(encoding="utf-8"))
        if digest(stage / "build-evidence/galata-compile-commands.json") != configuration["compile_commands_sha256"]:
            raise ValueError("effective compilation command digest mismatch")
        link_count = check_document_links(stage)
        inventory_path = (stage / metadata["dependency_inventory"]).resolve()
        if not inventory_path.is_relative_to(stage):
            raise ValueError("dependency inventory escapes archive")
        notices = inventory_path.parent
        inventory = json.loads(inventory_path.read_text(encoding="utf-8"))
        if not {"eigen3", "yaml-cpp"}.issubset({item["name"] for item in inventory}):
            raise ValueError("archive lacks runtime dependency attribution")
        for item in inventory:
            notice_digest = hashlib.sha256((notices / item["notice"]).read_bytes()).hexdigest()
            if notice_digest != item["notice_sha256"]:
                raise ValueError(f"notice hash mismatch for {item['name']}")
        for filename in ("LICENSE", "NOTICE", "THIRD_PARTY_LICENSES.md"):
            if not (stage / filename).read_bytes().strip():
                raise ValueError(f"missing distribution notice: {filename}")
        for item in metadata["runtime_files"]:
            runtime = (stage / item["path"]).resolve()
            if not runtime.is_relative_to(stage / "bin") or digest(runtime) != item["sha256"]:
                raise ValueError("runtime hash mismatch or invalid binary path")
        binary = stage / "bin" / ("galata.exe" if os.name == "nt" else "galata")
        subprocess.run([str(binary), "--version"], cwd=stage, check=True)
        studies = {"continuous-feedback": ("response.csv", "evidence.json"),
                   "nt33a-trim-and-linearise": ("trim-and-modes.md",),
                   "nt33a-control-design": ("control-design.md", "linear-response.csv",
                                            "nonlinear-response.csv")}
        for study, reports in studies.items():
            output = scratch / study
            output.mkdir()
            subprocess.run([str(binary), "run", f"examples/{study}/study.yaml",
                            "--output-dir", str(output)], cwd=stage, check=True)
            for report in reports:
                if not (output / report).is_file() or not (output / report).stat().st_size:
                    raise ValueError(f"extracted CLI did not produce {study}/{report}")
            manifests = list(output.glob("run-*.json"))
            if len(manifests) != 1:
                raise ValueError("extracted CLI must produce one content-addressed run manifest")
            manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
            if manifests[0].name != "run-" + digest(manifests[0]) + ".json":
                raise ValueError("run manifest content does not match its identity")
            built = metadata["build_source"]
            if (manifest["executable"]["sha256"] != digest(binary)
                    or manifest["build"]["source_commit"] != built["commit"]
                    or manifest["build"]["source_status"] != built["status"]
                    or manifest["build"]["source_tree_sha256"] != built["source_tree_sha256"]
                    or manifest["build"]["configuration_sha256"] != metadata["build_configuration"]["sha256"]
                    or manifest["build"]["dependency_manifest_sha256"] != built["dependency_manifest_sha256"]):
                raise ValueError("runtime provenance differs from packaged binary/build metadata")
        print(f"Extracted archive smoke, complete source snapshot, {link_count} local links "
              f"and dependency notices passed: {archive}")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: check-release-archive.py <release.tar.gz-or-release.zip>")
    try:
        check(Path(sys.argv[1]).resolve())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        sys.exit(f"archive smoke failed: {error}")
