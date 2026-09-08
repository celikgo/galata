#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Bind all release metadata and archive hashes to the CI-verified source."""
import argparse
import json
from pathlib import Path
import sys

from ci_evidence import check_evidence
from provenance_support import digest


def check(directory, evidence, expected):
    check_evidence(evidence, expected)
    expected_platforms = {"linux-x86_64", "macos-arm64"}
    platforms = set()
    metadata_files = list(directory.glob("*.package.json"))
    if len(metadata_files) != len(expected_platforms):
        raise ValueError("release must contain exactly one metadata record per supported platform")
    for path in metadata_files:
        metadata = json.loads(path.read_text(encoding="utf-8"))
        platform = metadata["platform"]
        if platform not in expected_platforms or platform in platforms:
            raise ValueError("missing, duplicate or unexpected release platform")
        platforms.add(platform)
        if metadata.get("ci_evidence") != evidence or metadata.get("archive_smoke") != "passed":
            raise ValueError("package lacks this run's complete CI evidence or archive smoke result")
        built, packaged = metadata["build_source"], metadata["packaged_source"]
        if (built["commit"] != expected or packaged["commit"] != expected
                or built["status"] != "clean" or packaged["status"] != "clean"
                or built["source_tree_sha256"] != packaged["source_files_sha256"]):
            raise ValueError("packaged source/build identity differs from the clean CI-verified source")
        for key in ("cli_archive", "source_archive"):
            record = metadata[key]
            archive = directory / record["file"]
            if archive.parent != directory or digest(archive) != record["sha256"]:
                raise ValueError("release archive identity does not match package metadata")
    print(f"All release packages match complete CI evidence for {expected}.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("evidence", type=Path)
    parser.add_argument("source_sha")
    args = parser.parse_args()
    try:
        check(args.directory.resolve(), json.loads(args.evidence.read_text(encoding="utf-8")), args.source_sha)
    except (OSError, ValueError, KeyError, TypeError) as error:
        sys.exit(f"release evidence failed: {error}")
