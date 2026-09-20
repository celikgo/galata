#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify controlled, independently reviewed aircraft-validation evidence.

This verifier checks the bytes and structure of an evidence package.  It does
not create flight evidence, certify a model, or decide airworthiness.  A
package is eligible for the broader-aircraft readiness gate only when it
contains at least two distinct aircraft, with model, calibration, envelope,
validation-report and independent-review records bound by SHA-256.
"""

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import sys


SCHEMA = "galata.aircraft-validation-evidence.v1"
RESULT_SCHEMA = "galata.aircraft-validation-verification.v1"
SHA256 = re.compile(r"[0-9a-f]{64}\Z")
MAX_MANIFEST_BYTES = 16 * 1024 * 1024
MAX_EVIDENCE_BYTES = 256 * 1024 * 1024
REQUIRED_RECORD_KEYS = {
    "aircraft_id",
    "configuration_id",
    "model",
    "calibration",
    "envelope",
    "validation_report",
    "review_record",
    "independent_review_complete",
}
REQUIRED_FILE_KEYS = {"path", "sha256"}


def digest(path):
    result = hashlib.sha256()
    total = 0
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            total += len(chunk)
            if total > MAX_EVIDENCE_BYTES:
                raise ValueError(f"evidence file is too large: {path}")
            result.update(chunk)
    return result.hexdigest()


def _regular_file(path, label):
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{label} must be a regular non-symlink file: {path}")
    return path


def _relative_path(value, label):
    if not isinstance(value, str) or not value or "\\" in value or value.startswith("/"):
        raise ValueError(f"{label} path must be a relative POSIX path")
    path = PurePosixPath(value)
    if (not path.parts or any(part in {"", ".", ".."} for part in path.parts)
            or ":" in path.parts[0]):
        raise ValueError(f"{label} path is unsafe: {value!r}")
    return path


def _evidence_file(root, descriptor, label):
    if not isinstance(descriptor, dict) or set(descriptor) != REQUIRED_FILE_KEYS:
        raise ValueError(f"{label} must contain only path and sha256")
    relative = _relative_path(descriptor["path"], label)
    sha256 = descriptor["sha256"]
    if not isinstance(sha256, str) or not SHA256.fullmatch(sha256):
        raise ValueError(f"{label} has an invalid SHA-256 digest")
    # Refuse links in every path component, not only links at the leaf.
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise ValueError(f"{label} path contains a symlink: {relative}")
    path = root / Path(*relative.parts)
    _regular_file(path, label)
    actual = digest(path)
    if actual != sha256:
        raise ValueError(f"{label} SHA-256 digest does not match {relative}")
    return {
        "path": relative.as_posix(),
        "sha256": actual,
        "size_bytes": path.stat().st_size,
    }


def _record(root, value, index):
    label = f"aircraft[{index}]"
    if not isinstance(value, dict) or set(value) != REQUIRED_RECORD_KEYS:
        raise ValueError(f"{label} has an unexpected field set")
    aircraft_id = value["aircraft_id"]
    configuration_id = value["configuration_id"]
    if (not isinstance(aircraft_id, str) or not aircraft_id.strip()
            or not isinstance(configuration_id, str) or not configuration_id.strip()):
        raise ValueError(f"{label} must name an aircraft and configuration")
    if type(value["independent_review_complete"]) is not bool:
        raise ValueError(f"{label}.independent_review_complete must be boolean")
    files = {
        name: _evidence_file(root, value[name], f"{label}.{name}")
        for name in ("model", "calibration", "envelope", "validation_report", "review_record")
    }
    return {
        "aircraft_id": aircraft_id,
        "configuration_id": configuration_id,
        "independent_review_complete": value["independent_review_complete"],
        "files": files,
    }


def verify(manifest_path):
    manifest_path = Path(manifest_path)
    _regular_file(manifest_path, "aircraft evidence manifest")
    manifest_path = manifest_path.resolve()
    if manifest_path.stat().st_size > MAX_MANIFEST_BYTES:
        raise ValueError("aircraft evidence manifest is too large")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"aircraft evidence manifest could not be read: {error}") from error
    if (not isinstance(manifest, dict)
            or set(manifest) != {"schema", "status", "evidence_class", "aircraft"}):
        raise ValueError("aircraft evidence manifest has an unexpected field set")
    if manifest["schema"] != SCHEMA:
        raise ValueError(f"manifest schema must be {SCHEMA}")
    if manifest["status"] != "complete":
        raise ValueError("aircraft evidence manifest status must be complete")
    if manifest["evidence_class"] != "independent_validation":
        raise ValueError("aircraft evidence_class must be independent_validation")
    entries = manifest["aircraft"]
    if not isinstance(entries, list) or not entries:
        raise ValueError("aircraft evidence manifest must contain a non-empty aircraft list")
    records = [_record(manifest_path.parent, entry, index)
               for index, entry in enumerate(entries)]
    identities = {(record["aircraft_id"], record["configuration_id"]) for record in records}
    if len(identities) != len(records):
        raise ValueError("aircraft evidence contains duplicate aircraft/configuration records")
    aircraft_ids = {record["aircraft_id"] for record in records}
    reviewed = all(record["independent_review_complete"] for record in records)
    reasons = []
    if len(aircraft_ids) < 2:
        reasons.append("at least two distinct aircraft are required for broader scope")
    if not reviewed:
        reasons.append("every aircraft configuration requires completed independent review")
    return {
        "schema": RESULT_SCHEMA,
        "status": "verified",
        "manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "evidence_class": manifest["evidence_class"],
        "aircraft_count": len(records),
        "distinct_aircraft_count": len(aircraft_ids),
        "independent_review_complete": reviewed,
        "eligible": not reasons,
        "reasons": reasons,
        "records": records,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    arguments = parser.parse_args()
    try:
        result = verify(arguments.manifest)
    except (OSError, ValueError, TypeError) as error:
        print(json.dumps({
            "schema": RESULT_SCHEMA,
            "status": "failed",
            "reasons": [str(error)],
        }, indent=2, sort_keys=True))
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
