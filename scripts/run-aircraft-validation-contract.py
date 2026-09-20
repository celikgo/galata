#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run the broader-aircraft evidence boundary with non-qualifying fixtures."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "scripts" / "check-aircraft-validation.py"
MODELS = (
    ("galata-f16", "nominal-derivative-slice", ROOT / "models" / "f16" / "f16-nominal.yaml"),
    ("galata-gtm-t2", "nominal-derivative-slice", ROOT / "models" / "gtm" / "gtm-t2-nominal.yaml"),
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def descriptor(root: Path, path: Path) -> dict:
    return {
        "path": path.relative_to(root).as_posix(),
        "sha256": sha256(path),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    output = arguments.output_dir.resolve()
    if output.exists():
        raise SystemExit(f"output directory already exists: {output}")
    output.mkdir(parents=True)

    records = []
    for aircraft_id, configuration_id, model_source in MODELS:
        evidence = output / "evidence" / aircraft_id / configuration_id
        evidence.mkdir(parents=True)
        model = evidence / "model.yaml"
        shutil.copy2(model_source, model)
        files = {"model": descriptor(output, model)}
        for role, text in {
            "calibration": "synthetic contract fixture; no calibrated aircraft instrumentation\n",
            "envelope": "synthetic contract fixture; no controlled operating envelope\n",
            "validation_report": "synthetic contract fixture; no flight-validation report\n",
            "review_record": "synthetic contract fixture; independent review not performed\n",
        }.items():
            path = evidence / f"{role}.txt"
            path.write_text(text, encoding="utf-8")
            files[role] = descriptor(output, path)
        records.append({
            "aircraft_id": aircraft_id,
            "configuration_id": configuration_id,
            **files,
            "independent_review_complete": False,
        })

    manifest = output / "aircraft-validation.manifest"
    manifest.write_text(json.dumps({
        "schema": "galata.aircraft-validation-evidence.v1",
        "status": "complete",
        "evidence_class": "independent_validation",
        "aircraft": records,
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    checked = subprocess.run(
        [sys.executable, str(CHECKER), str(manifest)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=60,
    )
    if checked.returncode != 0:
        raise RuntimeError(checked.stdout + checked.stderr)
    verification = json.loads(checked.stdout)
    summary = {
        "schema": "galata.aircraft-validation-contract.v1",
        "status": "not_ready_by_design",
        "manifest": str(manifest),
        "verification": verification,
    }
    (output / "aircraft-validation-contract-summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"aircraft validation contract failed: {error}") from error
