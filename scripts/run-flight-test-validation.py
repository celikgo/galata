#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Execute and attest a package-backed Galata flight-test validation run.

The receipt is deliberately narrower than an airworthiness or certification
decision.  It proves that the supplied campaign package was verified, bound to
the validation pipeline, retained in the run input ledger, and that the
generated report exposed both numerical and flight-test evidence gates.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys


SCHEMA = "galata.flight-test-validation-receipt.v1"
MAX_MANIFEST_BYTES = 2 * 1024 * 1024
MAX_REPORT_BYTES = 64 * 1024 * 1024


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_bytes(path, maximum, label):
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"{label} must be a regular non-symlink file: {path}")
    if path.stat().st_size > maximum:
        raise ValueError(f"{label} exceeds its size limit: {path}")
    return path.read_bytes()


def _run_json(cli, arguments):
    process = subprocess.run(
        [str(cli), *map(str, arguments)],
        capture_output=True,
        text=True,
        timeout=180,
    )
    if process.returncode != 0:
        message = (process.stderr or process.stdout).strip().replace("\n", " ")
        raise RuntimeError(message or f"command exited with {process.returncode}")
    try:
        return json.loads(process.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"command did not emit JSON: {error}") from error


def _parse_campaign_manifest(path):
    bytes_ = _read_bytes(path, MAX_MANIFEST_BYTES, "campaign manifest")
    fields = {}
    files = {}
    for line_number, line in enumerate(bytes_.decode("utf-8").splitlines(), 1):
        if not line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key.startswith("file."):
            match = re.fullmatch(r"file\.(\d+)\.(role|path|sha256)", key)
            if not match:
                raise ValueError(f"invalid campaign file field on line {line_number}")
            files.setdefault(int(match.group(1)), {})[match.group(2)] = value
        else:
            fields[key] = value
    expected_roles = {
        "test_plan",
        "flight_record",
        "calibration_manifest",
        "configuration_manifest",
        "reviewer_attestation",
    }
    entries = []
    for index in sorted(files):
        entry = files[index]
        if set(entry) != {"role", "path", "sha256"}:
            raise ValueError(f"campaign file entry {index} is incomplete")
        if entry["role"] not in expected_roles:
            raise ValueError(f"unexpected campaign file role: {entry['role']}")
        relative = Path(entry["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"campaign file path escapes its package: {relative}")
        entries.append({
            "role": entry["role"],
            "path": relative,
            "sha256": entry["sha256"],
        })
    if {entry["role"] for entry in entries} != expected_roles:
        raise ValueError("campaign manifest does not contain all five required roles")
    return {
        "sha256": hashlib.sha256(bytes_).hexdigest(),
        "evidence_class": fields.get("evidence_class"),
        "files": entries,
    }


def _resolve_inside(root, relative, label):
    root = root.resolve()
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise ValueError(f"{label} escapes its root: {relative}") from error
    return candidate


def _input_index(run_manifest):
    return {Path(item["path"]).resolve(): item for item in run_manifest.get("inputs", [])}


def _report_from_run(run_manifest):
    output_root = Path(run_manifest["output_directory"]).resolve()
    for item in run_manifest.get("outputs", []):
        relative = Path(item["path"])
        candidate = _resolve_inside(output_root, relative, "run output")
        data = _read_bytes(candidate, MAX_REPORT_BYTES, "validation report")
        text = data.decode("utf-8")
        if "Flight-test evidence gate:" in text:
            return candidate, text
    raise ValueError("run did not produce a report containing the flight-test evidence gate")


def _gate(report, label):
    match = re.search(rf"\*\*{re.escape(label)}: ([a-z_]+)\*\*", report)
    return match.group(1) if match else "not_reported"


def _receipt(arguments):
    cli = arguments.cli.resolve()
    campaign_manifest = arguments.campaign_manifest.resolve()
    study = arguments.study.resolve()
    campaign = _parse_campaign_manifest(campaign_manifest)
    verification = _run_json(cli, ["flighttest", "verify", campaign_manifest])
    if verification.get("status") != "verified":
        raise RuntimeError("flight-test campaign verifier did not return status=verified")

    run_arguments = ["run", study, "--output-dir", arguments.output_dir, "--json"]
    if arguments.overwrite:
        run_arguments.append("--overwrite")
    run = _run_json(cli, run_arguments)
    run_manifest_path = Path(run["manifest_path"]).resolve()
    run_manifest = json.loads(
        _read_bytes(run_manifest_path, MAX_REPORT_BYTES, "run manifest")
    )
    if run_manifest.get("schema") != "galata.run.v1" or run_manifest.get("status") != "completed":
        raise ValueError("run manifest is not a completed galata.run.v1 manifest")

    validation_stages = [
        stage for stage in run_manifest.get("stages", [])
        if stage.get("capability") == "identify.validate.vehicle"
    ]
    if not validation_stages:
        raise ValueError("study did not execute identify.validate.vehicle")

    inputs = _input_index(run_manifest)
    manifest_input = inputs.get(campaign_manifest)
    reasons = []
    campaign_binding_verified = True
    if manifest_input is None:
        campaign_binding_verified = False
        reasons.append("campaign manifest is not retained in the run input ledger")
    elif manifest_input.get("sha256") != campaign["sha256"]:
        campaign_binding_verified = False
        reasons.append("campaign manifest digest in the run ledger does not match")

    for entry in campaign["files"]:
        source = _resolve_inside(campaign_manifest.parent, entry["path"], "campaign evidence")
        if _sha256(source) != entry["sha256"]:
            raise ValueError(f"campaign evidence changed after verification: {entry['role']}")
        ledger = inputs.get(source)
        if ledger is None:
            reasons.append(f"campaign evidence is not retained in the run input ledger: {entry['role']}")
        elif ledger.get("sha256") != entry["sha256"]:
            reasons.append(f"run ledger digest mismatch for campaign evidence: {entry['role']}")

    report_path, report = _report_from_run(run_manifest)
    numerical_gate = _gate(report, "Numerical acceptance gate")
    flight_gate = _gate(report, "Flight-test evidence gate")
    digest_match = re.search(
        r"Campaign package manifest SHA-256: `([0-9a-f]{64})`", report
    )
    if digest_match is None:
        campaign_binding_verified = False
        reasons.append("validation report does not retain the campaign manifest digest")
    elif digest_match.group(1) != campaign["sha256"]:
        campaign_binding_verified = False
        reasons.append("validation report campaign digest does not match the verified package")

    if numerical_gate != "pass":
        reasons.append(f"numerical acceptance gate is {numerical_gate}")
    if flight_gate != "pass":
        reasons.append(f"flight-test evidence gate is {flight_gate}")
    if campaign["evidence_class"] != "measured_flight":
        reasons.append("only measured_flight provenance can produce a production validation receipt")
    run_inputs_verified = not any(
        "run input ledger" in reason or "run ledger" in reason
        for reason in reasons
    )
    if reasons:
        status = "not_ready"
    else:
        status = "gate_passed"
    return {
        "schema": SCHEMA,
        "status": status,
        "campaign_manifest": str(campaign_manifest),
        "campaign_manifest_sha256": campaign["sha256"],
        "campaign_verification": verification,
        "study": str(study),
        "run_manifest": str(run_manifest_path),
        "validation_stage_count": len(validation_stages),
        "validation_stage_ids": [stage["id"] for stage in validation_stages],
        "report": str(report_path),
        "campaign_binding_verified": campaign_binding_verified,
        "evidence_class": campaign["evidence_class"],
        "numerical_acceptance_gate": numerical_gate,
        "flight_test_evidence_gate": flight_gate,
        "run_inputs_verified": run_inputs_verified,
        "reasons": reasons,
        "qualification_state": "not_qualified",
        "airworthiness_claim": False,
        "certification_claim": False,
        "external_authority_acceptance_required": True,
    }


def _write_receipt(path, receipt, overwrite):
    if path.exists() and not overwrite:
        raise FileExistsError(f"receipt already exists; pass --overwrite: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp-{Path.cwd().name}")
    temporary.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", type=Path, required=True)
    parser.add_argument("--campaign-manifest", type=Path, required=True)
    parser.add_argument("--study", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--receipt", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    arguments = parser.parse_args()
    try:
        receipt = _receipt(arguments)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        receipt = {
            "schema": SCHEMA,
            "status": "failed",
            "reasons": [str(error)],
            "qualification_state": "not_qualified",
            "airworthiness_claim": False,
            "certification_claim": False,
            "external_authority_acceptance_required": True,
        }
        try:
            _write_receipt(arguments.receipt, receipt, arguments.overwrite)
        except OSError as write_error:
            print(f"galata flight-test validation: {write_error}", file=sys.stderr)
        print(json.dumps(receipt, sort_keys=True), file=sys.stdout)
        return 1
    try:
        _write_receipt(arguments.receipt, receipt, arguments.overwrite)
    except OSError as error:
        print(f"galata flight-test validation: {error}", file=sys.stderr)
        return 1
    print(json.dumps(receipt, sort_keys=True))
    return 0 if receipt["status"] == "gate_passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
