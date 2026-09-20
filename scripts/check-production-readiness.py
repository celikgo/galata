#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run the repository-controlled production readiness preflight.

This command joins the independent verifiers so a local candidate cannot be
mistaken for a flight release.  A successful byte check is recorded separately
from an eligibility check, and the command never emits a qualified,
airworthiness or certification claim.
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys


SCHEMA = "galata.production-readiness-audit.v1"


def _path_value(value):
    return str(value) if value is not None else None


def _missing_check(identifier, requirement, path):
    return {
        "id": identifier,
        "requirement": requirement,
        "path": _path_value(path),
        "verification": "not_run",
        "eligible": False,
        "reasons": ["required input was not supplied"],
    }


def _run_json(cli, arguments):
    process = subprocess.run(
        [str(cli), *map(str, arguments)],
        capture_output=True,
        text=True,
        timeout=120,
    )
    if process.returncode != 0:
        message = (process.stderr or process.stdout).strip().replace("\n", " ")
        raise RuntimeError(message or f"command exited with {process.returncode}")
    try:
        return json.loads(process.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"command did not emit JSON: {error}") from error


def _developer_id_identity_count():
    """Return the locally visible Developer ID Application identity count."""
    if sys.platform != "darwin":
        return None
    try:
        process = subprocess.run(
            ["/usr/bin/security", "find-identity", "-v", "-p", "codesigning"],
            capture_output=True,
            text=True,
            timeout=30,
            check=True,
        )
    except (OSError, subprocess.SubprocessError):
        return 0
    return sum("Developer ID Application:" in line for line in process.stdout.splitlines())


def _verified_check(identifier, requirement, path, command, predicate, reasons):
    command = [str(part) for part in command]
    result = {
        "id": identifier,
        "requirement": requirement,
        "path": _path_value(path),
        "command": command,
        "verification": "failed",
        "eligible": False,
        "reasons": [],
    }
    if path is None:
        return _missing_check(identifier, requirement, path)
    try:
        payload = _run_json(command[0], command[1:])
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        result["reasons"] = [str(error)]
        return result
    result["result"] = payload
    result["verification"] = "verified" if payload.get("status") == "verified" else "failed"
    if result["verification"] != "verified":
        result["reasons"] = ["verifier did not return status=verified"]
        return result
    result["eligible"] = bool(predicate(payload))
    result["reasons"] = [] if result["eligible"] else list(reasons(payload))
    return result


def _desktop_check(package_path, dmg_metadata_path):
    identifier = "desktop_distribution"
    requirement = "Developer ID signing, notarization, CI approval and stable release metadata"
    result = {
        "id": identifier,
        "requirement": requirement,
        "path": _path_value(package_path),
        "dmg_metadata": _path_value(dmg_metadata_path),
        "verification": "failed",
        "eligible": False,
        "reasons": [],
    }
    if package_path is None:
        return _missing_check(identifier, requirement, package_path)
    try:
        metadata = json.loads(package_path.read_text(encoding="utf-8"))
        distribution = metadata["distribution"]
        result["verification"] = "verified"
        result["distribution"] = distribution
        developer_id_count = _developer_id_identity_count()
        if developer_id_count is not None:
            result["developer_id_application_identity_count"] = developer_id_count
        if dmg_metadata_path is not None:
            dmg = json.loads(dmg_metadata_path.read_text(encoding="utf-8"))
            result["dmg_distribution"] = dmg.get("distribution")
            if dmg.get("distribution") != distribution:
                result["verification"] = "failed"
                result["reasons"].append("ZIP and DMG distribution metadata differ")
        expected = {
            "signing": "developer_id",
            "notarization": "submitted",
            "ci_approval": "claimed",
            "stable_release": True,
        }
        if distribution != expected:
            result["reasons"].extend([
                "candidate is not Developer ID signed and notarized",
                "stable CI-approved release metadata is not present",
            ])
            if developer_id_count == 0:
                result["reasons"].append(
                    "no valid Developer ID Application signing identity is available on this host"
                )
        else:
            result["eligible"] = developer_id_count is None or developer_id_count > 0
            if not result["eligible"]:
                result["reasons"].append(
                    "Developer ID Application signing identity is unavailable"
                )
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        result["reasons"] = [f"desktop metadata could not be verified: {error}"]
    return result


def _flight_test_check(arguments, cli):
    result = _verified_check(
        "flight_test_validation",
        "measured_flight campaign plus a verified package-backed validation receipt",
        arguments.flighttest,
        [cli, "flighttest", "verify", arguments.flighttest]
        if arguments.flighttest is not None else [cli],
        lambda payload: payload.get("evidence_class") == "measured_flight"
        and payload.get("safety_review_complete") is True,
        lambda payload: [
            "flight-test evidence_class must be measured_flight",
        ],
    )
    if result["verification"] != "verified":
        return result
    package_reasons = list(result["reasons"])
    if arguments.flight_validation_receipt is None:
        result["eligible"] = False
        result["reasons"] = package_reasons + [
            "package-backed flight-validation receipt was not supplied",
        ]
        return result
    try:
        receipt = json.loads(
            arguments.flight_validation_receipt.read_text(encoding="utf-8")
        )
        result["flight_validation_receipt"] = receipt
        if receipt.get("schema") != "galata.flight-test-validation-receipt.v1":
            raise ValueError("receipt schema is not galata.flight-test-validation-receipt.v1")
        if receipt.get("campaign_manifest_sha256") != result["result"].get("manifest_sha256"):
            raise ValueError("receipt campaign manifest digest does not match the verified package")
        receipt_reasons = []
        if receipt.get("status") != "gate_passed":
            receipt_reasons.append("flight-validation receipt status is not gate_passed")
        if receipt.get("evidence_class") != "measured_flight":
            receipt_reasons.append("flight-validation receipt evidence_class must be measured_flight")
        if receipt.get("numerical_acceptance_gate") != "pass":
            receipt_reasons.append("numerical acceptance gate did not pass")
        if receipt.get("flight_test_evidence_gate") != "pass":
            receipt_reasons.append("flight-test evidence gate did not pass")
        if receipt.get("campaign_binding_verified") is not True:
            receipt_reasons.append("campaign binding was not verified")
        if receipt.get("run_inputs_verified") is not True:
            receipt_reasons.append("run input ledger was not verified")
        declared_campaign = receipt.get("campaign_manifest")
        if not isinstance(declared_campaign, str) or not declared_campaign:
            receipt_reasons.append("flight-validation receipt does not name its campaign manifest")
        else:
            try:
                if Path(declared_campaign).resolve() != arguments.flighttest.resolve():
                    receipt_reasons.append(
                        "flight-validation receipt campaign path does not match the supplied package"
                    )
            except OSError:
                receipt_reasons.append(
                    "flight-validation receipt campaign path could not be resolved"
                )
        result["eligible"] = result["eligible"] and not receipt_reasons
        result["reasons"] = package_reasons + receipt_reasons
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        result["verification"] = "failed"
        result["eligible"] = False
        result["reasons"] = [f"flight-validation receipt could not be verified: {error}"]
    return result


def audit(arguments):
    cli = arguments.cli.resolve()
    checks = []
    checks.append(_flight_test_check(arguments, cli))
    checks.append(_verified_check(
        "onboard_deployment",
        "verified executable deployment bundle",
        arguments.deployment_dir,
        [cli, "onboard", "verify-deployment", arguments.deployment_dir]
        if arguments.deployment_dir is not None else [cli],
        lambda payload: payload.get("contains_executable") is True,
        lambda payload: [
            "deployment bundle is not an executable target handoff",
        ],
    ))
    if arguments.target_evidence is None or arguments.deployment is None:
        checks.append(_missing_check(
            "hardware_target",
            "target_hil or flight_target evidence bound to the deployment",
            arguments.target_evidence,
        ))
    else:
        checks.append(_verified_check(
            "hardware_target",
            "target_hil or flight_target evidence bound to the deployment",
            arguments.target_evidence,
            [cli, "onboard", "target", "verify", arguments.deployment, arguments.target_evidence],
            lambda payload: payload.get("evidence_class") in {"target_hil", "flight_target"},
            lambda payload: [
                "target evidence_class must be target_hil or flight_target",
            ],
        ))

    qualification_path = arguments.dossier
    if qualification_path is None or arguments.flighttest is None \
            or arguments.flight_validation_receipt is None \
            or arguments.deployment is None or arguments.deployment_dir is None \
            or arguments.target_evidence is None:
        checks.append(_missing_check(
            "qualification_chain",
            "traceable chain with a gate-passed flight-validation receipt eligible for external authority review",
            qualification_path,
        ))
    else:
        checks.append(_verified_check(
            "qualification_chain",
            "traceable chain eligible for external authority review",
            qualification_path,
            [cli, "qualification", "verify-chain", qualification_path,
             "--flighttest", arguments.flighttest,
             "--flight-validation-receipt", arguments.flight_validation_receipt,
             "--deployment", arguments.deployment,
             "--deployment-dir", arguments.deployment_dir,
             "--target-evidence", arguments.target_evidence],
            lambda payload: payload.get("qualification_eligibility")
            == "eligible_for_authority_review",
            lambda payload: list(payload.get("qualification_eligibility_reasons", []))
            or ["qualification chain is not eligible for external authority review"],
        ))
    checks.append(_desktop_check(arguments.desktop_package, arguments.dmg_metadata))

    checks.append(_verified_check(
        "broader_aircraft_scope",
        "independently validated aircraft models and operating envelopes",
        arguments.aircraft_evidence,
        [sys.executable, str(Path(__file__).with_name("check-aircraft-validation.py")),
         arguments.aircraft_evidence]
        if arguments.aircraft_evidence is not None else [sys.executable],
        lambda payload: payload.get("eligible") is True
        and payload.get("evidence_class") == "independent_validation",
        lambda payload: list(payload.get("reasons", []))
        or ["independent aircraft-validation evidence is not eligible for broader scope"],
    ))

    failures = [check for check in checks if check["verification"] == "failed"]
    gaps = [check for check in checks if not check["eligible"]]
    report = {
        "schema": SCHEMA,
        "status": "failed" if failures else ("not_ready" if gaps else "ready_for_external_review"),
        "checks": checks,
        "failure_count": len(failures),
        "readiness_gap_count": len(gaps),
        "qualification_state": "not_qualified",
        "airworthiness_claim": False,
        "certification_claim": False,
        "external_authority_acceptance_required": True,
    }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", type=Path, required=True)
    parser.add_argument("--flighttest", type=Path)
    parser.add_argument("--flight-validation-receipt", type=Path)
    parser.add_argument("--deployment", type=Path)
    parser.add_argument("--deployment-dir", type=Path)
    parser.add_argument("--target-evidence", type=Path)
    parser.add_argument("--dossier", type=Path)
    parser.add_argument("--desktop-package", type=Path)
    parser.add_argument("--dmg-metadata", type=Path)
    parser.add_argument("--aircraft-evidence", type=Path)
    parser.add_argument("--json-output", type=Path)
    arguments = parser.parse_args()
    report = audit(arguments)
    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(serialized, end="")
    if arguments.json_output is not None:
        arguments.json_output.write_text(serialized, encoding="utf-8")
    if report["status"] == "failed":
        return 1
    if report["status"] == "not_ready":
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
