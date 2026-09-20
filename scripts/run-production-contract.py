#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run Galata's repository-controlled evidence chain with synthetic inputs.

The output is intentionally not production-eligible. The purpose is to prove
that package creation, validation receipt binding, onboard deployment,
host-SIL target evidence and qualification-chain traceability compose without
turning placeholders into aircraft or authority evidence.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "examples" / "production-evidence-contract"
CAMPAIGN_EXAMPLE = ROOT / "examples" / "fixed-wing-validation-contract"
ONBOARD_EXAMPLE = ROOT / "examples" / "onboard-deployment"


def command(cli: Path, *arguments: object, expected: tuple[int, ...] = (0,)) -> dict:
    result = subprocess.run(
        [str(cli), *(str(argument) for argument in arguments)],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=180,
    )
    if result.returncode not in expected:
        raise RuntimeError(
            f"command failed with {result.returncode}: {result.stdout}\n{result.stderr}"
        )
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"command did not emit JSON: {result.stdout}") from error


def flight_validation(cli: Path, output: Path, campaign: Path) -> dict:
    source = CAMPAIGN_EXAMPLE / "study.yaml"
    study = output / "study.yaml"
    text = source.read_text(encoding="utf-8")
    text = text.replace(
        "path: ../../models/nt33a/nt33a-fc1.yaml",
        f"path: {ROOT / 'models' / 'nt33a' / 'nt33a-fc1.yaml'}",
    )
    text = text.replace(
        "path: trajectory.csv",
        f"path: {campaign / 'evidence' / 'flight_record'}",
    )
    text = text.replace(
        "      acceptance:\n        outputs:",
        f"      acceptance:\n        campaign_manifest: {campaign / 'campaign.manifest'}\n        outputs:",
    )
    study.write_text(text, encoding="utf-8")
    receipt = output / "flight-validation-receipt.json"
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "run-flight-test-validation.py"),
            "--cli",
            str(cli),
            "--campaign-manifest",
            str(campaign / "campaign.manifest"),
            "--study",
            str(study),
            "--output-dir",
            str(output / "validation-run"),
            "--receipt",
            str(receipt),
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=180,
    )
    if result.returncode != 2:
        raise RuntimeError(
            "synthetic validation must return exit code 2:\n"
            + result.stdout
            + result.stderr
        )
    return json.loads(receipt.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cli",
        type=Path,
        default=ROOT / "build" / "ci-macos" / "src" / "cli" / "galata",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    cli = arguments.cli.resolve()
    output = arguments.output_dir.resolve()
    if not cli.is_file() or cli.is_symlink():
        raise SystemExit(f"CLI is not a regular file: {cli}")
    if output.exists():
        raise SystemExit(f"output directory already exists: {output}")
    output.mkdir(parents=True)

    campaign = output / "campaign"
    campaign_files = {
        "test_plan": FIXTURE / "evidence" / "test_plan.txt",
        "flight_record": CAMPAIGN_EXAMPLE / "trajectory.csv",
        "calibration_manifest": FIXTURE / "evidence" / "calibration_manifest.txt",
        "configuration_manifest": FIXTURE / "evidence" / "configuration_manifest.txt",
        "reviewer_attestation": FIXTURE / "evidence" / "reviewer_attestation.txt",
    }
    create = [
        "flighttest",
        "create",
        campaign,
        "--aircraft-id",
        "synthetic-airframe-01",
        "--configuration",
        "synthetic-configuration-2026-09",
        "--evidence-class",
        "synthetic_contract",
        "--test-plan-id",
        "SYNTHETIC-FT-001",
        "--reviewer-id",
        "synthetic-reviewer",
        "--safety-review-complete",
        "true",
    ]
    for role, path in campaign_files.items():
        create.extend(("--file", f"{role}={path}"))
    campaign_result = command(cli, *create)
    campaign_verification = command(cli, "flighttest", "verify", campaign / "campaign.manifest")
    receipt = flight_validation(cli, output, campaign)

    deployment = output / "deployment"
    deployment_result = command(
        cli,
        "onboard",
        "deploy",
        ONBOARD_EXAMPLE / "onboard.manifest",
        deployment,
        "--runtime",
        cli,
        "--artifact",
        f"model={ONBOARD_EXAMPLE / 'model-artifact.bin'}",
        "--artifact",
        f"controller={ONBOARD_EXAMPLE / 'controller-artifact.bin'}",
    )
    deployment_verification = command(cli, "onboard", "verify-deployment", deployment)

    target = output / "target-evidence"
    target_arguments = [
        "onboard",
        "target",
        "create",
        target,
        deployment / "onboard.manifest",
        "--evidence-class",
        "host_sil",
        "--controller-worst-case-s",
        "0.001",
        "--cycle-worst-case-s",
        "0.004",
        "--watchdog-response-s",
        "0.008",
    ]
    for role in (
        "timing_report",
        "hardware_hil_report",
        "failsafe_report",
        "signing_record",
        "target_configuration",
    ):
        target_arguments.extend(
            ("--file", f"{role}={ONBOARD_EXAMPLE / 'host-sil-evidence' / role}")
        )
    target_result = command(cli, *target_arguments)
    target_verification = command(
        cli,
        "onboard",
        "target",
        "verify",
        deployment / "onboard.manifest",
        target / "target-evidence.manifest",
    )

    dossier = output / "qualification"
    dossier_files = {
        "requirements_matrix": FIXTURE / "evidence" / "requirements_matrix.txt",
        "software_release": FIXTURE / "evidence" / "software_release.txt",
        "verification_report": FIXTURE / "evidence" / "verification_report.txt",
        "flight_test_campaign": campaign / "campaign.manifest",
        "hardware_hil_report": target / "target-evidence.manifest",
        "safety_case": FIXTURE / "evidence" / "safety_case.txt",
        "independent_review": FIXTURE / "evidence" / "independent_review.txt",
        "authority_decision": FIXTURE / "evidence" / "authority_decision.txt",
        "maintenance_plan": FIXTURE / "evidence" / "maintenance_plan.txt",
    }
    dossier_create = [
        "qualification",
        "create",
        dossier,
        "--product-id",
        "galata",
        "--product-version",
        "0.3.0",
        "--intended-use",
        "synthetic repository integration contract",
        "--aircraft-id",
        "synthetic-airframe-01",
        "--configuration",
        "synthetic-configuration-2026-09",
        "--qualification-basis",
        "synthetic traceability contract",
        "--authority-id",
        "external-authority-pending",
    ]
    for role, path in dossier_files.items():
        dossier_create.extend(("--file", f"{role}={path}"))
    dossier_result = command(cli, *dossier_create)
    chain = command(
        cli,
        "qualification",
        "verify-chain",
        dossier / "qualification.manifest",
        "--flighttest",
        campaign / "campaign.manifest",
        "--flight-validation-receipt",
        output / "flight-validation-receipt.json",
        "--deployment",
        deployment / "onboard.manifest",
        "--deployment-dir",
        deployment,
        "--target-evidence",
        target / "target-evidence.manifest",
    )

    summary = {
        "schema": "galata.production-contract.v1",
        "status": "not_ready_by_design",
        "campaign": campaign_result,
        "campaign_verification": campaign_verification,
        "flight_validation_receipt": {
            "status": receipt.get("status"),
            "evidence_class": receipt.get("evidence_class"),
            "numerical_acceptance_gate": receipt.get("numerical_acceptance_gate"),
            "flight_test_evidence_gate": receipt.get("flight_test_evidence_gate"),
        },
        "deployment": deployment_result,
        "deployment_verification": deployment_verification,
        "target_evidence": {
            "create_status": target_result.get("status"),
            "verification_status": target_verification.get("status"),
            "evidence_class": target_verification.get("evidence_class"),
            "target_acceptance_state": target_verification.get("target_acceptance_state"),
        },
        "qualification": {
            "create_status": dossier_result.get("status"),
            "chain_status": chain.get("status"),
            "deployment_runtime_verified": chain.get("deployment_runtime_verified"),
            "deployment_runtime_sha256": chain.get("deployment_runtime_sha256"),
            "qualification_eligibility": chain.get("qualification_eligibility"),
            "qualification_state": chain.get("qualification_state"),
            "reasons": chain.get("qualification_eligibility_reasons", []),
        },
        "output_dir": str(output),
    }
    (output / "production-contract-summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"production contract failed: {error}") from error
