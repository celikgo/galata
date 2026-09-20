#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The production preflight reports missing external evidence explicitly."""

import json
import hashlib
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()
SCRIPT = ROOT / "scripts" / "check-production-readiness.py"
_SPEC = importlib.util.spec_from_file_location("galata_production_readiness", SCRIPT)
READINESS = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
_SPEC.loader.exec_module(READINESS)


@unittest.skipUnless(os.name == "posix" and CLI.is_file(),
                     "requires the POSIX project CLI")
class ProductionReadinessAudit(unittest.TestCase):
    def test_missing_external_inputs_are_reported_without_claims(self):
        with tempfile.TemporaryDirectory(prefix="galata-readiness-audit-") as directory:
            output = Path(directory) / "readiness.json"
            result = subprocess.run(
                [sys.executable, SCRIPT, "--cli", CLI, "--json-output", output],
                capture_output=True,
                text=True,
                timeout=45,
            )
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["schema"], "galata.production-readiness-audit.v1")
        self.assertEqual(report["status"], "not_ready")
        self.assertEqual(report["failure_count"], 0)
        self.assertGreaterEqual(report["readiness_gap_count"], 6)
        self.assertEqual(report["qualification_state"], "not_qualified")
        self.assertFalse(report["airworthiness_claim"])
        self.assertFalse(report["certification_claim"])
        self.assertTrue(report["external_authority_acceptance_required"])
        self.assertEqual(
            {check["id"] for check in report["checks"]},
            {
                "flight_test_validation",
                "onboard_deployment",
                "hardware_target",
                "qualification_chain",
                "desktop_distribution",
                "broader_aircraft_scope",
            },
        )

    def test_flight_receipt_must_bind_the_verified_campaign_path(self):
        with tempfile.TemporaryDirectory(prefix="galata-readiness-receipt-") as directory:
            root = Path(directory)
            campaign = root / "campaign.manifest"
            campaign.write_text("campaign bytes\n", encoding="utf-8")
            receipt_path = root / "receipt.json"
            receipt_path.write_text(json.dumps({
                "schema": "galata.flight-test-validation-receipt.v1",
                "status": "gate_passed",
                "campaign_manifest": str(campaign.resolve()),
                "campaign_manifest_sha256": hashlib.sha256(
                    campaign.read_bytes()).hexdigest(),
                "evidence_class": "measured_flight",
                "numerical_acceptance_gate": "pass",
                "flight_test_evidence_gate": "pass",
                "campaign_binding_verified": True,
                "run_inputs_verified": True,
            }), encoding="utf-8")
            arguments = SimpleNamespace(
                flighttest=campaign,
                flight_validation_receipt=receipt_path,
            )
            verified_package = {
                "status": "verified",
                "manifest_sha256": json.loads(receipt_path.read_text(encoding="utf-8"))[
                    "campaign_manifest_sha256"
                ],
                "evidence_class": "measured_flight",
                "safety_review_complete": True,
            }
            with patch.object(READINESS, "_run_json", return_value=verified_package):
                accepted = READINESS._flight_test_check(arguments, Path("/fake/galata"))
            self.assertTrue(accepted["eligible"])
            self.assertEqual(accepted["reasons"], [])

            receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
            receipt["campaign_manifest"] = str((root / "other.manifest").resolve())
            receipt_path.write_text(json.dumps(receipt), encoding="utf-8")
            with patch.object(READINESS, "_run_json", return_value=verified_package):
                rejected = READINESS._flight_test_check(arguments, Path("/fake/galata"))
            self.assertFalse(rejected["eligible"])
            self.assertIn(
                "flight-validation receipt campaign path does not match the supplied package",
                rejected["reasons"],
            )

    def test_supplied_path_arguments_remain_json_serializable(self):
        with patch.object(READINESS, "_run_json", return_value={
            "status": "verified",
            "contains_executable": True,
        }):
            result = READINESS._verified_check(
                "onboard_deployment",
                "verified executable deployment bundle",
                Path("/tmp/deployment"),
                [Path("/tmp/galata"), "onboard", "verify-deployment",
                 Path("/tmp/deployment")],
                lambda payload: payload["contains_executable"],
                lambda payload: [],
            )
        self.assertTrue(result["eligible"])
        self.assertTrue(all(isinstance(part, str) for part in result["command"]))
        json.dumps(result)


if __name__ == "__main__":
    unittest.main()
