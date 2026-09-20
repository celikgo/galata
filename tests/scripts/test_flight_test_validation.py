#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The package-backed flight-test validation receipt is conservative."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()
SCRIPT = ROOT / "scripts" / "run-flight-test-validation.py"


@unittest.skipUnless(os.name == "posix" and CLI.is_file(),
                     "requires the POSIX project CLI")
class FlightTestValidationReceipt(unittest.TestCase):
    def invoke(self, root, *arguments):
        return subprocess.run(
            [sys.executable, SCRIPT, "--cli", CLI, *map(str, arguments)],
            cwd=root,
            capture_output=True,
            text=True,
            timeout=180,
        )

    def test_synthetic_campaign_executes_but_cannot_become_flight_validation(self):
        example = ROOT / "examples" / "fixed-wing-validation-contract"
        with tempfile.TemporaryDirectory(prefix="galata-flight-validation-receipt-") as directory:
            root = Path(directory)
            package = root / "campaign"
            sources = {
                "test_plan": root / "test-plan.txt",
                "flight_record": example / "trajectory.csv",
                "calibration_manifest": root / "calibration.json",
                "configuration_manifest": root / "configuration.json",
                "reviewer_attestation": root / "reviewer.txt",
            }
            sources["test_plan"].write_text("approved test plan\n", encoding="utf-8")
            sources["calibration_manifest"].write_text("calibration-v1\n", encoding="utf-8")
            sources["configuration_manifest"].write_text("configuration-v1\n", encoding="utf-8")
            sources["reviewer_attestation"].write_text("independent review\n", encoding="utf-8")
            create = [
                "flighttest", "create", package,
                "--aircraft-id", "airframe-01",
                "--configuration", "configuration-2026-09",
                "--evidence-class", "synthetic_contract",
                "--test-plan-id", "FT-001",
                "--reviewer-id", "independent-reviewer",
                "--safety-review-complete", "true",
            ]
            for role, path in sources.items():
                create.extend(["--file", f"{role}={path}"])
            created = subprocess.run(
                [str(CLI), *map(str, create)],
                cwd=root,
                capture_output=True,
                text=True,
                timeout=60,
            )
            self.assertEqual(created.returncode, 0, created.stdout + created.stderr)

            study = root / "study.yaml"
            text = (example / "study.yaml").read_text(encoding="utf-8")
            text = text.replace(
                "path: ../../models/nt33a/nt33a-fc1.yaml",
                f"path: {ROOT / 'models' / 'nt33a' / 'nt33a-fc1.yaml'}",
            )
            text = text.replace(
                "path: trajectory.csv",
                f"path: {package / 'evidence' / 'flight_record'}",
            )
            text = text.replace(
                "      acceptance:\n        outputs:",
                f"      acceptance:\n        campaign_manifest: {package / 'campaign.manifest'}\n        outputs:",
            )
            study.write_text(text, encoding="utf-8")

            output = root / "run"
            receipt_path = root / "receipt.json"
            result = self.invoke(
                root,
                "--campaign-manifest", package / "campaign.manifest",
                "--study", study,
                "--output-dir", output,
                "--receipt", receipt_path,
            )
            self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
            receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
            self.assertEqual(receipt["schema"], "galata.flight-test-validation-receipt.v1")
            self.assertEqual(receipt["status"], "not_ready")
            self.assertEqual(receipt["evidence_class"], "synthetic_contract")
            self.assertEqual(receipt["numerical_acceptance_gate"], "pass")
            self.assertEqual(receipt["flight_test_evidence_gate"], "unresolved")
            self.assertTrue(receipt["campaign_binding_verified"])
            self.assertTrue(receipt["run_inputs_verified"])
            self.assertIn(
                "only measured_flight provenance",
                " ".join(receipt["reasons"]),
            )
            self.assertEqual(receipt["qualification_state"], "not_qualified")
            self.assertFalse(receipt["airworthiness_claim"])
            self.assertFalse(receipt["certification_claim"])

            direct_receipt_path = root / "direct-receipt.json"
            direct = subprocess.run(
                [
                    str(CLI), "flighttest", "validate",
                    package / "campaign.manifest", study,
                    "--output-dir", root / "direct-run",
                    "--receipt", direct_receipt_path,
                ],
                cwd=root,
                capture_output=True,
                text=True,
                timeout=60,
            )
            self.assertEqual(direct.returncode, 2, direct.stdout + direct.stderr)
            direct_summary = json.loads(direct.stdout)
            self.assertEqual(direct_summary["schema"],
                             "galata.flight-test-validation-receipt.v1")
            self.assertEqual(direct_summary["status"], "not_ready")
            direct_receipt = json.loads(direct_receipt_path.read_text(encoding="utf-8"))
            self.assertTrue(direct_receipt["campaign_binding_verified"])
            self.assertEqual(direct_receipt["numerical_acceptance_gate"], "pass")
            self.assertEqual(direct_receipt["flight_test_evidence_gate"], "unresolved")

            (package / "evidence" / "calibration_manifest").write_text(
                "tampered\n", encoding="utf-8"
            )
            failed = self.invoke(
                root,
                "--campaign-manifest", package / "campaign.manifest",
                "--study", study,
                "--output-dir", root / "tampered-run",
                "--receipt", root / "tampered-receipt.json",
            )
            self.assertEqual(failed.returncode, 1, failed.stdout + failed.stderr)
            tampered_receipt = json.loads(
                (root / "tampered-receipt.json").read_text(encoding="utf-8")
            )
            self.assertEqual(tampered_receipt["status"], "failed")


if __name__ == "__main__":
    unittest.main()
