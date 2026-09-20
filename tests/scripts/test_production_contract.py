#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The synthetic production contract composes every local evidence boundary."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()
SCRIPT = ROOT / "scripts" / "run-production-contract.py"


@unittest.skipUnless(os.name == "posix" and CLI.is_file(),
                     "requires the POSIX project CLI")
class ProductionEvidenceContract(unittest.TestCase):
    def test_all_local_evidence_boundaries_compose_without_claiming_readiness(self):
        with tempfile.TemporaryDirectory(prefix="galata-production-contract-") as directory:
            output = Path(directory) / "contract"
            result = subprocess.run(
                [sys.executable, SCRIPT, "--cli", CLI, "--output-dir", output],
                cwd=ROOT,
                capture_output=True,
                text=True,
                timeout=180,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            summary = json.loads(
                (output / "production-contract-summary.json").read_text(encoding="utf-8")
            )
            self.assertEqual(summary["status"], "not_ready_by_design")
            self.assertEqual(summary["campaign_verification"]["status"], "verified")
            self.assertEqual(summary["flight_validation_receipt"]["status"], "not_ready")
            self.assertEqual(summary["flight_validation_receipt"]["evidence_class"],
                             "synthetic_contract")
            self.assertEqual(summary["deployment_verification"]["status"], "verified")
            self.assertEqual(summary["target_evidence"]["verification_status"], "verified")
            self.assertEqual(summary["target_evidence"]["evidence_class"], "host_sil")
            self.assertEqual(summary["qualification"]["chain_status"], "verified")
            self.assertTrue(summary["qualification"]["deployment_runtime_verified"])
            self.assertRegex(summary["qualification"]["deployment_runtime_sha256"], r"^[0-9a-f]{64}$")
            self.assertEqual(summary["qualification"]["qualification_eligibility"],
                             "not_ready")
            self.assertEqual(summary["qualification"]["qualification_state"],
                             "not_qualified")


if __name__ == "__main__":
    unittest.main()
