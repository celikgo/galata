#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The broader-aircraft contract verifies bytes without fabricating review."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "run-aircraft-validation-contract.py"


class AircraftValidationContract(unittest.TestCase):
    def test_two_model_records_are_verified_but_not_eligible(self):
        with tempfile.TemporaryDirectory(prefix="galata-aircraft-contract-") as directory:
            result = subprocess.run(
                [sys.executable, SCRIPT, "--output-dir", Path(directory) / "contract"],
                cwd=ROOT,
                capture_output=True,
                text=True,
                timeout=60,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            summary = json.loads(result.stdout)
            verification = summary["verification"]
            self.assertEqual(summary["status"], "not_ready_by_design")
            self.assertEqual(verification["status"], "verified")
            self.assertEqual(verification["distinct_aircraft_count"], 2)
            self.assertFalse(verification["eligible"])
            self.assertFalse(verification["independent_review_complete"])
            self.assertIn("completed independent review", verification["reasons"][0])


if __name__ == "__main__":
    unittest.main()
