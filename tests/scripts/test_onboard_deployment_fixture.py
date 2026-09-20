#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exercise the repository's reproducible onboard deployment fixture end to end."""

import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/ci-macos/src/cli/galata")).resolve()
MANIFEST = ROOT / "examples/onboard-deployment/onboard.manifest"
MODEL = ROOT / "examples/onboard-deployment/model-artifact.bin"
CONTROLLER = ROOT / "examples/onboard-deployment/controller-artifact.bin"
EVIDENCE = ROOT / "examples/onboard-deployment/host-sil-evidence"


@unittest.skipUnless(os.name == "posix" and CLI.is_file() and MANIFEST.is_file(),
                     "requires the POSIX project CLI and repository fixture")
class OnboardDeploymentFixture(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-onboard-fixture-")
        self.root = Path(self.temporary.name)
        self.deployment = self.root / "deployment"
        self.target = self.root / "target-evidence"

    def tearDown(self):
        self.temporary.cleanup()

    def command(self, *arguments):
        result = subprocess.run([str(CLI), *map(str, arguments)], cwd=ROOT,
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        try:
            return json.loads(result.stdout)
        except ValueError as error:
            self.fail(f"successful command must emit JSON: {error}\n{result.stdout}")

    def test_fixture_deploys_and_binds_host_sil_evidence(self):
        manifest = self.command("onboard", "verify", MANIFEST)
        self.assertEqual(manifest["status"], "verified")
        self.assertEqual(manifest["artifact_count"], 2)

        deployed = self.command(
            "onboard", "deploy", MANIFEST, self.deployment,
            "--runtime", CLI,
            "--artifact", f"model={MODEL}",
            "--artifact", f"controller={CONTROLLER}",
        )
        self.assertEqual(deployed["status"], "staged")
        self.assertTrue(deployed["contains_executable"])
        self.assertEqual(self.command("onboard", "verify-deployment", self.deployment)["status"],
                         "verified")

        created = self.command(
            "onboard", "target", "create", self.target,
            self.deployment / "onboard.manifest",
            "--evidence-class", "host_sil",
            "--controller-worst-case-s", "0.001",
            "--cycle-worst-case-s", "0.004",
            "--watchdog-response-s", "0.008",
            "--file", f"timing_report={EVIDENCE / 'timing_report'}",
            "--file", f"hardware_hil_report={EVIDENCE / 'hardware_hil_report'}",
            "--file", f"failsafe_report={EVIDENCE / 'failsafe_report'}",
            "--file", f"signing_record={EVIDENCE / 'signing_record'}",
            "--file", f"target_configuration={EVIDENCE / 'target_configuration'}",
        )
        self.assertEqual(created["status"], "staged")
        self.assertEqual(created["evidence_class"], "host_sil")
        self.assertFalse(created["physical_tests_applicable"])

        verified = self.command(
            "onboard", "target", "verify",
            self.deployment / "onboard.manifest",
            self.target / "target-evidence.manifest",
        )
        self.assertEqual(verified["status"], "verified")
        self.assertEqual(verified["evidence_class"], "host_sil")
        self.assertEqual(verified["deployment_manifest_sha256"], manifest["manifest_sha256"])
        self.assertFalse(verified["physical_tests_applicable"])


if __name__ == "__main__":
    unittest.main()
