#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify that target integration evidence is bound to one deployment."""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()


@unittest.skipUnless(os.name == "posix" and CLI.is_file(), "requires the POSIX project CLI")
class OnboardTargetEvidenceCLI(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-onboard-target-evidence-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.deployment = self.root / "onboard.manifest"
        self.deployment.write_text(
            "format=galata-onboard-interface-manifest-v1\n"
            "qualification_state=not_qualified\n"
            "contains_executable=false\n"
            "target_platform=bench-flight-computer\n"
            "target.hardware_id=bench-airframe-01\n"
            "target.flight_computer_id=bench-fcu-v1\n"
            "target.firmware_id=bench-firmware-001\n"
            "target.emergency_stop_id=bench-estop-01\n"
            "model_description=controlled model\n"
            "controller_description=controlled controller\n"
            "failsafe_action=disarm on link loss\n"
            "runtime.max_controller_time_s=0.005\n"
            "interface.id=bench-link-v1\n"
            "interface.sample_period_s=0.01\n"
            "interface.external_arming_required=true\n"
            "hardware.profile_id=bench-serial-v1\n"
            "hardware.transport=serial\n"
            "hardware.endpoint=/dev/controlled|115200\n"
            "hardware.receive_timeout_ms=20\n"
            "hardware.transmit_timeout_ms=20\n"
            "hardware.watchdog_timeout_s=0.02\n"
            "hardware.emergency_stop_required=true\n"
            "channel.sensor.0.name=airspeed_m_s\n"
            "channel.sensor.0.unit=m/s\n"
            "channel.sensor.0.frame=body\n"
            "channel.actuator.0.name=elevator_rad\n"
            "channel.actuator.0.unit=rad\n"
            "channel.actuator.0.frame=body\n"
            "artifact.0.role=controller\n"
            + "artifact.0.sha256=" + "a" * 64 + "\n"
            "artifact.1.role=model\n"
            + "artifact.1.sha256=" + "b" * 64 + "\n",
            encoding="utf-8",
        )
        self.deployment_sha = hashlib.sha256(self.deployment.read_bytes()).hexdigest()
        self.evidence_root = self.root / "target-evidence"
        (self.evidence_root / "evidence").mkdir(parents=True)
        roles = [
            "timing_report",
            "hardware_hil_report",
            "failsafe_report",
            "signing_record",
            "target_configuration",
        ]
        entries = []
        for role in roles:
            path = self.evidence_root / "evidence" / role
            path.write_text(role + " passed\n", encoding="utf-8")
            entries.append(
                (role, f"evidence/{role}", hashlib.sha256(path.read_bytes()).hexdigest())
            )
        lines = [
            "format=galata-target-integration-evidence-v2",
            "qualification_state=not_qualified",
            "target_acceptance_state=passed",
            "evidence_class=host_sil",
            "target.hardware_id=bench-airframe-01",
            "target.flight_computer_id=bench-fcu-v1",
            "target.firmware_id=bench-firmware-001",
            "target.emergency_stop_id=bench-estop-01",
            f"deployment_manifest_sha256={self.deployment_sha}",
            "interface.id=bench-link-v1",
            "hardware.profile_id=bench-serial-v1",
            "measurement.controller_worst_case_s=0.001",
            "measurement.cycle_worst_case_s=0.004",
            "measurement.watchdog_response_s=0.008",
            "test.emergency_stop=not_applicable",
            "test.loss_of_link=not_applicable",
            "test.hil=not_applicable",
            "signing.state=not_applicable",
        ]
        for index, (role, path, digest) in enumerate(entries):
            lines.extend(
                [
                    f"file.{index}.role={role}",
                    f"file.{index}.path={path}",
                    f"file.{index}.sha256={digest}",
                ]
            )
        self.manifest = self.evidence_root / "target-evidence.manifest"
        self.manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def invoke(self):
        return subprocess.run(
            [str(CLI), "onboard", "target", "verify", self.deployment, self.manifest],
            cwd=self.root,
            capture_output=True,
            text=True,
            timeout=30,
        )

    def test_verified_evidence_is_bound_and_tamper_is_refused(self):
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        output = json.loads(result.stdout)
        self.assertEqual(output["schema"], "galata.onboard-target-evidence-verification.v2")
        self.assertEqual(output["status"], "verified")
        self.assertEqual(output["target_acceptance_state"], "passed")
        self.assertEqual(output["evidence_class"], "host_sil")
        self.assertFalse(output["physical_tests_applicable"])
        self.assertEqual(output["qualification_state"], "not_qualified")
        self.assertEqual(output["file_count"], 5)

        (self.evidence_root / "evidence" / "timing_report").write_text(
            "tampered\n", encoding="utf-8"
        )
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("hash mismatch", result.stderr)

    def test_extra_evidence_file_is_refused(self):
        (self.evidence_root / "evidence" / "unexpected").write_text("unexpected\n", encoding="utf-8")
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("extra or missing", result.stderr)

    def test_legacy_and_unknown_provenance_are_refused(self):
        manifest = self.manifest.read_text(encoding="utf-8")
        self.manifest.write_text(
            manifest.replace("evidence_class=host_sil", "evidence_class=unknown"),
            encoding="utf-8",
        )
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)

        self.manifest.write_text(
            manifest.replace(
                "galata-target-integration-evidence-v2", "galata-target-integration-evidence-v1"
            ),
            encoding="utf-8",
        )
        result = self.invoke()
        self.assertNotEqual(result.returncode, 0)

    def test_creator_publishes_an_atomic_package_that_the_verifier_accepts(self):
        destination = self.root / "created-target-evidence"
        command = [
            str(CLI),
            "onboard",
            "target",
            "create",
            str(destination),
            str(self.deployment),
            "--evidence-class",
            "host_sil",
            "--controller-worst-case-s",
            "0.001",
            "--cycle-worst-case-s",
            "0.004",
            "--watchdog-response-s",
            "0.008",
        ]
        for role in [
            "timing_report",
            "hardware_hil_report",
            "failsafe_report",
            "signing_record",
            "target_configuration",
        ]:
            command.extend(["--file", f"{role}={self.evidence_root / 'evidence' / role}"])
        created = subprocess.run(
            command, cwd=self.root, capture_output=True, text=True, timeout=30
        )
        self.assertEqual(created.returncode, 0, created.stdout + created.stderr)
        output = json.loads(created.stdout)
        self.assertEqual(output["schema"], "galata.onboard-target-evidence-package.v2")
        self.assertEqual(output["status"], "staged")
        self.assertEqual(output["evidence_class"], "host_sil")
        self.assertFalse(output["physical_tests_applicable"])
        self.assertTrue((destination / "target-evidence.manifest").is_file())
        self.assertTrue((destination / "target-evidence.manifest.sha256").is_file())
        self.assertFalse((self.root / "created-target-evidence.staging").exists())
        created_manifest = (destination / "target-evidence.manifest").read_text(encoding="utf-8")
        self.assertIn("test.emergency_stop=not_applicable", created_manifest)
        self.assertIn("signing.state=not_applicable", created_manifest)

        verified = subprocess.run(
            [
                str(CLI),
                "onboard",
                "target",
                "verify",
                str(self.deployment),
                str(destination / "target-evidence.manifest"),
            ],
            cwd=self.root,
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(verified.returncode, 0, verified.stdout + verified.stderr)
        verified_output = json.loads(verified.stdout)
        self.assertEqual(verified_output["file_count"], 5)
        self.assertFalse(verified_output["physical_tests_applicable"])

    def test_target_hil_creator_requires_and_records_physical_passes(self):
        destination = self.root / "created-target-hil-evidence"
        command = [
            str(CLI),
            "onboard",
            "target",
            "create",
            str(destination),
            str(self.deployment),
            "--evidence-class",
            "target_hil",
            "--controller-worst-case-s",
            "0.001",
            "--cycle-worst-case-s",
            "0.004",
            "--watchdog-response-s",
            "0.008",
            "--emergency-stop-passed",
            "true",
            "--loss-of-link-passed",
            "true",
            "--hil-passed",
            "true",
            "--signing-verified",
            "true",
        ]
        for role in [
            "timing_report",
            "hardware_hil_report",
            "failsafe_report",
            "signing_record",
            "target_configuration",
        ]:
            command.extend(["--file", f"{role}={self.evidence_root / 'evidence' / role}"])
        created = subprocess.run(
            command, cwd=self.root, capture_output=True, text=True, timeout=30
        )
        self.assertEqual(created.returncode, 0, created.stdout + created.stderr)
        output = json.loads(created.stdout)
        self.assertTrue(output["physical_tests_applicable"])
        created_manifest = (destination / "target-evidence.manifest").read_text(encoding="utf-8")
        self.assertIn("test.emergency_stop=passed", created_manifest)
        self.assertIn("signing.state=verified", created_manifest)

    def test_host_sil_physical_claim_is_refused(self):
        destination = self.root / "invalid-host-sil-evidence"
        command = [
            str(CLI),
            "onboard",
            "target",
            "create",
            str(destination),
            str(self.deployment),
            "--evidence-class",
            "host_sil",
            "--controller-worst-case-s",
            "0.001",
            "--cycle-worst-case-s",
            "0.004",
            "--watchdog-response-s",
            "0.008",
            "--emergency-stop-passed",
            "true",
        ]
        for role in [
            "timing_report",
            "hardware_hil_report",
            "failsafe_report",
            "signing_record",
            "target_configuration",
        ]:
            command.extend(["--file", f"{role}={self.evidence_root / 'evidence' / role}"])
        created = subprocess.run(
            command, cwd=self.root, capture_output=True, text=True, timeout=30
        )
        self.assertNotEqual(created.returncode, 0)
        self.assertIn("host_sil omits", created.stderr)


if __name__ == "__main__":
    unittest.main()
