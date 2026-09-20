# SPDX-License-Identifier: Apache-2.0
"""The target-neutral onboard handoff CLI must verify and stage atomically."""
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
class OnboardCLI(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-onboard-cli-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.model = self.root / "model artifact.bin"
        self.controller = self.root / "controller artifact.bin"
        self.model.write_bytes(b"model-artifact-v1\n")
        self.controller.write_bytes(b"controller-artifact-v1\n")
        model_sha = hashlib.sha256(self.model.read_bytes()).hexdigest()
        controller_sha = hashlib.sha256(self.controller.read_bytes()).hexdigest()
        self.manifest = self.root / "onboard.manifest"
        self.manifest.write_text(
            "format=galata-onboard-interface-manifest-v1\n"
            "qualification_state=not_qualified\n"
            "contains_executable=false\n"
            "target_platform=example-flight-computer\n"
            "target.hardware_id=airframe-01\n"
            "target.flight_computer_id=fcu-example-v1\n"
            "target.firmware_id=firmware-build-001\n"
            "target.emergency_stop_id=estop-chain-01\n"
            "model_description=controlled model\n"
            "controller_description=controlled controller\n"
            "failsafe_action=disarm on link loss\n"
            "runtime.max_controller_time_s=0.005\n"
            "interface.id=example-link-v1\n"
            "interface.sample_period_s=0.01\n"
            "interface.external_arming_required=true\n"
            "hardware.profile_id=example-sil-profile-v1\n"
            "hardware.transport=replay\n"
            "hardware.endpoint=reviewed-sil-replay\n"
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
            f"artifact.0.role=controller\nartifact.0.sha256={controller_sha}\n"
            f"artifact.1.role=model\nartifact.1.sha256={model_sha}\n",
            encoding="utf-8",
        )

    def invoke(self, *arguments):
        return subprocess.run([str(CLI), *map(str, arguments)], cwd=self.root,
                              capture_output=True, text=True, timeout=45)

    def command(self, *arguments):
        result = self.invoke(*arguments)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        try:
            return json.loads(result.stdout)
        except ValueError as error:
            self.fail(f"successful onboard command must emit JSON: {error}\n{result.stdout}")

    def refused(self, *arguments):
        result = self.invoke(*arguments)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((result.stdout + result.stderr).strip())

    def test_verify_and_stage_refuse_tampering(self):
        verified = self.command("onboard", "verify", self.manifest)
        self.assertEqual(verified["schema"], "galata.onboard-verification.v1")
        self.assertEqual(verified["status"], "verified")
        self.assertEqual(verified["artifact_count"], 2)
        self.assertEqual(verified["artifact_roles"], ["controller", "model"])
        self.assertEqual(verified["transport_profile"]["transport"], "replay")
        self.assertEqual(verified["transport_profile"]["endpoint"], "reviewed-sil-replay")
        self.assertEqual(verified["transport_profile"]["watchdog_timeout_s"], 0.02)
        self.assertEqual(verified["target_identity"]["hardware_id"], "airframe-01")
        self.assertEqual(verified["target_identity"]["flight_computer_id"], "fcu-example-v1")
        self.assertFalse(verified["contains_executable"])

        destination = self.root / "staged package"
        staged = self.command(
            "onboard", "stage", self.manifest, destination,
            "--artifact", f"model={self.model}",
            "--artifact", f"controller={self.controller}",
        )
        self.assertEqual(staged["schema"], "galata.onboard-stage.v1")
        self.assertEqual(staged["status"], "staged")
        self.assertEqual(staged["artifact_count"], 2)
        self.assertEqual(staged["transport_profile"]["id"], "example-sil-profile-v1")
        self.assertEqual((destination / "artifacts" / "model").read_bytes(), self.model.read_bytes())
        self.assertEqual((destination / "artifacts" / "controller").read_bytes(),
                         self.controller.read_bytes())
        self.assertTrue((destination / "onboard.manifest.sha256").is_file())

        failed_destination = self.root / "failed staging"
        self.controller.write_bytes(b"tampered\n")
        self.refused(
            "onboard", "stage", self.manifest, failed_destination,
            "--artifact", f"model={self.model}",
            "--artifact", f"controller={self.controller}",
        )
        self.assertFalse(failed_destination.exists())

        self.manifest.write_text(
            self.manifest.read_text(encoding="utf-8").replace(
                "qualification_state=not_qualified\n", "qualification_state=qualified\n"
            ),
            encoding="utf-8",
        )
        self.refused("onboard", "verify", self.manifest)

    def test_replay_self_test_is_explicitly_not_a_target_deployment(self):
        result = self.command("onboard", "self-test")
        self.assertEqual(result["schema"], "galata.onboard-self-test.v1")
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["completed_cycles"], 3)
        self.assertEqual(result["observed_cycles"], 3)
        self.assertGreaterEqual(result["observed_controller_worst_case_s"], 0.0)
        self.assertGreaterEqual(result["observed_cycle_worst_case_s"],
                                result["observed_controller_worst_case_s"])
        self.assertEqual(result["transport"], "replay")
        self.assertFalse(result["target_executable"])
        self.assertEqual(result["qualification_state"], "not_qualified")
        self.assertFalse(result["hardware_timing_claim"])
        self.assertFalse(result["airworthiness_claim"])


if __name__ == "__main__":
    unittest.main()
