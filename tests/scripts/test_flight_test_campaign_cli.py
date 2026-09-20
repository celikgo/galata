#SPDX - License - Identifier : Apache - 2.0
"""The flight-test campaign verifier checks package completeness and byte identity."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()


@unittest.skipUnless(os.name == "posix" and CLI.is_file(),
                     "requires the POSIX project CLI")
class FlightTestCampaignCLI(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-flight-test-cli-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "controlled records").mkdir()
        self.files = {
            "test_plan": self.root / "test plan.pdf",
            "flight_record": self.root / "controlled records" / "flight record.csv",
            "calibration_manifest": self.root / "calibration.json",
            "configuration_manifest": self.root / "configuration.json",
            "reviewer_attestation": self.root / "reviewer attestation.txt",
        }
        for role, path in self.files.items():
            path.write_bytes(f"{role}-v1\n".encode("utf-8"))
        self.manifest = self.root / "campaign.manifest"
        self.write_manifest()

    def write_manifest(self, **overrides):
        paths = {
            role: str(path.relative_to(self.root))
            for role, path in self.files.items()
        }
        paths.update(overrides.get("paths", {}))
        roles = [
            "test_plan",
            "flight_record",
            "calibration_manifest",
            "configuration_manifest",
            "reviewer_attestation",
        ]
        lines = [
            "format=galata-flight-test-evidence-v2",
            "aircraft_id=airframe-01",
            "aircraft_configuration=configuration-2026-09",
            f"evidence_class={overrides.get('evidence_class', 'synthetic_contract')}",
            "test_plan_id=FT-001",
            "reviewer_id=independent-reviewer",
            f"safety_review_complete={overrides.get('safety_review_complete', 'true')}",
        ]
        for index, role in enumerate(roles):
            path = self.root / paths[role]
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            lines.extend([
                f"file.{index}.role={role}",
                f"file.{index}.path={paths[role]}",
                f"file.{index}.sha256={digest}",
            ])
        self.manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def invoke(self, *arguments):
        return subprocess.run([str(CLI), *map(str, arguments)], cwd=self.root,
                              capture_output=True, text=True, timeout=45)

    def command(self, *arguments):
        result = self.invoke(*arguments)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        try:
            return json.loads(result.stdout)
        except ValueError as error:
            self.fail(f"successful flight-test command must emit JSON: {error}\n{result.stdout}")

    def refused(self, *arguments):
        result = self.invoke(*arguments)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((result.stdout + result.stderr).strip())

    def test_verifies_records_and_refuses_tampering(self):
        verified = self.command("flighttest", "verify", self.manifest)
        self.assertEqual(verified["schema"],
                         "galata.flight-test-evidence-verification.v2")
        self.assertEqual(verified["status"], "verified")
        self.assertEqual(verified["aircraft_id"], "airframe-01")
        self.assertEqual(verified["evidence_class"], "synthetic_contract")
        self.assertEqual(verified["file_count"], 5)
        self.assertEqual(verified["file_roles"], [
            "test_plan", "flight_record", "calibration_manifest",
            "configuration_manifest", "reviewer_attestation",
        ])
        self.assertTrue(verified["evidence_references_verified"])
        self.assertFalse(verified["airworthiness_claim"])
        self.assertFalse(verified["certification_claim"])

        self.files["flight_record"].write_bytes(b"tampered\n")
        self.refused("flighttest", "verify", self.manifest)

    def test_creates_and_verifies_an_atomic_campaign_package(self):
        destination = self.root / "created-package"
        arguments = [
            "flighttest", "create", destination,
            "--aircraft-id", "airframe-01",
            "--configuration", "configuration-2026-09",
            "--evidence-class", "synthetic_contract",
            "--test-plan-id", "FT-001",
            "--reviewer-id", "independent-reviewer",
            "--safety-review-complete", "true",
        ]
        for role, path in self.files.items():
            arguments.extend(["--file", f"{role}={path}"])

        created = self.command(*arguments)
        self.assertEqual(created["schema"], "galata.flight-test-evidence-package.v2")
        self.assertEqual(created["status"], "staged")
        self.assertEqual(created["file_count"], 5)
        self.assertTrue(created["evidence_references_verified"])
        manifest = destination / "campaign.manifest"
        self.assertTrue(manifest.is_file())
        verified = self.command("flighttest", "verify", manifest)
        self.assertEqual(verified["status"], "verified")
        self.assertEqual(verified["file_count"], 5)
        for role in self.files:
            self.assertEqual(
                (destination / "evidence" / role).read_bytes(),
                self.files[role].read_bytes(),
            )

        self.refused(*arguments)
        self.refused(
            "flighttest", "create", self.root / "unsafe-package",
            "--aircraft-id", "airframe-01",
            "--configuration", "configuration-2026-09",
            "--evidence-class", "synthetic_contract",
            "--test-plan-id", "FT-001",
            "--reviewer-id", "independent-reviewer",
            "--safety-review-complete", "false",
        )

    def test_refuses_escape_symlink_and_incomplete_safety_marker(self):
        outside = self.root.parent / "galata-flight-test-outside.txt"
        outside.write_bytes(b"outside\n")
        self.addCleanup(outside.unlink)
        self.write_manifest(paths={"test_plan": "../galata-flight-test-outside.txt"})
        self.refused("flighttest", "verify", self.manifest)

        self.write_manifest()
        linked = self.root / "linked-plan.pdf"
        linked.symlink_to(self.files["test_plan"])
        self.write_manifest(paths={"test_plan": linked.name})
        self.refused("flighttest", "verify", self.manifest)

        self.write_manifest(safety_review_complete="false")
        self.refused("flighttest", "verify", self.manifest)

        self.write_manifest(evidence_class="unclassified")
        self.refused("flighttest", "verify", self.manifest)

        self.write_manifest()
        legacy = self.manifest.read_text(encoding="utf-8").replace(
            "galata-flight-test-evidence-v2", "galata-flight-test-evidence-v1"
        )
        self.manifest.write_text(legacy, encoding="utf-8")
        self.refused("flighttest", "verify", self.manifest)


    def test_verified_campaign_package_can_drive_the_validation_gate(self):
        example = ROOT / "examples" / "fixed-wing-validation-contract"
        package = self.root / "validated-campaign"
        sources = {
            "test_plan": self.root / "approved-test-plan.txt",
            "flight_record": example / "trajectory.csv",
            "calibration_manifest": self.root / "approved-calibration.json",
            "configuration_manifest": self.root / "approved-configuration.json",
            "reviewer_attestation": self.root / "reviewer-attestation.txt",
        }
        sources["test_plan"].write_text("approved test plan\n", encoding="utf-8")
        sources["calibration_manifest"].write_text("calibration-v1\n", encoding="utf-8")
        sources["configuration_manifest"].write_text("configuration-v1\n", encoding="utf-8")
        sources["reviewer_attestation"].write_text("independent review\n", encoding="utf-8")
        arguments = [
            "flighttest", "create", package,
            "--aircraft-id", "airframe-01",
            "--configuration", "configuration-2026-09",
            "--evidence-class", "synthetic_contract",
            "--test-plan-id", "FT-001",
            "--reviewer-id", "independent-reviewer",
            "--safety-review-complete", "true",
        ]
        for role, path in sources.items():
            arguments.extend(["--file", f"{role}={path}"])
        self.command(*arguments)

        study = self.root / "campaign-validation.yaml"
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
        output = self.root / "campaign-validation-output"
        result = self.invoke("run", study, "--output-dir", output, "--json")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = (output / "validation.md").read_text(encoding="utf-8")
        self.assertIn("Flight-test evidence gate: unresolved", report)
        self.assertIn("only measured_flight provenance can satisfy", report)
        self.assertIn("Campaign package manifest SHA-256:", report)

        (package / "evidence" / "calibration_manifest").write_text(
            "tampered-calibration\n", encoding="utf-8"
        )
        self.refused("run", study, "--output-dir", output, "--json")


if __name__ == "__main__":
    unittest.main()
