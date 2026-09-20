#!/ usr / bin / env python3
#SPDX - License - Identifier : Apache - 2.0
"""The qualification dossier verifier checks completeness without making an approval claim."""
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
class QualificationDossierCLI(unittest.TestCase):
    ROLES = (
        "requirements_matrix",
        "software_release",
        "verification_report",
        "flight_test_campaign",
        "hardware_hil_report",
        "safety_case",
        "independent_review",
        "authority_decision",
        "maintenance_plan",
    )

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-qualification-cli-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.files = {}
        for role in self.ROLES:
            path = self.root / "evidence" / role / "record.txt"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((role + "-v1\n").encode("utf-8"))
            self.files[role] = path
        self.manifest = self.root / "qualification.manifest"
        self.write_manifest()

    def write_manifest(self, acceptance_state="not_qualified", roles=None):
        selected = self.ROLES if roles is None else tuple(roles)
        lines = [
            "format=galata-qualification-evidence-v1",
            "product_id=galata",
            "product_version=0.3.0",
            "intended_use=controlled engineering review",
            "aircraft_id=airframe-01",
            "aircraft_configuration=configuration-2026-09",
            "qualification_basis=application-specific review basis",
            "authority_id=external-authority-pending",
            f"acceptance_state={acceptance_state}",
        ]
        for index, role in enumerate(selected):
            path = self.files[role]
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            lines.extend([
                f"file.{index}.role={role}",
                f"file.{index}.path={path.relative_to(self.root)}",
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
            self.fail(f"successful qualification command must emit JSON: {error}\n{result.stdout}")

    def refused(self, *arguments):
        result = self.invoke(*arguments)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((result.stdout + result.stderr).strip())

    def test_verifies_complete_dossier_without_claiming_acceptance(self):
        verified = self.command("qualification", "verify", self.manifest)
        self.assertEqual(verified["schema"],
                         "galata.qualification-evidence-verification.v1")
        self.assertEqual(verified["status"], "verified")
        self.assertTrue(verified["evidence_package_complete"])
        self.assertEqual(verified["qualification_state"], "not_qualified")
        self.assertTrue(verified["external_authority_acceptance_required"])
        self.assertFalse(verified["airworthiness_claim"])
        self.assertFalse(verified["certification_claim"])
        self.assertEqual(verified["file_roles"], list(self.ROLES))

    def test_creates_and_verifies_an_atomic_dossier_package(self):
        destination = self.root / "assembled-dossier"
        arguments = [
            "qualification", "create", destination,
            "--product-id", "galata",
            "--product-version", "0.3.0",
            "--intended-use", "controlled engineering review",
            "--aircraft-id", "airframe-01",
            "--configuration", "configuration-2026-09",
            "--qualification-basis", "application-specific review basis",
            "--authority-id", "external-authority-pending",
        ]
        for role in self.ROLES:
            arguments.extend(("--file", f"{role}={self.files[role]}"))

        staged = self.command(*arguments)
        self.assertEqual(staged["schema"], "galata.qualification-evidence-package.v1")
        self.assertEqual(staged["status"], "staged")
        self.assertTrue(staged["evidence_references_verified"])
        self.assertEqual(staged["qualification_state"], "not_qualified")
        self.assertFalse(staged["airworthiness_claim"])
        self.assertTrue((destination / "qualification.manifest").is_file())
        for role in self.ROLES:
            self.assertEqual((destination / "evidence" / role).read_bytes(),
                             self.files[role].read_bytes())

        verified = self.command("qualification", "verify", destination / "qualification.manifest")
        self.assertEqual(verified["status"], "verified")
        self.assertTrue(verified["evidence_package_complete"])
        self.assertEqual(verified["file_roles"], list(self.ROLES))
        self.refused(*arguments)

    def test_creator_requires_all_roles_and_preserves_staging_atomicity(self):
        destination = self.root / "incomplete-dossier"
        arguments = [
            "qualification", "create", destination,
            "--product-id", "galata",
            "--product-version", "0.3.0",
            "--intended-use", "controlled engineering review",
            "--aircraft-id", "airframe-01",
            "--configuration", "configuration-2026-09",
            "--qualification-basis", "application-specific review basis",
            "--authority-id", "external-authority-pending",
        ]
        for role in self.ROLES[:-1]:
            arguments.extend(("--file", f"{role}={self.files[role]}"))
        self.refused(*arguments)
        self.assertFalse(destination.exists())
        self.assertFalse(Path(str(destination) + ".staging").exists())

    def test_refuses_tampering_missing_role_qualified_marker_and_symlink(self):
        self.files["hardware_hil_report"].write_bytes(b"changed\n")
        self.refused("qualification", "verify", self.manifest)

        self.write_manifest()
        self.write_manifest(roles=self.ROLES[:-1])
        self.refused("qualification", "verify", self.manifest)

        self.write_manifest()
        self.write_manifest(acceptance_state="qualified")
        self.refused("qualification", "verify", self.manifest)

        self.write_manifest()
        target = self.files["safety_case"]
        replacement = self.root / "outside-safety-case.txt"
        replacement.write_bytes(target.read_bytes())
        target.unlink()
        target.symlink_to(replacement)
        self.refused("qualification", "verify", self.manifest)

    def test_verify_chain_binds_dossier_flight_campaign_and_target_evidence(self):
        campaign_sources = {}
        campaign_root = self.root / "campaign-sources"
        for role in (
            "test_plan",
            "flight_record",
            "calibration_manifest",
            "configuration_manifest",
            "reviewer_attestation",
        ):
            path = campaign_root / role
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(role + " controlled\n", encoding="utf-8")
            campaign_sources[role] = path
        campaign_package = self.root / "campaign-package"
        campaign_arguments = [
            "flighttest", "create", campaign_package,
            "--aircraft-id", "airframe-01",
            "--configuration", "configuration-2026-09",
            "--evidence-class", "synthetic_contract",
            "--test-plan-id", "FT-CHAIN-001",
            "--reviewer-id", "independent-reviewer",
            "--safety-review-complete", "true",
        ]
        for role, path in campaign_sources.items():
            campaign_arguments.extend(("--file", f"{role}={path}"))
        self.command(*campaign_arguments)
        campaign_manifest = campaign_package / "campaign.manifest"
        validation_receipt = self.root / "flight-validation-receipt.json"
        validation_receipt.write_text(json.dumps({
            "schema": "galata.flight-test-validation-receipt.v1",
            "status": "not_ready",
            "campaign_manifest": str(campaign_manifest.resolve()),
            "campaign_manifest_sha256": hashlib.sha256(
                campaign_manifest.read_bytes()).hexdigest(),
            "evidence_class": "synthetic_contract",
            "campaign_binding_verified": True,
            "run_inputs_verified": True,
            "numerical_acceptance_gate": "pass",
            "flight_test_evidence_gate": "unresolved",
        }) + "\n", encoding="utf-8")

        model_artifact = self.root / "chain-model.bin"
        controller_artifact = self.root / "chain-controller.bin"
        model_artifact.write_bytes(b"chain model artifact\n")
        controller_artifact.write_bytes(b"chain controller artifact\n")
        model_sha = hashlib.sha256(model_artifact.read_bytes()).hexdigest()
        controller_sha = hashlib.sha256(controller_artifact.read_bytes()).hexdigest()
        deployment = self.root / "chain-onboard.manifest"
        deployment.write_text(
            "format=galata-onboard-interface-manifest-v1\n"
            "qualification_state=not_qualified\n"
            "contains_executable=false\n"
            "target_platform=bench-flight-computer\n"
            "target.hardware_id=chain-airframe-01\n"
            "target.flight_computer_id=chain-fcu-v1\n"
            "target.firmware_id=chain-firmware-001\n"
            "target.emergency_stop_id=chain-estop-01\n"
            "model_description=controlled model\n"
            "controller_description=controlled controller\n"
            "failsafe_action=disarm on link loss\n"
            "runtime.max_controller_time_s=0.005\n"
            "interface.id=chain-link-v1\n"
            "interface.sample_period_s=0.01\n"
            "interface.external_arming_required=true\n"
            "hardware.profile_id=chain-serial-v1\n"
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
            + "artifact.0.sha256=" + controller_sha + "\n"
            + "artifact.1.role=model\n"
            + "artifact.1.sha256=" + model_sha + "\n",
            encoding="utf-8",
        )
        deployment_directory = self.root / "chain-deployment"
        self.command(
            "onboard", "deploy", deployment, deployment_directory,
            "--runtime", CLI,
            "--artifact", f"model={model_artifact}",
            "--artifact", f"controller={controller_artifact}",
        )
        target_sources = {}
        target_source_root = self.root / "target-sources"
        for role in (
            "timing_report",
            "hardware_hil_report",
            "failsafe_report",
            "signing_record",
            "target_configuration",
        ):
            path = target_source_root / role
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(role + " passed\n", encoding="utf-8")
            target_sources[role] = path
        target_package = self.root / "target-package"
        target_arguments = [
            "onboard", "target", "create", target_package, deployment,
            "--evidence-class", "host_sil",
            "--controller-worst-case-s", "0.001",
            "--cycle-worst-case-s", "0.004",
            "--watchdog-response-s", "0.008",
        ]
        for role, path in target_sources.items():
            target_arguments.extend(("--file", f"{role}={path}"))
        self.command(*target_arguments)

        dossier_sources = dict(self.files)
        dossier_sources["flight_test_campaign"] = campaign_package / "campaign.manifest"
        dossier_sources["hardware_hil_report"] = target_package / "target-evidence.manifest"
        dossier_package = self.root / "chain-dossier"
        dossier_arguments = [
            "qualification", "create", dossier_package,
            "--product-id", "galata",
            "--product-version", "0.3.0",
            "--intended-use", "controlled engineering review",
            "--aircraft-id", "airframe-01",
            "--configuration", "configuration-2026-09",
            "--qualification-basis", "application-specific review basis",
            "--authority-id", "external-authority-pending",
        ]
        for role in self.ROLES:
            dossier_arguments.extend(("--file", f"{role}={dossier_sources[role]}"))
        self.command(*dossier_arguments)

        verified = self.command(
            "qualification", "verify-chain", dossier_package / "qualification.manifest",
            "--flighttest", campaign_manifest,
            "--flight-validation-receipt", validation_receipt,
            "--deployment", deployment,
            "--deployment-dir", deployment_directory,
            "--target-evidence", target_package / "target-evidence.manifest",
        )
        self.assertEqual(verified["schema"], "galata.qualification-chain-verification.v1")
        self.assertEqual(verified["status"], "verified")
        self.assertTrue(verified["dossier_evidence_verified"])
        self.assertTrue(verified["flight_test_evidence_verified"])
        self.assertFalse(verified["flight_validation_receipt_verified"])
        self.assertTrue(verified["target_evidence_verified"])
        self.assertTrue(verified["traceability_links_verified"])
        self.assertTrue(verified["deployment_runtime_verified"])
        self.assertEqual(
            verified["deployment_runtime_sha256"],
            hashlib.sha256((deployment_directory / "bin" / "galata").read_bytes()).hexdigest(),
        )
        self.assertFalse(verified["flight_test_eligibility"])
        self.assertFalse(verified["target_hardware_eligibility"])
        self.assertFalse(verified["authority_decision_eligibility"])
        self.assertEqual(verified["qualification_eligibility"], "not_ready")
        self.assertEqual(
            verified["qualification_eligibility_reasons"],
            [
                "flight_test_requires_measured_flight_evidence_and_completed_safety_review",
                "flight_validation_receipt_must_be_gate_passed_and_bound_to_the_campaign",
                "target_hardware_requires_target_hil_or_flight_target_evidence",
                "external_authority_decision_is_not_identified",
            ],
        )
        self.assertEqual(verified["qualification_state"], "not_qualified")
        self.assertFalse(verified["airworthiness_claim"])
        self.assertFalse(verified["certification_claim"])

        (target_package / "evidence" / "timing_report").write_text(
            "tampered timing\n", encoding="utf-8"
        )
        self.refused(
            "qualification", "verify-chain", dossier_package / "qualification.manifest",
            "--flighttest", campaign_manifest,
            "--flight-validation-receipt", validation_receipt,
            "--deployment", deployment,
            "--deployment-dir", deployment_directory,
            "--target-evidence", target_package / "target-evidence.manifest",
        )


if __name__ == "__main__":
    unittest.main()
