#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Tests for the controlled broader-aircraft evidence boundary."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "check-aircraft-validation.py"
_SPEC = importlib.util.spec_from_file_location("galata_aircraft_validation", SCRIPT)
CHECK = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
_SPEC.loader.exec_module(CHECK)


class AircraftValidationEvidence(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="galata-aircraft-evidence-")
        self.root = Path(self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def add_file(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        return {
            "path": name,
            "sha256": hashlib.sha256(content.encode()).hexdigest(),
        }

    def record(self, aircraft_id, configuration_id):
        prefix = f"evidence/{aircraft_id}/{configuration_id}"
        return {
            "aircraft_id": aircraft_id,
            "configuration_id": configuration_id,
            "model": self.add_file(prefix + "/model.json", "model\n"),
            "calibration": self.add_file(prefix + "/calibration.json", "calibration\n"),
            "envelope": self.add_file(prefix + "/envelope.json", "envelope\n"),
            "validation_report": self.add_file(prefix + "/validation-report.json", "report\n"),
            "review_record": self.add_file(prefix + "/review.json", "review\n"),
            "independent_review_complete": True,
        }

    def write_manifest(self, records):
        path = self.root / "aircraft-validation.manifest"
        path.write_text(json.dumps({
            "schema": "galata.aircraft-validation-evidence.v1",
            "status": "complete",
            "evidence_class": "independent_validation",
            "aircraft": records,
        }, indent=2) + "\n", encoding="utf-8")
        return path

    def test_two_distinct_aircraft_are_eligible(self):
        manifest = self.write_manifest([
            self.record("cessna-172", "cruise-v1"),
            self.record("uh-60a", "level-1-v1"),
        ])
        result = CHECK.verify(manifest)
        self.assertEqual(result["status"], "verified")
        self.assertTrue(result["eligible"])
        self.assertEqual(result["distinct_aircraft_count"], 2)
        self.assertEqual(result["aircraft_count"], 2)
        completed = subprocess.run(
            [sys.executable, SCRIPT, manifest], capture_output=True, text=True, check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertTrue(json.loads(completed.stdout)["eligible"])

    def test_one_aircraft_is_verified_but_not_broader_scope_eligible(self):
        result = CHECK.verify(self.write_manifest([self.record("cessna-172", "cruise-v1")]))
        self.assertEqual(result["status"], "verified")
        self.assertFalse(result["eligible"])
        self.assertIn("at least two distinct aircraft", result["reasons"][0])

    def test_tampering_and_escape_are_refused(self):
        records = [self.record("cessna-172", "cruise-v1"), self.record("uh-60a", "level-1-v1")]
        manifest = self.write_manifest(records)
        (self.root / "evidence/cessna-172/cruise-v1/model.json").write_text(
            "tampered\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            CHECK.verify(manifest)

        records = [self.record("cessna-172", "cruise-v1"), self.record("uh-60a", "level-1-v1")]
        records[0]["model"] = {"path": "../outside.json", "sha256": "0" * 64}
        manifest = self.write_manifest(records)
        with self.assertRaisesRegex(ValueError, "relative POSIX|unsafe"):
            CHECK.verify(manifest)

    def test_symlinked_evidence_is_refused(self):
        records = [self.record("cessna-172", "cruise-v1"), self.record("uh-60a", "level-1-v1")]
        linked = self.root / "evidence/uh-60a/level-1-v1/review.json"
        replacement = self.root / "replacement-review.json"
        replacement.write_text("review\n", encoding="utf-8")
        linked.unlink()
        linked.symlink_to(replacement)
        records[1]["review_record"]["sha256"] = hashlib.sha256(
            replacement.read_bytes()).hexdigest()
        manifest = self.write_manifest(records)
        with self.assertRaisesRegex(ValueError, "symlink"):
            CHECK.verify(manifest)

    def test_symlinked_manifest_is_refused(self):
        manifest = self.write_manifest([
            self.record("cessna-172", "cruise-v1"),
            self.record("uh-60a", "level-1-v1"),
        ])
        linked = self.root / "linked.manifest"
        linked.symlink_to(manifest)
        with self.assertRaisesRegex(ValueError, "symlink"):
            CHECK.verify(linked)


if __name__ == "__main__":
    unittest.main()
