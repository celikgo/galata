# SPDX-License-Identifier: Apache-2.0
"""A tag, CI record and downloadable packages must identify the same source."""
from contextlib import redirect_stdout
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from ci_evidence import REQUIRED_JOBS
from provenance_support import digest

SPEC = importlib.util.spec_from_file_location("release_evidence", ROOT / "scripts/check-release-evidence.py")
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


class ReleaseEvidence(unittest.TestCase):
    SHA = "a" * 40

    def fixture(self, directory):
        evidence = {"schema": "galata.ci-evidence.v1", "source_sha": self.SHA,
                    "jobs": {name: {"result": "success", "outputs": {"source_sha": self.SHA}}
                             for name in REQUIRED_JOBS}}
        for platform in ("linux-x86_64", "macos-arm64", "windows-x86_64"):
            metadata = {"platform": platform, "ci_evidence": evidence, "archive_smoke": "passed",
                        "build_source": {"commit": self.SHA, "status": "clean", "source_tree_sha256": "b" * 64},
                        "packaged_source": {"commit": self.SHA, "status": "clean", "source_files_sha256": "b" * 64}}
            for key in ("cli_archive", "source_archive"):
                archive = directory / (platform + "-" + key + ".tar.gz")
                archive.write_bytes((platform + key).encode())
                metadata[key] = {"file": archive.name, "sha256": digest(archive)}
            (directory / (platform + ".package.json")).write_text(json.dumps(metadata), encoding="utf-8")
        return evidence

    def test_all_three_packages_need_matching_evidence_and_archive_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            evidence = self.fixture(directory)
            with redirect_stdout(io.StringIO()):
                CHECK.check(directory, evidence, self.SHA)
            for field, value in (("commit", "c" * 40), ("status", "dirty"), ("source_tree_sha256", "c" * 64)):
                path = directory / "linux-x86_64.package.json"
                baseline = path.read_text()
                metadata = json.loads(baseline)
                metadata["build_source"][field] = value
                path.write_text(json.dumps(metadata))
                with self.assertRaises(ValueError):
                    CHECK.check(directory, evidence, self.SHA)
                path.write_text(baseline)
            wrong = copy.deepcopy(evidence)
            wrong["jobs"]["sanitizer"]["result"] = "skipped"
            with self.assertRaises(ValueError):
                CHECK.check(directory, wrong, self.SHA)
            (directory / "linux-x86_64-cli_archive.tar.gz").write_bytes(b"changed archive")
            with self.assertRaisesRegex(ValueError, "archive identity"):
                CHECK.check(directory, evidence, self.SHA)

    def test_missing_platform_cannot_be_replaced_by_another_platform(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            evidence = self.fixture(directory)
            path = directory / "linux-x86_64.package.json"
            metadata = json.loads(path.read_text())
            metadata["platform"] = "macos-arm64"
            path.write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError, "platform"):
                CHECK.check(directory, evidence, self.SHA)
            path.unlink()
            with self.assertRaisesRegex(ValueError, "exactly one"):
                CHECK.check(directory, evidence, self.SHA)


class TagIdentity(unittest.TestCase):
    def git(self, directory, *arguments):
        return subprocess.run(["git", "-C", str(directory), "-c", "user.name=Fixture",
                               "-c", "user.email=fixture@example.invalid", "-c", "commit.gpgsign=false",
                               "-c", "tag.gpgsign=false", *arguments], check=True, capture_output=True, text=True).stdout.strip()

    def test_annotated_tag_resolves_once_and_tag_movement_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self.git(directory, "init", "--quiet")
            (directory / "VERSION").write_text("0.3.0\n")
            self.git(directory, "add", "VERSION")
            self.git(directory, "commit", "--quiet", "-m", "fixture")
            expected = self.git(directory, "rev-parse", "HEAD")
            self.git(directory, "tag", "-a", "v0.3.0", "-m", "annotated fixture")
            output = directory / "outputs"
            command = [sys.executable, str(ROOT / "scripts/resolve-release.py"), "v0.3.0", "--expected-sha", expected]
            first = subprocess.run(command, cwd=directory, env={**os.environ, "GITHUB_OUTPUT": str(output)},
                                   capture_output=True, text=True)
            self.assertEqual(first.returncode, 0, first.stderr)
            self.assertIn("source_sha=" + expected, output.read_text())
            (directory / "another.cpp").write_text("changed source\n")
            self.git(directory, "add", "another.cpp")
            self.git(directory, "commit", "--quiet", "-m", "changed")
            self.git(directory, "tag", "--force", "v0.3.0")
            moved = subprocess.run(command, cwd=directory, capture_output=True, text=True)
            self.assertNotEqual(moved.returncode, 0)
            self.assertIn("tag moved", moved.stderr)


if __name__ == "__main__":
    unittest.main()
