# SPDX-License-Identifier: Apache-2.0
"""Adversarial inputs for the gates that decide whether evidence can ship."""
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


class ComparisonGates(unittest.TestCase):
    def compare(self, script, left, right, *arguments):
        with tempfile.TemporaryDirectory() as directory:
            a, b = Path(directory) / "a", Path(directory) / "b"
            a.write_text(left, encoding="utf-8")
            b.write_text(right, encoding="utf-8")
            interpreter = "bash" if script.endswith(".sh") else sys.executable
            return subprocess.run([interpreter, str(ROOT / "scripts" / script),
                                   str(a), str(b), *arguments], capture_output=True, text=True)

    def test_fingerprint_nonfinite_values_never_pass(self):
        for value in ("nan", "inf", "-inf", "1e400"):
            for key in ("value", "tier1.value"):
                with self.subTest(value=value, key=key):
                    prefix = "other\t1\n"
                    result = self.compare("compare-determinism.sh", prefix + f"{key}\t1\n",
                                          prefix + f"{key}\t{value}\n")
                    self.assertNotEqual(result.returncode, 0, result.stdout)

    def test_fingerprint_shape_is_checked_before_tier_exclusion(self):
        for left, right in (("", ""), ("tier1.x\t1\n", "tier1.x\t1\n"),
                            ("x\t1\nx\t1\n", "x\t1\n"),
                            ("x\t1\n", "x\t1\ntier1.extra\t1\n"),
                            ("\t1\n", "\t1\n"), ("x 1\n", "x 1\n")):
            with self.subTest(left=left, right=right):
                self.assertNotEqual(self.compare("compare-determinism.sh", left, right).returncode, 0)

    def test_fingerprint_tolerance_and_exclusion(self):
        for tolerance in ("nan", "inf", "0", "-1"):
            self.assertNotEqual(self.compare("compare-determinism.sh", "x\t1\n", "x\t1\n",
                                             tolerance).returncode, 0)
        self.assertEqual(self.compare("compare-determinism.sh", "x\t1\ntier1.y\t2\n",
                                      "x\t1.0000000001\ntier1.y\t3\n").returncode, 0)
        self.assertNotEqual(self.compare("compare-determinism.sh", "x\t1e308\n",
                                         "x\t-1e308\n").returncode, 0)

    def test_fingerprint_mode_labels_allow_internal_ascii_spaces_only(self):
        label = "modes.lateral.roll subsidence.real"
        result = self.compare("compare-determinism.sh", f"{label}\t-0.4\n",
                              f"{label}\t-0.4000000001\n")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for malformed in (" " + label, label + " ", " ", "roll\vsubsidence",
                          "roll\fsubsidence", "roll\u00a0subsidence", "roll\0subsidence",
                          "roll\x7fsubsidence"):
            with self.subTest(key=repr(malformed)):
                self.assertNotEqual(self.compare("compare-determinism.sh", f"{malformed}\t1\n",
                                                 f"{malformed}\t1\n").returncode, 0)
        self.assertNotEqual(self.compare("compare-determinism.sh", f"{label}\t1\n{label}\t1\n",
                                         f"{label}\t1\n").returncode, 0)

    def test_json_nonfinite_and_duplicate_members_never_pass(self):
        for value in ("NaN", "Infinity", "-Infinity", "1e400", "-1e400"):
            with self.subTest(value=value):
                self.assertNotEqual(self.compare("compare-run-json.py", '{"x":1}',
                                                 '{"x":' + value + '}').returncode, 0)
        self.assertNotEqual(self.compare("compare-run-json.py", '{"x":1,"x":1}',
                                         '{"x":1}').returncode, 0)

    def test_json_types_and_shapes_are_semantic(self):
        for left, right in (("true", "1"), ("false", "0"), ("null", "0"),
                            ('"1"', "1"), ("[]", "{}"), ("[1]", "[1,2]"),
                            ('{"a":1}', '{"b":1}')):
            with self.subTest(left=left, right=right):
                self.assertNotEqual(self.compare("compare-run-json.py", '{"value":' + left + '}',
                                                 '{"value":' + right + '}').returncode, 0)
        for empty in ("{}", "[]", "null", "1"):
            self.assertNotEqual(self.compare("compare-run-json.py", empty, empty).returncode, 0)

    def test_json_rounding_passes_but_a_moved_pole_fails(self):
        self.assertEqual(self.compare("compare-run-json.py", '{"pole":-0.0319000}',
                                      '{"pole":-0.0319001}').returncode, 0)
        self.assertNotEqual(self.compare("compare-run-json.py", '{"pole":-0.0319}',
                                         '{"pole":-0.0329}').returncode, 0)
        self.assertNotEqual(self.compare("compare-run-json.py", '{"x":1e308}',
                                         '{"x":-1e308}').returncode, 0)


class RequiredJobs(unittest.TestCase):
    SHA = "a" * 40

    def complete(self):
        return {name: {"result": "success", "outputs": {"source_sha": self.SHA}}
                for name in REQUIRED_JOBS}

    def result(self, needs, sha=None):
        return subprocess.run([sys.executable, str(ROOT / "scripts/check-ci-results.py")],
                              env={**os.environ, "NEEDS_JSON": json.dumps(needs),
                                   "EXPECTED_SOURCE_SHA": self.SHA if sha is None else sha},
                              capture_output=True, text=True).returncode

    def test_only_success_is_success(self):
        self.assertEqual(self.result(self.complete()), 0)
        for result in ("skipped", "cancelled", "failure", None):
            needs = self.complete()
            needs["engine"]["result"] = result
            self.assertNotEqual(self.result(needs), 0)
        for malformed in ({}, [], {"engine": {}}, {"engine": None}):
            self.assertNotEqual(self.result(malformed), 0)

    def test_missing_extra_or_wrong_source_evidence_never_passes(self):
        for name in REQUIRED_JOBS:
            missing = self.complete()
            del missing[name]
            self.assertNotEqual(self.result(missing), 0)
            for identity in (None, "", "b" * 40):
                changed = self.complete()
                changed[name]["outputs"]["source_sha"] = identity
                self.assertNotEqual(self.result(changed), 0)
        extra = self.complete()
        extra["unreviewed"] = {"result": "success", "outputs": {"source_sha": self.SHA}}
        self.assertNotEqual(self.result(extra), 0)
        for identity in ("main", "v0.3.0", "a" * 7, "", "a" * 39 + "\n"):
            self.assertNotEqual(self.result(self.complete(), identity), 0)


if __name__ == "__main__":
    unittest.main()
