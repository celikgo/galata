# SPDX-License-Identifier: Apache-2.0
"""Governance checks must cover new local source before it is committed."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class WorktreeGates(unittest.TestCase):
    def fixture(self, scratch, script):
        source = scratch / "source"
        (source / "scripts").mkdir(parents=True)
        subprocess.run(["git", "init", "--quiet", str(source)], check=True)
        shutil.copy2(ROOT / "scripts" / script, source / "scripts" / script)
        (source / "scripts/doc-references-allow.txt").write_text("# no exceptions\n", encoding="utf-8")
        (source / "README.md").write_text("Fixture documentation.\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(source), "add", "."], check=True)
        return source

    def run_gate(self, source, script):
        return subprocess.run(["bash", str(source / "scripts" / script)],
                              capture_output=True, text=True)

    def test_si_gate_rejects_a_conversion_in_an_untracked_solver(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.fixture(Path(directory), "check-si-boundary.sh")
            (source / "src/synth").mkdir(parents=True)
            (source / "src/synth/new.cpp").write_text("double converted = 0.3048 * value;\n", encoding="utf-8")
            result = self.run_gate(source, "check-si-boundary.sh")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("src/synth/new.cpp", result.stdout)
            self.assertIn("1 file(s) scanned", result.stdout)

    def test_reference_gate_reads_new_tests_and_new_documents(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.fixture(Path(directory), "check-doc-references.sh")
            (source / "tests/unit").mkdir(parents=True)
            (source / "tests/unit/new.cpp").write_text("TEST(NewSuite, NewCase) {}\n", encoding="utf-8")
            document = source / "NEW.md"
            document.write_text("Checked by `NewSuite.NewCase`.\n", encoding="utf-8")
            result = self.run_gate(source, "check-doc-references.sh")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("1 registered tests", result.stdout)
            document.write_text("Checked by `NewSuite.MissingCase`.\n", encoding="utf-8")
            result = self.run_gate(source, "check-doc-references.sh")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("NewSuite.MissingCase", result.stdout)

    def test_link_gate_reads_an_untracked_document_without_network(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.fixture(Path(directory), "check-doc-links.sh")
            (source / "NEW.md").write_text("[missing local file](absent.md)\n", encoding="utf-8")
            result = self.run_gate(source, "check-doc-links.sh")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("NEW.md points at absent.md", result.stdout)

    def test_reference_gate_reads_native_tests_and_native_source_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            source = self.fixture(Path(directory), "check-doc-references.sh")
            (source / "tests/desktop").mkdir(parents=True)
            native = source / "tests/desktop/new.mm"
            native.write_text("TEST(NativeSuite, NativeCase) {}\n", encoding="utf-8")
            document = source / "NEW.md"
            document.write_text(
                "Checked by `NativeSuite.NativeCase` in `tests/desktop/new.mm`.\n",
                encoding="utf-8")
            result = self.run_gate(source, "check-doc-references.sh")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("1 registered tests", result.stdout)
            document.write_text(
                "`NativeSuite.MissingCase` in `tests/desktop/missing.mm`.\n",
                encoding="utf-8")
            result = self.run_gate(source, "check-doc-references.sh")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("no test registered as NativeSuite.MissingCase", result.stdout)
            self.assertIn("no such file or directory: tests/desktop/missing.mm", result.stdout)


if __name__ == "__main__":
    unittest.main()
