# SPDX-License-Identifier: Apache-2.0
"""Content identity must change for relevant edits, not for generated artifacts."""
import hashlib
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from provenance_support import inventory, inventory_digest

SPEC = importlib.util.spec_from_file_location("build_provenance", ROOT / "scripts/build-provenance.py")
BUILD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILD)


class BuildProvenance(unittest.TestCase):
    def generate(self, root, build):
        with redirect_stdout(io.StringIO()):
            BUILD.generate(root, build)

    def fixture(self, scratch):
        root = scratch / "source with spaces"
        root.mkdir()
        subprocess.run(["git", "init", "--quiet", str(root)], check=True)
        (root / ".gitignore").write_text("build/\n*.secret\n", encoding="utf-8")
        (root / "tracked source.cpp").write_text("source\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(root), "add", "."], check=True)
        return root

    def test_current_untracked_source_and_deletions_have_distinct_identities(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.fixture(Path(directory))
            first = inventory_digest(inventory(root))
            self.assertEqual(first, inventory_digest(inventory(root)))
            (root / "new header.hpp").write_text("new\n", encoding="utf-8")
            second = inventory_digest(inventory(root))
            self.assertNotEqual(first, second)
            (root / "new header.hpp").write_text("changed\n", encoding="utf-8")
            third = inventory_digest(inventory(root))
            self.assertNotEqual(second, third)
            (root / "tracked source.cpp").unlink()
            self.assertNotEqual(third, inventory_digest(inventory(root)))

    def test_generated_and_ignored_bytes_cannot_feed_back_into_source_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.fixture(Path(directory))
            first = inventory_digest(inventory(root))
            (root / "build").mkdir()
            (root / "build/generated.hpp").write_text("generated\n", encoding="utf-8")
            (root / "credentials.secret").write_text("ignored\n", encoding="utf-8")
            self.assertEqual(first, inventory_digest(inventory(root)))

    def test_example_code_and_model_data_are_bound_regardless_of_extension(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.fixture(Path(directory))
            (root / "models").mkdir()
            (root / "examples/custom").mkdir(parents=True)
            tracked = (root / "models/aerodynamic-table.csv", root / "examples/custom/block.cpp",
                       root / "examples/custom/run-reference.json")
            for path in tracked:
                path.write_text("original source or data\n", encoding="utf-8")
            subprocess.run(["git", "-C", str(root), "add", "models", "examples"], check=True)
            previous = inventory_digest(inventory(root))
            for path in tracked:
                path.write_text("changed source or data\n", encoding="utf-8")
                current = inventory_digest(inventory(root))
                self.assertNotEqual(previous, current)
                previous = current
            untracked = root / "examples/custom/study-driver.py"
            untracked.write_text("original untracked code\n", encoding="utf-8")
            current = inventory_digest(inventory(root))
            self.assertNotEqual(previous, current)
            untracked.write_text("changed untracked code\n", encoding="utf-8")
            self.assertNotEqual(current, inventory_digest(inventory(root)))

    def test_only_declared_generated_example_outputs_are_excluded(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.fixture(Path(directory))
            first = inventory_digest(inventory(root))
            study = root / "examples/nt33a-control-design"
            study.mkdir(parents=True)
            for name in ("control-design.md", "control-design.html", "linear-response.csv",
                         "nonlinear-response.csv", "run-local.json", "report.md.out"):
                (study / name).write_text("generated output\n", encoding="utf-8")
            # Legacy tracked generated reports remain excluded as documented.
            subprocess.run(["git", "-C", str(root), "add", str(study / "control-design.md")], check=True)
            self.assertEqual(first, inventory_digest(inventory(root)))
            reference = study / "reference-response.csv"
            reference.write_text("independent reference data\n", encoding="utf-8")
            self.assertNotEqual(first, inventory_digest(inventory(root)))

    def test_configuration_binds_effective_target_flags_and_serialized_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.fixture(Path(directory))
            build = root / "build"
            build.mkdir()
            compiler = Path(directory) / "compiler with spaces"
            compiler.write_bytes(b"compiler identity")
            cache = (f"CMAKE_CXX_COMPILER:FILEPATH={compiler}\n"
                     "CMAKE_BUILD_TYPE:STRING=Debug\nCMAKE_CXX_FLAGS:STRING=-fno-fast-math\n"
                     "UNRELATED_SECRET:STRING=must-not-appear\n")
            (build / "CMakeCache.txt").write_text(cache, encoding="utf-8")
            command = {"directory": str(build), "file": str(root / "tracked source.cpp"),
                       "command": '"compiler with spaces" -ffp-contract=off -c "tracked source.cpp"'}
            (build / "compile_commands.json").write_text(json.dumps([command]), encoding="utf-8")
            self.generate(root, build)
            first = (build / "galata-build-configuration.json").read_bytes()
            source_first = (build / "galata-source-inventory.json").read_bytes()
            configuration = json.loads(first)
            self.assertNotIn(b"must-not-appear", first)
            self.assertEqual(configuration["compile_commands_sha256"], hashlib.sha256(
                (build / "galata-compile-commands.json").read_bytes()).hexdigest())
            self.generate(root, build)
            self.assertEqual(first, (build / "galata-build-configuration.json").read_bytes())
            self.assertEqual(source_first, (build / "galata-source-inventory.json").read_bytes())
            command["command"] += " -DCHANGED_TARGET_BEHAVIOR=1"
            (build / "compile_commands.json").write_text(json.dumps([command]), encoding="utf-8")
            self.generate(root, build)
            self.assertNotEqual(first, (build / "galata-build-configuration.json").read_bytes())
            self.assertEqual(source_first, (build / "galata-source-inventory.json").read_bytes())


if __name__ == "__main__":
    unittest.main()
