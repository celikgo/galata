# SPDX-License-Identifier: Apache-2.0
"""Release source snapshots retain local source edits without bundling run debris."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location("package_release", ROOT / "scripts/package-release.py")
PACKAGING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PACKAGING)
CHECK_SPEC = importlib.util.spec_from_file_location("check_release_archive", ROOT / "scripts/check-release-archive.py")
CHECKING = importlib.util.module_from_spec(CHECK_SPEC)
CHECK_SPEC.loader.exec_module(CHECKING)


class ReleasePackaging(unittest.TestCase):
    def test_shared_galata_archives_are_explicitly_unsupported(self):
        for value in ("ON", "1", "true", "Yes"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "require static Galata libraries"):
                    PACKAGING.check_archive_configuration({"BUILD_SHARED_LIBS": value})
        PACKAGING.check_archive_configuration({})
        PACKAGING.check_archive_configuration({"BUILD_SHARED_LIBS": "OFF"})

    def fixture(self, scratch):
        source = scratch / "source"
        source.mkdir()
        subprocess.run(["git", "init", "--quiet", str(source)], check=True)
        (source / ".gitignore").write_text("*.secret\n", encoding="utf-8")
        (source / "tracked.cpp").write_text("old source\n", encoding="utf-8")
        (source / "deleted.cpp").write_text("deleted source\n", encoding="utf-8")
        subprocess.run(["git", "-C", str(source), "add", "."], check=True)
        (source / "tracked.cpp").write_text("current edited source\n", encoding="utf-8")
        (source / "deleted.cpp").unlink()
        (source / "new.hpp").write_text("new uncommitted header\n", encoding="utf-8")
        (source / "local.secret").write_text("ignored local data\n", encoding="utf-8")
        (source / "build").mkdir()
        (source / "build/generated.hpp").write_text("generated\n", encoding="utf-8")
        study = source / "examples/nt33a-control-design"
        study.mkdir(parents=True)
        for filename in ("study.yaml", "README.md", "control-design.md", "linear-response.csv", "run-old.json"):
            (study / filename).write_text(filename + "\n", encoding="utf-8")
        return source

    def test_snapshot_is_current_source_including_untracked_files(self):
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory)
            source = self.fixture(scratch)
            target = scratch / "snapshot"
            metadata = PACKAGING.snapshot(source, target)
            names = {item["path"] for item in metadata["files"]}
            self.assertEqual(names, {".gitignore", "tracked.cpp", "new.hpp",
                                     "examples/nt33a-control-design/study.yaml",
                                     "examples/nt33a-control-design/README.md"})
            self.assertEqual((target / "tracked.cpp").read_text(), "current edited source\n")
            self.assertEqual(metadata["status"], "dirty")
            for item in metadata["files"]:
                self.assertEqual(item["sha256"], hashlib.sha256((target / item["path"]).read_bytes()).hexdigest())
            self.assertEqual(json.loads((target / "SOURCE-SNAPSHOT.json").read_text()), metadata)

    def test_symlink_cannot_import_files_outside_source(self):
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory)
            source = self.fixture(scratch)
            outside = scratch / "outside"
            outside.write_text("outside source\n", encoding="utf-8")
            try:
                (source / "linked.cpp").symlink_to(outside)
            except OSError:
                self.skipTest("symlink creation is unavailable on this host")
            with self.assertRaisesRegex(ValueError, "symlinks"):
                PACKAGING.snapshot(source, scratch / "snapshot")

    def test_extracted_source_does_not_inherit_an_enclosing_repository_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory)
            source = self.fixture(scratch)
            extracted = scratch / "extracted"
            original = PACKAGING.snapshot(source, extracted)
            nested = source / "downloaded source"
            shutil.copytree(extracted, nested)
            for index, archive_source in enumerate((extracted, nested)):
                with self.subTest(source=archive_source):
                    metadata = PACKAGING.snapshot(archive_source, scratch / f"repackaged-{index}")
                    self.assertEqual(metadata["commit"], "unknown")
                    self.assertEqual(metadata["status"], "unknown")
                    self.assertEqual(metadata["working_tree_changes"], [])
                    self.assertEqual(metadata["source_files_sha256"], original["source_files_sha256"])

    def test_cli_source_inventory_and_local_links_detect_omitted_files(self):
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory).resolve()
            source = self.fixture(scratch)
            (source / "README.md").write_text("[Included API](new.hpp)\n", encoding="utf-8")
            original_notices = source / "third_party/licenses"
            original_notices.mkdir(parents=True)
            (original_notices / "eigen3.txt").write_text("source reference notice\n", encoding="utf-8")
            snapshot = scratch / "snapshot"
            metadata = PACKAGING.snapshot(source, snapshot)
            stage = scratch / "cli-stage"
            shutil.copytree(snapshot, stage)
            build_notices = stage / "third_party/build-licenses"
            build_notices.mkdir()
            (build_notices / "eigen3.txt").write_text("actual resolved notice\n", encoding="utf-8")
            package_metadata = {"packaged_source": {"source_files_sha256": metadata["source_files_sha256"]}}
            CHECKING.check_source_snapshot(stage, package_metadata)
            self.assertEqual(CHECKING.check_document_links(stage), 1)
            self.assertEqual((stage / "third_party/licenses/eigen3.txt").read_text(), "source reference notice\n")
            (stage / "new.hpp").unlink()
            with self.assertRaisesRegex(ValueError, "missing, duplicate or changed"):
                CHECKING.check_source_snapshot(stage, package_metadata)
            with self.assertRaisesRegex(ValueError, "broken extracted documentation link"):
                CHECKING.check_document_links(stage)

    def test_archives_preserve_bytes_and_modes_with_repeatable_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            scratch = Path(directory)
            stage = scratch / "galata-test"
            (stage / "bin").mkdir(parents=True)
            binary = stage / "bin/galata"
            binary.write_bytes(b"exact executable bytes\n")
            binary.chmod(0o755)
            for suffix in (".tar.gz", ".zip"):
                first, second = scratch / ("first" + suffix), scratch / ("second" + suffix)
                PACKAGING.archive_tree(stage, first)
                PACKAGING.archive_tree(stage, second)
                self.assertEqual(first.read_bytes(), second.read_bytes())
                if suffix == ".tar.gz":
                    with tarfile.open(first) as archive:
                        member = archive.getmember("galata-test/bin/galata")
                        self.assertEqual(member.mode, 0o755)
                        self.assertEqual(member.mtime, 0)
                        self.assertEqual(archive.extractfile(member).read(), binary.read_bytes())
                else:
                    with zipfile.ZipFile(first) as archive:
                        self.assertEqual(archive.read("galata-test/bin/galata"), binary.read_bytes())


if __name__ == "__main__":
    unittest.main()
