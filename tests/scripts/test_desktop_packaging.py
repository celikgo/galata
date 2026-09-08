# SPDX-License-Identifier: Apache-2.0
"""Desktop archive boundary tests using synthetic files, never trusted executables.

The public contract is a local-only candidate with a complete byte/mode inventory,
one matching source/configuration identity, safe extraction and system-only macOS
loader paths. These tests do not launch the synthetic app or claim runtime QA.
"""
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import plistlib
import sys
import tarfile
import tempfile
import unittest
import zipfile


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location("desktop_check", ROOT / "scripts/check-desktop-package.py")
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


def digest_bytes(data):
    return hashlib.sha256(data).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")


class DesktopPackaging(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-desktop-contract-")
        self.addCleanup(self.temporary.cleanup)
        self.scratch = Path(self.temporary.name)

    def fixture(self):
        stage = self.scratch / "candidate"
        resources = stage / "Galata Preview.app/Contents/Resources"
        (resources / "licenses").mkdir(parents=True)
        executables = stage / "Galata Preview.app/Contents/MacOS"
        executables.mkdir()
        for name in ("Galata Preview", "galata"):
            path = executables / name
            path.write_bytes(b"synthetic, never executable\n" + name.encode())
            path.chmod(0o755)
        plist = {"CFBundleExecutable": "Galata Preview", "CFBundleIdentifier": "org.galata.desktop.preview",
                 "CFBundleShortVersionString": "0.3.0", "CFBundleVersion": "0.3.0",
                 "LSMinimumSystemVersion": "26.0"}
        (stage / "Galata Preview.app/Contents/Info.plist").write_bytes(plistlib.dumps(plist))
        for name in ("LICENSE", "NOTICE", "THIRD_PARTY_LICENSES.md", "START_HERE.txt"):
            (resources / name).write_text("synthetic " + name, encoding="utf-8")
        dependencies = []
        for name in ("eigen3", "yaml-cpp"):
            data = ("synthetic notice " + name).encode()
            (resources / "licenses" / (name + ".txt")).write_bytes(data)
            dependencies.append({"name": name, "notice": name + ".txt", "notice_sha256": digest_bytes(data)})
        write_json(resources / "licenses/dependencies.json", dependencies)
        source_bytes = b"independent synthetic source\n"
        source_records = [{"path": "source.cpp", "sha256": digest_bytes(source_bytes), "executable": False}]
        snapshot = {"files": source_records, "source_files_sha256": CHECK.inventory_digest(source_records),
                    "commit": "1" * 40, "status": "dirty"}
        write_json(stage / "SOURCE-SNAPSHOT.json", snapshot)
        (stage / "source").mkdir()
        source_archive = stage / "source/source.tar.gz"
        with tarfile.open(source_archive, "w:gz") as archive:
            for name, data in (("snapshot/source.cpp", source_bytes),
                               ("snapshot/SOURCE-SNAPSHOT.json", json.dumps(snapshot).encode())):
                member = tarfile.TarInfo(name)
                member.size = len(data)
                member.mode = 0o644
                archive.addfile(member, io.BytesIO(data))
        evidence = stage / "build-evidence"
        evidence.mkdir()
        write_json(evidence / "galata-compile-commands.json", [{"file": "/synthetic/src/desktop/main.mm"}])
        configuration = {"cache": {"GALATA_BUILD_DESKTOP": "ON", "CMAKE_BUILD_TYPE": "RelWithDebInfo"},
                         "compile_commands_sha256": CHECK.digest(evidence / "galata-compile-commands.json")}
        write_json(evidence / "galata-build-configuration.json", configuration)
        metadata = {"schema": "galata.desktop-package.v1", "channel": "local-candidate", "version": "0.3.0",
                    "platform": "macos-arm64", "minimum_macos": "26.0", "build_type": "RelWithDebInfo",
                    "distribution": {"signing": "ad-hoc", "notarization": "not-submitted",
                                     "ci_approval": "not-claimed", "stable_release": False},
                    "ui_sha256": CHECK.digest(executables / "Galata Preview"),
                    "built_ui_sha256": CHECK.digest(executables / "Galata Preview"),
                    "worker_sha256": CHECK.digest(executables / "galata"),
                    "built_cli_sha256": CHECK.digest(executables / "galata"),
                    "build_source": {"commit": snapshot["commit"], "status": snapshot["status"],
                                     "source_tree_sha256": snapshot["source_files_sha256"]},
                    "build_configuration": {"file": "build-evidence/galata-build-configuration.json",
                                            "sha256": CHECK.digest(evidence / "galata-build-configuration.json")},
                    "source_archive": {"file": "source/source.tar.gz", "sha256": CHECK.digest(source_archive)}}
        write_json(evidence / "galata-desktop-build.json", {
            "schema": "galata.desktop-build-stamp.v1", "source_tree_sha256": snapshot["source_files_sha256"],
            "configuration_sha256": metadata["build_configuration"]["sha256"],
            "ui_sha256": metadata["built_ui_sha256"], "worker_sha256": metadata["built_cli_sha256"]})
        self.refresh(stage, metadata)
        return stage, metadata

    def refresh(self, stage, metadata):
        metadata["files"] = CHECK.file_inventory(stage)
        metadata["files_sha256"] = CHECK.inventory_digest(metadata["files"])
        write_json(stage / "PACKAGE.json", metadata)

    def zip(self, path, entries):
        with zipfile.ZipFile(path, "w") as archive:
            for name, data, mode in entries:
                entry = zipfile.ZipInfo(name)
                entry.create_system = 3
                entry.external_attr = mode << 16
                archive.writestr(entry, data)

    def test_complete_candidate_round_trip_preserves_bytes_modes_and_relocation(self):
        stage, metadata = self.fixture()
        self.assertEqual(CHECK.check_stage(stage), metadata)
        archive = self.scratch / "candidate.zip"
        self.zip(archive, [("candidate/" + path.relative_to(stage).as_posix(), path.read_bytes(),
                            0o100755 if path.stat().st_mode & 0o111 else 0o100644)
                           for path in sorted(stage.rglob("*")) if path.is_file()])
        evidence = CHECK.check(archive, metadata_only=True)
        self.assertEqual(evidence["inventory"], "passed")
        self.assertEqual(evidence["runtime"], "not-run")
        self.assertEqual(evidence["archive_sha256"], CHECK.digest(archive))

    def test_missing_extra_changed_bytes_and_lost_executable_mode_are_refused(self):
        stage, _ = self.fixture()
        worker = stage / "Galata Preview.app/Contents/MacOS/galata"
        original = worker.read_bytes()
        for mutation in ("missing", "extra", "changed", "mode"):
            with self.subTest(mutation=mutation):
                if mutation == "missing":
                    worker.unlink()
                elif mutation == "extra":
                    (stage / "unexpected.txt").write_text("unexpected", encoding="utf-8")
                elif mutation == "changed":
                    worker.write_bytes(b"different worker")
                else:
                    worker.chmod(0o644)
                with self.assertRaisesRegex(ValueError, "inventory mismatch"):
                    CHECK.check_stage(stage)
                worker.write_bytes(original)
                worker.chmod(0o755)
                (stage / "unexpected.txt").unlink(missing_ok=True)

    def test_reinventoried_wrong_worker_still_fails_built_cli_identity(self):
        stage, metadata = self.fixture()
        worker = stage / "Galata Preview.app/Contents/MacOS/galata"
        worker.write_bytes(b"another executable")
        metadata["worker_sha256"] = CHECK.digest(worker)
        self.refresh(stage, metadata)
        with self.assertRaisesRegex(ValueError, "differs from the built CLI"):
            CHECK.check_stage(stage)

    def test_stable_notarized_or_ci_approved_claims_are_refused(self):
        stage, metadata = self.fixture()
        for key, value in (("stable_release", True), ("notarization", "accepted"),
                           ("ci_approval", "passed"), ("signing", "Developer ID")):
            with self.subTest(key=key):
                changed = copy.deepcopy(metadata)
                changed["distribution"][key] = value
                write_json(stage / "PACKAGE.json", changed)
                with self.assertRaisesRegex(ValueError, "local, ad-hoc candidate"):
                    CHECK.check_stage(stage)

    def test_source_and_compilation_evidence_must_match_built_identity(self):
        stage, metadata = self.fixture()
        original = copy.deepcopy(metadata)
        metadata["build_source"]["source_tree_sha256"] = "0" * 64
        write_json(stage / "PACKAGE.json", metadata)
        with self.assertRaisesRegex(ValueError, "built source identity"):
            CHECK.check_stage(stage)
        metadata = original
        commands = stage / "build-evidence/galata-compile-commands.json"
        commands.write_text("[]", encoding="utf-8")
        self.refresh(stage, metadata)
        with self.assertRaisesRegex(ValueError, "compilation commands digest"):
            CHECK.check_stage(stage)

    def test_source_archive_cannot_omit_files_even_when_archive_hash_is_updated(self):
        stage, metadata = self.fixture()
        archive = stage / metadata["source_archive"]["file"]
        with tarfile.open(archive, "w:gz"):
            pass
        metadata["source_archive"]["sha256"] = CHECK.digest(archive)
        self.refresh(stage, metadata)
        with self.assertRaisesRegex(ValueError, "source archive is missing"):
            CHECK.check_stage(stage)

    def test_build_stamp_refuses_stale_ui_worker_source_or_configuration(self):
        stage, metadata = self.fixture()
        path = stage / "build-evidence/galata-desktop-build.json"
        original = json.loads(path.read_text(encoding="utf-8"))
        for key in ("source_tree_sha256", "configuration_sha256", "ui_sha256", "worker_sha256"):
            with self.subTest(key=key):
                stamp = {**original, key: "0" * 64}
                write_json(path, stamp)
                self.refresh(stage, metadata)
                with self.assertRaisesRegex(ValueError, "build stamp is stale"):
                    CHECK.check_stage(stage)

    def test_unsafe_link_special_duplicate_and_conflicting_archive_members_are_refused_before_writes(self):
        cases = [[(name, b"unsafe", 0o100644)] for name in
                 ("../escape", "/absolute", "candidate/../escape", "candidate//empty", "C:/windows",
                  "candidate\\file", "candidate/./dot", "candidate/control\x01")]
        cases += [[("candidate/link", b"outside", 0o120777)],
                  [("candidate/fifo", b"", 0o010644)],
                  [("candidate/suid", b"", 0o104755)],
                  [("candidate/File", b"a", 0o100644), ("candidate/file", b"b", 0o100644)],
                  [("candidate/file", b"a", 0o100644), ("candidate/file/child", b"b", 0o100644)],
                  [("candidate/file/child", b"a", 0o100644), ("candidate/file", b"b", 0o100644)],
                  [("one/file", b"a", 0o100644), ("two/file", b"b", 0o100644)]]
        for index, entries in enumerate(cases):
            with self.subTest(entries=entries):
                archive = self.scratch / f"unsafe-{index}.zip"
                destination = self.scratch / f"extract-{index}"
                self.zip(archive, entries)
                with self.assertRaises(ValueError):
                    CHECK.extract_zip(archive, destination)
                self.assertFalse(destination.exists(), "validate all members before writing any output")

    def test_symlink_in_staged_app_is_refused(self):
        stage, _ = self.fixture()
        (stage / "linked").symlink_to(self.scratch / "outside")
        with self.assertRaisesRegex(ValueError, "symlinks"):
            CHECK.check_stage(stage)

    def test_loader_rejects_developer_paths_unbundled_libraries_and_rpaths(self):
        allowed = "binary:\n\t/usr/lib/libSystem.B.dylib (compatibility version 1.0.0)\n"
        self.assertEqual(CHECK.check_loader_output(allowed, ""), ["/usr/lib/libSystem.B.dylib"])
        for dependency in ("/opt/homebrew/lib/libyaml-cpp.dylib", "/Users/developer/build/lib.dylib",
                           "@rpath/libgalata.dylib", "@executable_path/../missing.dylib"):
            with self.subTest(dependency=dependency):
                with self.assertRaisesRegex(ValueError, "non-system runtime dependency"):
                    CHECK.check_loader_output("binary:\n\t" + dependency + " (compatibility version 1.0.0)", "")
        with self.assertRaisesRegex(ValueError, "non-system runtime search path"):
            CHECK.check_loader_output(allowed, "cmd LC_RPATH\ncmdsize 64\npath /Users/developer/build (offset 12)\n")

    def test_minimum_os_cannot_understate_either_executable_requirement(self):
        commands = "cmd LC_BUILD_VERSION\ncmdsize 32\nplatform 1\nminos 26.0\nsdk 26.4\n"
        CHECK.check_minimum_os(commands, "26")
        CHECK.check_minimum_os(commands, "26.4")
        with self.assertRaisesRegex(ValueError, "deployment requirement"):
            CHECK.check_minimum_os(commands, "15.0")
        with self.assertRaisesRegex(ValueError, "deployment requirement"):
            CHECK.check_minimum_os("", "26.0")


if __name__ == "__main__":
    unittest.main()
