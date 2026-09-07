# SPDX-License-Identifier: Apache-2.0
"""Public revision recovery contract, independent of implementation details.

Revisions are unordered retained content, not an inferred chronological audit.
These checks use exact persisted bytes for identity, never computed numerical
outputs as an expected physics result. Each invocation uses the desktop worker's
public CLI and a bounded, independently declared fixture.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()


@unittest.skipUnless(os.name == "posix" and CLI.is_file(), "requires the POSIX project CLI")
class ProjectRecovery(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-recovery-test-")
        self.addCleanup(self.temporary.cleanup)
        self.scratch = Path(self.temporary.name)
        self.project = self.scratch / "retained project.galata"
        self.first = self.command("create", self.project)

    def invoke(self, *arguments):
        return subprocess.run([str(CLI), "project", *map(str, arguments)], cwd=self.scratch,
                              capture_output=True, text=True, timeout=45)

    def command(self, *arguments):
        result = self.invoke(*arguments)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        try:
            return json.loads(result.stdout)
        except ValueError as error:
            self.fail(f"successful project command must emit JSON: {error}\n{result.stdout}")

    def refused(self, *arguments):
        result = self.invoke(*arguments)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((result.stdout + result.stderr).strip(), "refusal needs a diagnostic")

    def draft(self, view):
        draft = {"schema": view["draft_schema"],
                 **{key: copy.deepcopy(view[key]) for key in ("model", "presentation", "simulation")}}
        if "origin_sha256" in view:
            draft["origin_sha256"] = view["origin_sha256"]
        return draft

    def save(self, draft, view):
        path = self.scratch / "draft.json"
        path.write_text(json.dumps(draft, allow_nan=False), encoding="utf-8")
        return self.command("save", self.project, path, "--expected-revision", view["revision"])

    def changed(self, view):
        draft = self.draft(view)
        draft["simulation"]["steps"] = 2
        draft["simulation"]["sample_stride"] = 1
        return self.save(draft, view)

    def snapshot(self):
        """The pointer is the only retained document a restore may replace."""
        return {str(path.relative_to(self.project)): path.read_bytes()
                for path in self.project.rglob("*")
                if path.is_file() and not path.is_symlink()}

    def retain_draft(self, draft):
        data = json.dumps(draft, allow_nan=False).encode()
        digest = hashlib.sha256(data).hexdigest()
        (self.project / "revisions" / (digest + ".json")).write_bytes(data)
        return digest

    def import_tiny(self):
        """Declare xdot=u, y=x with explicit SI channel types and two steps."""
        source = self.scratch / "source"
        source.mkdir()
        (source / "plant.yaml").write_text(
            "description: Independent recovery transport fixture\n"
            "citation: Analytic integrator xdot=u\nunits: SI\n"
            "states: [distance]\ninputs: [speed]\noutputs: [distance]\n"
            "a: [[0]]\nb: [[1]]\nc: [[1]]\nd: [[0]]\n", encoding="utf-8")
        study = {
            "version": 1,
            "stages": [
                {"id": "source", "capability": "model.linear.statespace", "input": {"path": "plant.yaml"}},
                {"id": "graph", "capability": "model.linear_graph", "input": {
                    "system": {"from": "source"},
                    "channel_types": {
                        "states": [{"dimension": [1, 0, 0, 0, 0, 0, 0, 0], "frame": "none"}],
                        "inputs": [{"dimension": [1, 0, -1, 0, 0, 0, 0, 0], "frame": "none"}],
                        "outputs": [{"dimension": [1, 0, 0, 0, 0, 0, 0, 0], "frame": "none"}]},
                    "initial_state": [0], "command": [1],
                    "model_path": "model.yaml", "adapter_path": "adapter.json"}},
                {"id": "response", "capability": "sim.model", "input": {
                    "model": {"from": "graph"}, "step_s": 0.125, "steps": 2,
                    "csv_path": "response.csv", "evidence_path": "evidence.json"}}
            ]}
        study_path = source / "study.yaml"
        study_path.write_text(json.dumps(study), encoding="utf-8")
        self.project = self.scratch / "imported project.galata"
        view = self.command("import-linear", self.project, study_path)
        source.rename(self.scratch / "retired source")
        return view

    def test_history_has_exact_retained_ids_and_one_current_marker(self):
        second = self.changed(self.first)
        history = self.command("revisions", self.project)
        self.assertEqual(history["schema"], "galata.project-history.v1")
        self.assertEqual(history["current_revision"], second["revision"])
        self.assertEqual(history["current_status"], "valid")
        self.assertEqual(history["ordering"], "revision_filename_ascending")
        entries = history["revisions"]
        self.assertEqual([entry["filename"] for entry in entries],
                         sorted(view["revision"] + ".json" for view in (self.first, second)))
        self.assertEqual({entry["revision"] for entry in entries},
                         {self.first["revision"], second["revision"]})
        self.assertEqual([entry["revision"] for entry in entries if entry["is_current"]],
                         [second["revision"]])
        for entry in entries:
            self.assertEqual(entry["status"], "valid")
            self.assertEqual(entry["model_profile"], "continuous-scalar.v1")
            expected = self.first if entry["revision"] == self.first["revision"] else second
            self.assertEqual(entry["block_count"], len(expected["model"]["blocks"]))
            self.assertEqual(entry["simulation"], expected["simulation"])
        before = self.snapshot()
        selected = self.command("revision", self.project, self.first["revision"])
        self.assertEqual(selected["schema"], "galata.project-revision.v1")
        self.assertEqual(selected["revision"], self.first["revision"])
        self.assertEqual(selected["draft"], self.draft(self.first))
        self.assertEqual(self.snapshot(), before)

    def test_restore_changes_only_head_and_retains_newer_runs_and_revisions(self):
        first_head = (self.project / "project.json").read_bytes()
        second = self.changed(self.first)
        run = self.command("run", self.project)
        self.assertEqual(run["status"], "completed")
        before = self.snapshot()
        restored = self.command("restore", self.project, self.first["revision"],
                                "--expected-revision", second["revision"])
        self.assertEqual(restored["revision"], self.first["revision"])
        self.assertEqual(self.draft(restored), self.draft(self.first))
        self.assertEqual((self.project / "project.json").read_bytes(), first_head)
        after = self.snapshot()
        self.assertEqual(set(before), set(after))
        for name in before.keys() - {"project.json"}:
            self.assertEqual(after[name], before[name], name)
        retained_run = next(item for item in restored["runs"] if item["id"] == run["id"])
        self.assertEqual(retained_run["revision"], second["revision"])
        self.assertEqual(retained_run["status"], "completed")
        self.assertEqual(len(self.command("revisions", self.project)["revisions"]), 2)
        same = self.command("restore", self.project, restored["revision"],
                            "--expected-revision", restored["revision"])
        self.assertEqual(same["revision"], restored["revision"])
        self.assertEqual(self.snapshot(), after)

    def test_stale_missing_and_malformed_targets_preserve_owned_documents(self):
        second = self.changed(self.first)
        before = self.snapshot()
        for target, expected in ((self.first["revision"], self.first["revision"]),
                                 ("0" * 64, second["revision"]),
                                 ("../project", second["revision"])):
            with self.subTest(target=target, expected=expected):
                self.refused("restore", self.project, target, "--expected-revision", expected)
                self.assertEqual(self.snapshot(), before)
        head_path = self.project / "project.json"
        for damaged in (b"{", json.dumps({"schema": "galata.project.v999",
                                          "revision": second["revision"]}).encode()):
            with self.subTest(head=damaged):
                head_path.write_bytes(damaged)
                damaged_snapshot = self.snapshot()
                self.refused("restore", self.project, self.first["revision"],
                             "--expected-revision", second["revision"])
                self.assertEqual(self.snapshot(), damaged_snapshot)
        head_path.write_bytes(before["project.json"])

    def test_corrupt_target_is_visible_invalid_and_cannot_replace_head(self):
        second = self.changed(self.first)
        target = self.project / "revisions" / (self.first["revision"] + ".json")
        target.write_bytes(target.read_bytes() + b"\n")
        before = self.snapshot()
        self.refused("restore", self.project, self.first["revision"],
                     "--expected-revision", second["revision"])
        self.refused("revision", self.project, self.first["revision"])
        self.assertEqual(self.snapshot(), before)
        entries = self.command("revisions", self.project)["revisions"]
        damaged = next(entry for entry in entries if entry["revision"] == self.first["revision"])
        self.assertEqual(damaged["status"], "invalid")
        self.assertFalse(damaged["is_current"])
        self.assertTrue(damaged["diagnostic"])

    def test_history_limit_counts_invalid_entries_and_refuses_1025_without_changes(self):
        revisions = self.project / "revisions"
        invalid_names = {f"damaged-{index:04d}.json" for index in range(1023)}
        for name in invalid_names:
            (revisions / name).write_text("{}", encoding="utf-8")
        before = self.snapshot()
        history = self.command("revisions", self.project)
        entries = history["revisions"]
        self.assertEqual(len(entries), 1024)
        self.assertEqual({entry["filename"] for entry in entries},
                         invalid_names | {self.first["revision"] + ".json"})
        valid = [entry for entry in entries if entry["status"] == "valid"]
        self.assertEqual(len(valid), 1)
        self.assertEqual(valid[0]["revision"], self.first["revision"])
        self.assertTrue(valid[0]["is_current"])
        self.assertEqual(sum(entry["status"] == "invalid" for entry in entries), 1023)
        self.assertEqual(history["current_status"], "valid")
        self.assertEqual(self.snapshot(), before)
        (revisions / "one-entry-too-many.json").write_text("{}", encoding="utf-8")
        before = self.snapshot()
        self.refused("revisions", self.project)
        self.assertEqual(self.snapshot(), before)

    @unittest.skipIf(os.name != "posix" or (hasattr(os, "geteuid") and os.geteuid() == 0),
                     "directory permissions require a non-root POSIX user")
    def test_restore_write_failure_keeps_head_and_retained_documents(self):
        second = self.changed(self.first)
        before = self.snapshot()
        lock = self.project / "project.lock"
        self.assertTrue(lock.is_file())
        self.assertTrue(os.access(lock, os.W_OK), "the existing lock must remain writable")
        original_mode = stat.S_IMODE(self.project.stat().st_mode)
        try:
            self.project.chmod(original_mode & ~0o222)
            self.assertFalse(os.access(self.project, os.W_OK))
            self.assertTrue(os.access(lock, os.W_OK), "refusal must not be caused by an unwritable lock")
            selected = self.command("revision", self.project, self.first["revision"])
            self.assertEqual(selected["draft"], self.draft(self.first))
            self.refused("restore", self.project, self.first["revision"],
                         "--expected-revision", second["revision"])
            self.assertEqual(self.snapshot(), before)
        finally:
            self.project.chmod(original_mode)
        self.assertEqual(self.command("inspect", self.project)["revision"], second["revision"])

    def test_history_exposes_malformed_symlink_directory_and_corrupt_current(self):
        self.changed(self.first)
        revisions = self.project / "revisions"
        (revisions / "bad-name.json").write_text("{}", encoding="utf-8")
        (revisions / ("1" * 64 + ".json")).mkdir()
        outside = self.scratch / "outside.json"
        outside.write_text("outside content", encoding="utf-8")
        (revisions / ("2" * 64 + ".json")).symlink_to(outside)
        head = json.loads((self.project / "project.json").read_text())
        current = revisions / (head["revision"] + ".json")
        current.write_bytes(current.read_bytes() + b"\n")
        before = self.snapshot()
        history = self.command("revisions", self.project)
        self.assertEqual(history["current_revision"], head["revision"])
        self.assertEqual(history["current_status"], "invalid")
        self.assertTrue(history["current_diagnostic"])
        entries = {entry["filename"]: entry for entry in history["revisions"]}
        self.assertEqual(len(entries), 5)
        for name in ("bad-name.json", "1" * 64 + ".json", "2" * 64 + ".json", current.name):
            self.assertEqual(entries[name]["status"], "invalid", name)
            self.assertTrue(entries[name]["diagnostic"], name)
        self.assertTrue(entries[current.name]["is_current"])
        self.assertEqual(entries[self.first["revision"] + ".json"]["status"], "valid")
        self.refused("restore", self.project, self.first["revision"],
                     "--expected-revision", head["revision"])
        self.assertEqual(self.snapshot(), before)
        self.assertEqual(outside.read_text(), "outside content")
        current.rename(self.scratch / "missing current revision.json")
        missing = self.command("revisions", self.project)
        self.assertEqual(missing["current_status"], "invalid")
        self.assertTrue(missing["current_diagnostic"])
        self.assertEqual(len(missing["revisions"]), 4)
        self.assertFalse(any(entry["is_current"] for entry in missing["revisions"]))
        before = self.snapshot()
        self.refused("restore", self.project, self.first["revision"],
                     "--expected-revision", head["revision"])
        self.assertEqual(self.snapshot(), before)

    def test_valid_uncompiled_draft_can_be_restored_without_claiming_execution(self):
        draft = self.draft(self.first)
        draft["model"]["connections"] = []
        unfinished = self.save(draft, self.first)
        self.command("restore", self.project, self.first["revision"],
                     "--expected-revision", unfinished["revision"])
        history = self.command("revisions", self.project)
        entry = next(item for item in history["revisions"] if item["revision"] == unfinished["revision"])
        self.assertEqual(entry["status"], "valid")
        restored = self.command("restore", self.project, unfinished["revision"],
                                "--expected-revision", self.first["revision"])
        self.assertEqual(restored["model"]["connections"], [])
        self.refused("run", self.project)
        viewed = self.command("inspect", self.project)
        self.assertEqual(viewed["revision"], unfinished["revision"])
        self.assertEqual(viewed["runs"][0]["status"], "failed")

    def test_import_restore_retains_origin_and_recovers_original_relation(self):
        imported = self.import_tiny()
        draft = self.draft(imported)
        state = next(block for block in draft["model"]["blocks"] if block["kind"] == "integrator")
        state["initial_value"] = 3
        edited = self.save(draft, imported)
        self.assertEqual(edited["origin"]["relation"], "modified_from_import")
        before = self.snapshot()
        restored = self.command("restore", self.project, imported["revision"],
                                "--expected-revision", edited["revision"])
        self.assertEqual(restored["draft_schema"], "galata.project-draft.v2")
        self.assertEqual(restored["origin_sha256"], imported["origin_sha256"])
        self.assertEqual(restored["origin"]["relation"], "matches_imported_model")
        after = self.snapshot()
        self.assertEqual(set(after), set(before))
        for name in before.keys() - {"project.json"}:
            self.assertEqual(after[name], before[name], name)
        entries = {entry["revision"]: entry for entry in self.command("revisions", self.project)["revisions"]}
        self.assertEqual(entries[imported["revision"]]["origin_relation"], "matches_imported_model")
        self.assertEqual(entries[edited["revision"]]["origin_relation"], "modified_from_import")

    def test_restore_refuses_valid_foreign_origin_and_schema_downgrade(self):
        imported = self.import_tiny()
        origin_path = self.project / "origins" / (imported["origin_sha256"] + ".json")
        # A different retained attachment identity, with independently valid
        # bindings, must not be adopted merely because its semantic content fits.
        alternate_bytes = origin_path.read_bytes() + b"\n"
        alternate_sha = hashlib.sha256(alternate_bytes).hexdigest()
        (origin_path.parent / (alternate_sha + ".json")).write_bytes(alternate_bytes)
        foreign = self.draft(imported)
        foreign["origin_sha256"] = alternate_sha
        foreign_revision = self.retain_draft(foreign)
        downgrade_revision = self.retain_draft(self.draft(self.first))
        entries = {entry["revision"]: entry for entry in self.command("revisions", self.project)["revisions"]}
        before = self.snapshot()
        for revision in (foreign_revision, downgrade_revision):
            with self.subTest(revision=revision):
                self.assertEqual(entries[revision]["status"], "valid")
                self.refused("restore", self.project, revision,
                             "--expected-revision", imported["revision"])
                self.assertEqual(self.snapshot(), before)


if __name__ == "__main__":
    unittest.main()
