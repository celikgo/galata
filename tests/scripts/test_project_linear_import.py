# SPDX-License-Identifier: Apache-2.0
"""Public import/edit/run contract with an independently authored polynomial.

xdot=2, x(0)=1/2, y=3*x+u/4, u=2: RK4 is exact at the dyadic sample times.
This exercises project transport and origin attribution, not aircraft validity.
"""
import copy
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()


@unittest.skipUnless(os.name == "posix" and CLI.is_file(), "requires the POSIX project CLI")
class ProjectLinearImport(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = tempfile.TemporaryDirectory(prefix="galata-import-fixture-")
        cls.addClassCleanup(cls.fixture.cleanup)
        cls.fixture_root = Path(cls.fixture.name)
        source = cls.fixture_root / "source"
        source.mkdir()
        cls.source_bytes = ("description: Independent polynomial transport fixture\n"
                            "citation: Analytic xdot=2, y=3x+u/4\nunits: SI\n"
                            "states: [distance]\ninputs: [speed]\noutputs: [measured]\n"
                            "a: [[0]]\nb: [[1]]\nc: [[3]]\nd: [[0.25]]\n").encode()
        (source / "plant.yaml").write_bytes(cls.source_bytes)
        cls.study_text = """version: 1
stages:
  - id: source
    capability: model.linear.statespace
    input: {path: plant.yaml}
  - id: graph
    capability: model.linear_graph
    input:
      system: {from: source}
      channel_types:
        states: [{dimension: [1,0,0,0,0,0,0,0], frame: body}]
        inputs: [{dimension: [1,0,-1,0,0,0,0,0], frame: none}]
        outputs: [{dimension: [1,0,0,0,0,0,0,0], frame: ned}]
      initial_state: [0.5]
      command: [2]
      model_path: exports/model.yaml
      adapter_path: exports/adapter.json
  - id: response
    capability: sim.model
    input: {model: {from: graph}, step_s: 0.125, steps: 2, csv_path: response.csv, evidence_path: evidence.json}
"""
        study = source / "study.yaml"
        study.write_text(cls.study_text)
        cls.template = cls.fixture_root / "template.galata"
        result = subprocess.run([str(CLI), "project", "import-linear", str(cls.template), str(study)],
                                capture_output=True, text=True, timeout=45)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)
        cls.import_view = json.loads(result.stdout)
        # All following operations must work after the original source disappears.
        source.rename(cls.fixture_root / "retired source")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-import-test-")
        self.addCleanup(self.temporary.cleanup)
        self.scratch = Path(self.temporary.name)
        self.project = self.scratch / "relocated project.galata"
        shutil.copytree(self.template, self.project)

    def invoke(self, *arguments, success=True):
        result = subprocess.run([str(CLI), "project", *map(str, arguments)],
                                capture_output=True, text=True, timeout=45)
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            return json.loads(result.stdout)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((result.stdout + result.stderr).strip())
        return result

    def view(self):
        return self.invoke("inspect", self.project)

    def draft(self, view):
        return {"schema": view["draft_schema"], "origin_sha256": view["origin_sha256"],
                **{key: copy.deepcopy(view[key]) for key in ("model", "presentation", "simulation")}}

    def save(self, draft, view, success=True):
        path = self.scratch / "draft.json"
        path.write_text(json.dumps(draft))
        return self.invoke("save", self.project, path, "--expected-revision", view["revision"],
                           success=success)

    def test_relocated_import_preserves_exact_original_sources_and_mapping(self):
        view = self.view()
        self.assertEqual(view["draft_schema"], "galata.project-draft.v2")
        self.assertEqual(view["model"]["profile"], "continuous-linear.v1")
        self.assertEqual(view["origin"]["relation"], "matches_imported_model")
        original = view["origin"]["manifest"]
        self.assertEqual(bytes.fromhex(original["study"]["bytes_hex"]), self.study_text.encode())
        records = [r for r in original["inputs"] if Path(r["path"]).name == "plant.yaml"]
        self.assertEqual(len(records), 1)
        self.assertEqual(bytes.fromhex(records[0]["bytes_hex"]), self.source_bytes)
        adapter = view["origin"]["adapter"]
        self.assertEqual(adapter["source_system"]["a"], [[0]])
        self.assertEqual(adapter["source_system"]["d"], [[0.25]])
        self.assertEqual(adapter["source_system"]["state_names"], ["distance"])
        self.assertEqual(adapter["mapping"]["state_ids"], ["state_000"])
        self.assertEqual(adapter["channel_types"]["outputs"][0]["frame"], "ned")
        self.assertTrue(Path(view["origin"]["manifest_path"]).resolve().is_relative_to(self.project.resolve()))
        self.assertTrue(Path(view["origin"]["manifest_path"]).is_file())

    def test_saved_run_solves_polynomial_and_snapshots_original_context(self):
        view = self.view()
        run = self.invoke("run", self.project)
        self.assertEqual(run["status"], "completed")
        self.assertEqual(run["origin_relation"], "matches_imported_model")
        evidence = json.loads(Path(run["evidence_json"]).read_text())
        self.assertEqual(evidence["profile"], "continuous-linear.v1")
        for axis in ("numerical_accuracy", "model_validity", "engineering_acceptance"):
            self.assertEqual(evidence[axis], "not_assessed")
        rows = list(csv.DictReader(io.StringIO(Path(run["trajectory_csv"]).read_text())))
        self.assertEqual(len(rows), 3)
        for index, row in enumerate(rows):
            x = 0.5 + 2 * index * 0.125
            self.assertEqual(float(row["state:state_000"]), x)
            self.assertEqual(float(row["output:output_000"]), 3 * x + 0.5)
            self.assertEqual(float(row["output:control_output_000"]), 2)
        origin_bytes = (self.project / "origins" / (view["origin_sha256"] + ".json")).read_bytes()
        self.assertEqual(hashlib.sha256(origin_bytes).hexdigest(), view["origin_sha256"])
        manifest = json.loads(Path(run["manifest_path"]).read_text())
        context = [r for r in manifest["inputs"] if Path(r["path"]).name == "origin.json"]
        self.assertEqual(len(context), 1)
        self.assertEqual(bytes.fromhex(context[0]["bytes_hex"]), origin_bytes)
        self.assertFalse(manifest["linearization_evidence"])
        self.assertEqual(self.view()["runs"][0]["status"], "completed")

    def test_layout_and_equation_edits_keep_origin_and_separate_run_relation(self):
        first = self.view()
        draft = self.draft(first)
        draft["presentation"]["positions"]["state_000"]["x"] += 10
        layout = self.save(draft, first)
        self.assertNotEqual(layout["revision"], first["revision"])
        self.assertEqual(layout["origin"]["relation"], "matches_imported_model")
        old_run = self.invoke("run", self.project)
        draft = self.draft(layout)
        state = next(b for b in draft["model"]["blocks"] if b["id"] == "state_000")
        state["initial_value"] = 1
        edited = self.save(draft, layout)
        self.assertEqual(edited["origin_sha256"], first["origin_sha256"])
        self.assertEqual(edited["origin"]["relation"], "modified_from_import")
        self.assertEqual(edited["runs"][0]["origin_relation"], "matches_imported_model")
        self.assertEqual(edited["runs"][0]["id"], old_run["id"])
        new_run = self.invoke("run", self.project)
        self.assertEqual(new_run["origin_relation"], "modified_from_import")
        self.assertEqual(new_run["status"], "completed")

    def test_save_cannot_drop_or_replace_origin(self):
        first = self.view()
        for change in ("drop", "replace"):
            draft = self.draft(first)
            if change == "drop":
                draft["schema"] = "galata.project-draft.v1"
                del draft["origin_sha256"]
            else:
                draft["origin_sha256"] = "0" * 64
            self.save(draft, first, success=False)
            self.assertEqual(self.view()["revision"], first["revision"])

    def test_import_artifact_tamper_refuses_review_without_rewriting_head(self):
        view = self.view()
        head = (self.project / "project.json").read_bytes()
        root = Path(view["origin"]["manifest_path"]).parent
        (root / "exports" / "adapter.json").write_text("{}\n")
        self.invoke("inspect", self.project, success=False)
        self.assertEqual((self.project / "project.json").read_bytes(), head)

    def test_invalid_import_never_publishes_head_or_overwrites_existing_project(self):
        head = (self.project / "project.json").read_bytes()
        bad = self.scratch / "bad.yaml"
        bad.write_text("version: 1\nstages: []\n")
        self.invoke("import-linear", self.project, bad, success=False)
        self.assertEqual((self.project / "project.json").read_bytes(), head)
        destination = self.scratch / "failed.galata"
        self.invoke("import-linear", destination, bad, success=False)
        self.assertFalse((destination / "project.json").exists())


if __name__ == "__main__":
    unittest.main()
