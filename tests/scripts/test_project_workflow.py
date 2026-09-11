# SPDX-License-Identifier: Apache-2.0
"""Public project/worker acceptance; no captured Galata numerical references.

The synthetic fixture solves x' = 1 - x, x(0) = 0. Identity comparisons below
test integration consistency; the endpoint check uses its analytic solution.
Run on the supported POSIX preview platforms with GALATA_PROJECT_CLI pointing
at the actual built executable. A present but incompatible executable must fail.
"""
import copy
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()
# Wall-clock deadline for ONE CLI invocation: a hang guard, so a wedged worker
# fails here naming its own command rather than arriving as an opaque ctest kill
# of the whole module. 45 s is the uninstrumented default and is unchanged;
# tests/CMakeLists.txt scales it for the sanitizer build and records why.
TIMEOUT_S = float(os.environ.get("GALATA_PROJECT_TIMEOUT_S", "45"))


def synthetic_model():
    """Independently authored scalar feedback equation in the public vocabulary."""
    length = {"dimension": [1, 0, 0, 0, 0, 0, 0, 0], "frame": "none"}
    velocity = {"dimension": [1, 0, -1, 0, 0, 0, 0, 0], "frame": "none"}
    return {
        "schema": "galata.model.v1", "profile": "continuous-scalar.v1",
        "blocks": [
            {"id": "command", "kind": "constant", "output": velocity, "value": 1.0},
            {"id": "feedback", "kind": "gain", "output": velocity,
             "coefficient": {"value": 1.0, "dimension": [0, 0, -1, 0, 0, 0, 0, 0]}},
            {"id": "rate", "kind": "sum", "output": velocity, "signs": [1, -1]},
            {"id": "x", "kind": "integrator", "output": length, "initial_value": 0.0},
            {"id": "y", "kind": "output", "output": length},
        ],
        "connections": [
            {"source": "command", "target": "rate", "input": 0},
            {"source": "feedback", "target": "rate", "input": 1},
            {"source": "rate", "target": "x", "input": 0},
            {"source": "x", "target": "feedback", "input": 0},
            {"source": "x", "target": "y", "input": 0},
        ],
    }


@unittest.skipUnless(os.name == "posix", "project worker preview requires a POSIX host")
@unittest.skipUnless(CLI.is_file(), f"project executable is missing: {CLI}")
class ProjectWorkflow(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-project-test-")
        self.addCleanup(self.temporary.cleanup)
        self.scratch = Path(self.temporary.name)
        self.project = self.scratch / "project with spaces"
        self.view = self.command("create", self.project)
        self.draft_sequence = 0

    def invoke(self, *arguments):
        return subprocess.run([str(CLI), "project", *map(str, arguments)],
                              cwd=self.scratch, capture_output=True, text=True, timeout=TIMEOUT_S)

    def command(self, *arguments):
        result = self.invoke(*arguments)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        try:
            return json.loads(result.stdout)
        except ValueError as error:
            self.fail(f"successful project command did not emit one JSON document: {error}\n"
                      f"{result.stdout}\n{result.stderr}")

    def refused(self, *arguments):
        result = self.invoke(*arguments)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((result.stdout + result.stderr).strip(), "refusal needs a diagnostic")
        return result

    def inspect(self):
        return self.command("inspect", self.project)

    def draft(self, view=None):
        view = self.inspect() if view is None else view
        return {"schema": "galata.project-draft.v1",
                **{key: copy.deepcopy(view[key])
                   for key in ("model", "presentation", "simulation")}}

    def write_draft(self, draft):
        self.draft_sequence += 1
        path = self.scratch / f"draft-{self.draft_sequence}.json"
        path.write_text(json.dumps(draft, allow_nan=False), encoding="utf-8")
        return path

    def save(self, draft, revision=None):
        revision = self.inspect()["revision"] if revision is None else revision
        return self.command("save", self.project, self.write_draft(draft),
                            "--expected-revision", revision)

    def feedback_draft(self, long_run=False):
        draft = self.draft()
        draft["model"] = synthetic_model()
        draft["presentation"] = {"schema": "galata.presentation.v1", "positions": {
            block["id"]: {"x": index * 120, "y": 100}
            for index, block in enumerate(draft["model"]["blocks"])}}
        draft["simulation"] = {"initial_time_s": 0.0, "step_s": 0.01,
                               "steps": 1000000 if long_run else 100,
                               "sample_stride": 1000000 if long_run else 1}
        return draft

    def artifact(self, run, key):
        path = Path(run[key])
        if not path.is_absolute():
            path = (self.project if path.parts[0] == "runs"
                    else self.project / "runs" / run["id"]) / path
        run_root = (self.project / "runs" / run["id"]).resolve()
        self.assertTrue(path.resolve().is_relative_to(run_root),
                        f"{key} escapes the owned run: {path}")
        self.assertTrue(path.is_file(), f"missing {key}: {path}")
        return path

    def run_evidence(self):
        result = self.command("run", self.project)
        self.assertEqual(result["status"], "completed", result)
        evidence = json.loads(self.artifact(result, "evidence_json").read_text())
        trajectory = self.artifact(result, "trajectory_csv").read_bytes()
        self.artifact(result, "manifest_path")
        self.assertEqual(evidence["execution"], "completed")
        for axis in ("numerical_accuracy", "model_validity", "engineering_acceptance"):
            self.assertEqual(evidence[axis], "not_assessed")
        self.assertEqual(evidence["trajectory_sha256"], hashlib.sha256(trajectory).hexdigest())
        return result, evidence, trajectory

    def start_long_job(self):
        revision = self.save(self.feedback_draft(long_run=True))["revision"]
        process = subprocess.Popen([str(CLI), "project", "run", str(self.project)],
                                   cwd=self.scratch, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True)

        def cleanup():
            if process.poll() is None:
                process.kill()
            process.communicate(timeout=10)

        self.addCleanup(cleanup)
        # THIS DEADLINE IS CHECKED AT THE TOP OF THE LOOP, so the first
        # `inspect()` always completes however long it takes, and under
        # instrumentation it takes longer than the deadline by itself. The
        # effective requirement is therefore "the first inspect sees the job
        # running", and the message below is accurate only for the
        # uninstrumented build. Left at 15 s deliberately: it has not failed,
        # and scaling a guard that is passing would relax a requirement on no
        # evidence. If it ever fires under the sanitizer, this is why, and the
        # fix is to observe the worker's state without a second full CLI
        # invocation rather than to raise the number.
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            for run in self.inspect()["runs"]:
                if run["status"] == "running":
                    return process, run, revision
            if process.poll() is not None:
                stdout, stderr = process.communicate()
                self.fail(f"long job ended before its running state was observable: {stdout}{stderr}")
            time.sleep(0.01)
        self.fail("worker did not expose a running job within 15 seconds")

    def test_create_round_trip_and_retained_revision_identity(self):
        self.assertEqual(self.view["schema"], "galata.project-view.v1")
        self.assertRegex(self.view["revision"], r"^[0-9a-f]{64}$")
        self.assertEqual(self.view, self.inspect())
        head = json.loads((self.project / "project.json").read_text())
        self.assertEqual(head, {"schema": "galata.project.v1", "revision": self.view["revision"]})
        revision_path = self.project / "revisions" / (head["revision"] + ".json")
        original = revision_path.read_bytes()
        self.assertEqual(hashlib.sha256(original).hexdigest(), head["revision"])
        updated = self.save(self.feedback_draft())
        self.assertEqual(revision_path.read_bytes(), original)
        self.assertEqual(self.inspect()["revision"], updated["revision"])

    def test_create_refuses_nonempty_directory_without_overwriting(self):
        sentinel = self.project / "keep.txt"
        sentinel.write_text("retain me", encoding="utf-8")
        head = (self.project / "project.json").read_bytes()
        self.refused("create", self.project)
        self.assertEqual(sentinel.read_text(), "retain me")
        self.assertEqual((self.project / "project.json").read_bytes(), head)

    def test_stale_save_refuses_without_changing_head_or_old_revision(self):
        first = self.inspect()
        draft = self.feedback_draft()
        current = self.save(draft, first["revision"])
        draft["simulation"]["steps"] += 1
        self.refused("save", self.project, self.write_draft(draft),
                     "--expected-revision", first["revision"])
        self.assertEqual(self.inspect()["revision"], current["revision"])
        self.assertTrue((self.project / "revisions" / (first["revision"] + ".json")).is_file())

    def test_layout_edit_preserves_semantics_and_exact_trajectory(self):
        draft = self.feedback_draft()
        first = self.save(draft)
        old_run, old_evidence, old_csv = self.run_evidence()
        draft["presentation"]["positions"]["feedback"] = {"x": -45.5, "y": 801.25}
        second = self.save(draft)
        new_run, new_evidence, new_csv = self.run_evidence()
        self.assertNotEqual(first["revision"], second["revision"])
        self.assertNotEqual(old_run["id"], new_run["id"])
        self.assertEqual(old_evidence["model_semantic_sha256"],
                         new_evidence["model_semantic_sha256"])
        self.assertEqual(old_csv, new_csv)
        for run, revision in ((old_run, first["revision"]), (new_run, second["revision"])):
            request = json.loads((self.project / "runs" / run["id"] / "request.json").read_text())
            self.assertEqual(request["revision"], revision)

    def test_semantic_and_solver_edits_have_separate_identities(self):
        draft = self.feedback_draft()
        self.save(draft)
        _, original, original_csv = self.run_evidence()
        draft["simulation"]["step_s"] = 0.02
        self.save(draft)
        _, solver, solver_csv = self.run_evidence()
        self.assertEqual(original["model_semantic_sha256"], solver["model_semantic_sha256"])
        self.assertNotEqual(original["solver"], solver["solver"])
        self.assertNotEqual(original_csv, solver_csv)
        draft["model"]["blocks"][1]["coefficient"]["value"] = 2.0
        self.save(draft)
        _, changed, changed_csv = self.run_evidence()
        self.assertNotEqual(solver["model_semantic_sha256"], changed["model_semantic_sha256"])
        self.assertNotEqual(solver_csv, changed_csv)

    def test_completed_feedback_meets_prespecified_analytic_endpoint_budget(self):
        self.save(self.feedback_draft())
        run, _, trajectory = self.run_evidence()
        rows = list(csv.DictReader(io.StringIO(trajectory.decode("utf-8"))))
        self.assertEqual(len(rows), 101)
        self.assertEqual(float(rows[0]["time_s"]), 0.0)
        self.assertEqual(float(rows[-1]["time_s"]), 1.0)
        self.assertEqual(float(rows[0]["state:x"]), 0.0)
        # RK4 global error is O(h^4); h=.01, T=1, smooth stable scalar equation.
        # 1e-9 is fixed here before observing product output, not fitted to it.
        self.assertLess(abs(float(rows[-1]["state:x"]) - (1 - math.exp(-1))), 1e-9)
        self.assertEqual(float(rows[-1]["state:x"]), float(rows[-1]["output:y"]))
        listed = next(item for item in self.inspect()["runs"] if item["id"] == run["id"])
        self.assertEqual(listed["status"], "completed")

    def stateless_draft(self, source_id="constant", output_id="scope", value=1.0):
        draft = self.feedback_draft()
        unitless = {"dimension": [0] * 8, "frame": "none"}
        draft["model"] = {"schema": "galata.model.v1", "profile": "continuous-scalar.v1",
                          "blocks": [
                              {"id": source_id, "kind": "constant", "output": unitless,
                               "value": value},
                              {"id": output_id, "kind": "output", "output": unitless}],
                          "connections": [{"source": source_id, "target": output_id,
                                           "input": 0}]}
        draft["presentation"]["positions"] = {source_id: {"x": 0, "y": 0},
                                               output_id: {"x": 120, "y": 0}}
        draft["simulation"] = {"initial_time_s": 0.0, "step_s": 1.0,
                               "steps": 1, "sample_stride": 1}
        return draft

    def test_boolean_looking_ids_remain_strings_through_saved_run(self):
        draft = self.stateless_draft(source_id="true", output_id="false")
        saved = self.save(draft)
        self.assertEqual({block["id"] for block in saved["model"]["blocks"]}, {"true", "false"})
        self.assertEqual(saved["model"]["connections"], draft["model"]["connections"])
        _, evidence, trajectory = self.run_evidence()
        self.assertEqual(evidence["output_ids"], ["false"])
        rows = list(csv.DictReader(io.StringIO(trajectory.decode("utf-8"))))
        self.assertEqual([float(row["output:false"]) for row in rows], [1.0, 1.0])

    def test_minimum_binary64_subnormal_survives_stateless_round_trip(self):
        minimum_subnormal = float.fromhex("0x0.0000000000001p-1022")
        saved = self.save(self.stateless_draft(value=minimum_subnormal))
        constant = next(block for block in saved["model"]["blocks"] if block["kind"] == "constant")
        self.assertEqual(constant["value"], minimum_subnormal)
        _, evidence, trajectory = self.run_evidence()
        self.assertEqual(evidence["state_ids"], [])
        rows = list(csv.DictReader(io.StringIO(trajectory.decode("utf-8"))))
        self.assertEqual([float(row["output:scope"]) for row in rows],
                         [minimum_subnormal, minimum_subnormal])

    def test_large_integer_input_index_is_not_rounded_through_binary64(self):
        draft = self.feedback_draft()
        exact_index = 2 ** 53 + 1
        draft["model"]["connections"][0]["input"] = exact_index
        saved = self.save(draft)
        edge = next(edge for edge in saved["model"]["connections"] if edge["source"] == "command")
        self.assertIsInstance(edge["input"], int)
        self.assertEqual(edge["input"], exact_index)
        reopened = self.inspect()
        edge = next(edge for edge in reopened["model"]["connections"] if edge["source"] == "command")
        self.assertEqual(edge["input"], exact_index)
        self.refused("run", self.project)
        self.assertEqual(self.inspect()["runs"][0]["status"], "failed")

    def test_closed_draft_versions_fields_and_finite_values(self):
        baseline = self.inspect()["revision"]
        cases = []
        for field in (None, "model", "presentation"):
            draft = self.feedback_draft()
            target = draft if field is None else draft[field]
            target["schema"] = "unknown.future.v99"
            cases.append(draft)
        for field in (None, "presentation", "simulation"):
            draft = self.feedback_draft()
            target = draft if field is None else draft[field]
            target["execute"] = "unexpected command"
            cases.append(draft)
        draft = self.feedback_draft()
        draft["model"]["blocks"][0]["value"] = "1.0"
        cases.append(draft)
        draft = self.feedback_draft()
        draft["presentation"]["positions"]["command"]["x"] = True
        cases.append(draft)
        draft = self.feedback_draft()
        draft["simulation"]["steps"] = 1000001
        cases.append(draft)
        for index, draft in enumerate(cases):
            with self.subTest(case=index):
                baseline = self.inspect()["revision"]
                self.refused("save", self.project, self.write_draft(draft),
                             "--expected-revision", baseline)
                self.assertEqual(self.inspect()["revision"], baseline)

    def test_malformed_duplicate_key_and_oversized_drafts_are_refused(self):
        draft = self.write_draft(self.feedback_draft())
        baseline = self.inspect()["revision"]
        valid = draft.read_text()
        cases = ("{", '{"schema":"galata.project-draft.v1",' + valid[1:],
                 valid.replace('"step_s": 0.01', '"step_s": NaN'), " " * (3 * 1024 * 1024))
        for index, text in enumerate(cases):
            with self.subTest(case=index):
                baseline = self.inspect()["revision"]
                draft.write_text(text, encoding="utf-8")
                self.refused("save", self.project, draft, "--expected-revision", baseline)
                self.assertEqual(self.inspect()["revision"], baseline)

    def test_unwired_and_type_invalid_drafts_save_but_runs_fail_with_evidence(self):
        for defect in ("unwired", "type"):
            with self.subTest(defect=defect):
                draft = self.feedback_draft()
                if defect == "unwired":
                    draft["model"]["connections"].pop()
                else:
                    draft["model"]["blocks"][-1]["output"]["frame"] = "body"
                revision = self.save(draft)["revision"]
                before = {run["id"] for run in self.inspect()["runs"]}
                self.refused("run", self.project)
                added = [run for run in self.inspect()["runs"] if run["id"] not in before]
                self.assertEqual(len(added), 1)
                self.assertEqual(added[0]["status"], "failed")
                self.assertTrue(added[0]["diagnostic"])
                run_root = self.project / "runs" / added[0]["id"]
                self.assertEqual(json.loads((run_root / "request.json").read_text())["revision"],
                                 revision)
                self.assertEqual(json.loads((run_root / "result.json").read_text())["status"],
                                 "failed")

    def test_algebraic_loop_cannot_become_completed(self):
        draft = self.feedback_draft()
        unitless = {"dimension": [0] * 8, "frame": "none"}
        draft["model"] = {"schema": "galata.model.v1", "profile": "continuous-scalar.v1",
                          "blocks": [
                              {"id": "loop", "kind": "gain", "output": unitless,
                               "coefficient": {"value": 1.0, "dimension": [0] * 8}},
                              {"id": "out", "kind": "output", "output": unitless}],
                          "connections": [
                              {"source": "loop", "target": "loop", "input": 0},
                              {"source": "loop", "target": "out", "input": 0}]}
        draft["presentation"]["positions"] = {"loop": {"x": 0, "y": 0},
                                               "out": {"x": 120, "y": 0}}
        self.save(draft)
        self.refused("run", self.project)
        runs = self.inspect()["runs"]
        self.assertEqual(len(runs), 1)
        self.assertEqual(runs[0]["status"], "failed")
        self.assertIn("loop", runs[0]["diagnostic"].lower())

    def test_tampered_trajectory_is_never_listed_as_completed(self):
        self.save(self.feedback_draft())
        run, _, _ = self.run_evidence()
        result_path = self.project / "runs" / run["id"] / "result.json"
        original_result = result_path.read_bytes()
        trajectory = self.artifact(run, "trajectory_csv")
        trajectory.write_bytes(trajectory.read_bytes() + b"tampered\n")
        listed = next(item for item in self.inspect()["runs"] if item["id"] == run["id"])
        self.assertNotEqual(listed["status"], "completed")
        self.assertTrue(listed["diagnostic"])
        self.assertEqual(result_path.read_bytes(), original_result,
                         "integrity review must preserve the originally recorded outcome")

    def test_rehashed_manifest_for_different_source_is_not_completed(self):
        self.save(self.feedback_draft())
        run, _, _ = self.run_evidence()
        manifest_path = self.artifact(run, "manifest_path")
        manifest = json.loads(manifest_path.read_text())
        source_records = [item for item in manifest["inputs"]
                          if Path(item["path"]).name == "model.yaml"]
        self.assertEqual(len(source_records), 1)
        record = source_records[0]
        # All hashes below are internally consistent, but the manifest now
        # claims to have consumed different source bytes than the owned request.
        # A comment edit preserves the equation while changing exact provenance.
        changed_source = bytes.fromhex(record["bytes_hex"]) + b"\n# different source revision\n"
        record["bytes_hex"] = changed_source.hex()
        record["size_bytes"] = len(changed_source)
        record["sha256"] = hashlib.sha256(changed_source).hexdigest()
        changed_manifest = (json.dumps(manifest, separators=(",", ":")) + "\n").encode("utf-8")
        changed_digest = hashlib.sha256(changed_manifest).hexdigest()
        changed_path = manifest_path.with_name("run-" + changed_digest + ".json")
        changed_path.write_bytes(changed_manifest)
        result_path = self.project / "runs" / run["id"] / "result.json"
        terminal = json.loads(result_path.read_text())
        terminal["artifacts"]["manifest_path"] = {
            "path": changed_path.relative_to(self.project.resolve()).as_posix(), "sha256": changed_digest}
        terminal_bytes = (json.dumps(terminal, separators=(",", ":")) + "\n").encode("utf-8")
        result_path.write_bytes(terminal_bytes)
        for artifact in terminal["artifacts"].values():
            self.assertEqual(hashlib.sha256((self.project / artifact["path"]).read_bytes()).hexdigest(),
                             artifact["sha256"])
        listed = next(item for item in self.inspect()["runs"] if item["id"] == run["id"])
        self.assertEqual(listed["status"], "invalid", listed)
        self.assertTrue(listed["diagnostic"])
        self.assertEqual(result_path.read_bytes(), terminal_bytes)

    def test_moved_project_retains_verified_run_and_owned_artifact_paths(self):
        self.save(self.feedback_draft())
        run, _, trajectory = self.run_evidence()
        destination = self.scratch / "relocated project"
        self.project.rename(destination)
        self.project = destination
        listed = next(item for item in self.inspect()["runs"] if item["id"] == run["id"])
        self.assertEqual(listed["status"], "completed", listed)
        self.assertEqual(self.artifact(listed, "trajectory_csv").read_bytes(), trajectory)
        self.assertEqual(self.artifact(listed, "evidence_json").parent,
                         self.artifact(listed, "trajectory_csv").parent)

    def test_revision_tampering_and_head_path_escape_fail_closed(self):
        head_path = self.project / "project.json"
        head = json.loads(head_path.read_text())
        revision_path = self.project / "revisions" / (head["revision"] + ".json")
        original = revision_path.read_bytes()
        revision_path.write_bytes(original + b"\n")
        self.refused("inspect", self.project)
        revision_path.write_bytes(original)
        head["revision"] = "../../outside"
        outside = self.scratch / "outside.json"
        outside.write_text("do not read or alter", encoding="utf-8")
        head_path.write_text(json.dumps(head), encoding="utf-8")
        self.refused("inspect", self.project)
        self.assertEqual(outside.read_text(), "do not read or alter")

    def test_revision_symlink_is_refused(self):
        revision_path = self.project / "revisions" / (self.inspect()["revision"] + ".json")
        outside = self.scratch / "external-revision.json"
        outside.write_bytes(revision_path.read_bytes())
        original = outside.read_bytes()
        revision_path.unlink()
        revision_path.symlink_to(outside)
        self.refused("inspect", self.project)
        self.assertEqual(outside.read_bytes(), original)

    def test_hard_kill_is_interrupted_and_working_edit_does_not_mutate_request(self):
        process, running, submitted_revision = self.start_long_job()
        draft = self.draft()
        draft["simulation"]["steps"] = 100
        draft["simulation"]["sample_stride"] = 1
        draft["model"]["blocks"][1]["coefficient"]["value"] = 2.0
        latest = self.save(draft)
        self.assertNotEqual(latest["revision"], submitted_revision)
        process.kill()
        process.communicate(timeout=10)
        self.assertNotEqual(process.returncode, 0)
        view = self.inspect()
        interrupted = next(run for run in view["runs"] if run["id"] == running["id"])
        self.assertEqual(interrupted["status"], "interrupted", interrupted)
        self.assertTrue(interrupted["diagnostic"])
        request = json.loads((self.project / "runs" / running["id"] / "request.json").read_text())
        self.assertEqual(request["revision"], submitted_revision)
        self.assertEqual(view["revision"], latest["revision"])
        # The crashed process must not leave a save/run lock requiring manual repair.
        self.save(self.draft())
        self.run_evidence()

    def test_sigterm_records_cancelled_and_preserves_request(self):
        process, running, revision = self.start_long_job()
        process.send_signal(signal.SIGTERM)
        # Cancellation is cooperative, including during provenance preflight.
        # Instrumented workers hash a much larger executable; permit the same
        # command deadline as invoke(), without relaxing the required outcome.
        process.communicate(timeout=TIMEOUT_S)
        self.assertNotEqual(process.returncode, 0)
        listed = next(run for run in self.inspect()["runs"] if run["id"] == running["id"])
        self.assertEqual(listed["status"], "cancelled", listed)
        self.assertTrue(listed["diagnostic"])
        run_root = self.project / "runs" / running["id"]
        self.assertEqual(json.loads((run_root / "request.json").read_text())["revision"], revision)
        self.assertEqual(json.loads((run_root / "result.json").read_text())["status"], "cancelled")


if __name__ == "__main__":
    unittest.main()
