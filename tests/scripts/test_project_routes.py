# SPDX-License-Identifier: Apache-2.0
"""Public presentation-route contract, with independently authored coordinates.

Presentation v2 adds up to 256 exact connection-keyed orthogonal polylines of
2 through 64 finite canvas points. It must not change model, run, origin or
draft-schema meaning. These checks exercise the CLI, retained bytes and run
identity; no GUI geometry or numerical output is used as a reference value.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()
# Wall-clock deadline for ONE CLI invocation: a hang guard, so a wedged worker
# fails here naming its own command rather than arriving as an opaque ctest kill
# of the whole module. 45 s is the uninstrumented default and is unchanged;
# tests/CMakeLists.txt scales it for the sanitizer build and records why.
TIMEOUT_S = float(os.environ.get("GALATA_PROJECT_TIMEOUT_S", "45"))


@unittest.skipUnless(os.name == "posix" and CLI.is_file(), "requires the POSIX project CLI")
class ProjectRoutes(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="galata-routes-test-")
        self.addCleanup(self.temporary.cleanup)
        self.scratch = Path(self.temporary.name)
        self.project = self.scratch / "route project.galata"
        self.first = self.command("create", self.project)

    def invoke(self, *arguments):
        return subprocess.run([str(CLI), "project", *map(str, arguments)], cwd=self.scratch,
                              capture_output=True, text=True, timeout=TIMEOUT_S)

    def command(self, *arguments):
        result = self.invoke(*arguments)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        try:
            return json.loads(result.stdout)
        except ValueError as error:
            self.fail(f"successful project command must emit JSON: {error}\n{result.stdout}")

    def draft(self, view):
        draft = {"schema": view["draft_schema"],
                 **{key: copy.deepcopy(view[key]) for key in ("model", "presentation", "simulation")}}
        if "origin_sha256" in view:
            draft["origin_sha256"] = view["origin_sha256"]
        return draft

    def route_draft(self, view=None):
        draft = self.draft(self.first if view is None else view)
        draft["presentation"]["schema"] = "galata.presentation.v2"
        draft["presentation"]["routes"] = [{
            **draft["model"]["connections"][0],
            "points": [{"x": -40, "y": 24}, {"x": 80, "y": 24},
                       {"x": 80, "y": 120}, {"x": 240, "y": 120}]}]
        return draft

    def save(self, draft, view=None):
        path = self.scratch / "draft.json"
        path.write_text(json.dumps(draft, allow_nan=False), encoding="utf-8")
        return self.command("save", self.project, path, "--expected-revision",
                            (self.first if view is None else view)["revision"])

    def snapshot(self):
        return {str(path.relative_to(self.project)): path.read_bytes()
                for path in self.project.rglob("*") if path.is_file() and not path.is_symlink()}

    def refused_draft(self, draft, view=None):
        before = self.snapshot()
        path = self.scratch / "invalid-draft.json"
        # Nonfinite JSON spellings are deliberately supplied to the strict YAML
        # boundary so rejection, rather than Python serialization, is tested.
        path.write_text(json.dumps(draft), encoding="utf-8")
        result = self.invoke("save", self.project, path, "--expected-revision",
                             (self.first if view is None else view)["revision"])
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertTrue((result.stdout + result.stderr).strip(), "refusal needs a diagnostic")
        self.assertEqual(self.snapshot(), before, "refusal changed retained project content")

    def evidence(self):
        run = self.command("run", self.project)
        self.assertEqual(run["status"], "completed", run)
        return run, json.loads(Path(run["evidence_json"]).read_text()), Path(run["trajectory_csv"]).read_bytes()

    def test_route_save_reopen_and_run_preserve_model_identity_and_trajectory(self):
        old_run, old_evidence, old_csv = self.evidence()
        before = self.snapshot()
        draft = self.route_draft()
        second = self.save(draft)
        self.assertNotEqual(second["revision"], self.first["revision"])
        self.assertEqual(second["draft_schema"], self.first["draft_schema"])
        self.assertEqual(second["model"], self.first["model"])
        self.assertEqual(second["simulation"], self.first["simulation"])
        self.assertEqual(second["presentation"], draft["presentation"])
        self.assertEqual(self.command("inspect", self.project), second)
        selected = self.command("revision", self.project, second["revision"])
        self.assertEqual(selected["draft"], draft)
        for name, data in before.items():
            if name != "project.json":
                self.assertEqual((self.project / name).read_bytes(), data, name)
        new_run, new_evidence, new_csv = self.evidence()
        self.assertEqual(old_evidence["model_semantic_sha256"], new_evidence["model_semantic_sha256"])
        self.assertEqual(old_csv, new_csv)
        self.assertEqual(old_run["revision"], self.first["revision"])
        self.assertEqual(new_run["revision"], second["revision"])
        self.assertEqual(self.command("inspect", self.project)["presentation"], draft["presentation"])

    def test_v1_bytes_remain_compatible_and_both_presentation_versions_restore(self):
        before = self.snapshot()
        self.assertEqual(self.first["presentation"]["schema"], "galata.presentation.v1")
        # Revisions bind exact submitted bytes, including key order and spacing.
        # A reserialized but equal JSON object may legitimately have a new ID.
        original_path = self.project / "revisions" / (self.first["revision"] + ".json")
        unchanged = self.command("save", self.project, original_path, "--expected-revision",
                                 self.first["revision"])
        self.assertEqual(unchanged["revision"], self.first["revision"])
        self.assertEqual(self.snapshot(), before)
        draft = self.route_draft()
        draft["presentation"]["routes"] = []
        second = self.save(draft)
        retained = self.snapshot()
        for target, current in ((self.first, second), (second, self.first)):
            restored = self.command("restore", self.project, target["revision"],
                                    "--expected-revision", current["revision"])
            self.assertEqual(self.draft(restored), self.draft(target))
            for name, data in retained.items():
                if name != "project.json":
                    self.assertEqual((self.project / name).read_bytes(), data, name)
        history = self.command("revisions", self.project)
        self.assertTrue(all(entry["status"] == "valid" for entry in history["revisions"]))
        back_to_v1 = self.save(self.draft(self.first), second)
        self.assertEqual(self.draft(back_to_v1), self.draft(self.first))
        self.assertEqual(original_path.read_bytes(), before[str(original_path.relative_to(self.project))])
        submitted = (self.scratch / "draft.json").read_bytes()
        self.assertEqual(back_to_v1["revision"], hashlib.sha256(submitted).hexdigest())

    def test_invalid_route_shapes_coordinates_and_closed_keys_preserve_head(self):
        cases = {
            "missing points": lambda route: route.pop("points"),
            "points map": lambda route: route.update(points={}),
            "empty points": lambda route: route.update(points=[]),
            "one point": lambda route: route.update(points=[{"x": 0, "y": 0}]),
            "diagonal": lambda route: route.update(points=[{"x": 0, "y": 0}, {"x": 1, "y": 1}]),
            "zero segment": lambda route: route.update(points=[{"x": 0, "y": 0}] * 2),
            "missing coordinate": lambda route: route["points"][0].pop("x"),
            "unknown coordinate": lambda route: route["points"][0].update(z=0),
            "unknown route key": lambda route: route.update(weight=1),
            "string coordinate": lambda route: route["points"][0].update(x="-40"),
            "boolean coordinate": lambda route: route["points"][0].update(x=True),
            "nan": lambda route: route["points"][0].update(x=float("nan")),
            "infinity": lambda route: route["points"][0].update(x=float("inf")),
            "above canvas bound": lambda route: route["points"][0].update(x=100001),
            "below canvas bound": lambda route: route["points"][0].update(x=-100001),
        }
        for name, mutate in cases.items():
            with self.subTest(case=name):
                draft = self.route_draft()
                mutate(draft["presentation"]["routes"][0])
                self.refused_draft(draft)

    def test_unknown_duplicate_and_stale_connection_routes_preserve_head(self):
        cases = {
            "unknown source": lambda route: route.update(source="absent"),
            "unknown target": lambda route: route.update(target="absent"),
            "wrong existing connection": lambda route: route.update(input=17),
            "fractional input": lambda route: route.update(input=0.5),
            "floating integer input": lambda route: route.update(input=0.0),
            "string input": lambda route: route.update(input="0"),
            "boolean input": lambda route: route.update(input=False),
            "negative input": lambda route: route.update(input=-1),
            "overflow input": lambda route: route.update(input=2**64),
        }
        for name, mutate in cases.items():
            with self.subTest(case=name):
                draft = self.route_draft()
                mutate(draft["presentation"]["routes"][0])
                self.refused_draft(draft)
        duplicate = self.route_draft()
        duplicate["presentation"]["routes"] *= 2
        self.refused_draft(duplicate)
        stale = self.route_draft()
        stale["model"]["connections"].pop(0)
        self.refused_draft(stale)

    def test_version_contract_requires_routes_array_and_rejects_v1_route_payload(self):
        for schema, routes in (("galata.presentation.v1", []), ("galata.presentation.v999", []),
                               ("galata.presentation.v2", None), ("galata.presentation.v2", {})):
            with self.subTest(schema=schema, routes=routes):
                draft = self.route_draft()
                draft["presentation"].update(schema=schema, routes=routes)
                self.refused_draft(draft)
        missing = self.route_draft()
        missing["presentation"].pop("routes")
        self.refused_draft(missing)

    def test_point_and_route_count_boundaries_and_large_exact_input(self):
        draft = self.route_draft()
        points = [{"x": (index + 1) // 2, "y": index // 2} for index in range(64)]
        points[0]["x"] = -100000
        points[-1]["x"] = 100000
        draft["presentation"]["routes"][0]["points"] = points
        at_limit = self.save(draft)
        self.assertEqual(at_limit["presentation"]["routes"][0]["points"], points)
        too_many = copy.deepcopy(draft)
        too_many["presentation"]["routes"][0]["points"].append({"x": 100000, "y": 100000})
        self.refused_draft(too_many, at_limit)

        # Draft wiring permits an exact index outside actual block ports. The
        # presentation key must preserve it without rounding through binary64.
        wide = self.route_draft(at_limit)
        wide["model"]["connections"][0]["input"] = 2**53 + 1
        wide["presentation"]["routes"][0]["input"] = 2**53 + 1
        wide_view = self.save(wide, at_limit)
        self.assertEqual(wide_view["presentation"]["routes"][0]["input"], 2**53 + 1)
        mismatch = copy.deepcopy(wide)
        mismatch["presentation"]["routes"][0]["input"] -= 1
        self.refused_draft(mismatch, wide_view)

        dimensionless = {"dimension": [0] * 8, "frame": "none"}
        bounded = self.route_draft(wide_view)
        bounded["model"]["blocks"] = [
            {"id": "source", "kind": "constant", "output": dimensionless, "value": 1},
            *[{"id": f"sink_{index}", "kind": "output", "output": dimensionless}
              for index in range(257)]]
        bounded["model"]["connections"] = [
            {"source": "source", "target": f"sink_{index}", "input": 0} for index in range(257)]
        bounded["presentation"]["positions"] = {}
        bounded["presentation"]["routes"] = [
            {**wire, "points": [{"x": 0, "y": 0}, {"x": 20, "y": 0}]}
            for wire in bounded["model"]["connections"][:256]]
        bounded_view = self.save(bounded, wide_view)
        self.assertEqual(len(bounded_view["presentation"]["routes"]), 256)
        bounded["presentation"]["routes"].append({
            **bounded["model"]["connections"][-1],
            "points": [{"x": 0, "y": 0}, {"x": 20, "y": 0}]})
        self.refused_draft(bounded, bounded_view)

    def test_imported_origin_and_draft_schema_survive_route_edit_and_restore(self):
        source = self.scratch / "source"
        source.mkdir()
        (source / "plant.yaml").write_text(
            "description: Independent route transport fixture\ncitation: Analytic xdot=u\nunits: SI\n"
            "states: [distance]\ninputs: [speed]\noutputs: [distance]\n"
            "a: [[0]]\nb: [[1]]\nc: [[1]]\nd: [[0]]\n", encoding="utf-8")
        distance = {"dimension": [1, 0, 0, 0, 0, 0, 0, 0], "frame": "none"}
        speed = {"dimension": [1, 0, -1, 0, 0, 0, 0, 0], "frame": "none"}
        study = {"version": 1, "stages": [
            {"id": "source", "capability": "model.linear.statespace", "input": {"path": "plant.yaml"}},
            {"id": "graph", "capability": "model.linear_graph", "input": {
                "system": {"from": "source"},
                "channel_types": {"states": [distance], "inputs": [speed], "outputs": [distance]},
                "initial_state": [0], "command": [1],
                "model_path": "model.yaml", "adapter_path": "adapter.json"}},
            {"id": "response", "capability": "sim.model", "input": {
                "model": {"from": "graph"}, "step_s": 0.125, "steps": 2,
                "csv_path": "response.csv", "evidence_path": "evidence.json"}}]}
        study_path = source / "study.json"
        study_path.write_text(json.dumps(study), encoding="utf-8")
        self.project = self.scratch / "imported routes.galata"
        imported = self.command("import-linear", self.project, study_path)
        before = self.snapshot()
        routed = self.save(self.route_draft(imported), imported)
        self.assertEqual(routed["draft_schema"], "galata.project-draft.v2")
        self.assertEqual(routed["origin_sha256"], imported["origin_sha256"])
        self.assertEqual(routed["origin"], imported["origin"])
        self.assertEqual(routed["origin"]["relation"], "matches_imported_model")
        for name, data in before.items():
            if name != "project.json":
                self.assertEqual(hashlib.sha256((self.project / name).read_bytes()).digest(),
                                 hashlib.sha256(data).digest(), name)
        for target, current in ((imported, routed), (routed, imported)):
            restored = self.command("restore", self.project, target["revision"],
                                    "--expected-revision", current["revision"])
            self.assertEqual(self.draft(restored), self.draft(target))
            self.assertEqual(restored["origin"], imported["origin"])


if __name__ == "__main__":
    unittest.main()
