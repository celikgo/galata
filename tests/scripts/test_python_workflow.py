"""Executable acceptance tests for the installed Python engineering workflow."""

from __future__ import annotations

import csv
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from galata_workflow import (  # noqa: E402
    ArtifactError,
    GalataWorkflow,
    InvalidParameterError,
    Run,
)
from galata_workflow.plotting import (  # noqa: E402
    plot_ensemble_member_values,
    plot_open_closed,
    plot_sweep,
    plot_trajectory_playback,
    plot_trajectory_projection,
    plot_time_history,
    plot_time_history_from_rows,
)


EXECUTABLE = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()
MODEL = ROOT / "models/souxmar-heli/souxmar-heli.yaml"
SHARED_MODELS = (
    ("fixed-wing", ROOT / "models/nt33a/nt33a-fc1.yaml", 69.4944, 0.0),
    ("multirotor", ROOT / "examples/quadrotor-sampled-control/quad-heterogeneous.yaml", 0.0, 120.0),
    ("helicopter", MODEL, 0.0, 100.0),
)


def _fake_run(directory: Path, files: dict[str, str]) -> Run:
    output_dir = directory / "results"
    output_dir.mkdir(parents=True)
    outputs = []
    for name, content in files.items():
        path = output_dir / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        outputs.append({"path": name, "sha256": hashlib.sha256(content.encode()).hexdigest()})
    manifest_path = output_dir / "manifest.json"
    manifest = {"status": "completed", "output_directory": str(output_dir), "outputs": outputs}
    manifest_path.write_text(json.dumps(manifest))
    return Run(manifest_path, manifest)


@unittest.skipUnless(EXECUTABLE.is_file(), f"requires the POSIX project CLI: {EXECUTABLE}")
class PythonWorkflowAcceptance(unittest.TestCase):
    def test_shared_vehicle_operations_execute_for_all_families(self):
        with tempfile.TemporaryDirectory(prefix="galata-shared-python-") as directory:
            root = Path(directory)
            workflow = GalataWorkflow(EXECUTABLE)
            for kind, path, airspeed, altitude in SHARED_MODELS:
                with self.subTest(kind=kind):
                    model = workflow.load_model(path)
                    original_mass = float(model.parameters["mass.mass_kg"]["value"])
                    override = {"mass.mass_kg": original_mass * 1.001}
                    study = workflow.configure(
                        model, root / kind,
                        parameter_overrides=override,
                        flight_condition={"airspeed_m_s": airspeed, "altitude_m": altitude})
                    trim = workflow.trim(study)
                    residual = trim.get("diagnostics", {}).get("residual_norm",
                                                               trim.get("residual_norm"))
                    self.assertIsNotNone(residual)
                    self.assertLess(float(residual), 1e-7)
                    self.assertEqual(trim["parameter_overrides"], override)
                    linear = workflow.linearize(trim)
                    self.assertGreater(len(linear["a"]), 0)
                    self.assertGreater(len(linear["b"]), 0)
                    controller = workflow.design(linear)
                    self.assertGreater(len(controller["gain_k"]), 0)
                    simulation = workflow.simulate(
                        controller,
                        initial_state_perturbation={"roll_rate_rad_s": 0.02},
                        steps=100, sample_stride=10,
                        response_requirements={
                            "signal_requirements": {
                                "roll_rad": {"settling_band_rad": 0.2,
                                              "settling_dwell_s": 0.02,
                                              "settling_time_s": 1.0},
                                "pitch_rad": {"settling_band_rad": 0.2,
                                               "settling_dwell_s": 0.02,
                                               "settling_time_s": 1.0}}})
                    evaluation = workflow.evaluate(simulation)
                    self.assertTrue(evaluation["execution_completed"])
                    if kind != "helicopter":
                        self.assertEqual(simulation["criteria_status"], "pass")
                    else:
                        self.assertIn("criteria_passed", simulation.data)
                    self.assertTrue(simulation.closed_csv.is_file())
                    header = simulation.closed_csv.read_text().splitlines()[0]
                    if kind == "helicopter":
                        self.assertIn("output_roll_rad", header)
                    else:
                        self.assertIn("state:", header)

    def test_composed_operations_are_real_and_cli_agrees(self):
        with tempfile.TemporaryDirectory(prefix="galata-python-acceptance-") as directory:
            scratch = Path(directory)
            workflow = GalataWorkflow(EXECUTABLE)
            model = workflow.load_model(MODEL)
            self.assertEqual(model.sha256, hashlib.sha256(MODEL.read_bytes()).hexdigest())
            self.assertGreater(len(model.states), 10)
            self.assertIn("mass.mass_kg", model.parameters)
            with self.assertRaises(InvalidParameterError):
                workflow.configure(model, scratch / "invalid", {"not.a.parameter": 1.0})

            study = workflow.configure(
                model, scratch / "composed", {"mass.mass_kg": 2830.0},
                {"airspeed_m_s": 0.0, "altitude_m": 100.0})
            trim = workflow.trim(study)
            self.assertEqual(trim["schema"], "galata.helicopter.trim.v1")
            self.assertLess(trim["diagnostics"]["residual_norm"], 1e-8)
            self.assertEqual(trim["parameter_overrides"]["mass.mass_kg"], 2830.0)
            linear = workflow.linearize(trim)
            self.assertEqual(len(linear["a"]), len(linear["state_names"]))
            self.assertEqual(len(linear["b"][0]), len(linear["input_names"]))
            controller = workflow.design(linear)
            self.assertEqual(len(controller["gain_k"]), len(controller["plant_input_names"]))
            simulation = workflow.simulate(controller, steps=1000, sample_stride=10)
            evaluation = workflow.evaluate(simulation)
            self.assertTrue(evaluation["execution_completed"])
            self.assertEqual(simulation["schema"], "galata.helicopter.response.v1")
            self.assertEqual(len(simulation["metrics"]), 2)
            self.assertTrue(simulation.closed_csv.stat().st_size > 1000)

            # Run the equivalent composed trim directly through the CLI and
            # compare numerical fields, independent of Python's orchestration.
            direct_pipeline = scratch / "direct.json"
            direct_pipeline.write_text(json.dumps({"version": 1, "stages": [
                {"id": "aircraft", "capability": "model.helicopter",
                 "input": {"path": str(MODEL), "parameter_overrides": {"mass.mass_kg": 2830.0}}},
                {"id": "trim", "capability": "trim.helicopter",
                 "input": {"helicopter": {"from": "aircraft"}, "airspeed_m_s": 0.0,
                           "altitude_m": 100.0}},
                {"id": "report", "capability": "report.helicopter_trim_json",
                 "input": {"trim": {"from": "trim"}, "path": "trim.json"}},
            ]}))
            direct_dir = scratch / "direct-output"
            direct = subprocess.run([str(EXECUTABLE), "run", str(direct_pipeline),
                                     "--output-dir", str(direct_dir), "--json"],
                                    check=True, capture_output=True, text=True)
            direct_manifest = Path(json.loads(direct.stdout)["manifest_path"])
            direct_trim = json.loads((direct_manifest.parent / "trim.json").read_text())
            self.assertAlmostEqual(direct_trim["diagnostics"]["residual_norm"],
                                   trim["diagnostics"]["residual_norm"], places=14)

            plot = scratch / "result.svg"
            plot_time_history(simulation.closed_csv, ["output_roll_rad"], plot)
            bundle = workflow.export(simulation, scratch / "bundle",
                                     outputs=["response.json", "open.csv", "closed.csv"],
                                     plots=[plot])
            bundle_metadata = json.loads((bundle / "bundle.json").read_text())
            self.assertEqual(bundle_metadata["schema"], "galata.bundle.v1")
            self.assertTrue(bundle_metadata["configuration"])
            self.assertTrue(bundle_metadata["plots"])
            self.assertTrue((bundle / "provenance.json").is_file())
            self.assertTrue((bundle / "results" / "closed.csv").is_file())
            with self.assertRaises(FileExistsError):
                workflow.export(simulation, bundle, outputs=["response.json"])

    def test_full_comparison_requires_the_same_nonempty_output_set(self):
        with tempfile.TemporaryDirectory(prefix="galata-comparison-") as directory:
            root = Path(directory)
            first = _fake_run(root / "first", {"a.csv": "time_s,value\n0,1\n"})
            same = _fake_run(root / "same", {"a.csv": "time_s,value\n0,1\n"})
            changed = _fake_run(root / "changed", {"a.csv": "time_s,value\n0,2\n"})
            extra = _fake_run(root / "extra", {"a.csv": "time_s,value\n0,1\n", "b.csv": "x\n1\n"})
            missing = _fake_run(root / "missing", {})
            self.assertTrue(GalataWorkflow.compare(first, same)["bit_identical_outputs"])
            self.assertFalse(GalataWorkflow.compare(first, changed)["bit_identical_outputs"])
            self.assertFalse(GalataWorkflow.compare(first, extra)["bit_identical_outputs"])
            self.assertFalse(GalataWorkflow.compare(first, missing)["bit_identical_outputs"])
            self.assertFalse(GalataWorkflow.compare(missing, missing)["bit_identical_outputs"])
            selected_missing = GalataWorkflow.compare(first, missing, outputs=["a.csv"])
            self.assertFalse(selected_missing["bit_identical_outputs"])
            self.assertEqual(selected_missing["missing_from_second"], ["a.csv"])
            self.assertTrue(GalataWorkflow.compare_subset(first, extra, ["a.csv"])["bit_identical"])
            self.assertFalse(GalataWorkflow.compare_numeric(first, changed, ["a.csv"], atol=0.1)["passed"])

    def test_plot_contracts_and_attitude_playback(self):
        with tempfile.TemporaryDirectory(prefix="galata-plot-contract-") as directory:
            root = Path(directory)
            rows = [{"time_s": 0.0, "roll_rad": 0.0, "position_north_m": 0.0,
                     "position_east_m": 0.0, "position_down_m": 0.0,
                     "quaternion_w": 1.0, "quaternion_x": 0.0, "quaternion_y": 0.0,
                     "quaternion_z": 0.0},
                    {"time_s": 1.0, "roll_rad": None, "position_north_m": 1.0,
                     "position_east_m": 0.0, "position_down_m": -1.0,
                     "quaternion_w": 1.0, "quaternion_x": 0.0, "quaternion_y": 0.0,
                     "quaternion_z": 0.0},
                    {"time_s": 2.0, "roll_rad": 0.2, "position_north_m": 2.0,
                     "position_east_m": 0.0, "position_down_m": -2.0,
                     "quaternion_w": 1.0, "quaternion_x": 0.0, "quaternion_y": 0.0,
                     "quaternion_z": 0.0}]
            history = root / "history.svg"
            plot_time_history_from_rows(rows, ["roll_rad"], history, "roll")
            self.assertGreaterEqual(history.read_text().count("polyline"), 2)

            open_csv, closed_csv = root / "open.csv", root / "closed.csv"
            for path, times in ((open_csv, [0.0, 1.0]), (closed_csv, [0.0, 0.5, 1.0])):
                with path.open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(["time_s", "roll_rad"])
                    for time in times:
                        writer.writerow([time, time])
            with self.assertRaises(ValueError):
                plot_open_closed(open_csv, closed_csv, "roll_rad", root / "bad.svg")
            plot_open_closed(open_csv, closed_csv, "roll_rad", root / "aligned.svg", alignment="linear")

            sweep = root / "sweep.svg"
            plot_sweep([1.0, 2.0], [3.0, 4.0], sweep, "mass_kg", "peak_error", "kg", "rad")
            self.assertIn("mass_kg [kg]", sweep.read_text())
            ensemble = root / "ensemble.svg"
            plot_ensemble_member_values([0.9, 1.0, 1.1], ensemble,
                                        statuses=["passed", "failed", "excluded"],
                                        included=[True, False, False], denominator=1)
            ensemble_text = ensemble.read_text()
            self.assertIn("not a population distribution", ensemble_text)
            self.assertIn("included=1/3", ensemble_text)

            playback_csv = root / "trajectory.csv"
            with playback_csv.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows([{key: (0.0 if value is None else value) for key, value in row.items()}
                                  for row in rows])
            projection = root / "projection.svg"
            plot_trajectory_projection(playback_csv, projection)
            playback = root / "playback.html"
            plot_trajectory_playback(playback_csv, playback)
            text = playback.read_text()
            self.assertIn('type="range"', text)
            self.assertIn("quaternion", text)
            self.assertIn("up = -down", text)

    def test_installed_package_executes_outside_repository_and_rejects_missing_artifacts(self):
        with tempfile.TemporaryDirectory(prefix="galata-install-") as directory:
            target = Path(directory) / "site"
            subprocess.run([sys.executable, "-m", "pip", "install", "--no-deps",
                            "--target", str(target), str(ROOT)], check=True,
                           capture_output=True, text=True)
            script = f"""
from pathlib import Path
from galata_workflow import ArtifactError, GalataWorkflow
from galata_workflow.plotting import plot_time_history
workflow = GalataWorkflow({str(EXECUTABLE)!r})
model = workflow.load_model({str(MODEL)!r})
study = workflow.configure(model, Path('outside-repo-run'), {{'mass.mass_kg': 2830.0}})
trim = workflow.trim(study)
linear = workflow.linearize(trim)
controller = workflow.design(linear)
simulation = workflow.simulate(controller, steps=100, sample_stride=10)
assert trim['diagnostics']['residual_norm'] < 1e-8
assert len(linear['a']) == 16 and len(controller['gain_k']) == 4
assert simulation.closed_csv.stat().st_size > 1000
plot = Path('outside-repo.svg')
plot_time_history(simulation.closed_csv, ['output_roll_rad'], plot)
bundle = workflow.export(simulation, Path('outside-repo-bundle'), plots=[plot])
assert (bundle / 'configuration').is_dir()
assert (bundle / 'provenance.json').is_file()
try:
    workflow.trim(object())
except ArtifactError:
    print('structured-error')
else:
    raise SystemExit('missing artifact was accepted')
"""
            env = dict(os.environ)
            env["PYTHONPATH"] = str(target)
            result = subprocess.run([sys.executable, "-c", script], cwd=directory,
                                    env=env, capture_output=True, text=True, check=True)
            self.assertIn("structured-error", result.stdout)


if __name__ == "__main__":
    unittest.main()
