"""Contract tests for the installable, machine-readable Python workflow."""

import csv
import hashlib
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))

from galata_workflow.plotting import (  # noqa: E402
    plot_ensemble_distribution,
    plot_sweep,
    plot_trajectory_playback,
    plot_time_history_from_rows,
)
from galata_workflow.workflow import GalataWorkflow  # noqa: E402


class PythonWorkflowContract(unittest.TestCase):
    def test_model_provenance_and_named_channel_plots_are_machine_readable(self):
        with tempfile.TemporaryDirectory(prefix="galata-python-contract-") as directory:
            scratch = Path(directory)
            model = scratch / "model.yaml"
            model.write_text("description: contract fixture\n", encoding="utf-8")
            loaded = GalataWorkflow().load_model(model)
            self.assertEqual(loaded.sha256, hashlib.sha256(model.read_bytes()).hexdigest())

            rows = [
                {"time_s": 0.0, "position_north_m": 0.0, "position_east_m": 1.0, "position_down_m": 0.0, "roll_rad": 0.0},
                {"time_s": 1.0, "position_north_m": 2.0, "position_east_m": 3.0, "position_down_m": -1.0, "roll_rad": 0.1},
            ]
            history = scratch / "history.svg"
            plot_time_history_from_rows(rows, ["roll_rad"], history, "roll (rad)",
                                        {"reference step": [1.0]})
            self.assertIn("roll_rad", history.read_text(encoding="utf-8"))
            self.assertIn("reference step", history.read_text(encoding="utf-8"))

            trajectory_csv = scratch / "trajectory.csv"
            with trajectory_csv.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            playback = scratch / "playback.svg"
            plot_trajectory_playback(trajectory_csv, playback)
            self.assertIn("N/E/up", playback.read_text(encoding="utf-8"))

            ensemble = scratch / "ensemble.svg"
            plot_ensemble_distribution([0.9, 1.0, 1.1], ensemble,
                                       statuses=["passed", "failed", "excluded"])
            self.assertIn("ensemble distribution", ensemble.read_text(encoding="utf-8"))
            self.assertIn("failed/excluded", ensemble.read_text(encoding="utf-8"))

            sweep = scratch / "sweep.svg"
            plot_sweep([1.0, 2.0], [3.0, 4.0], sweep, "mass_scale", "peak_error")
            self.assertIn("mass_scale", sweep.read_text(encoding="utf-8"))

    def test_workflow_operations_are_explicit_and_do_not_parse_reports(self):
        with tempfile.TemporaryDirectory(prefix="galata-python-operations-") as directory:
            study = Path(directory) / "study.yaml"
            study.write_text("stages: []\n", encoding="utf-8")
            workflow = GalataWorkflow()
            configured = workflow.configure(study, Path(directory) / "out")
            self.assertEqual(workflow.trim(configured)["operation"], "trim")
            self.assertEqual(workflow.linearize(configured)["operation"], "linearize")
            self.assertEqual(workflow.design(configured)["operation"], "design")


if __name__ == "__main__":
    unittest.main()
