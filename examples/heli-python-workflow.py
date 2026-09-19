#!/usr/bin/env python3
"""Executable load -> configure -> trim -> linearize -> design -> simulate
-> evaluate -> compare -> plot -> export workflow."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from galata_workflow import GalataWorkflow
from galata_workflow.plotting import plot_open_closed, plot_time_history, plot_trajectory_playback


root = Path(__file__).resolve().parents[1]
workflow = GalataWorkflow(root / "build/dev/src/cli/galata")
model = workflow.load_model(root / "models/souxmar-heli/souxmar-heli.yaml")
study = workflow.configure(root / "examples/heli-performance-acceptance/study.yaml",
                           root / "build/python-workflow")
trim = workflow.trim(study)
linear = workflow.linearize(study)
design = workflow.design(study)
first = workflow.simulate(study)
evaluation = workflow.evaluate(first)
second = workflow.simulate(study)
comparison = workflow.compare(first, second)
output_dir = root / "build/python-workflow"
plot_time_history(first.output("closed.csv"), ["output_roll_rad", "output_pitch_rad"],
                  output_dir / "attitude.svg", "closed-loop attitude response")
plot_open_closed(first.output("open.csv"), first.output("closed.csv"), "roll_rad",
                 output_dir / "open-closed-roll.svg")
plot_trajectory_playback(first.output("closed.csv"), output_dir / "trajectory-playback.svg")
manifest_copy = workflow.export(first, output_dir / "exported")

print({"model_sha256": model.sha256, "trim": trim, "linearize": linear,
       "design": design, "evaluation": evaluation, "cli_agreement": comparison,
       "manifest": str(manifest_copy)})
