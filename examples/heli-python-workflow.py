#!/usr/bin/env python3
"""Executable Python engineering workflow backed by Galata's C++ capabilities.

Run from any directory after building Galata, for example:
  python examples/heli-python-workflow.py --executable /abs/path/galata
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from galata_workflow import GalataWorkflow, load_csv
from galata_workflow.plotting import (
    plot_open_closed,
    plot_trajectory_playback,
    plot_trajectory_projection,
    plot_time_history,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("build/python-workflow"))
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    workflow = GalataWorkflow(args.executable)
    model = workflow.load_model(root / "models/souxmar-heli/souxmar-heli.yaml")
    study = workflow.configure(
        model,
        args.output_dir,
        parameter_overrides={"mass.mass_kg": 2830.0 * 1.01},
        flight_condition={"airspeed_m_s": 0.0, "altitude_m": 100.0},
    )
    trim = workflow.trim(study)
    linearization = workflow.linearize(trim)
    controller = workflow.design(linearization)
    simulation = workflow.simulate(
        controller,
        attitude_perturbation_body_rad=(0.04, -0.03, 0.0),
        initial_state_perturbation={"roll_rate_rad_s": 0.08, "pitch_rate_rad_s": -0.06},
        steps=1000,
        sample_stride=10,
    )
    evaluation = workflow.evaluate(simulation)
    repeated = workflow.simulate(
        controller,
        attitude_perturbation_body_rad=(0.04, -0.03, 0.0),
        initial_state_perturbation={"roll_rate_rad_s": 0.08, "pitch_rate_rad_s": -0.06},
        steps=1000,
        sample_stride=10,
    )
    comparison = workflow.compare(
        simulation.run, repeated.run, outputs=["response.json", "open.csv", "closed.csv"])

    args.output_dir.mkdir(parents=True, exist_ok=True)
    attitude_plot = plot_time_history(
        simulation.closed_csv, ["output_roll_rad", "output_pitch_rad"],
        args.output_dir / "attitude.svg", "closed-loop attitude response")
    comparison_plot = plot_open_closed(
        simulation.open_csv, simulation.closed_csv, "roll_rad",
        args.output_dir / "open-closed-roll.svg")
    projection = plot_trajectory_projection(
        simulation.closed_csv, args.output_dir / "trajectory-projection.svg")
    playback = plot_trajectory_playback(
        simulation.closed_csv, args.output_dir / "trajectory-playback.html")
    bundle = workflow.export(
        simulation, args.output_dir / "bundle",
        outputs=["trim.json", "linear.json", "controller.json", "response.json",
                 "open.csv", "closed.csv"],
        plots=[attitude_plot, comparison_plot, projection, playback],
        overwrite=False)

    summary = {
        "model_sha256": model.sha256,
        "parameter_overrides": study.parameter_overrides,
        "trim_residual_norm": trim["diagnostics"]["residual_norm"],
        "linearization_shape": [len(linearization["a"]), len(linearization["a"][0])],
        "controller_gain_shape": [len(controller["gain_k"]), len(controller["gain_k"][0])],
        "trajectory_samples": len(load_csv(simulation.closed_csv)),
        "evaluation": evaluation,
        "cli_python_agreement": comparison,
        "plots": [str(attitude_plot), str(comparison_plot), str(projection), str(playback)],
        "bundle": str(bundle),
    }
    print(json.dumps(summary, indent=2))
    return 0 if evaluation["execution_completed"] and comparison["bit_identical_outputs"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
