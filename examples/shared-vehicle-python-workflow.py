#!/usr/bin/env python3
"""Run one composed Python workflow through all built-in vehicle adapters.

The operations are intentionally written here, rather than hidden in a
pre-authored YAML study: each family goes through load -> configure -> trim ->
linearize -> design -> disturbed simulation -> evaluation -> plotting -> export.
The C++ CLI remains the numerical authority.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from galata_workflow import GalataWorkflow
from galata_workflow.plotting import (
    plot_open_closed,
    plot_trajectory_playback,
    plot_trajectory_projection,
    plot_time_history,
)


CASES = {
    "fixed-wing": ("models/nt33a/nt33a-fc1.yaml", 69.4944, 0.0, "quaternion_x"),
    "multirotor": ("examples/quadrotor-sampled-control/quad-heterogeneous.yaml", 0.0, 120.0,
                   "quaternion_x"),
    "helicopter": ("models/souxmar-heli/souxmar-heli.yaml", 0.0, 100.0, "roll_rad"),
}


def run_case(workflow: GalataWorkflow, repo_root: Path, output_root: Path, kind: str, source: str,
             airspeed: float, altitude: float, signal: str) -> dict:
    model = workflow.load_model(repo_root / source)
    mass = float(model.parameters["mass.mass_kg"]["value"])
    study = workflow.configure(
        model, output_root / kind,
        parameter_overrides={"mass.mass_kg": mass * 1.001},
        flight_condition={"airspeed_m_s": airspeed, "altitude_m": altitude},
    )
    trim = workflow.trim(study)
    linear = workflow.linearize(trim)
    controller = workflow.design(linear)
    response_requirements = {
        "signal_requirements": {
            "roll_rad": {"settling_band_rad": 0.2, "settling_dwell_s": 0.05,
                          "settling_time_s": 5.0},
            "pitch_rad": {"settling_band_rad": 0.2, "settling_dwell_s": 0.05,
                           "settling_time_s": 5.0},
        }
    }
    simulation = workflow.simulate(
        controller,
        attitude_perturbation_body_rad=(0.04, -0.03, 0.0),
        initial_state_perturbation={"roll_rate_rad_s": 0.02},
        steps=500,
        sample_stride=10,
        response_requirements=response_requirements,
    )
    evaluation = workflow.evaluate(simulation)
    repeated = workflow.simulate(
        controller,
        attitude_perturbation_body_rad=(0.04, -0.03, 0.0),
        initial_state_perturbation={"roll_rate_rad_s": 0.02},
        steps=500,
        sample_stride=10,
        response_requirements=response_requirements,
    )
    comparison = workflow.compare(
        simulation.run, repeated.run,
        outputs=["response.json", "open.csv", "closed.csv"],
    )

    output = study.output_dir / "plots"
    output.mkdir(parents=True, exist_ok=True)
    history_signal = f"output_{signal}" if kind == "helicopter" else f"output:{signal}"
    history = plot_time_history(simulation.closed_csv, [history_signal],
                                output / "response.svg", f"{kind} response")
    open_closed = plot_open_closed(simulation.open_csv, simulation.closed_csv, signal,
                                   output / "open-closed.svg")
    projection = plot_trajectory_projection(simulation.closed_csv, output / "trajectory.svg")
    playback = plot_trajectory_playback(simulation.closed_csv, output / "playback.html")
    bundle = workflow.export(
        simulation, study.output_dir / "bundle",
        outputs=["response.json", "open.csv", "closed.csv"],
        plots=[history, open_closed, projection, playback],
    )
    return {
        "kind": kind,
        "model_sha256": model.sha256,
        "parameter_overrides": dict(study.parameter_overrides),
        "trim_residual": trim.get("diagnostics", {}).get("residual_norm",
                                                             trim.get("residual_norm")),
        "linearization_shape": [len(linear["a"]), len(linear["a"][0])],
        "controller_shape": [len(controller["gain_k"]), len(controller["gain_k"][0])],
        "evaluation": evaluation,
        "serial_repeat_bitwise": comparison["bit_identical_outputs"],
        "bundle": str(bundle),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("build/shared-python-workflow"))
    parser.add_argument("--only", choices=sorted(CASES), action="append")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    workflow = GalataWorkflow(args.executable)
    selected = args.only or list(CASES)
    output_root = args.output_dir.expanduser().resolve()
    summaries = [run_case(workflow, root, output_root, kind, *CASES[kind]) for kind in selected]
    print(json.dumps(summaries, indent=2))
    return 0 if all(item["serial_repeat_bitwise"] for item in summaries) else 1


if __name__ == "__main__":
    raise SystemExit(main())
