#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Record representative public-CLI project workloads, without acceptance limits.

The desktop launches these same project commands. This measures process startup,
worker execution, persistence and verified review together; it does not measure
canvas rendering or prove responsiveness, numerical accuracy or model validity.
Synthetic channels solve xdot = command_rate - x/(1 s), with command_rate in m/s.
Run in a new/empty output directory; keep its projects for reproducible review.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
WORKLOADS = (
    {"name": "scalar-1", "channels": 1, "steps": 500, "sample_stride": 1},
    {"name": "scalar-16", "channels": 16, "steps": 2000, "sample_stride": 10},
    {"name": "scalar-64", "channels": 64, "steps": 4000, "sample_stride": 20},
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def json_bytes(value):
    return (json.dumps(value, allow_nan=False, sort_keys=True, indent=2) + "\n").encode()


def synthetic_draft(workload):
    length = {"dimension": [1, 0, 0, 0, 0, 0, 0, 0], "frame": "none"}
    velocity = {"dimension": [1, 0, -1, 0, 0, 0, 0, 0], "frame": "none"}
    blocks, connections, positions = [], [], {}
    for index in range(workload["channels"]):
        names = {name: f"{name}_{index:03d}" for name in ("command", "gain", "rate", "state", "scope")}
        blocks.extend([
            {"id": names["command"], "kind": "constant", "output": velocity,
             "value": 1 + index / 8},
            {"id": names["gain"], "kind": "gain", "output": velocity,
             "coefficient": {"value": 1, "dimension": [0, 0, -1, 0, 0, 0, 0, 0]}},
            {"id": names["rate"], "kind": "sum", "output": velocity, "signs": [1, -1]},
            {"id": names["state"], "kind": "integrator", "output": length, "initial_value": 0},
            {"id": names["scope"], "kind": "output", "output": length},
        ])
        connections.extend([
            {"source": names["command"], "target": names["rate"], "input": 0},
            {"source": names["gain"], "target": names["rate"], "input": 1},
            {"source": names["rate"], "target": names["state"], "input": 0},
            {"source": names["state"], "target": names["gain"], "input": 0},
            {"source": names["state"], "target": names["scope"], "input": 0},
        ])
        for column, name in enumerate(names.values()):
            positions[name] = {"x": column * 180, "y": index * 90}
    return {
        "schema": "galata.project-draft.v1",
        "model": {"schema": "galata.model.v1", "profile": "continuous-scalar.v1",
                  "blocks": blocks, "connections": connections},
        "presentation": {"schema": "galata.presentation.v1", "positions": positions},
        "simulation": {"initial_time_s": 0, "step_s": 0.01, "steps": workload["steps"],
                       "sample_stride": workload["sample_stride"]},
    }


def invoke(cli, timeout, measurements, operation, *arguments, repetition=None):
    started = time.perf_counter_ns()
    result = subprocess.run([str(cli), "project", *map(str, arguments)], capture_output=True,
                            text=True, timeout=timeout)
    elapsed = time.perf_counter_ns() - started
    measurements.append({"operation": operation, "repetition": repetition,
                         "wall_time_ns": elapsed, "exit_code": result.returncode,
                         "stdout_bytes": len(result.stdout.encode()),
                         "stderr_bytes": len(result.stderr.encode())})
    if result.returncode:
        raise RuntimeError(f"{operation} failed ({result.returncode}):\n{result.stdout}{result.stderr}")
    try:
        return json.loads(result.stdout)
    except ValueError as error:
        raise RuntimeError(f"{operation} did not return JSON: {error}") from error


def output_metrics(run):
    csv_path = Path(run["trajectory_csv"])
    with csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        columns = next(reader)
        row_count = sum(1 for _ in reader)
    manifest_bytes = Path(run["manifest_path"]).read_bytes()
    manifest = json.loads(manifest_bytes)
    evidence = json.loads(Path(run["evidence_json"]).read_bytes())
    return {
        "run_id": run["id"], "revision": run["revision"], "status": run["status"],
        "trajectory_rows": row_count, "trajectory_columns": len(columns),
        "state_count": sum(name.startswith("state:") for name in columns),
        "output_count": sum(name.startswith("output:") for name in columns),
        "trajectory_bytes": csv_path.stat().st_size,
        "evidence_bytes": Path(run["evidence_json"]).stat().st_size,
        "manifest_bytes": len(manifest_bytes), "manifest_sha256": digest(manifest_bytes),
        "generated_study_sha256": manifest["study"]["sha256"],
        "model_semantic_sha256": evidence["model_semantic_sha256"],
        "source_records": [{"path": record["path"], "sha256": record["sha256"],
                            "size_bytes": record["size_bytes"]} for record in manifest["inputs"]],
    }


def measure(cli, output, timeout, repetitions, workload, observations, study=None):
    measurements, runs = [], []
    project = output / (workload["name"] + ".galata")
    case = {"definition": workload, "project_path": str(project),
            "status": "running", "measurements": measurements, "runs": runs}
    observations.append(case)
    if study is None:
        initial = invoke(cli, timeout, measurements, "create", "create", project)
        draft_bytes = json_bytes(synthetic_draft(workload))
        draft_path = output / (workload["name"] + "-draft.json")
        draft_path.write_bytes(draft_bytes)
        case["draft_sha256"] = digest(draft_bytes)
        view = invoke(cli, timeout, measurements, "save", "save", project, draft_path,
                      "--expected-revision", initial["revision"])
    else:
        source_bytes = study.read_bytes()
        case["import_study_sha256"] = digest(source_bytes)
        case["import_study_path"] = str(study)
        view = invoke(cli, timeout, measurements, "import", "import-linear", project, study)
        case["origin_sha256"] = view["origin_sha256"]
        case["source_records"] = [{"path": record["path"], "sha256": record["sha256"],
                                   "size_bytes": record["size_bytes"]}
                                  for record in view["origin"]["manifest"]["inputs"]]
        initial = view
    case["revision"] = view["revision"]
    case["block_count"] = len(view["model"]["blocks"])
    case["connection_count"] = len(view["model"]["connections"])
    case["simulation"] = view["simulation"]
    for repetition in range(1, repetitions + 1):
        invoke(cli, timeout, measurements, "inspect_before_run", "inspect", project,
               repetition=repetition)
        run = invoke(cli, timeout, measurements, "run", "run", project, repetition=repetition)
        if run.get("status") != "completed":
            raise RuntimeError(f"run did not complete: {run}")
        runs.append(output_metrics(run))
        reviewed = invoke(cli, timeout, measurements, "inspect_after_run", "inspect", project,
                          repetition=repetition)
        measurements[-1]["retained_run_count"] = len(reviewed["runs"])
        invoke(cli, timeout, measurements, "revisions", "revisions", project,
               repetition=repetition)
        invoke(cli, timeout, measurements, "revision", "revision", project, view["revision"],
               repetition=repetition)
    if initial["revision"] != view["revision"]:
        invoke(cli, timeout, measurements, "restore_older", "restore", project, initial["revision"],
               "--expected-revision", view["revision"])
        invoke(cli, timeout, measurements, "restore_workload", "restore", project, view["revision"],
               "--expected-revision", initial["revision"])
    owned_files = [path for path in project.rglob("*") if path.is_file()]
    case["retained_file_count"] = len(owned_files)
    case["retained_bytes"] = sum(path.stat().st_size for path in owned_files)
    case["status"] = "completed"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", type=Path, default=ROOT / "build/dev/src/cli/galata")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=45,
                        help="per-command operational timeout in seconds; not an acceptance limit")
    parser.add_argument("--nt33a-study", type=Path,
                        help="optional explicit path to the NT-33A graph study")
    arguments = parser.parse_args()
    if not 1 <= arguments.repetitions <= 20 or not 0 < arguments.timeout <= 600:
        parser.error("repetitions must be 1..20 and timeout must be positive and at most 600 seconds")
    cli = arguments.cli.resolve(strict=True)
    study = arguments.nt33a_study.resolve(strict=True) if arguments.nt33a_study else None
    output = arguments.output_dir.resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error("output-dir must be a new or empty directory")
    output.mkdir(parents=True, exist_ok=True)
    version = subprocess.run([str(cli), "--version"], capture_output=True, text=True,
                             check=True, timeout=arguments.timeout).stdout.strip()
    report = {
        "schema": "galata.project-workload-observation.v1",
        "assessment": "observations_only_no_acceptance_budget",
        "scope": "public CLI startup, worker execution, persistence and verified review",
        "limitations": ["Does not measure GUI rendering or event-loop responsiveness.",
                        "Sequential repetitions share the host filesystem cache and accumulate retained runs.",
                        "No numerical-accuracy, model-validity or engineering-acceptance claim."],
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {"system": platform.system(), "release": platform.release(),
                        "machine": platform.machine(), "platform": platform.platform(),
                        "logical_cpu_count": os.cpu_count(), "python": platform.python_version()},
        "tool": {"path": str(cli), "sha256": digest(cli.read_bytes()), "version": version},
        "recorder_sha256": digest(Path(__file__).read_bytes()),
        "repetitions": arguments.repetitions,
        "operational_command_timeout_s": arguments.timeout,
        "workloads": [],
    }
    report_path = output / "observations.json"
    try:
        for workload in WORKLOADS:
            measure(cli, output, arguments.timeout, arguments.repetitions,
                    workload, report["workloads"])
            report_path.write_bytes(json_bytes(report))
        if study:
            measure(cli, output, arguments.timeout, arguments.repetitions,
                    {"name": "nt33a-imported-graph"}, report["workloads"], study)
        report["status"] = "completed"
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        report["status"] = "failed"
        report["diagnostic"] = str(error)
        report_path.write_bytes(json_bytes(report))
        print(f"Measurement failed; retained record: {report_path}\n{error}", file=sys.stderr)
        return 1
    report["finished_utc"] = datetime.now(timezone.utc).isoformat()
    report_path.write_bytes(json_bytes(report))
    print(report_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
