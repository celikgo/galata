"""Execution-backed Python operations for the Galata engineering workflow.

The Python package is deliberately a thin, machine-readable client of the
authoritative C++ CLI. It does not recreate trim, linearisation, control, or
simulation algorithms in Python.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Union


PathLike = Union[Path, str]


class WorkflowError(RuntimeError):
    """Base class for actionable workflow failures."""


class ExecutableNotFoundError(WorkflowError):
    """The requested authoritative Galata executable is unavailable."""


class OperationError(WorkflowError):
    """The C++ capability refused or failed an operation."""


class InvalidParameterError(WorkflowError, ValueError):
    """A model parameter override is unknown or invalid."""


class ArtifactError(WorkflowError, ValueError):
    """An operation received an incompatible or incomplete artifact."""


class ComparisonError(WorkflowError, ValueError):
    """Comparison inputs are malformed or the requested policy is invalid."""


def _finite(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def load_csv(path: PathLike) -> List[Dict[str, Optional[float]]]:
    """Load a Galata CSV while preserving blank cells as missing observations."""
    with Path(path).open(newline="") as stream:
        rows: List[Dict[str, Optional[float]]] = []
        for raw in csv.DictReader(stream):
            rows.append({name: None if value in (None, "") else float(value)
                         for name, value in raw.items()})
        return rows


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class Model:
    path: Path
    sha256: str
    schema: Mapping[str, Any]

    @property
    def kind(self) -> str:
        """Validated vehicle family used to select compatibility adapters."""
        if self.schema.get("schema") == "galata.helicopter.schema.v1":
            return "helicopter"
        return str(self.schema.get("kind", "unknown"))

    @property
    def parameters(self) -> Mapping[str, Mapping[str, Any]]:
        parameters = self.schema.get("parameters", {})
        if isinstance(parameters, Mapping):
            return parameters
        return {str(item["name"]): item for item in parameters}

    @property
    def states(self) -> Sequence[Mapping[str, Any]]:
        return self.schema.get("states", [])

    @property
    def controls(self) -> Sequence[Mapping[str, Any]]:
        return self.schema.get("controls", [])

    @property
    def outputs(self) -> Sequence[Mapping[str, Any]]:
        return self.schema.get("outputs", [])


@dataclass(frozen=True)
class Study:
    model: Model
    output_dir: Path
    flight_condition: Mapping[str, Any]
    parameter_overrides: Mapping[str, float] = field(default_factory=dict)
    template_path: Optional[Path] = None

    @property
    def path(self) -> Optional[Path]:
        """Compatibility view; composed studies do not depend on a template."""
        return self.template_path


@dataclass(frozen=True)
class Run:
    manifest_path: Path
    manifest: Mapping[str, Any]

    @property
    def output_dir(self) -> Path:
        return Path(str(self.manifest["output_directory"])).resolve()

    def output(self, relative_path: str) -> Path:
        candidate = (self.output_dir / relative_path).resolve()
        try:
            candidate.relative_to(self.output_dir)
        except ValueError as error:
            raise ArtifactError(f"output path escapes run directory: {relative_path}") from error
        if not candidate.is_file():
            raise ArtifactError(f"run output is missing: {relative_path}")
        return candidate

    def output_digests(self) -> Mapping[str, str]:
        return {str(item["path"]): str(item["sha256"])
                for item in self.manifest.get("outputs", [])}

    def selected_outputs(self, outputs: Optional[Iterable[str]] = None) -> Mapping[str, str]:
        digests = self.output_digests()
        if outputs is None:
            return dict(digests)
        selected = list(outputs)
        missing = sorted(set(selected) - set(digests))
        if missing:
            raise ComparisonError(f"selected outputs are missing from run: {missing}")
        return {name: digests[name] for name in selected}


@dataclass(frozen=True)
class OperationResult:
    data: Mapping[str, Any]
    run: Run
    artifact_path: Path
    study: Study

    def __getitem__(self, key: str) -> Any:
        return self.data[key]

    def get(self, key: str, default: Any = None) -> Any:
        return self.data.get(key, default)


@dataclass(frozen=True)
class TrimResult(OperationResult):
    pass


@dataclass(frozen=True)
class LinearizationResult(OperationResult):
    trim: TrimResult


@dataclass(frozen=True)
class ControllerResult(OperationResult):
    linearization: LinearizationResult


@dataclass(frozen=True)
class SimulationResult(OperationResult):
    controller: ControllerResult
    open_csv: Path
    closed_csv: Path
    response_json: Path


def _matrix_diagonal(size: int, value: float) -> List[List[float]]:
    return [[value if i == j else 0.0 for j in range(size)] for i in range(size)]


def _default_q(state_names: Sequence[str]) -> List[List[float]]:
    """Use the documented Souxmar attitude-hold design weights by channel."""
    weights = []
    for name in state_names:
        if name in ("roll_rad", "pitch_rad"):
            weights.append(20.0)
        elif name.endswith("rate_rad_s") or name in (
                "main_rotor_speed_rad_s", "main_inflow_ratio", "tail_inflow_ratio",
                "collective_rad", "longitudinal_cyclic_rad", "lateral_cyclic_rad",
                "pedal_rad"):
            weights.append(5.0)
        elif name == "engine_torque_n_m":
            weights.append(0.01)
        else:
            weights.append(1.0)
    return [[weights[i] if i == j else 0.0 for j in range(len(weights))]
            for i in range(len(weights))]


class GalataWorkflow:
    """Compose real C++ capabilities and return structured result artifacts."""

    def __init__(self, executable: PathLike = "build/dev/src/cli/galata"):
        requested = Path(executable)
        if requested.is_absolute() or requested.parent != Path("."):
            resolved = requested.expanduser().resolve()
        else:
            found = shutil.which(str(requested))
            resolved = Path(found).resolve() if found else requested.resolve()
        if not resolved.is_file() or not os.access(resolved, os.X_OK):
            raise ExecutableNotFoundError(
                f"Galata executable not found or not executable: {resolved}; "
                "build it or pass executable=..."
            )
        self.executable = resolved

    def _run_pipeline(self, stages: Sequence[Mapping[str, Any]], study: Study,
                      label: str) -> Run:
        root = study.output_dir.resolve()
        root.mkdir(parents=True, exist_ok=True)
        run_dir = Path(tempfile.mkdtemp(prefix=f"{label}-", dir=str(root)))
        pipeline_path = run_dir / "pipeline.json"
        pipeline_path.write_text(json.dumps({"version": 1, "stages": list(stages)}, indent=2))
        output_dir = run_dir / "results"
        completed = subprocess.run(
            [str(self.executable), "run", str(pipeline_path), "--output-dir",
             str(output_dir), "--json"], capture_output=True, text=True)
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout).strip()
            raise OperationError(f"Galata {label} failed with exit {completed.returncode}: {detail}")
        try:
            summary = json.loads(completed.stdout)
            manifest_path = Path(str(summary["manifest_path"])).resolve()
            manifest = json.loads(manifest_path.read_text())
        except (KeyError, json.JSONDecodeError, OSError) as error:
            raise OperationError(f"Galata {label} returned no valid run manifest") from error
        if manifest.get("status") != "completed":
            raise OperationError(f"Galata {label} did not complete: {manifest.get('status')}")
        return Run(manifest_path, manifest)

    @staticmethod
    def _model_stage(study: Study) -> Dict[str, Any]:
        if study.model.kind == "helicopter":
            capability = "model.helicopter"
            extra = {}
        else:
            capability = "model.vehicle"
            extra = {"kind": study.model.kind}
        return {"id": "aircraft", "capability": capability,
                "input": {"path": str(study.model.path), **extra,
                          "parameter_overrides": dict(study.parameter_overrides)}}

    @staticmethod
    def _trim_stage(study: Study) -> Dict[str, Any]:
        condition = {"airspeed_m_s": 0.0, "altitude_m": 100.0, "delta_isa_k": 0.0}
        condition.update(study.flight_condition)
        if study.model.kind == "helicopter":
            return {"id": "trim", "capability": "trim.helicopter",
                    "input": {"helicopter": {"from": "aircraft"}, **condition}}
        return {"id": "trim", "capability": "trim.vehicle",
                "input": {"vehicle": {"from": "aircraft"}, **condition}}

    def load_model(self, path: PathLike) -> Model:
        model_path = Path(path).expanduser().resolve()
        if not model_path.is_file():
            raise WorkflowError(f"model file does not exist: {model_path}")
        with tempfile.TemporaryDirectory(prefix="galata-model-") as temporary:
            output = Path(temporary)
            source = model_path.read_text()
            if "main_rotor:" in source or "drivetrain:" in source:
                kind = "helicopter"
                capability = "model.helicopter"
                report_capability = "report.helicopter_schema_json"
                reference_key = "helicopter"
            elif "rotors:" in source:
                kind = "multirotor"
                capability = "model.vehicle"
                report_capability = "report.vehicle_schema_json"
                reference_key = "vehicle"
            else:
                kind = "fixed-wing"
                capability = "model.vehicle"
                report_capability = "report.vehicle_schema_json"
                reference_key = "vehicle"
            model_input: Dict[str, Any] = {"path": str(model_path)}
            if capability == "model.vehicle":
                model_input["kind"] = kind
            pipeline = [{"id": "aircraft", "capability": capability, "input": model_input},
                        {"id": "schema", "capability": report_capability,
                         "input": {reference_key: {"from": "aircraft"}, "path": "model-schema.json"}}]
            study = Study(Model(model_path, _sha256(model_path), {}), output, {})
            run = self._run_pipeline(pipeline, study, "model")
            schema = json.loads(run.output("model-schema.json").read_text())
        if schema.get("schema") not in ("galata.helicopter.schema.v1", "galata.vehicle.schema.v1"):
            raise OperationError(f"unexpected model schema: {schema.get('schema')}")
        identity = schema.get("identity", schema.get("model", {}))
        return Model(model_path, str(identity["sha256"]), schema)

    def inspect(self, model: Model) -> Mapping[str, Any]:
        """Return the authoritative named model schema."""
        self._require_model(model)
        return model.schema

    @staticmethod
    def _require_model(model: Any) -> Model:
        if not isinstance(model, Model) or model.schema.get("schema") not in (
                "galata.helicopter.schema.v1", "galata.vehicle.schema.v1"):
            raise ArtifactError("expected a validated Model returned by load_model()")
        return model

    def configure(self, model: Model, output_dir: PathLike,
                  parameter_overrides: Optional[Mapping[str, Any]] = None,
                  flight_condition: Optional[Mapping[str, Any]] = None) -> Study:
        model = self._require_model(model)
        overrides: Dict[str, float] = {}
        allowed = set(model.parameters)
        for name, value in (parameter_overrides or {}).items():
            if name not in allowed:
                raise InvalidParameterError(
                    f"unknown parameter '{name}'; available parameters: {sorted(allowed)}")
            overrides[name] = _finite(value, f"parameter {name}")
        condition = dict(flight_condition or {})
        unknown = sorted(set(condition) - {"airspeed_m_s", "altitude_m", "delta_isa_k", "heading_rad"})
        if unknown:
            raise ValueError(f"unknown flight-condition fields: {unknown}")
        for name, value in condition.items():
            condition[name] = _finite(value, name)
        return Study(model, Path(output_dir).expanduser().resolve(), condition, overrides)

    def trim(self, study: Study) -> TrimResult:
        if not isinstance(study, Study):
            raise ArtifactError("trim() requires a Study returned by configure()")
        report_capability = ("report.helicopter_trim_json" if study.model.kind == "helicopter"
                             else "report.vehicle_trim_json")
        stages = [self._model_stage(study), self._trim_stage(study),
                  {"id": "trim_report", "capability": report_capability,
                   "input": {"trim": {"from": "trim"}, "path": "trim.json"}}]
        run = self._run_pipeline(stages, study, "trim")
        artifact = run.output("trim.json")
        return TrimResult(json.loads(artifact.read_text()), run, artifact, study)

    def linearize(self, trim: TrimResult, drop_position_and_heading: bool = True
                  ) -> LinearizationResult:
        if not isinstance(trim, TrimResult):
            raise ArtifactError("linearize() requires the result of trim()")
        generic = trim.study.model.kind != "helicopter"
        linear_capability = "linearize.shared" if generic else "linearize.vehicle"
        stages = [self._model_stage(trim.study), self._trim_stage(trim.study),
                  {"id": "linear", "capability": linear_capability,
                   "input": {"trim": {"from": "trim"},
                             "drop_position_and_heading": bool(drop_position_and_heading)}},
                  {"id": "linear_report", "capability": "report.linear_system_json",
                   "input": {"system": {"from": "linear"}, "path": "linear.json"}}]
        run = self._run_pipeline(stages, trim.study, "linearize")
        artifact = run.output("linear.json")
        data = json.loads(artifact.read_text())
        evidence = next((item for item in run.manifest.get("linearization_evidence", [])
                         if item.get("stage_id") == "linear"), None)
        if evidence:
            data = dict(data)
            data["diagnostics"] = evidence
        return LinearizationResult(data, run, artifact, trim.study, trim)

    def design(self, linearization: LinearizationResult,
               q: Optional[Sequence[Sequence[float]]] = None,
               r: Optional[Sequence[Sequence[float]]] = None) -> ControllerResult:
        if not isinstance(linearization, LinearizationResult):
            raise ArtifactError("design() requires the result of linearize()")
        input_count = len(linearization.data.get("input_names", []))
        q_matrix = [list(map(float, row)) for row in q] if q is not None else _default_q(
            linearization.data.get("state_names", []))
        r_matrix = [list(map(float, row)) for row in r] if r is not None else _matrix_diagonal(input_count, 0.1)
        generic = linearization.study.model.kind != "helicopter"
        linear_capability = "linearize.shared" if generic else "linearize.vehicle"
        stages = [self._model_stage(linearization.study), self._trim_stage(linearization.study),
                  {"id": "linear", "capability": linear_capability,
                   "input": {"trim": {"from": "trim"}, "drop_position_and_heading": True}},
                  {"id": "law", "capability": "synth.lqr",
                   "input": {"system": {"from": "linear"}, "break_at": "plant_input",
                             "q": q_matrix, "r": r_matrix}},
                  {"id": "law_report", "capability": "report.control_law_json",
                   "input": {"law": {"from": "law"}, "path": "controller.json"}}]
        run = self._run_pipeline(stages, linearization.study, "design")
        artifact = run.output("controller.json")
        return ControllerResult(json.loads(artifact.read_text()), run, artifact,
                                linearization.study, linearization)

    def simulate(self, controller: ControllerResult,
                 attitude_perturbation_body_rad: Sequence[float] = (0.04, -0.03, 0.0),
                 initial_state_perturbation: Optional[Mapping[str, float]] = None,
                 step_s: float = 0.002, steps: int = 2500, sample_stride: int = 10,
                 controller_period_s: float = 0.01, delay_periods: int = 0,
                 response_requirements: Optional[Mapping[str, Any]] = None) -> SimulationResult:
        if not isinstance(controller, ControllerResult):
            raise ArtifactError("simulate() requires the result of design()")
        perturbation = [_finite(value, "attitude perturbation")
                        for value in attitude_perturbation_body_rad]
        if len(perturbation) != 3:
            raise ValueError("attitude_perturbation_body_rad must contain roll, pitch, yaw")
        perturbation_map = {str(key): _finite(value, str(key))
                            for key, value in (initial_state_perturbation or {}).items()}
        requirements = dict(response_requirements or {
            "validity_envelope_departures": 0, "saturation_duration_s": 2.0,
            "control_effort_peak_rad": 1.0, "control_effort_rms_rad": 1.0,
        })
        signal_requirements = requirements.pop("signal_requirements", {
            "roll_rad": {"settling_band_rad": 0.02, "settling_dwell_s": 0.5,
                          "peak_tracking_error_rad": 0.2, "final_tracking_error_rad": 0.1,
                          "rms_tracking_error_rad": 0.1, "settling_time_s": 5.0},
            "pitch_rad": {"settling_band_rad": 0.02, "settling_dwell_s": 0.5,
                           "peak_tracking_error_rad": 0.2, "final_tracking_error_rad": 0.1,
                           "rms_tracking_error_rad": 0.1, "settling_time_s": 5.0},
        })
        common: Dict[str, Any] = {
            "step_s": _finite(step_s, "step_s"), "steps": int(steps),
            "sample_stride": int(sample_stride),
            "attitude_perturbation_body_rad": perturbation,
            "initial_state_perturbation": perturbation_map,
        }
        if common["steps"] <= 0 or common["sample_stride"] <= 0:
            raise ValueError("steps and sample_stride must be positive")
        if controller.study.model.kind != "helicopter":
            return self._simulate_shared_vehicle(controller, perturbation, step_s, steps,
                                                 sample_stride, perturbation_map,
                                                 response_requirements, signal_requirements)
        # The controller artifact contains the selected Q/R matrices. Reusing
        # them makes the composed simulation depend on the preceding design.
        stages: List[Mapping[str, Any]] = [
            self._model_stage(controller.study), self._trim_stage(controller.study),
            {"id": "linear", "capability": "linearize.vehicle",
             "input": {"trim": {"from": "trim"}, "drop_position_and_heading": True}},
            {"id": "law", "capability": "synth.lqr",
             "input": {"system": {"from": "linear"}, "break_at": "plant_input",
                       "q": controller.data.get("q"), "r": controller.data.get("r")}},
            {"id": "trim_report", "capability": "report.helicopter_trim_json",
             "input": {"trim": {"from": "trim"}, "path": "trim.json"}},
            {"id": "linear_report", "capability": "report.linear_system_json",
             "input": {"system": {"from": "linear"}, "path": "linear.json"}},
            {"id": "law_report", "capability": "report.control_law_json",
             "input": {"law": {"from": "law"}, "path": "controller.json"}},
            {"id": "open", "capability": "sim.helicopter", "input":
             {"trim": {"from": "trim"}, **common}},
            {"id": "closed", "capability": "sim.helicopter.closed_loop", "input":
             {"trim": {"from": "trim"}, "law": {"from": "law"},
              "controller_period_s": _finite(controller_period_s, "controller_period_s"),
              "delay_periods": int(delay_periods),
              "controller": {"type": "state_feedback", "missing_measurement": "refuse"},
              **common}},
            {"id": "response", "capability": "analyze.helicopter_response", "input":
             {"open_loop": {"from": "open"}, "closed_loop": {"from": "closed"},
              "signals": ["roll_rad", "pitch_rad"],
              "signal_requirements": signal_requirements, "requirements": requirements}},
            {"id": "response_report", "capability": "report.helicopter_response_json",
             "input": {"response": {"from": "response"}, "path": "response.json"}},
            {"id": "open_csv", "capability": "report.helicopter_csv",
             "input": {"trajectory": {"from": "open"}, "path": "open.csv"}},
            {"id": "closed_csv", "capability": "report.helicopter_csv",
             "input": {"trajectory": {"from": "closed"}, "path": "closed.csv"}},
        ]
        run = self._run_pipeline(stages, controller.study, "simulate")
        response_path, open_csv, closed_csv = (run.output(name) for name in
                                               ("response.json", "open.csv", "closed.csv"))
        return SimulationResult(json.loads(response_path.read_text()), run, response_path,
                                controller.study, controller, open_csv, closed_csv, response_path)

    @staticmethod
    def _shared_chart_names(model: Model) -> List[str]:
        names = ["position_north_m", "position_east_m", "position_down_m",
                 "velocity_u_m_s", "velocity_v_m_s", "velocity_w_m_s",
                 "roll_rad", "pitch_rad", "yaw_rad", "roll_rate_rad_s",
                 "pitch_rate_rad_s", "yaw_rate_rad_s"]
        states = list(model.states)
        for item in states[13:]:
            names.append(str(item.get("name", "")))
        return names

    def _simulate_shared_vehicle(self, controller: ControllerResult,
                                 perturbation: Sequence[float], step_s: float, steps: int,
                                 sample_stride: int,
                                 initial_state_perturbation: Mapping[str, float],
                                 requirements: Mapping[str, Any],
                                 signal_requirements: Mapping[str, Any]) -> SimulationResult:
        chart_names = self._shared_chart_names(controller.study.model)
        initial = [0.0] * len(chart_names)
        for name, value in zip(("roll_rad", "pitch_rad", "yaw_rad"), perturbation):
            initial[chart_names.index(name)] = value
        linear_capability = "linearize.shared"
        for name, value in initial_state_perturbation.items():
            if name not in chart_names:
                raise ArtifactError(
                    f"initial_state_perturbation names unknown shared chart channel '{name}'")
            initial[chart_names.index(name)] = value
        common = {"step_s": _finite(step_s, "step_s"), "steps": int(steps),
                  "sample_stride": int(sample_stride),
                  "initial_chart_perturbation": initial}
        stages: List[Mapping[str, Any]] = [
            self._model_stage(controller.study), self._trim_stage(controller.study),
            {"id": "linear", "capability": linear_capability,
             "input": {"trim": {"from": "trim"}, "drop_position_and_heading": True}},
            {"id": "law", "capability": "synth.lqr",
             "input": {"system": {"from": "linear"}, "break_at": "plant_input",
                       "q": controller.data.get("q"), "r": controller.data.get("r")}},
            {"id": "open", "capability": "sim.vehicle",
             "input": {"trim": {"from": "trim"}, **common}},
            {"id": "closed", "capability": "sim.vehicle",
             "input": {"trim": {"from": "trim"}, "law": {"from": "law"}, **common}},
            {"id": "response", "capability": "report.vehicle_response_json",
             "input": {"open": {"from": "open"}, "closed": {"from": "closed"},
                       "signals": list(signal_requirements),
                       "signal_requirements": signal_requirements, "path": "response.json"}},
            {"id": "open_csv", "capability": "report.vehicle_csv",
             "input": {"trajectory": {"from": "open"}, "path": "open.csv"}},
            {"id": "closed_csv", "capability": "report.vehicle_csv",
             "input": {"trajectory": {"from": "closed"}, "path": "closed.csv"}},
        ]
        run = self._run_pipeline(stages, controller.study, "simulate-shared")
        response_path, open_csv, closed_csv = (run.output(name) for name in
                                               ("response.json", "open.csv", "closed.csv"))
        return SimulationResult(json.loads(response_path.read_text()), run, response_path,
                                controller.study, controller, open_csv, closed_csv, response_path)

    def evaluate(self, result: Union[SimulationResult, Run]) -> Mapping[str, Any]:
        if isinstance(result, SimulationResult):
            response, run = dict(result.data), result.run
        elif isinstance(result, Run):
            run = result
            candidates = [name for name in run.output_digests() if name.endswith("response.json")]
            if not candidates:
                raise ArtifactError("run has no structured response JSON artifact")
            response = json.loads(run.output(candidates[0]).read_text())
        else:
            raise ArtifactError("evaluate() requires SimulationResult or Run")
        return {"schema": response.get("schema"),
                "execution_completed": run.manifest.get("status") == "completed",
                "criteria_passed": bool(response.get("criteria_passed", False)),
                "metrics": response.get("metrics", []),
                "requirements": response.get("requirements", {}),
                "outputs": sorted(run.output_digests())}

    @staticmethod
    def compare(first: Run, second: Run, outputs: Optional[Sequence[str]] = None
                ) -> Mapping[str, Any]:
        """Compare the complete explicitly selected output set bit-for-bit."""
        if outputs is None:
            left, right = dict(first.output_digests()), dict(second.output_digests())
            declared = set(left) | set(right)
            missing_declared_first = set()
            missing_declared_second = set()
        else:
            selected = list(outputs)
            left_all, right_all = first.output_digests(), second.output_digests()
            left = {name: left_all[name] for name in selected if name in left_all}
            right = {name: right_all[name] for name in selected if name in right_all}
            declared = set(selected)
            missing_declared_first = declared - set(left_all)
            missing_declared_second = declared - set(right_all)
        left_set, right_set = set(left), set(right)
        matching = sorted(left_set & right_set)
        same_set = left_set == right_set and left_set == declared
        digest_matches = same_set and all(left[name] == right[name] for name in matching)
        return {"comparison": "full_output_bitwise",
                "selected_outputs": sorted(declared),
                "matching_output_paths": matching,
                "bit_identical_outputs": bool(same_set and bool(left_set) and digest_matches),
                "empty_output_set": not left_set and not right_set,
                "missing_from_second": sorted((left_set - right_set) | missing_declared_second),
                "missing_from_first": sorted((right_set - left_set) | missing_declared_first),
                "digest_mismatches": sorted(name for name in matching if left[name] != right[name])}

    @staticmethod
    def compare_subset(first: Run, second: Run, outputs: Sequence[str]) -> Mapping[str, Any]:
        """Compare only a caller-declared common subset."""
        left, right = first.output_digests(), second.output_digests()
        selected = list(outputs)
        missing = sorted((set(selected) - set(left)) | (set(selected) - set(right)))
        return {"comparison": "selected_subset_bitwise", "selected_outputs": selected,
                "missing": missing, "bit_identical": not missing and bool(selected)
                and all(left[name] == right[name] for name in selected),
                "digest_mismatches": [name for name in selected
                                      if name in left and name in right and left[name] != right[name]]}

    @staticmethod
    def compare_numeric(first: Run, second: Run, outputs: Sequence[str],
                        rtol: float = 1e-9, atol: float = 1e-12) -> Mapping[str, Any]:
        if rtol < 0 or atol < 0:
            raise ComparisonError("numeric comparison tolerances must be non-negative")
        left, right = first.output_digests(), second.output_digests()
        details: Dict[str, Any] = {}
        passed = True
        for name in outputs:
            if name not in left or name not in right:
                details[name] = {"status": "missing"}
                passed = False
                continue
            try:
                first_rows, second_rows = load_csv(first.output(name)), load_csv(second.output(name))
            except (OSError, ValueError, ArtifactError) as error:
                details[name] = {"status": "not_numeric_csv", "error": str(error)}
                passed = False
                continue
            same = (len(first_rows) == len(second_rows) and
                    (not first_rows or list(first_rows[0]) == list(second_rows[0])))
            if same:
                for row_a, row_b in zip(first_rows, second_rows):
                    if list(row_a) != list(row_b):
                        same = False
                        break
                    for key in row_a:
                        x, y = row_a[key], row_b[key]
                        if x is None or y is None:
                            same = x is None and y is None
                        elif abs(x - y) > atol + rtol * abs(y):
                            same = False
                        if not same:
                            break
                    if not same:
                        break
            details[name] = {"status": "pass" if same else "fail",
                             "rows_first": len(first_rows), "rows_second": len(second_rows)}
            passed = passed and same
        return {"comparison": "numeric_tolerance", "selected_outputs": list(outputs),
                "rtol": rtol, "atol": atol, "passed": passed, "details": details}

    @staticmethod
    def export(run: Union[Run, OperationResult], destination: PathLike,
               outputs: Optional[Sequence[str]] = None,
               plots: Optional[Sequence[PathLike]] = None,
               overwrite: bool = False) -> Path:
        """Create a self-contained result bundle with validated relative links."""
        if isinstance(run, OperationResult):
            run = run.run
        if not isinstance(run, Run):
            raise ArtifactError("export() requires a Run or operation result")
        destination_path = Path(destination).expanduser().resolve()
        if destination_path.exists():
            if not overwrite:
                raise FileExistsError(f"refusing to overwrite existing result bundle: {destination_path}")
            if not destination_path.is_dir() or any(destination_path.iterdir()):
                raise FileExistsError(f"overwrite requires an empty directory: {destination_path}")
        else:
            destination_path.mkdir(parents=True)
        selected = run.selected_outputs(outputs)
        shutil.copyfile(run.manifest_path, destination_path / "manifest.json")
        result_dir = destination_path / "results"
        result_dir.mkdir()
        files = []
        for relative, digest in selected.items():
            source = run.output(relative)
            target = result_dir / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            actual = _sha256(target)
            if actual != digest:
                raise ArtifactError(f"digest changed while bundling {relative}")
            files.append({"path": str(Path("results") / relative), "sha256": actual})
        plot_files = []
        seen_plot_names = set()
        for plot in plots or []:
            source = Path(plot).expanduser().resolve()
            if not source.is_file():
                raise ArtifactError(f"plot selected for bundle is missing: {source}")
            if source.name in seen_plot_names:
                raise ArtifactError(f"plot basenames collide in bundle: {source.name}")
            seen_plot_names.add(source.name)
            target = destination_path / "plots" / source.name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            plot_files.append({"path": str(Path("plots") / source.name),
                               "sha256": _sha256(target)})
        configuration_files = []
        for index, item in enumerate(run.manifest.get("inputs", [])):
            encoded = item.get("bytes_hex")
            if not encoded:
                continue
            try:
                bytes_value = bytes.fromhex(str(encoded))
            except ValueError as error:
                raise ArtifactError(f"manifest input {index} has invalid bytes_hex") from error
            name = Path(str(item.get("path", f"input-{index}.json"))).name
            target = destination_path / "configuration" / f"{index:02d}-{name}"
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(bytes_value)
            if _sha256(target) != item.get("sha256"):
                raise ArtifactError(f"configuration digest mismatch for {name}")
            configuration_files.append({"path": str(target.relative_to(destination_path)),
                                        "sha256": _sha256(target)})
        provenance = {
            "build": run.manifest.get("build"),
            "executable": run.manifest.get("executable"),
            "runtime": run.manifest.get("runtime"),
            "study": run.manifest.get("study"),
            "stages": run.manifest.get("stages", []),
            "source_output_directory": run.manifest.get("output_directory"),
        }
        (destination_path / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
        metadata = {"schema": "galata.bundle.v1", "source_manifest": "manifest.json",
                    "selected_outputs": files, "plots": plot_files,
                    "configuration": configuration_files,
                    "provenance": "provenance.json", "relative_to": ".", "portable": True}
        (destination_path / "bundle.json").write_text(json.dumps(metadata, indent=2) + "\n")
        for item in files + plot_files + configuration_files:
            if not (destination_path / item["path"]).is_file():
                raise ArtifactError(f"bundle link is missing: {item['path']}")
        return destination_path
