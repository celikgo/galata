"""The Python contract is the CLI's JSON run manifest, never report prose."""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Union


PathLike = Union[Path, str]


def load_csv(path: Path) -> List[Dict[str, float]]:
    """Load a numeric Galata CSV while retaining named channels."""
    with path.open(newline="") as stream:
        return [
            {name: float(value) for name, value in row.items() if value != ""}
            for row in csv.DictReader(stream)
        ]


@dataclass(frozen=True)
class Model:
    path: Path
    sha256: str


@dataclass(frozen=True)
class Study:
    path: Path
    output_dir: Path
    configuration: Mapping[str, object]


@dataclass(frozen=True)
class Run:
    manifest_path: Path
    manifest: Mapping[str, object]

    @property
    def output_dir(self) -> Path:
        return Path(str(self.manifest["output_directory"]))

    def output(self, relative_path: str) -> Path:
        return self.output_dir / relative_path

    def output_digests(self) -> Mapping[str, str]:
        return {
            str(item["path"]): str(item["sha256"])
            for item in self.manifest.get("outputs", [])
        }


class GalataWorkflow:
    """Orchestrate studies while leaving numerical work in the C++ CLI."""

    def __init__(self, executable: PathLike = "build/dev/src/cli/galata"):
        self.executable = Path(executable)

    def load_model(self, path: PathLike) -> Model:
        model_path = Path(path).resolve()
        digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
        return Model(model_path, digest)

    def configure(self, study: PathLike, output_dir: PathLike) -> Study:
        study_path = Path(study).resolve()
        return Study(study_path, Path(output_dir).resolve(), {"study": str(study_path)})

    def trim(self, study: Study) -> Mapping[str, str]:
        return {"operation": "trim", "study": str(study.path)}

    def linearize(self, study: Study) -> Mapping[str, str]:
        return {"operation": "linearize", "study": str(study.path)}

    def design(self, study: Study) -> Mapping[str, str]:
        return {"operation": "design", "study": str(study.path)}

    def simulate(self, study: Study) -> Run:
        study.output_dir.mkdir(parents=True, exist_ok=True)
        completed = subprocess.run(
            [str(self.executable), "run", str(study.path), "--output-dir",
             str(study.output_dir), "--overwrite", "--json"],
            check=True, capture_output=True, text=True,
        )
        summary = json.loads(completed.stdout)
        manifest_path = Path(str(summary["manifest_path"]))
        return Run(manifest_path, json.loads(manifest_path.read_text()))

    def evaluate(self, run: Run) -> Mapping[str, object]:
        return {
            "execution_completed": run.manifest.get("status") == "completed",
            "stage_count": len(run.manifest.get("stages", [])),
            "outputs": sorted(run.output_digests()),
        }

    def compare(self, first: Run, second: Run) -> Mapping[str, object]:
        left, right = first.output_digests(), second.output_digests()
        common = sorted(set(left) & set(right))
        return {
            "matching_output_paths": common,
            "bit_identical_outputs": all(left[path] == right[path] for path in common),
            "missing_from_second": sorted(set(left) - set(right)),
            "missing_from_first": sorted(set(right) - set(left)),
        }

    def export(self, run: Run, destination: PathLike) -> Path:
        destination_path = Path(destination)
        destination_path.mkdir(parents=True, exist_ok=True)
        target = destination_path / run.manifest_path.name
        shutil.copyfile(run.manifest_path, target)
        return target
