#!/usr/bin/env python3
"""Exercise the native UH-60A model through the shared CLI vehicle path."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
CLI = Path(os.environ.get("GALATA_PROJECT_CLI", ROOT / "build/dev/src/cli/galata")).resolve()
STUDY = ROOT / "examples" / "uh60a-hover-trim" / "study.yaml"


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="galata-uh60a-") as scratch:
        result = subprocess.run(
            [str(CLI), "run", str(STUDY), "--output-dir", scratch],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
            timeout=60,
        )
        if result.returncode != 0:
            raise SystemExit(
                f"UH-60A study failed with {result.returncode}:\n"
                f"{result.stdout}\n{result.stderr}"
            )
        required = (
            "model.helicopter",
            "trim.helicopter",
            "linearize.vehicle",
            "analyze.modes",
            "5 stages completed.",
            "UH-60A Black Hawk Level-1 study model",
        )
        missing = [text for text in required if text not in result.stdout]
        if missing:
            raise SystemExit(f"UH-60A study output omitted {missing}:\n{result.stdout}")


if __name__ == "__main__":
    main()
