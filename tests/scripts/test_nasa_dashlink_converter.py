#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The public DASHlink converter preserves source shape and explicit timing."""
import csv
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zipfile


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "convert-nasa-dashlink.py"


def write_npy(archive: zipfile.ZipFile, name: str, shape: tuple[int, ...], values: list[float]) -> None:
    header = {
        "descr": "<f8",
        "fortran_order": False,
        "shape": shape,
    }
    text = repr(header)
    header_length = 10 + len(text) + 1
    padding = (-header_length) % 16
    encoded = (text + " " * padding + "\n").encode("latin1")
    blob = b"\x93NUMPY\x01\x00" + struct.pack("<H", len(encoded)) + encoded
    blob += struct.pack("<" + "d" * len(values), *values)
    archive.writestr(name, blob)


class DashlinkConverter(unittest.TestCase):
    def test_converts_selected_instance_without_inventing_sample_period(self):
        with tempfile.TemporaryDirectory(prefix="galata-dashlink-converter-") as directory:
            root = Path(directory)
            source = root / "source.npz"
            values = [float(index) for index in range(2 * 3 * 20)]
            with zipfile.ZipFile(source, "w") as archive:
                write_npy(archive, "data.npy", (2, 3, 20), values)
                write_npy(archive, "label.npy", (2,), [0.0, 2.0])
            csv_path = root / "window.csv"
            metadata_path = root / "window.json"
            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(source),
                    str(csv_path),
                    "--metadata",
                    str(metadata_path),
                    "--index",
                    "1",
                    "--sample-period-s",
                    "0.25",
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            with csv_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[0][0], "time_s")
            self.assertEqual(len(rows), 4)
            self.assertEqual(rows[1][0], "0.0")
            self.assertEqual(rows[2][0], "0.25")
            self.assertEqual(rows[1][1], "60.0")
            self.assertEqual(rows[3][-1], "119.0")
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(metadata["schema"], "galata.public-flight-data-reference.v1")
            self.assertEqual(metadata["input"]["shape"], [2, 3, 20])
            self.assertEqual(metadata["input"]["instance_index"], 1)
            self.assertEqual(metadata["input"]["label_name"], "path_high")
            self.assertEqual(metadata["output"]["sample_period_s"], 0.25)
            self.assertIn("de-identified", " ".join(metadata["limitations"]))

    def test_rejects_unknown_shape(self):
        with tempfile.TemporaryDirectory(prefix="galata-dashlink-converter-") as directory:
            root = Path(directory)
            source = root / "source.npz"
            with zipfile.ZipFile(source, "w") as archive:
                write_npy(archive, "data.npy", (1, 2, 19), [0.0] * 38)
            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(source), str(root / "out.csv"),
                 "--metadata", str(root / "out.json"), "--sample-period-s", "0.1"],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("expected data shape", result.stderr)


if __name__ == "__main__":
    unittest.main()
