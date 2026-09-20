#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Convert one documented NASA DASHlink NPZ flight window to Galata CSV.

The converter intentionally uses only the Python standard library. The public
DASHlink four-class data set is an NPZ containing ``data`` with shape
``(instances, samples, 20)`` and an optional ``label`` array. This tool does
not infer a sample period, units, aircraft identity or model configuration:
the caller must provide the sample period, and the emitted metadata preserves
the public source identity and the selected instance label.
"""

from __future__ import annotations

import argparse
import ast
import csv
import json
import math
from pathlib import Path
import struct
import sys
import zipfile


SOURCE_URL = "https://c3.ndc.nasa.gov/dashlink/resources/1018/"
DATASET_URL = (
    "https://c3.ndc.nasa.gov/dashlink/static/media/dataset/"
    "DASHlink_full_fourclass_raw.npz"
)
VARIABLES = (
    "aileron_position_lh_deg",
    "aileron_position_rh_deg",
    "corrected_angle_of_attack_deg",
    "baro_correct_altitude_ft",
    "computed_airspeed_kt",
    "selected_course_deg",
    "drift_angle_deg",
    "elevator_position_left_deg",
    "te_flap_position",
    "glideslope_deviation_pct",
    "selected_heading_deg",
    "localizer_deviation_pct",
    "core_speed_avg_pct",
    "total_pressure_mbar",
    "pitch_angle_deg",
    "roll_angle_deg",
    "rudder_position_deg",
    "true_heading_deg",
    "vertical_acceleration_g",
    "wind_speed_kt",
)
LABELS = ("nominal", "speed_high", "path_high", "flaps_late_setting")


def _read_npy(blob: bytes) -> tuple[tuple[int, ...], str, list[float]]:
    if blob[:6] != b"\x93NUMPY":
        raise ValueError("array member is not a NumPy .npy file")
    major, minor = blob[6], blob[7]
    if (major, minor) == (1, 0):
        header_size = struct.unpack_from("<H", blob, 8)[0]
        header_start = 10
    elif (major, minor) in ((2, 0), (3, 0)):
        header_size = struct.unpack_from("<I", blob, 8)[0]
        header_start = 12
    else:
        raise ValueError(f"unsupported .npy format {major}.{minor}")
    header_end = header_start + header_size
    try:
        header = ast.literal_eval(blob[header_start:header_end].decode("latin1"))
    except (SyntaxError, UnicodeDecodeError, ValueError) as error:
        raise ValueError("invalid .npy header") from error
    if not isinstance(header, dict) or header.get("fortran_order") is not False:
        raise ValueError("only C-order arrays are supported")
    descr = header.get("descr")
    shape = header.get("shape")
    if descr not in ("<f4", "<f8", "|f4", "|f8") or not isinstance(shape, tuple):
        raise ValueError("only little-endian float32/float64 arrays are supported")
    if not all(isinstance(extent, int) and extent >= 0 for extent in shape):
        raise ValueError("array shape contains an invalid extent")
    item_size = 4 if descr.endswith("4") else 8
    count = math.prod(shape)
    payload = blob[header_end:]
    if len(payload) != count * item_size:
        raise ValueError(".npy payload length does not match its shape")
    fmt = "<" + ("f" if item_size == 4 else "d") * count
    values = list(struct.unpack(fmt, payload)) if count else []
    if not all(math.isfinite(value) for value in values):
        raise ValueError("array contains a non-finite value")
    return tuple(shape), descr, values


def _read_npz(path: Path) -> tuple[tuple[int, ...], list[float], list[int] | None]:
    try:
        with zipfile.ZipFile(path) as archive:
            names = set(archive.namelist())
            data_name = "data.npy" if "data.npy" in names else "data"
            if data_name not in names:
                raise ValueError("NPZ does not contain the required data array")
            shape, _, values = _read_npy(archive.read(data_name))
            labels = None
            if "label.npy" in names:
                label_shape, _, label_values = _read_npy(archive.read("label.npy"))
                if len(label_shape) != 1 or label_shape[0] != shape[0]:
                    raise ValueError("label array does not match the data instance count")
                labels = [int(value) for value in label_values]
            return shape, values, labels
    except zipfile.BadZipFile as error:
        raise ValueError("input is not a valid NPZ archive") from error


def convert(source: Path, destination: Path, metadata_path: Path, index: int,
            sample_period_s: float) -> None:
    if not math.isfinite(sample_period_s) or sample_period_s <= 0.0:
        raise ValueError("sample period must be positive and finite")
    shape, values, labels = _read_npz(source)
    if len(shape) != 3 or shape[2] != len(VARIABLES):
        raise ValueError(
            "expected data shape (instances, samples, 20); "
            f"received {shape!r}"
        )
    if index < 0 or index >= shape[0]:
        raise ValueError(f"instance index {index} is outside 0..{shape[0] - 1}")

    samples, width = shape[1], shape[2]
    first = index * samples * width
    rows = []
    for sample in range(samples):
        offset = first + sample * width
        rows.append([sample * sample_period_s, *values[offset:offset + width]])

    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["time_s", *VARIABLES])
        writer.writerows(rows)

    metadata = {
        "schema": "galata.public-flight-data-reference.v1",
        "source": {
            "dataset_url": DATASET_URL,
            "resource_url": SOURCE_URL,
            "record_type": "NASA DASHlink de-identified regional-jet recorder data",
            "license": "not stated on the source resource",
        },
        "input": {
            "path": str(source),
            "array": "data",
            "shape": list(shape),
            "instance_index": index,
            "label": labels[index] if labels is not None else None,
            "label_name": (LABELS[labels[index]]
                           if labels is not None and 0 <= labels[index] < len(LABELS)
                           else None),
        },
        "output": {
            "path": str(destination),
            "sample_period_s": sample_period_s,
            "sample_count": samples,
            "channels": list(VARIABLES),
        },
        "limitations": [
            "The source data is de-identified and does not identify the aircraft model or configuration.",
            "The source resource does not provide calibration, test-plan or independent-review evidence.",
            "This conversion is not flight-test approval, airworthiness evidence or qualification.",
        ],
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="downloaded DASHlink .npz file")
    parser.add_argument("output", type=Path, help="Galata CSV output path")
    parser.add_argument("--metadata", type=Path, required=True,
                        help="JSON provenance sidecar output path")
    parser.add_argument("--index", type=int, default=0,
                        help="zero-based data instance to export (default: 0)")
    parser.add_argument("--sample-period-s", type=float, required=True,
                        help="sample period supplied by the caller; never inferred")
    args = parser.parse_args(argv)
    try:
        convert(args.input, args.output, args.metadata, args.index, args.sample_period_s)
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        print(f"convert-nasa-dashlink: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
