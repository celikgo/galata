#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Compare galata's ULog reader against pyulog, PX4's own Python tooling.

ADR-0016 chose to parse ULog with an in-repo reader rather than a dependency,
and it accepted an obligation in exchange: the reader must be checked against an
implementation that is not ours. `tests/data/make_ulog_fixture.py --verify`
discharges half of that — it proves the FIXTURE is valid ULog that pyulog can
read. This script discharges the other half, which is the half that matters: it
proves galata's READER decodes the same values pyulog does.

WHAT IS COMPARED, and why each item is here rather than assumed:

  TIMESTAMPS      ULog timestamps are microseconds since boot as uint64. galata
                  presents seconds as double. A reader that divided by the wrong
                  power of ten, or that lost the low bits by going through a
                  float, produces a plausible-looking timebase; comparing
                  against pyulog's own microsecond integers catches both.
  TOPIC INSTANCES A duplicated topic is logged as the same message format with a
                  new message id and multi_id 1. A reader that ignored multi_id
                  would MERGE the two instances into one channel that is neither
                  — and would pass every single-instance test. The fixture writes
                  a second `sensor_combined`, and this checks galata selects the
                  instance the study asked for.
  FIELD TYPES     uint32 fields sit between float arrays in `sensor_combined`.
                  A reader that mistook one for a float would shift every
                  subsequent field by nothing at all — the widths match — and
                  return garbage for the integer while every float still looked
                  fine. So the integer fields are compared exactly.
  ARRAYS          `q[4]`, `xyz[3]`, `control[12]`. Array element addressing is
                  where an off-by-one is invisible: element 1 of a quaternion is
                  a small number whatever index you actually read. Every element
                  of every array is compared, by name.
  VALUES          Every declared channel, every sample.

HOW EXACT. galata converts to double; pyulog hands back numpy float32 for a
float field. A float32 promoted to double is exact, so the comparison for float
fields is exact equality after promotion, and for integer fields exact equality
outright. Timestamps are compared as integers, in microseconds, after undoing
galata's scaling — no tolerance anywhere. A tolerance here would hide precisely
the class of defect the comparison exists to find.

WHAT IT CATCHES, MEASURED RATHER THAN ASSERTED. Three deliberate defects were
injected and the comparison was re-run:

  a wrong resample rate (250 -> 200 Hz)   CAUGHT, as a sample-count mismatch
  a galata-side scale of 1 + 1e-7 on one
    channel, which pyulog knows nothing
    about                                 CAUGHT, at sample 0

  a wrong ARRAY INDEX in the table below  NOT caught
  a wrong TOPIC INSTANCE in the table     NOT caught

The last two are not caught, and that is a real limitation of the method rather
than a defect in the reader. `CHANNELS` drives BOTH sides: it generates the
galata study and it selects the pyulog series. Perturbing it moves both sides
together, so this compares two DECODINGS of one specification and cannot detect
a mis-specified channel. The specification itself is checked by the in-tree
`UlogImport.*` tests, which assert values against what
`tests/data/make_ulog_fixture.py` visibly wrote.

WHAT THIS DOES NOT ESTABLISH. It is a comparison against a SYNTHETIC fixture
this repository wrote. pyulog reading it independently makes the reader's
agreement meaningful, but no real PX4 flight log has been read by either
implementation, and nothing here says how the reader behaves on one: real logs
carry topics, formats, appended data, and corruption this fixture does not. That
limitation is separate and stays open.

Usage:
    scripts/compare-ulog-against-pyulog.py --galata <cli> --python <python-with-pyulog>
"""

import argparse
import csv
import json
import pathlib
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parent.parent

# (galata channel name, ulog topic, ulog field, multi_id). Field names are
# pyulog's own addressing, so an array element is `q[1]` on both sides.
CHANNELS = [
    ("q_w", "vehicle_attitude", "q[0]", 0),
    ("q_x", "vehicle_attitude", "q[1]", 0),
    ("q_y", "vehicle_attitude", "q[2]", 0),
    ("q_z", "vehicle_attitude", "q[3]", 0),
    ("rate_x", "vehicle_angular_velocity", "xyz[0]", 0),
    ("rate_y", "vehicle_angular_velocity", "xyz[1]", 0),
    ("rate_z", "vehicle_angular_velocity", "xyz[2]", 0),
    ("pos_x", "vehicle_local_position", "x", 0),
    ("pos_z", "vehicle_local_position", "z", 0),
    ("vel_y", "vehicle_local_position", "vy", 0),
    ("gyro_x", "sensor_combined", "gyro_rad[0]", 0),
    ("gyro_z", "sensor_combined", "gyro_rad[2]", 0),
    ("accel_z", "sensor_combined", "accelerometer_m_s2[2]", 0),
    # An INTEGER field between two float arrays. See the header.
    ("gyro_dt", "sensor_combined", "gyro_integral_dt", 0),
    ("accel_dt", "sensor_combined", "accelerometer_integral_dt", 0),
    # THE SECOND INSTANCE of the same topic, offset by 100 in the fixture.
    ("imu1_gyro_x", "sensor_combined", "gyro_rad[0]", 1),
    ("imu1_accel_z", "sensor_combined", "accelerometer_m_s2[2]", 1),
    # The twelfth element of a twelve-element array, and the first.
    ("motor_0", "actuator_motors", "control[0]", 0),
    ("motor_11", "actuator_motors", "control[11]", 0),
    ("battery_v", "battery_status", "voltage_v", 0),
]

# The record's timebase. `data.import.ulog` resamples onto a declared grid; the
# fixture's own period is 4000 us, so this rate reproduces its instants exactly
# and no interpolation is involved in the comparison.
RESAMPLE_HZ = 250.0
TIME_SCALE_US = 1_000_000


def study(fixture: pathlib.Path) -> str:
    channels = "\n".join(
        f'        - {{topic: {topic}, field: "{field}", name: {name}, '
        f'unit: "si", frame: none, multi_id: {multi}}}'
        for name, topic, field, multi in CHANNELS
    )
    return f"""version: 1
stages:
  - id: log
    capability: data.import.ulog
    input:
      path: {fixture.name}
      resample_hz: {RESAMPLE_HZ}
      description: "the generated ULog fixture, for the pyulog comparison"
      channels:
{channels}
  - id: out
    capability: report.record
    input:
      record: {{from: log}}
      path: galata.csv
      evidence_path: galata.yaml
"""


def read_galata(path: pathlib.Path):
    with path.open(newline="") as handle:
        rows = list(csv.reader(handle))
    header, body = rows[0], rows[1:]
    columns = {name: index for index, name in enumerate(header)}
    times = [float(row[columns["time_s"]]) for row in body]
    values = {
        name: [float(row[columns[name]]) for row in body]
        for name, _, _, _ in CHANNELS
    }
    return times, values


def read_pyulog(python: str, fixture: pathlib.Path):
    """Decode with pyulog in ITS OWN interpreter, and hand back plain JSON.

    Run out of process on purpose: pyulog is a test-time tool and must not be
    importable from anything this repository ships or builds.
    """
    program = f"""
import json, sys
from pyulog import ULog
log = ULog({str(fixture)!r})
wanted = {json.dumps([[t, f, m] for _, t, f, m in CHANNELS])}
series = {{}}
for topic, field, multi in wanted:
    match = [d for d in log.data_list if d.name == topic and d.multi_id == multi]
    if len(match) != 1:
        raise SystemExit(f"pyulog found {{len(match)}} series for {{topic}} instance {{multi}}")
    data = match[0]
    if field not in data.data:
        raise SystemExit(f"pyulog has no field {{field}} on {{topic}} instance {{multi}}")
    series[f"{{topic}}|{{field}}|{{multi}}"] = {{
        "timestamp_us": [int(v) for v in data.data["timestamp"]],
        "values": [float(v) for v in data.data[field]],
        "type": str(data.data[field].dtype),
    }}
instances = {{}}
for d in log.data_list:
    instances.setdefault(d.name, []).append(d.multi_id)
json.dump({{"series": series, "instances": {{k: sorted(v) for k, v in instances.items()}}}},
          sys.stdout)
"""
    finished = subprocess.run([python, "-c", program], capture_output=True, text=True, check=False)
    if finished.returncode != 0:
        sys.stderr.write(finished.stderr)
        raise SystemExit("pyulog decode failed")
    return json.loads(finished.stdout)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--galata", required=True, help="path to the galata CLI")
    parser.add_argument("--python", required=True, help="interpreter with pyulog installed")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--keep", help="directory to retain the run artefacts in")
    arguments = parser.parse_args()

    with tempfile.TemporaryDirectory() as scratch:
        work = pathlib.Path(arguments.keep) if arguments.keep else pathlib.Path(scratch)
        work.mkdir(parents=True, exist_ok=True)
        fixture = work / "fixture.ulg"

        subprocess.run([arguments.python, str(REPO / "tests/data/make_ulog_fixture.py"),
                        str(fixture), "--samples", str(arguments.samples), "--verify"],
                       check=True)

        (work / "study.yaml").write_text(study(fixture))
        subprocess.run([arguments.galata, "run", str(work / "study.yaml"), "--overwrite"],
                       check=True, stdout=subprocess.DEVNULL)

        times, galata = read_galata(work / "galata.csv")
        reference = read_pyulog(arguments.python, fixture)

    problems = []

    # TOPIC INSTANCES. The fixture logs sensor_combined twice; if pyulog does not
    # see both as separate series then the fixture is not exercising instances
    # and every instance comparison below is vacuous.
    seen = reference["instances"].get("sensor_combined", [])
    if seen != [0, 1]:
        problems.append(f"pyulog saw sensor_combined instances {seen}, expected [0, 1]")

    for name, topic, field, multi in CHANNELS:
        entry = reference["series"][f"{topic}|{field}|{multi}"]
        expected_times = entry["timestamp_us"]
        expected = entry["values"]
        actual = galata[name]

        if len(actual) != len(expected):
            problems.append(f"{name}: galata has {len(actual)} samples, pyulog {len(expected)}")
            continue

        # TIMESTAMPS, compared as integer microseconds. galata's seconds are
        # scaled back rather than pyulog's being converted forward, so the
        # comparison is against the integers the file actually holds.
        for index, (galata_seconds, reference_us) in enumerate(zip(times, expected_times)):
            scaled = round(galata_seconds * TIME_SCALE_US)
            if scaled != reference_us:
                problems.append(
                    f"timestamp {index}: galata {galata_seconds} s -> {scaled} us, "
                    f"pyulog {reference_us} us")
                break

        # VALUES, exactly. A float32 promoted to double is exact, and an integer
        # field is an integer, so any difference is a decoding defect.
        for index, (got, want) in enumerate(zip(actual, expected)):
            if got != want:
                problems.append(
                    f"{name} ({topic}.{field} instance {multi}) sample {index}: "
                    f"galata {got!r}, pyulog {want!r}")
                break

        # FIELD TYPES. pyulog reports the dtype it decoded; an integer field
        # must not have arrived as a float on either side.
        if field.endswith("_dt") and not entry["type"].startswith("uint"):
            problems.append(f"{name}: pyulog decoded {field} as {entry['type']}, expected uint")

    print(f"compared {len(CHANNELS)} channel(s) over {len(times)} sample(s): "
          f"timestamps as integer microseconds, values exactly, "
          f"{sum(1 for _, _, _, m in CHANNELS if m == 1)} from a second topic instance, "
          f"{sum(1 for _, _, f, _ in CHANNELS if f.endswith('_dt'))} integer field(s), "
          f"{sum(1 for _, _, f, _ in CHANNELS if '[' in f)} array element(s)")

    if problems:
        for problem in problems:
            print(f"  DISAGREEMENT: {problem}", file=sys.stderr)
        print(f"{len(problems)} disagreement(s) between galata's reader and pyulog",
              file=sys.stderr)
        return 1
    print("galata's ULog reader agrees with pyulog on every compared value.")
    print("NOT established: behaviour on a real PX4 flight log. Neither implementation has "
          "read one here.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
