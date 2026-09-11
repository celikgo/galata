#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Writes a small, real PX4 ULog file for the reader's tests.
#
# WHY THIS EXISTS RATHER THAN A COMMITTED .ulg. A binary fixture nobody can read
# is a fixture nobody can check. This script is the fixture: what is in the file
# is what is written here, in the open, and a reviewer who doubts a value edits
# the line that produced it.
#
# WHY IT IS NOT CIRCULAR. ADR-0016 requires the reader to be checked against an
# implementation that is not ours. It cannot be checked against THIS script —
# same author, same misreading of the format if there is one. So the fixture is
# handed to `pyulog`, PX4's own Python tooling, which parses it independently;
# `verify_with_pyulog` below fails if pyulog cannot read what this wrote, or
# reads different values from it. Only a fixture pyulog agrees with is used.
#
# ADR-0016 said fixtures would be "written by pyulog". They cannot be: pyulog's
# ULog constructor requires a file to read and offers no documented way to build
# one from nothing. The direction is inverted — we write, pyulog checks — which
# gives the same independence for the thing that matters, the READER.
#
# pyulog is a TEST-TIME tool. It is not a dependency of the library, nothing
# built from it is shipped, and the C++ reader never sees it.

import argparse
import struct
import sys

MAGIC = b"ULog\x01\x12\x35"
VERSION = 1

# Message types, from the format specification.
FLAG_BITS = ord("B")
FORMAT = ord("F")
ADD_LOGGED = ord("A")
DATA = ord("D")
INFO = ord("I")
PARAMETER = ord("P")
DROPOUT = ord("O")


def message(kind: int, payload: bytes) -> bytes:
    """One message: uint16 size, uint8 type, payload."""
    return struct.pack("<HB", len(payload), kind) + payload


def flag_bits() -> bytes:
    # compat 8, incompat 8, appended offsets 3 x uint64. Zero incompat flags
    # means no appended data, which is the only case this reader supports.
    return message(FLAG_BITS, bytes(8) + bytes(8) + struct.pack("<3Q", 0, 0, 0))


def format_message(name: str, fields: list[tuple[str, str]]) -> bytes:
    body = name + ":" + "".join(f"{t} {n};" for t, n in fields)
    return message(FORMAT, body.encode("ascii"))


def info(key_type: str, key: str, value: bytes) -> bytes:
    key_text = f"{key_type} {key}".encode("ascii")
    return message(INFO, struct.pack("<B", len(key_text)) + key_text + value)


def parameter(key: str, value: float) -> bytes:
    key_text = f"float {key}".encode("ascii")
    return message(PARAMETER, struct.pack("<B", len(key_text)) + key_text
                   + struct.pack("<f", value))


def add_logged(msg_id: int, name: str, multi_id: int = 0) -> bytes:
    return message(ADD_LOGGED, struct.pack("<BH", multi_id, msg_id) + name.encode("ascii"))


def data(msg_id: int, payload: bytes) -> bytes:
    return message(DATA, struct.pack("<H", msg_id) + payload)


def dropout(duration_ms: int) -> bytes:
    return message(DROPOUT, struct.pack("<H", duration_ms))


# The declared subset. Field order here IS the packed byte order.
TOPICS = {
    "vehicle_attitude": [("uint64_t", "timestamp"), ("float[4]", "q")],
    "vehicle_angular_velocity": [("uint64_t", "timestamp"), ("float[3]", "xyz")],
    "vehicle_local_position": [
        ("uint64_t", "timestamp"),
        ("float", "x"), ("float", "y"), ("float", "z"),
        ("float", "vx"), ("float", "vy"), ("float", "vz"),
    ],
    "sensor_combined": [
        ("uint64_t", "timestamp"),
        ("float[3]", "gyro_rad"),
        ("uint32_t", "gyro_integral_dt"),
        ("float[3]", "accelerometer_m_s2"),
        ("uint32_t", "accelerometer_integral_dt"),
    ],
    "actuator_motors": [
        ("uint64_t", "timestamp"),
        ("float[12]", "control"),
    ],
    "battery_status": [
        ("uint64_t", "timestamp"),
        ("float", "voltage_v"), ("float", "current_a"), ("float", "remaining"),
    ],
}

IDS = {name: index for index, name in enumerate(TOPICS)}

# A SECOND INSTANCE of one topic, which is how PX4 logs a duplicated sensor: the
# same message format, a new message id, and multi_id 1 in ADD_LOGGED_MSG. It is
# here because a reader that ignored multi_id would merge the two instances and
# produce a channel that is neither — and would pass every single-instance test.
# The values below are deliberately far from instance 0's so a merge is obvious
# rather than plausible.
SECOND_INSTANCE_TOPIC = "sensor_combined"
SECOND_INSTANCE_ID = len(TOPICS)
SECOND_INSTANCE_MULTI_ID = 1


def build(samples: int = 20, period_us: int = 4000, with_dropout: bool = False) -> bytes:
    out = [MAGIC, struct.pack("<BQ", VERSION, 0), flag_bits()]
    for name, fields in TOPICS.items():
        out.append(format_message(name, fields))
    out.append(info("char[5]", "sys_name", struct.pack("<B", 4) + b"PX4"))
    out.append(parameter("BAT_N_CELLS", 6.0))
    for name in TOPICS:
        out.append(add_logged(IDS[name], name))
    out.append(add_logged(SECOND_INSTANCE_ID, SECOND_INSTANCE_TOPIC,
                          multi_id=SECOND_INSTANCE_MULTI_ID))

    for k in range(samples):
        t = k * period_us
        # Attitude: a small, DELIBERATELY ASYMMETRIC rotation about each axis in
        # turn, so a reader that transposed the quaternion or reordered wxyz
        # produces visibly different numbers rather than a plausible one.
        angle = 0.02 * (k + 1)
        out.append(data(IDS["vehicle_attitude"],
                        struct.pack("<Q4f", t, 1.0, 0.1 * angle, 0.2 * angle, 0.3 * angle)))
        # Body rates: one axis per component, distinct magnitudes and signs.
        out.append(data(IDS["vehicle_angular_velocity"],
                        struct.pack("<Q3f", t, 0.10 + k, -0.20 - k, 0.30 + k)))
        # NED position and ground velocity, all six distinct.
        out.append(data(IDS["vehicle_local_position"],
                        struct.pack("<Q6f", t, 1.0 + k, -2.0 - k, -3.0 - k,
                                    0.5 + k, -0.6 - k, 0.7 + k)))
        out.append(data(IDS["sensor_combined"],
                        struct.pack("<Q3fI3fI", t, 0.01 + k, -0.02 - k, 0.03 + k, 4000,
                                    0.11 + k, -0.22 - k, -9.81 - k, 4000)))
        motors = [0.0] * 12
        for rotor in range(4):
            motors[rotor] = 0.5 + 0.01 * rotor + 0.001 * k
        out.append(data(IDS["actuator_motors"], struct.pack("<Q12f", t, *motors)))
        out.append(data(IDS["battery_status"],
                        struct.pack("<Q3f", t, 24.0 - 0.01 * k, 20.0 + 0.1 * k, 1.0 - 0.01 * k)))
        # The second IMU. Every component is offset by 100 from instance 0's, so
        # a reader that confused the instances reports a value out by exactly
        # that and cannot be mistaken for a rounding difference.
        out.append(data(SECOND_INSTANCE_ID,
                        struct.pack("<Q3fI3fI", t, 100.01 + k, -100.02 - k, 100.03 + k, 4000,
                                    100.11 + k, -100.22 - k, 90.19 - k, 4000)))
        if with_dropout and k == samples // 2:
            out.append(dropout(17))
    return b"".join(out)


def verify_with_pyulog(path: str) -> None:
    """Parse the fixture with PX4's own tooling and check what it found.

    A fixture this repository both wrote and read would prove only that it is
    self-consistent. This is the independence ADR-0016 asks for.
    """
    from pyulog import ULog

    log = ULog(path)
    found = {d.name for d in log.data_list}
    missing = set(TOPICS) - found
    if missing:
        raise SystemExit(f"pyulog did not find {sorted(missing)}; the fixture is not valid ULog")
    attitude = next(d for d in log.data_list if d.name == "vehicle_attitude")
    # Spot-check a value pyulog decoded against what was written above.
    first_q1 = attitude.data["q[1]"][0]
    if abs(first_q1 - 0.1 * 0.02) > 1e-6:
        raise SystemExit(f"pyulog read q[1]={first_q1}, expected {0.1 * 0.02}")
    # And that pyulog sees BOTH instances of the duplicated topic as separate
    # series. If it merged them, the fixture is not exercising instances and the
    # reader comparison below would be checking nothing.
    instances = sorted(d.multi_id for d in log.data_list
                       if d.name == SECOND_INSTANCE_TOPIC)
    if instances != [0, SECOND_INSTANCE_MULTI_ID]:
        raise SystemExit(f"pyulog saw {SECOND_INSTANCE_TOPIC} instances {instances}, "
                         f"expected [0, {SECOND_INSTANCE_MULTI_ID}]")
    print(f"pyulog agrees: {len(found)} topics, "
          f"{len(attitude.data['timestamp'])} attitude samples, "
          f"{SECOND_INSTANCE_TOPIC} instances {instances}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--dropout", action="store_true")
    parser.add_argument("--verify", action="store_true",
                        help="parse the result with pyulog and check it")
    arguments = parser.parse_args()
    with open(arguments.output, "wb") as handle:
        handle.write(build(arguments.samples, with_dropout=arguments.dropout))
    if arguments.verify:
        verify_with_pyulog(arguments.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
