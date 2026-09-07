#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Compares two generated run records NUMERICALLY.
#
#   scripts/compare-run-json.py <committed> <freshly-generated>
#
# Called by scripts/gen-modal-map.sh --check and scripts/gen-report.sh --check.
# ONE comparator for both, deliberately: docs/TESTING.md's rule is that anything
# shared between a test and a generated document lives in one place, because two
# implementations would be two answers to the same question — and "is this
# committed file still what the code produces" is one question.
#
# It exists because the values it compares are downstream of a central
# difference, and ADR-0004 deliberately does NOT claim a cross-platform bound
# for such values: dividing by h amplifies a platform math-library disagreement
# by 1/h. The committed file is generated on whatever machine last regenerated
# it and checked on CI's Linux runner, so a byte diff would be a flaky gate —
# and a flaky gate is worse than a strict one, because people learn to re-run it
# and then re-run past a real failure.
#
# The tolerance below is therefore a stated engineering bound, not a fudge. Both
# emitters round to six significant figures, so two runs that agree physically
# agree textually in almost every case; this catches the boundary ones without
# letting a genuine change through. A MOVED POLE is orders of magnitude larger
# than this: the model changing at all moves these values in the third figure.

import json
import math
import sys

RELATIVE_TOLERANCE = 1e-5
# Below this, a difference is not a relative difference at all. The spiral root
# is 0.032 and a mode that has moved to zero is a change; a value that is zero
# in both files is not.
ABSOLUTE_FLOOR = 1e-9

if len(sys.argv) != 3:
    sys.exit(f"usage: {sys.argv[0]} <committed.json> <generated.json>")

def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate object key: {key!r}")
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError(f"nonfinite JSON constant: {value}")


try:
    with open(sys.argv[1], encoding="utf-8") as handle:
        committed = json.load(handle, object_pairs_hook=unique_object,
                              parse_constant=reject_constant)
    with open(sys.argv[2], encoding="utf-8") as handle:
        generated = json.load(handle, object_pairs_hook=unique_object,
                              parse_constant=reject_constant)
except (OSError, ValueError) as error:
    sys.exit(f"invalid run record: {error}")

if any(not isinstance(record, dict) or not record for record in (committed, generated)):
    sys.exit("invalid run record: expected a nonempty object")

problems = []
worst, worst_at = 0.0, ""


def walk(a, b, path):
    global worst, worst_at
    if isinstance(a, dict) or isinstance(b, dict):
        if not (isinstance(a, dict) and isinstance(b, dict)) or set(a) != set(b):
            problems.append(f"{path or '/'}: object keys differ")
            return
        for k in a:
            walk(a[k], b[k], f"{path}/{k}")
    elif isinstance(a, list) or isinstance(b, list):
        if not (isinstance(a, list) and isinstance(b, list)) or len(a) != len(b):
            problems.append(f"{path or '/'}: list length differs")
            return
        for i, (u, v) in enumerate(zip(a, b)):
            walk(u, v, f"{path}[{i}]")
    elif isinstance(a, bool) or isinstance(b, bool) or a is None or b is None:
        if type(a) is not type(b) or a != b:
            problems.append(f"{path}: {a!r} vs {b!r}")
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if any(isinstance(value, float) and not math.isfinite(value) for value in (a, b)):
            problems.append(f"{path}: nonfinite number ({a} vs {b})")
            return
        delta = abs(a - b)
        if delta <= ABSOLUTE_FLOOR:
            return
        scale = max(abs(a), abs(b))
        relative = abs(a / scale - b / scale)
        if relative > worst:
            worst, worst_at = relative, path
        if relative > RELATIVE_TOLERANCE:
            problems.append(f"{path}: {a!r} vs {b!r} — {relative:.2e} relative")
    else:
        # Strings: the mode LABELS live here, and a relabelled mode is exactly
        # the failure this gate is for. Compared exactly, on purpose.
        if type(a) is not type(b) or a != b:
            problems.append(f"{path}: {a!r} vs {b!r}")


walk(committed, generated, "")

if problems:
    print(f"{len(problems)} difference(s) beyond {RELATIVE_TOLERANCE:g} relative:")
    for p in problems[:40]:
        print(f"  {p}")
    if len(problems) > 40:
        print(f"  ... and {len(problems) - 40} more")
    sys.exit(1)

if worst > 0.0:
    print(f"{sys.argv[1]} is current "
          f"(worst agreement {worst:.2e} relative, at {worst_at}, "
          f"tolerance {RELATIVE_TOLERANCE:g}).")
else:
    print(f"{sys.argv[1]} is current (identical).")
