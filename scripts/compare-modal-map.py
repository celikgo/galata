#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
#
# Compares two docs/assets/modal-map.json files NUMERICALLY.
#
#   scripts/compare-modal-map.py <committed> <freshly-generated>
#
# Called by scripts/gen-modal-map.sh --check. It exists because the poles it
# compares are downstream of a central difference, and ADR-0004 deliberately
# does NOT claim a cross-platform bound for such values: dividing by h amplifies
# a platform math-library disagreement by 1/h. The committed file is generated
# on whatever machine last regenerated it and checked on CI's Linux runner, so
# a byte diff would be a flaky gate — and a flaky gate is worse than a strict
# one, because people learn to re-run it and then re-run past a real failure.
#
# The tolerance below is therefore a stated engineering bound, not a fudge. The
# emitter rounds to six significant figures, so two runs that agree physically
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

committed = json.loads(open(sys.argv[1], encoding="utf-8").read())
generated = json.loads(open(sys.argv[2], encoding="utf-8").read())

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
        if a != b:
            problems.append(f"{path}: {a!r} vs {b!r}")
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if math.isnan(a) or math.isnan(b):
            problems.append(f"{path}: not a number ({a} vs {b})")
            return
        delta = abs(a - b)
        if delta <= ABSOLUTE_FLOOR:
            return
        relative = delta / max(abs(a), abs(b))
        if relative > worst:
            worst, worst_at = relative, path
        if relative > RELATIVE_TOLERANCE:
            problems.append(f"{path}: {a!r} vs {b!r} — {relative:.2e} relative")
    else:
        # Strings: the mode LABELS live here, and a relabelled mode is exactly
        # the failure this gate is for. Compared exactly, on purpose.
        if a != b:
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
