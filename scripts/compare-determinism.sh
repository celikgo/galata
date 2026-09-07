#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# ADR-0004 tier 2: same source, different platform, agreement to a bound.
#
#   scripts/compare-determinism.sh <fingerprint-a> <fingerprint-b> [relative-tolerance]
#
# NOT a byte comparison, and the reason is worth restating because a reader will
# reasonably ask why not. sqrt is required by IEEE 754 to be correctly rounded
# and is bit-identical everywhere. sin, cos, tan, asin, atan2, exp, log and pow
# are not: they come from the platform's math library — glibc, Apple's libm, the
# UCRT — and those disagree in their final bits. galata cannot avoid them; angle
# of attack is an atan2 and the atmosphere's pressure profile is a pow.
#
# So this reports the OBSERVED deviation as well as gating it. A bound with no
# measurement behind it is a guess with a number attached.

set -euo pipefail

a="${1:-}"
b="${2:-}"
tolerance="${3:-1e-9}"

if [ ! -f "$a" ] || [ ! -f "$b" ]; then
  printf '::error::usage: %s <fingerprint-a> <fingerprint-b> [relative-tolerance]\n' "$0"
  exit 2
fi

python3 - "$a" "$b" "$tolerance" <<'PY'
import math
import sys

path_a, path_b, tolerance = sys.argv[1], sys.argv[2], float(sys.argv[3])
if not math.isfinite(tolerance) or tolerance <= 0:
    sys.exit("::error::tolerance must be finite and positive")

# Keys beginning "tier1." are excluded from the cross-platform comparison.
#
# They are downstream of a central-difference Jacobian, which divides by the
# perturbation h and so amplifies a platform libm disagreement by 1/h. That can
# reach 1e-8 relative on a small matrix entry, past this gate, through nobody's
# error. They are still held BYTE-IDENTICAL within a platform by tier 1, which
# is the stronger claim anyway.
def load(path):
    values = {}
    with open(path) as handle:
        for line in handle:
            line = line.rstrip("\n")
            if not line or line.startswith("#"):
                continue
            if line.count("\t") != 1:
                raise ValueError(f"{path}: expected one tab between key and value")
            key, value = line.split("\t")
            # Mode labels include embedded ASCII spaces ("roll subsidence").
            # Preserve those exact keys while refusing ambiguous padding and
            # control/non-ASCII whitespace; a malformed key must not disappear.
            if (not key or key != key.strip(" ") or key in values
                    or any((character.isspace() and character != " ")
                           or ord(character) < 32 or ord(character) == 127 for character in key)):
                raise ValueError(f"{path}: empty, malformed or duplicate key {key!r}")
            number = float(value)
            if not math.isfinite(number):
                raise ValueError(f"{path}: nonfinite value at {key}")
            values[key] = number
    return values

try:
    a, b = load(path_a), load(path_b)
except (OSError, ValueError) as error:
    sys.exit(f"::error::{error}")

only_a, only_b = set(a) - set(b), set(b) - set(a)
if only_a or only_b:
    for key in sorted(only_a):
        print(f"::error::key present only in {path_a}: {key}")
    for key in sorted(only_b):
        print(f"::error::key present only in {path_b}: {key}")
    sys.exit(1)

# Tier-1-only values may differ, but must still be finite and present in both
# inputs. Validate their shape before excluding their numerical comparison.
skipped_a = sum(key.startswith("tier1.") for key in a)
a = {key: value for key, value in a.items() if not key.startswith("tier1.")}
b = {key: value for key, value in b.items() if not key.startswith("tier1.")}
if not a:
    sys.exit("::error::no cross-platform values to compare")

worst_key, worst = None, 0.0
identical = 0
failures = []

for key in sorted(a):
    x, y = a[key], b[key]
    if x == y:
        identical += 1
        continue
    scale = max(abs(x), abs(y))
    deviation = abs(x / scale - y / scale) if scale > 0 else 0.0
    if deviation > worst:
        worst, worst_key = deviation, key
    if deviation > tolerance:
        failures.append((key, x, y, deviation))

total = len(a)
print(f"Determinism tier 2: {total} values compared, {skipped_a} tier-1-only keys skipped.")
print(f"  bit-identical across platforms: {identical} of {total} "
      f"({100.0 * identical / total:.1f}%)")
if worst_key is None:
    print("  every value is bit-identical.")
else:
    print(f"  worst relative deviation: {worst:.3e} at {worst_key}")
print(f"  gate: {tolerance:.0e}")

if failures:
    print()
    for key, x, y, deviation in failures:
        print(f"::error::{key}: {x!r} vs {y!r}, relative deviation {deviation:.3e}")
    print()
    print("Cross-platform agreement is bounded, not exact (ADR-0004 tier 2), but these")
    print("exceed the published bound. Either a real divergence has been introduced, or")
    print("the bound needs revisiting with evidence — not silently.")
    sys.exit(1)

print("  PASS")
PY
