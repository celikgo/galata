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
import sys

path_a, path_b, tolerance = sys.argv[1], sys.argv[2], float(sys.argv[3])

# Keys beginning "tier1." are excluded from the cross-platform comparison.
#
# They are downstream of a central-difference Jacobian, which divides by the
# perturbation h and so amplifies a platform libm disagreement by 1/h. That can
# reach 1e-8 relative on a small matrix entry, past this gate, through nobody's
# error. They are still held BYTE-IDENTICAL within a platform by tier 1, which
# is the stronger claim anyway.
def load(path):
    values, skipped = {}, 0
    with open(path) as handle:
        for line in handle:
            line = line.rstrip("\n")
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition("\t")
            if key.startswith("tier1."):
                skipped += 1
                continue
            values[key] = float(value)
    return values, skipped

a, skipped_a = load(path_a)
b, skipped_b = load(path_b)

only_a, only_b = set(a) - set(b), set(b) - set(a)
if only_a or only_b:
    for key in sorted(only_a):
        print(f"::error::key present only in {path_a}: {key}")
    for key in sorted(only_b):
        print(f"::error::key present only in {path_b}: {key}")
    sys.exit(1)

worst_key, worst = None, 0.0
identical = 0
failures = []

for key in sorted(a):
    x, y = a[key], b[key]
    if x == y:
        identical += 1
        continue
    scale = max(abs(x), abs(y))
    deviation = abs(x - y) / scale if scale > 0 else abs(x - y)
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
