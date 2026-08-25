#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# ADR-0004 tier 1: same binary, same platform, twice, byte-identical.
#
#   scripts/check-determinism.sh <path-to-galata-determinism> [output-file]
#
# This is the strong claim and the one that can be gated absolutely. It is what
# a regression suite, a Monte Carlo whose worst case must be re-examinable, and
# a reproducible paper all rest on.
#
# The fingerprint prints every value with %.17g, which round-trips a double
# exactly, so byte-identical output means bit-identical values rather than
# values that merely print the same.
#
# A THIRD run goes out under a hostile locale. ADR-0004's locale-independence
# item is the weakest on its list, because it rests on a convention — nothing in
# galata calls setlocale or imbue, so the process stays in the "C" locale and
# printf's %.17g keeps its decimal point. A single setlocale(LC_ALL, "")
# anywhere in the library would adopt the environment's locale, turn every
# decimal point into a comma on a German machine, and pass tier 1 anyway, since
# both runs would be equally wrong. This catches exactly that: run the binary
# with a comma-decimal locale IN THE ENVIRONMENT and require the bytes not to
# move. It passes today because galata ignores the environment, which is the
# whole claim.

set -euo pipefail

cd "$(dirname "$0")/.."

binary="${1:-}"
output="${2:-}"

if [ -z "$binary" ] || [ ! -x "$binary" ]; then
  printf '::error::usage: %s <path-to-galata-determinism> [output-file]\n' "$0"
  exit 2
fi

first="$(mktemp)"
second="$(mktemp)"
trap 'rm -f "$first" "$second"' EXIT

"$binary" > "$first"
"$binary" > "$second"

if ! diff -u "$first" "$second"; then
  printf '\n::error::the same binary produced different output on two consecutive runs.\n'
  printf 'That is a correctness bug, not a flake. Usual causes: unordered container\n'
  printf 'iteration reaching the output, an unseeded PRNG, a tolerance-based loop exit,\n'
  printf 'address-dependent ordering, or uninitialised memory.\n'
  exit 1
fi

lines="$(grep -cv '^#' "$first")"
if [ "$lines" -lt 50 ]; then
  printf '::error::the fingerprint has only %s values; it is too thin to gate anything\n' "$lines"
  exit 1
fi

printf 'Determinism tier 1: %s values, byte-identical across two runs.\n' "$lines"

# --- the same binary, under a locale that formats 1/2 as "0,5" --------------
#
# The candidate is PROBED rather than assumed: a locale name that exists but
# behaves like C would give false confidence, so the probe requires the C
# library to actually report a comma as its decimal point.
hostile=""
for candidate in de_DE.UTF-8 fr_FR.UTF-8 de_DE.utf8 fr_FR.utf8 de_DE fr_FR German_Germany.1252; do
  if LC_ALL="$candidate" python3 -c '
import locale, sys
try:
    locale.setlocale(locale.LC_ALL, "")
except locale.Error:
    sys.exit(1)
sys.exit(0 if locale.localeconv()["decimal_point"] == "," else 1)
' 2>/dev/null; then
    hostile="$candidate"
    break
  fi
done

if [ -z "$hostile" ]; then
  # Named, not silent. A gate that is green because it did not run is worse
  # than no gate, and this is the one platform-dependent step in the script.
  printf 'Determinism tier 1: locale check SKIPPED — no comma-decimal locale on this machine.\n'
  printf '  Tried: de_DE.UTF-8 fr_FR.UTF-8 de_DE.utf8 fr_FR.utf8 de_DE fr_FR German_Germany.1252\n'
else
  third="$(mktemp)"
  # shellcheck disable=SC2064
  trap "rm -f '$first' '$second' '$third'" EXIT
  LC_ALL="$hostile" "$binary" > "$third"
  if ! diff -u "$first" "$third"; then
    printf '\n::error::the fingerprint changed under LC_ALL=%s.\n' "$hostile"
    printf 'Something in galata now adopts the process locale — a setlocale(LC_ALL, "") or an\n'
    printf 'imbue() on a stream that reaches the output. ADR-0004 relies on galata staying in\n'
    printf 'the "C" locale, because printf and iostreams both take their decimal point from\n'
    printf 'it. Tier 1 alone would NOT have caught this: two runs on the same machine would\n'
    printf 'be equally wrong and equally identical.\n'
    exit 1
  fi
  printf 'Determinism tier 1: byte-identical again under LC_ALL=%s.\n' "$hostile"
fi

if [ -n "$output" ]; then
  cp "$first" "$output"
  printf 'Fingerprint written to %s\n' "$output"
fi
