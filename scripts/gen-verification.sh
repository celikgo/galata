#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Regenerates docs/VERIFICATION.md from the validation reference data and the
# code that consumes it.
#
#   scripts/gen-verification.sh <path-to-galata-validation-report>   # write
#   scripts/gen-verification.sh <path> --check                       # diff only
#
# The --check form is what CI runs. It fails when the committed report no longer
# matches what the code produces, which is the whole point: a V&V report that
# has drifted from the implementation is worse than no report, because it looks
# like evidence.

set -euo pipefail

cd "$(dirname "$0")/.."

binary="${1:-}"
mode="${2:-write}"

if [ -z "$binary" ]; then
  printf '::error::usage: %s <path-to-galata-validation-report> [--check]\n' "$0"
  exit 2
fi

if [ ! -x "$binary" ]; then
  printf '::error::%s is not an executable\n' "$binary"
  exit 2
fi

target="docs/VERIFICATION.md"
generated="$(mktemp)"
trap 'rm -f "$generated"' EXIT

"$binary" tests/validation/reference > "$generated"

# An unresolved placeholder means a prose sentence refers to a measurement the
# report does not compute. The generator leaves it visible rather than dropping
# it silently, because a sentence with the number quietly removed still reads
# fine and says nothing.
if grep -n 'UNRESOLVED:' "$generated"; then
  printf '\n::error::the report contains unresolved measurement placeholders (above).\n'
  printf 'Either add the measurement in tools/validation/report_main.cpp, or fix the\n'
  printf 'placeholder name.\n'
  exit 1
fi

if [ "$mode" = "--check" ]; then
  if ! diff -u "$target" "$generated"; then
    printf '\n::error::%s is stale.\n' "$target"
    printf 'Regenerate it with:\n'
    printf '  scripts/gen-verification.sh %s\n' "$binary"
    exit 1
  fi
  printf '%s is current.\n' "$target"
  exit 0
fi

cp "$generated" "$target"
printf 'Wrote %s\n' "$target"
