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

if [ -n "$output" ]; then
  cp "$first" "$output"
  printf 'Fingerprint written to %s\n' "$output"
fi
