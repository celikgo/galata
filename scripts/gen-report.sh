#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Regenerates docs/assets/nt33a-fc1-run.json — the run that
# docs/reports/nt33a-fc1.html is drawn from.
#
#   scripts/gen-report.sh <path-to-galata-report-data>            # write
#   scripts/gen-report.sh <path> --check                          # diff only
#
# The --check form is what CI runs, for the same reason it diffs
# docs/VERIFICATION.md and docs/assets/modal-map.json: the page asserts that
# galata computed these numbers. If the model, the classifier or the margin code
# moves and the page does not, the page keeps making a claim that has stopped
# being true — and a stale page still looks like evidence, which is worse than
# no page at all.
#
# Redrawing the page from the JSON is a separate step and needs no compiler:
#   python3 scripts/gen-report-page.py

set -euo pipefail

cd "$(dirname "$0")/.."

binary="${1:-}"
mode="${2:-write}"

if [ -z "$binary" ]; then
  printf '::error::usage: %s <path-to-galata-report-data> [--check]\n' "$0"
  exit 2
fi

if [ ! -x "$binary" ]; then
  printf '::error::%s is not an executable\n' "$binary"
  exit 2
fi

target="docs/assets/nt33a-fc1-run.json"
generated="$(mktemp)"
trap 'rm -f "$generated"' EXIT

"$binary" > "$generated"

# A mode the classifier could not name would become an unlabelled row and an
# unlabelled cross. The page labels every pole it draws, so fail here instead.
if grep -q '"label": "unclassified"' "$generated"; then
  printf '::error::the chain produced an UNCLASSIFIED mode.\n'
  printf 'The report page labels every mode it reports, so an unnamed one means the\n'
  printf 'page would assert a classification that did not happen.\n'
  exit 1
fi

if [ "$mode" = "--check" ]; then
  # Compared NUMERICALLY by the same comparator the pole map uses. These values
  # are downstream of a central difference, and ADR-0004 does not claim a
  # cross-platform bound for such values: dividing by h amplifies a libm
  # disagreement by 1/h. A byte diff of a file generated on one machine and
  # checked on another would be a flaky gate, and a flaky gate is worse than a
  # strict one because people learn to re-run it and then re-run past a real
  # failure too.
  if ! python3 "$(dirname "$0")/compare-run-json.py" "$target" "$generated"; then
    printf '\n::error::%s no longer matches what the chain produces, so\n' "$target"
    printf 'docs/reports/nt33a-fc1.html is reporting numbers that are not current.\n'
    printf 'Regenerate both with:\n'
    printf '  scripts/gen-report.sh %s\n' "$binary"
    printf '  python3 scripts/gen-report-page.py\n'
    exit 1
  fi
  exit 0
fi

cp "$generated" "$target"
printf 'Wrote %s\n' "$target"
