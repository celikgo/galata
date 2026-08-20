#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Regenerates docs/assets/modal-map.json — the poles that the social preview
# card is drawn from.
#
#   scripts/gen-modal-map.sh <path-to-galata-modal-map>            # write
#   scripts/gen-modal-map.sh <path> --check                        # diff only
#
# The --check form is what CI runs, for the same reason it diffs
# docs/VERIFICATION.md: the picture asserts that galata computes these poles and
# labels them by eigenvector participation. If the model or the classifier moves
# and the picture does not, the card keeps making a claim that has stopped being
# true — on every page that links this repository, which is the one place a
# stale number is hardest to notice.
#
# Redrawing the PNG from the JSON is a separate step and needs librsvg:
#   python3 scripts/gen-social-preview.py

set -euo pipefail

cd "$(dirname "$0")/.."

binary="${1:-}"
mode="${2:-write}"

if [ -z "$binary" ]; then
  printf '::error::usage: %s <path-to-galata-modal-map> [--check]\n' "$0"
  exit 2
fi

if [ ! -x "$binary" ]; then
  printf '::error::%s is not an executable\n' "$binary"
  exit 2
fi

target="docs/assets/modal-map.json"
generated="$(mktemp)"
trap 'rm -f "$generated"' EXIT

"$binary" > "$generated"

# A mode the classifier could not name would silently become an unlabelled
# cross on the card. Fail here instead.
if grep -q '"label": "unclassified"' "$generated"; then
  printf '::error::the chain produced an UNCLASSIFIED mode.\n'
  printf 'The social preview labels every pole it draws, so an unnamed mode means\n'
  printf 'the picture would assert a classification that did not happen.\n'
  exit 1
fi

if [ "$mode" = "--check" ]; then
  # Compared NUMERICALLY, not as text. These poles come out of a central
  # difference, and ADR-0004 does not claim a cross-platform bound for values
  # downstream of one: dividing by h amplifies a libm disagreement by 1/h. A
  # byte diff of a file generated on one platform and checked on another would
  # be a flaky gate, and a flaky gate is worse than a strict one because people
  # learn to re-run it and then re-run it past a real failure too.
  if ! python3 "$(dirname "$0")/compare-modal-map.py" "$target" "$generated"; then
    printf '\n::error::%s no longer matches what the chain produces, so\n' "$target"
    printf 'docs/assets/social-preview.png is asserting poles that are not current.\n'
    printf 'Regenerate both with:\n'
    printf '  scripts/gen-modal-map.sh %s\n' "$binary"
    printf '  python3 scripts/gen-social-preview.py\n'
    exit 1
  fi
  exit 0
fi

cp "$generated" "$target"
printf 'Wrote %s\n' "$target"
