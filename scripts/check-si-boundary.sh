#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# ADR-0003: strict SI internally, conversion only at the boundary.
#
# The failure mode this catches is not someone calling feet_to_metres() inside a
# solver — nobody does that. It is someone writing `alt * 0.3048` inline because
# it seemed obvious at the time, and the reviewer not noticing because the line
# reads correctly.
#
# So the check is on numeric literals as much as on function names.
#
# Directories that do not exist yet are reported as skipped, by name, so the
# coverage of this gate is visible in the log rather than assumed.

set -euo pipefail

cd "$(dirname "$0")/.."

# The numerical core, headers as well as sources. Scanning only src/ would leave
# the obvious hole: an inline conversion in a header is still a conversion in the
# core, and headers are where inline arithmetic tends to live.
#
# UI code and file-format adapters are deliberately absent: converting is their
# job. include/galata/units.hpp is deliberately absent too — it is the definition
# site, and it sits outside include/galata/core/ precisely so that this list does
# not have to carve out an exception for it.
CORE_DIRS="src/core src/model src/numerics src/trim src/linearize src/synth src/analyze src/sim src/ident"
CORE_DIRS="$CORE_DIRS include/galata/core include/galata/model include/galata/numerics"
CORE_DIRS="$CORE_DIRS include/galata/trim include/galata/linearize include/galata/synth"
CORE_DIRS="$CORE_DIRS include/galata/analyze include/galata/sim include/galata/ident"

# Conversion factors from ADR-0003's table, plus the reciprocals and the
# rounded forms that appear in hand-written code. Matched as substrings of a
# numeric literal so that 0.3048, 0.30480 and 3.28084 are all caught.
FACTORS='0\.3048|3\.28084|3\.280839|1852(\.0)?[^0-9]|0\.5144|1\.94384|57\.2957|57\.29577|0\.0174532|273\.15|4\.44822|14\.5939|2\.20462|0\.45359|6076\.11|1\.68781'

# Conversion entry points. Calling one of these from the core is a direct
# violation regardless of literals.
CONVERSIONS='feet_to_metres|metres_to_feet|knots_to_metres_per_second|metres_per_second_to_knots|degrees_to_radians|radians_to_degrees|celsius_to_kelvin|kelvin_to_celsius|pounds_force_to_newtons|newtons_to_pounds_force|slugs_to_kilograms|kilograms_to_slugs'

fail=0
scanned=0
skipped=""

for dir in $CORE_DIRS; do
  if [ ! -d "$dir" ]; then
    skipped="$skipped $dir"
    continue
  fi

  files="$(git ls-files "$dir" | grep -E '\.(cpp|hpp|h|c|cc)$' || true)"
  [ -n "$files" ] || { skipped="$skipped $dir(empty)"; continue; }

  while IFS= read -r f; do
    [ -n "$f" ] || continue
    scanned=$((scanned + 1))

    # Lines carrying an explicit, reviewed exemption are excluded. The marker is
    # deliberately verbose so it cannot be typed by accident.
    body="$(grep -vn 'GALATA_SI_EXEMPT' "$f" || true)"
    [ -n "$body" ] || continue

    if hits="$(printf '%s\n' "$body" | grep -nE "$CONVERSIONS")"; then
      printf '::error file=%s::calls a unit conversion inside the numerical core (ADR-0003)\n' "$f"
      printf '%s\n' "$hits"
      fail=1
    fi

    # Strip comment bodies before looking for literals: a comment that says
    # "10,000 ft = 3048 m" is documentation, not a conversion.
    code="$(printf '%s\n' "$body" | sed -E 's://.*$::' )"
    if hits="$(printf '%s\n' "$code" | grep -nE "$FACTORS")"; then
      printf '::error file=%s::unit conversion factor as a literal inside the numerical core (ADR-0003)\n' "$f"
      printf '%s\n' "$hits"
      printf '  If this constant is legitimately not a unit conversion, append a\n'
      printf '  GALATA_SI_EXEMPT comment on the line explaining what it is.\n'
      fail=1
    fi
  done <<EOF
$files
EOF
done

printf 'SI boundary: %d file(s) scanned.\n' "$scanned"
[ -z "$skipped" ] || printf 'SI boundary: not yet present, skipped:%s\n' "$skipped"

if [ "$fail" -ne 0 ]; then
  printf 'SI boundary check FAILED.\n'
  exit 1
fi
printf 'SI boundary check OK.\n'
