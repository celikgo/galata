#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Charter rule 3: one source of version truth.
#
# The VERSION file at the repository root is that source. Every other surface
# that shows a version to a human or a package manager must derive from it.
# This script enumerates those surfaces and fails when any of them disagrees.
#
# Surfaces that do not exist yet are reported as skipped, by name. That is
# deliberate: a gate which silently checks nothing is worse than no gate, and
# the skip list is the honest record of which surfaces still have to be wired.

set -euo pipefail

cd "$(dirname "$0")/.."

fail=0
checked=0

note() { printf '  %s\n' "$1"; }
bad() {
  printf '::error::%s\n' "$1"
  fail=1
}

if [ ! -f VERSION ]; then
  bad "VERSION file is missing — it is the source of truth and must exist"
  exit 1
fi

version="$(tr -d '[:space:]' < VERSION)"

if ! printf '%s' "$version" | grep -qE '^[0-9]+\.[0-9]+\.[0-9]+$'; then
  bad "VERSION contains '$version', which is not major.minor.patch"
  exit 1
fi

printf 'VERSION = %s\n\n' "$version"

# --- vcpkg manifest -------------------------------------------------------
printf 'vcpkg.json version-string\n'
if [ -f vcpkg.json ]; then
  manifest_version="$(python3 -c 'import json;print(json.load(open("vcpkg.json")).get("version-string",""))')"
  checked=$((checked + 1))
  if [ "$manifest_version" = "$version" ]; then
    note "ok: $manifest_version"
  else
    bad "vcpkg.json version-string is '$manifest_version', VERSION is '$version'"
  fi
else
  note 'skipped: vcpkg.json does not exist'
fi

# --- CMake ----------------------------------------------------------------
# CMake must READ the VERSION file rather than restate the number. Checking the
# mechanism rather than the value is what makes this gate durable: a hardcoded
# literal that happens to match today would pass a value comparison and drift
# on the next release.
printf 'CMakeLists.txt derives its version from the VERSION file\n'
if [ -f CMakeLists.txt ]; then
  checked=$((checked + 1))
  if grep -qE 'file\(READ .*VERSION' CMakeLists.txt; then
    note 'ok: project() version comes from file(READ ... VERSION)'
  else
    bad 'CMakeLists.txt does not read the VERSION file'
  fi
  if grep -qE '^\s*project\(galata\s*$' CMakeLists.txt &&
    grep -qE '^\s*VERSION \$\{GALATA_VERSION\}\s*$' CMakeLists.txt; then
    note 'ok: project(galata VERSION ${GALATA_VERSION})'
  else
    bad 'project() does not take its VERSION from ${GALATA_VERSION}'
  fi
else
  note 'skipped: CMakeLists.txt does not exist'
fi

# --- Surfaces that report a version at runtime ----------------------------
# Each is checked once it exists. Until then it is named here so the gap is
# visible in the CI log rather than assumed away.
for surface in \
  "CLI --version:src/cli" \
  "Python bindings __version__:bindings/python" \
  "Desktop About box:src/desktop"; do
  label="${surface%%:*}"
  path="${surface#*:}"
  printf '%s\n' "$label"
  if [ -d "$path" ]; then
    checked=$((checked + 1))
    # Any file under the surface that hardcodes a three-component version
    # literal is a second source of truth by definition.
    if hits="$(grep -rEn '"[0-9]+\.[0-9]+\.[0-9]+"' "$path" 2>/dev/null)"; then
      bad "$label hardcodes a version literal instead of deriving it:"
      printf '%s\n' "$hits"
    else
      note 'ok: no hardcoded version literal'
    fi
  else
    note "skipped: $path does not exist yet"
  fi
done

printf '\n%d surface(s) checked.\n' "$checked"

if [ "$fail" -ne 0 ]; then
  printf 'Version consistency FAILED.\n'
  exit 1
fi

printf 'Version consistency OK.\n'
