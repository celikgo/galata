#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Regenerates the capability table in README.md from the registry itself.
#
#   scripts/gen-status-table.sh <path-to-galata>            # write
#   scripts/gen-status-table.sh <path-to-galata> --check    # diff only
#
# Charter rule 2: nothing is documented before it works. A hand-maintained
# status table is the classic way that rule gets broken — not by anyone lying,
# but by a row surviving a refactor that removed the thing it describes. This
# table is generated from the same registry the CLI dispatches through, so it
# cannot describe a capability that does not exist.

set -euo pipefail

cd "$(dirname "$0")/.."

binary="${1:-}"
mode="${2:-write}"

if [ -z "$binary" ] || [ ! -x "$binary" ]; then
  printf '::error::usage: %s <path-to-galata-binary> [--check]\n' "$0"
  exit 2
fi

begin='<!-- BEGIN GENERATED CAPABILITY TABLE -->'
end='<!-- END GENERATED CAPABILITY TABLE -->'

if ! grep -qF "$begin" README.md || ! grep -qF "$end" README.md; then
  printf '::error::README.md has no generated-capability-table markers\n'
  exit 1
fi

table_file="$(mktemp)"
updated="$(mktemp)"
trap 'rm -f "$updated" "$table_file"' EXIT

# Written to a file rather than passed through -v: awk does not accept a
# multi-line value in a -v assignment.
"$binary" capabilities --markdown > "$table_file"

awk -v begin="$begin" -v end="$end" -v table_file="$table_file" '
  $0 == begin {
    print
    while ((getline line < table_file) > 0) { print line }
    close(table_file)
    skipping = 1
    next
  }
  $0 == end { print; skipping = 0; next }
  !skipping { print }
' README.md > "$updated"

if [ "$mode" = "--check" ]; then
  if ! diff -u README.md "$updated"; then
    printf '\n::error::README.md capability table is stale.\n'
    printf 'Regenerate it with:\n  scripts/gen-status-table.sh %s\n' "$binary"
    exit 1
  fi
  printf 'README.md capability table is current.\n'
  exit 0
fi

cp "$updated" README.md
printf 'Updated the capability table in README.md\n'
