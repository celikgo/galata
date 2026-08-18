#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Charter rule 4: the clone URL in the README is the real clone URL.
#
# Generalised: every URL written in any Markdown file in this repository must
# resolve. A dead link in documentation is the cheapest possible way to signal
# that nobody has read the documentation recently, and this project's whole
# claim is that its documentation is checked.
#
# What counts as failure:
#   * DNS does not resolve (curl exit 6)            -> FAIL
#   * HTTP 404 or 410                               -> FAIL
#   * connection refused / TLS failure after retry  -> FAIL
# What does not:
#   * 401 / 403 / 405 / 429 — the host is alive and is refusing a CI robot,
#     which is a different fact from the URL being wrong. Reported, not fatal.
#
# Relative Markdown links are checked too: they are the ones that rot fastest,
# because a file rename does not touch the document that points at it.

set -euo pipefail

cd "$(dirname "$0")/.."

fail=0
checked=0
warned=0

docs="$(git ls-files '*.md')"
if [ -z "$docs" ]; then
  printf '::error::no Markdown files found — this gate would be vacuously green\n'
  exit 1
fi

# --- Absolute URLs --------------------------------------------------------
urls="$(
  printf '%s\n' "$docs" | while IFS= read -r f; do
    grep -oE 'https?://[^][ )>"'"'"'`]+' "$f" || true
  done | sed -E 's/[.,;:]+$//' | LC_ALL=C sort -u
)"

printf 'Absolute URLs\n'
if [ -z "$urls" ]; then
  printf '  none\n'
else
  while IFS= read -r url; do
    [ -n "$url" ] || continue
    checked=$((checked + 1))
    code="$(curl -sS -o /dev/null -L \
      --retry 2 --retry-delay 2 --max-time 25 \
      -A 'galata-ci-link-check (+https://github.com/celikgo/galata)' \
      -w '%{http_code}' "$url" 2>/dev/null)" || rc=$?
    rc="${rc:-0}"
    if [ "$rc" -eq 6 ]; then
      printf '::error::NXDOMAIN %s\n' "$url"
      fail=1
    elif [ "$rc" -ne 0 ]; then
      printf '::error::unreachable (curl exit %s) %s\n' "$rc" "$url"
      fail=1
    elif [ "$code" = "404" ] || [ "$code" = "410" ]; then
      printf '::error::HTTP %s %s\n' "$code" "$url"
      fail=1
    elif [ "$code" -ge 400 ] 2>/dev/null; then
      printf '  warn HTTP %s %s (host alive, refusing CI)\n' "$code" "$url"
      warned=$((warned + 1))
    else
      printf '  ok   HTTP %s %s\n' "$code" "$url"
    fi
    unset rc
  done <<EOF
$urls
EOF
fi

# --- Relative links -------------------------------------------------------
printf '\nRelative links\n'
relative_found=0
while IFS= read -r f; do
  [ -n "$f" ] || continue
  dir="$(dirname "$f")"
  targets="$(grep -oE '\]\([^)#][^)]*\)' "$f" | sed -E 's/^\]\(//; s/\)$//' || true)"
  [ -n "$targets" ] || continue
  while IFS= read -r target; do
    [ -n "$target" ] || continue
    case "$target" in
      http://* | https://* | mailto:* | '#'*) continue ;;
    esac
    # Strip any in-page anchor before resolving the path.
    path="${target%%#*}"
    [ -n "$path" ] || continue
    relative_found=1
    checked=$((checked + 1))
    if [ -e "$dir/$path" ]; then
      printf '  ok   %s -> %s\n' "$f" "$path"
    else
      printf '::error::%s points at %s, which does not exist\n' "$f" "$path"
      fail=1
    fi
  done <<EOF
$targets
EOF
done <<EOF
$docs
EOF
[ "$relative_found" -eq 1 ] || printf '  none\n'

printf '\n%d link(s) checked, %d warning(s).\n' "$checked" "$warned"

if [ "$fail" -ne 0 ]; then
  printf 'Documentation link check FAILED.\n'
  exit 1
fi
printf 'Documentation link check OK.\n'
