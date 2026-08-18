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
#   * DNS does not resolve (curl exit 6)  -> FAIL. The domain is wrong or gone,
#     which is exactly what rule 4 exists to catch.
#   * HTTP 404 or 410                     -> FAIL. The path is wrong or gone.
#
# What does not:
#   * 401 / 403 / 405 / 429 — the host is alive and is refusing a CI robot,
#     which is a different fact from the URL being wrong.
#   * A TIMEOUT, after retries. This gate's job is to catch URLs that are
#     WRONG, not hosts that are SLOW. ntrs.nasa.gov — which this project cites
#     heavily, because it is where the primary sources live — intermittently
#     takes more than half a minute to answer, and it failed this gate exactly
#     that way on a documentation-only commit.
#
#     Failing the build on that would make the gate flaky, and a flaky gate is
#     worse than a strict one: people learn to re-run it, and then they re-run
#     it past a real failure too. Timeouts are retried with a rising limit and
#     then reported loudly as warnings.
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

    # Three attempts with a rising time limit. Some archives answer a cold
    # request slowly and a warm one immediately.
    code=""
    rc=0
    for timeout in 25 45 75; do
      rc=0
      code="$(curl -sS -o /dev/null -L \
        --max-time "$timeout" \
        -A 'galata-ci-link-check (+https://github.com/celikgo/galata)' \
        -w '%{http_code}' "$url" 2>/dev/null)" || rc=$?
      # A DNS failure will not improve with a longer timeout.
      if [ "$rc" -eq 0 ] || [ "$rc" -eq 6 ]; then
        break
      fi
      sleep 3
    done

    if [ "$rc" -eq 6 ]; then
      printf '::error::NXDOMAIN %s\n' "$url"
      fail=1
    elif [ "$rc" -eq 28 ]; then
      printf '  WARN timeout after three attempts (75s) %s\n' "$url"
      printf '       the host is slow, which is not evidence the URL is wrong\n'
      warned=$((warned + 1))
    elif [ "$rc" -ne 0 ]; then
      printf '  WARN unreachable (curl exit %s) after three attempts %s\n' "$rc" "$url"
      warned=$((warned + 1))
    elif [ "$code" = "404" ] || [ "$code" = "410" ]; then
      printf '::error::HTTP %s %s\n' "$code" "$url"
      fail=1
    elif [ "$code" -ge 400 ] 2>/dev/null; then
      printf '  warn HTTP %s %s (host alive, refusing CI)\n' "$code" "$url"
      warned=$((warned + 1))
    else
      printf '  ok   HTTP %s %s\n' "$code" "$url"
    fi
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
if [ "$warned" -gt 0 ]; then
  printf 'Warnings are hosts that were slow or that refused a CI robot. They are not\n'
  printf 'failures: this gate catches URLs that are wrong, not hosts that are having a\n'
  printf 'bad day. A URL that warns on every run for weeks is worth investigating by\n'
  printf 'hand.\n'
fi

if [ "$fail" -ne 0 ]; then
  printf 'Documentation link check FAILED.\n'
  exit 1
fi
printf 'Documentation link check OK.\n'
