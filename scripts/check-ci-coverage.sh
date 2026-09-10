#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# ADR-0019: every pull request runs the required graph, whatever its base.
#
# WHAT THIS CATCHES AND WHY IT IS WORTH A GATE. A `branches:` filter under
# `pull_request:` in .github/workflows/ci.yml means a pull request based on
# anything else never fires the workflow — and GitHub reports that as ZERO
# CHECKS rather than as a failure. On the pull-request page that reads as
# "nothing to see here" instead of "this has never been tested". It is the
# quietest possible regression: nothing goes red, the tick column is simply
# empty, and a reviewer approving such a pull request is approving code against
# no evidence at all.
#
# It happened. On 2026-09-10 seven of fifteen open pull requests carried no
# checks for exactly this reason, and those seven were the whole
# measured-data and identification vertical plus sampled control. The filter is
# gone; this stops it coming back by accident, because the failure mode it
# produces is invisible to every other gate in this repository.
#
# WHAT THIS IS NOT: not a check that the jobs pass, and not a check that GitHub
# actually ran them. It is a check on one line of one file. Branch protection
# with the eleven contexts as required is the mechanical enforcement, and
# ADR-0019 records it as a follow-up rather than claiming it exists.
set -eu

cd "$(dirname "$0")/.."

workflow=.github/workflows/ci.yml
if [ ! -f "$workflow" ]; then
  echo "CI coverage: $workflow is missing" >&2
  exit 1
fi

# The `pull_request:` key and whatever is indented under it, up to the next
# top-level trigger key. `awk` rather than a YAML parser: this gate runs in the
# first seconds of the job, before anything is installed.
block=$(awk '
  /^  pull_request:[[:space:]]*$/ { inside = 1; next }
  inside && /^  [a-z_]+:/ { inside = 0 }
  inside { print }
' "$workflow")

if printf '%s\n' "$block" | grep -qE '^[[:space:]]*branches(-ignore)?:'; then
  echo "CI coverage: .github/workflows/ci.yml restricts the pull_request trigger by base branch." >&2
  echo "" >&2
  echo "A pull request whose base is not listed will fire no workflow, and GitHub reports" >&2
  echo "that as ZERO CHECKS rather than as a failure — which reads as 'nothing to see'" >&2
  echo "instead of 'never tested'. ADR-0019 removed this filter after seven stacked pull" >&2
  echo "requests were found carrying no checks at all. Remove the filter, or amend" >&2
  echo "docs/adr/0019-every-pull-request-runs-the-required-graph.md and say why." >&2
  exit 1
fi

# And the trigger must actually be there: deleting it would pass the check above
# for the wrong reason.
if ! grep -qE '^  pull_request:[[:space:]]*$' "$workflow"; then
  echo "CI coverage: .github/workflows/ci.yml has no unfiltered 'pull_request:' trigger." >&2
  echo "Without it no pull request runs this workflow at all, which is the same" >&2
  echo "invisible failure ADR-0019 exists to prevent." >&2
  exit 1
fi

echo "CI coverage check OK: every pull request fires the required graph, whatever its base."
