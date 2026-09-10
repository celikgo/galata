<!-- SPDX-License-Identifier: Apache-2.0 -->

# ADR-0019: Every pull request runs the required graph, and a stack is gated at its integration head

- **Status:** accepted
- **Date:** 2026-09-10
- **Deciders:** project maintainer, on the RFC-0002 combined-stack acceptance

## Context

`.github/workflows/ci.yml` triggered on `pull_request: branches: [main, master]`. A pull
request whose base is another **feature branch** therefore never fired it, and GitHub reports
that state as **zero checks** rather than as a failure — which on the pull-request page reads
as "nothing to see here" rather than "this has never been tested".

This was measured rather than reasoned about. On 2026-09-10 the quadrotor programme had
fifteen open pull requests. The eight based on `main` each carried 11 of 11 green. The seven
stacked ones — `#23`, `#25`, `#26`, `#29`, `#30`, `#31`, `#32` — carried none. Those seven are
`sim.sampled`, the public chart map, the ULog parser, and all three `identify.*` capabilities:
the whole measured-data and identification vertical, plus sampled control. No hosted job had
ever compiled any of it with GCC, with Clang on Linux, under the sanitisers, or through the
determinism comparison. Every one of those pull requests was reviewable, and one was
described as delivered.

Two further facts shape the decision. `main` carries **no branch protection**, so "required
check" is a convention this repository keeps rather than a rule GitHub enforces — an empty
rollup blocks nothing mechanically. And CI is not cheap: issue #16 records the sanitiser job
alone as 72 minutes of `ctest`, and a seven-deep stack contains its ancestors' code seven
times over, so running the full graph on every link re-tests the same translation units
repeatedly.

## Decision

**`ci.yml` fires on every `pull_request` event, with no base-branch filter. Validation happens
on each stack branch, at its own head. A stack additionally has one integration head that must
be green before any of it merges, and that head is where the combined behaviour is
established.**

Four parts.

**1. No pull request may be reviewed with an empty rollup.** The filter is removed rather than
extended, because a list of admitted base branches is a list somebody has to remember to add
to, and the failure it produces is silence. A missing check must look like a missing check.

**2. Each stack branch is validated at its own head, and that is not redundant with the
integration head.** A stacked branch's tree is its ancestors plus its own commits, so a green
run on link *n* does cover links 1 to *n−1* as they exist there — but it covers them **in that
combination only**. The integration head is a different tree: it is the merge of every branch,
including conflict resolutions that exist in no constituent branch. On this programme two such
resolutions were needed, and one of them had been committed with conflict markers still in the
file, which compiled nowhere and which no constituent branch's CI could have caught. A green
stack does not imply a green integration.

**3. The integration head is the merge gate.** A stack merges by merging the integration head,
once, after it is green — not by merging each constituent pull request in turn. The
constituents stay open as the review surface for their own concern and close as the
integration merges; merging them individually as well would apply the same commits twice and
would put an untested combination on `main` between the first merge and the last.

**4. The cost is accepted rather than paid for with coverage.** The full graph on every link of
a deep stack is real duplicated compute, and the place to reduce it is the sanitiser job's own
process-spawning overhead (issue #16), not the set of pull requests that get checked. Charter
rule 3 forbids widening a gate to make a number pass; narrowing one to save minutes is the
same move.

## Alternatives considered

**Validate only at a required integration head, leaving stack branches unchecked.** Its honest
best case is that it is the cheapest option that still gates `main` correctly — the integration
head is what merges, so gating it is sufficient for `main`'s health, and a deep stack costs one
graph instead of seven. Rejected because the pull request is the review surface. A reviewer
approving link 4 of a stack is approving code, and doing so against no evidence is the state
this ADR exists to end. It also localises failure badly: a compile error introduced at link 2
surfaces only at the integration head, where it must be bisected back through six branches
instead of being reported on the branch that caused it.

**Keep the filter and add every feature branch prefix to it.** Best case: minimal change,
and CI fires exactly where the maintainer intends. Rejected because it is a list that must be
maintained, and the consequence of forgetting an entry is not an error but a silent absence of
checking — the same defect in a new place. A filter whose omissions are invisible is worse
than no filter.

**Require branch protection with the eleven checks as required contexts.** Best case: it makes
"required" mechanical rather than conventional, and an empty rollup would then block merging
outright. Not rejected — it is the right complement to this decision — but it is a repository
administration change with consequences beyond CI triggers (it also blocks the maintainer's own
direct pushes, and it names contexts that must then be kept in step with the job matrix), so it
is recorded here as a follow-up rather than bundled in.

**A reduced matrix for stacked pull requests** — say Linux GCC only, deferring the sanitisers
and the determinism comparison to the integration head. Best case: most of the coverage for a
fraction of the cost, and the expensive jobs still run before anything merges. Rejected because
it reintroduces exactly the failure mode being fixed, in a quieter form: a stacked pull request
would show a green tick that means less than the same tick on a `main`-based one, and nothing on
the page would say so. A check that means different things in different places is a check a
reader will misread.

## Consequences

**Easier.** A pull request page now answers "has this been tested" the same way everywhere. A
stacked branch's failures are reported on the branch that caused them. The integration head's
own green run becomes a meaningful, separate claim about the combination, rather than the only
claim in existence.

**Harder.** A deep stack costs one full graph per link, and the wall clock is dominated by the
sanitiser job. Rebasing a stack re-runs everything below the rebase point. Contributors will
see more concurrent runs, and the `concurrency` group already keyed on `github.ref` is what
stops each branch cancelling its neighbours.

**Expensive to reverse.** Not very — this is one trigger block. What would be expensive is
undoing it *silently*: reinstating a base filter would return the repository to a state where
some pull requests are checked and some only look as though they are, and the reader cannot
tell which from the page. If it is ever reinstated, the pull-request template must say so.

**What this does not buy.** It is a convention, not an enforcement: `main` is unprotected, so a
merge with an empty or red rollup is still mechanically possible. It also does not make a green
stack a green integration — part 2 above is the reason, and the integration head's run is the
only evidence about the combination.

## Revisit when

Branch protection lands on `main`, at which point the eleven contexts become enforced and the
follow-up above is discharged; or issue #16 changes the cost structure enough that a different
trade is available. Also revisit if a stack ever exceeds roughly ten links, where the
duplicated compute stops being an accepted cost and starts being the reason a contributor
avoids stacking.
