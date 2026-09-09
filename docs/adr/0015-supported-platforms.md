# ADR-0015: Supported platforms are Linux and macOS

- **Status:** accepted
- **Date:** 2026-09-08
- **Deciders:** project maintainer

## Context

Until this record the engine was built and tested on four legs — Linux with
GCC, Linux with Clang, macOS with AppleClang, and Windows with MSVC — and every
release shipped a `windows-x86_64` archive beside the others.

Windows was the only failing check on the M2 branch, and it failed in the place
that matters most for a platform anyone is expected to install: the
installed-package consumer smoke test in `scripts/check-install.py`. A
relocated out-of-tree consumer linking the installed `galata::modeling` could
not parse a YAML document that galata had just written correctly. It first
appeared as an access violation, 0xC0000005, which identifies nothing. Naming
each invariant in the smoke test turned it into a yaml-cpp report of an unknown
escape character on a pure-ASCII document with LF line breaks, whose third line
is `blocks:` and which contains no backslash anywhere. A byte dump then
established that Windows writes the same document macOS does, and that the same
round trip passes throughout the Windows test suite against the build tree —
only the relocated consumer fails. That narrows the defect to what the
installed consumer resolves against rather than to galata's serialisation, and
there it stopped. It is filed as
[issue 12](https://github.com/celikgo/galata/issues/12) and was never root
caused.

The price of the fourth platform is visible in the branch this record was
written on, merged as [pull request
11](https://github.com/celikgo/galata/pull/11). Seven of its thirteen commits —
[5dcf158](https://github.com/celikgo/galata/commit/5dcf158f55cf29098e753407645c15b841feb693),
[ef638a8](https://github.com/celikgo/galata/commit/ef638a8b41b9fa665806a0441ffa0c7d4a2dc727),
[5b023af](https://github.com/celikgo/galata/commit/5b023afafff9d74dd6802cc5d93f552109a92cde),
[b8ba5a1](https://github.com/celikgo/galata/commit/b8ba5a1297027e44e96c436175100ad16afb147e),
[750add8](https://github.com/celikgo/galata/commit/750add82e4f79c42a291f0a91c04615eea9048f6),
[9d03aaa](https://github.com/celikgo/galata/commit/9d03aaa09484b2c5f7d9b18aefa675fb1a26ecf5)
and
[6066f2f](https://github.com/celikgo/galata/commit/6066f2f2fefa82cd16af6ff45f3fb1c47ffb33af)
— are hosted-matrix portability and diagnosis work rather than the increment
they were meant to deliver, and the last two of those are a debug probe for the
Windows failure and its revert. Each attempt is a round trip through a runner
nobody here can attach a debugger to. That branch was squash-merged as
`f19c562` and then deleted, so none of those seven commits is reachable from
main; they survive on the pull request, and `git fetch origin refs/pull/11/head`
brings them into a clone.

Meanwhile [the product plan](../PRODUCT_PLAN.md) already records that delivery
prioritises macOS, then Linux, per user direction. Nobody uses galata on
Windows. The platform with the weakest evidence was being maintained for a user
who does not exist.

Withdrawing it also undoes work this repository has already paid for.
[Pull request 2](https://github.com/celikgo/galata/pull/2), "Ship the Windows
DLLs with the packaged binary", exists for no purpose other than making the
Windows archive start on a machine that did not build it. This decision
withdraws what that pull request delivered, and a record that does not say so
is worth less than one that does.

## Decision

Withdraw Windows as a supported platform. Supported platforms are Linux (GCC
and Clang) and macOS (AppleClang).

The Windows leg leaves the engine matrix in
[the CI workflow](../../.github/workflows/ci.yml) and the fingerprint matrix in
[the determinism workflow](../../.github/workflows/determinism.yml). The
`windows-x86_64` packaging leg leaves
[the release workflow](../../.github/workflows/release.yml). The `ci-windows`
configure, build and test presets leave `CMakePresets.json`. The MSVC
developer-environment step leaves the setup-cpp composite action, which removes
this repository's only use of the third-party action `ilammy/msvc-dev-cmd@v1`.
`vcpkg.json` now declares support for Linux and macOS on x64 and arm64.
`scripts/package-release.py` loses the Windows platform entry, the `.exe`
suffix, the dependency-DLL staging block and the zip-versus-tarball choice, so
every supported platform ships one archive format;
`scripts/check-release-evidence.py` expects the two remaining platforms; and
`scripts/check-install.py` and `scripts/check-release-archive.py` lose their
`.exe` handling. The bug-report template no longer offers Windows in its
Platform dropdown.

The MSVC branches in `cmake/GalataDeterminism.cmake` and
`cmake/GalataWarnings.cmake` go as well. They are the easiest thing here to
argue for keeping, because leaving dead compiler branches in a CMake file costs
nothing to run. But [ADR-0004](0004-determinism-policy.md) claims determinism
is a tested property, and a floating-point mode selected for a compiler no job
exercises is a determinism claim with no test behind it. Deleting it is the
honest form of not testing it.

Three things deliberately do not change. The asan preset keeps its condition
excluding Windows hosts: that is a refusal to configure there, not a promise
about it. `scripts/ci_evidence.py` is untouched, because its required-job set
names workflow job ids — format, governance, engine, sanitizer, static-analysis
and determinism — rather than platform legs, and none of those jobs disappears.
And Windows-shaped input hygiene stays: [the study-file
guide](../STUDY_FILES.md) refuses Windows device names in output paths, and
refusing `CON` or `NUL` is portable defensive validation that says nothing
about which platforms are supported.

The archives already published under v0.1.0 and v0.2.0 stay where they are.
Each of those releases publishes one `SHA256SUMS.txt` covering every asset it
shipped, so deleting the Windows zip would invalidate the checksum file every
other platform in that release depends on.
`scripts/check-release-archive.py` therefore still reads zip archives on
purpose, so those two remain verifiable. They are historical and unmaintained,
and no future release ships one.

## Alternatives considered

**Root-cause the installed-consumer failure and fix it.** This is the option a
stranger would pick, and it has the better argument on the merits: a fault that
appears only in a relocated installed package is the shape of a real defect —
a mismatched runtime, two yaml-cpp copies resolved at load time, an ODR
violation — and such a defect can be latent on every platform while visible on
one. That branch has already been the beneficiary of exactly that effect twice,
in [5b023af](https://github.com/celikgo/galata/commit/5b023afafff9d74dd6802cc5d93f552109a92cde)
and [b8ba5a1](https://github.com/celikgo/galata/commit/b8ba5a1297027e44e96c436175100ad16afb147e),
where a compiler galata does not develop on found a dangling reference and an
uninitialised read that the local toolchain did not.
Against that: the failure reproduces only on a hosted runner, and two commits
were spent merely obtaining a legible symptom, which established what the
defect is *not* and left the question of which yaml-cpp the installed consumer
binds against unanswered. The remaining work is bounded by nothing, is paid in
push-and-wait iterations, and its success condition is a green check for a
platform with no user. Nor is it a one-off payment: succeeding restores the
state in which every later change is taxed by the same matrix. The messenger
this gives up is real, and named below as a cost.

**Keep the Windows leg but make it advisory.** The cheapest change on this
list, and it keeps evidence accruing without blocking a merge. But Windows is a
leg inside the `engine` job, not a job of its own, and the only way to stop one
leg failing the job is `continue-on-error`. That makes `engine` report success
while Windows is broken — and `scripts/ci_evidence.py` reads job results, not
matrix legs, so [ADR-0009](0009-release-evidence-and-source-identity.md)'s
release gate would then accept a complete, all-successful required job set that
conceals a failing platform. Splitting Windows into its own advisory job avoids
that, at the cost of a permanently red check nobody may block on. Neither is
evidence; both are a support claim standing next to its own counterexample.

**Keep Windows building but stop shipping a Windows archive.** This preserves
what is genuinely valuable about the platform — MSVC is a third front end, with
a different standard library and a different warning set — while retiring the
packaging surface. It is also the narrowest change that could make CI green.
The trouble is where the failure actually lives: `scripts/check-install.py`
tests the *installed* package, so this option either keeps that check, in which
case the job still fails, or deletes it, in which case the surviving Windows
evidence is that the tree compiles and its in-tree suite passes. A
configuration that is never installed and never shipped is a configuration
nobody consumes, and every MSVC portability commit would still be paid in full.
That is the whole cost of a platform for part of the assurance.

## Consequences

Three engine legs instead of four, and one determinism fingerprint fewer. The
release path has one archive format and one dependency-staging story rather
than two of each. `cmake/GalataDeterminism.cmake` and
`cmake/GalataWarnings.cmake` describe only compilers that are exercised. A
portability defect that appears only under MSVC no longer blocks this
repository — and no longer reaches it either.

That last point is the real cost, and it comes in two parts. Losing MSVC loses
a front end whose diagnostics differ from GCC's and Clang's, of the same kind
that caught defects on this branch; what survives is two Linux compilers, the
macOS compiler, the sanitizer job and static analysis. And determinism tier 2
drops from three platforms to two. Its published relative bound and its
every-pair comparison are unchanged — with two platforms, every pair is the one
pair, Linux against macOS — but the evidence behind the cross-platform claim is
narrower than it was. A libm disagreement that glibc and Apple's libm happen to
share is no longer contradicted by a third implementation. Tier 1, the
byte-identical same-platform guarantee, is now gated on Linux and macOS and is
otherwise unaffected.

Undoing this decision means restoring, specifically: the engine matrix leg in
[the CI workflow](../../.github/workflows/ci.yml); the fingerprint matrix leg
and the tier-2 fingerprint count in
[the determinism workflow](../../.github/workflows/determinism.yml); the
packaging leg in [the release workflow](../../.github/workflows/release.yml);
the `ci-windows` configure, build and test presets in `CMakePresets.json`; the
MSVC
developer-environment step in the setup-cpp composite action, together with a
fresh review of the third-party action it depends on; the MSVC branches in
`cmake/GalataDeterminism.cmake` and `cmake/GalataWarnings.cmake`; the `.exe`
suffix, dependency-DLL staging and zip selection in
`scripts/package-release.py`, and the `.exe` handling in
`scripts/check-install.py` and `scripts/check-release-archive.py`; the expected
platform set in `scripts/check-release-evidence.py`; the supports expression in
`vcpkg.json`; and the Platform entry in the bug-report template. None of that
is difficult, and all of it is recoverable from the diff of `f19c562`. What is
not recoverable that way is the reason the leg was red: restoring Windows means
starting from issue 12, not from a green build.

## Revisit when

Revisit when a named user requires galata on Windows, or when a Windows build
becomes a precondition of a delivery commitment that has been accepted — the
condition [the product plan](../PRODUCT_PLAN.md) leaves open by making delivery
priorities follow user direction. Reopening this record means reopening issue
12 first: the platform comes back when its installed package is explained, not
when its matrix leg is pasted back in.
