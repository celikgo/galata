---
name: preserving-determinism
description: What ADR-0004's two-tier determinism guarantee actually requires, what the determinism workflow compares and how, and the C++ and CMake constructs that break it silently — unordered container iteration, tolerance-based loop exits, -ffast-math and FMA contraction, long double, uninitialised reads, unordered parallel reductions, and locale-dependent formatting. Use when touching src/numerics, src/analyze, src/trim, src/linearize, tools/determinism, cmake/, or any code path whose values reach an output file — and before introducing threading anywhere.
---

# Preserving determinism

A tool whose output changes between runs cannot support a regression suite, cannot support a
Monte Carlo study whose worst case is meant to be re-examinable, and cannot support the claim
that a number is reproducible from the CLI. galata claims it, so galata tests it.

"Deterministic" is also a word that gets over-claimed, and ADR-0004 is deliberately blunt about
what is and is not promised.

## The guarantee, in two tiers

**Tier 1 — same binary, same platform: bit-identical.** Run the same pipeline twice on the same
machine and get byte-identical output. Gated on Linux, macOS and Windows.

**Tier 2 — same source, different platform: agreement to a published bound.** Output from
Linux, macOS and Windows agrees to **1e-9 relative**, and every pair is compared.

Tier 2 is not bit-identity, and the reason is worth stating rather than hiding. `sqrt` is
required by IEEE 754 to be correctly rounded, so it produces identical bits everywhere.
**Nothing else transcendental does.** `sin`, `cos`, `tan`, `asin`, `atan2`, `exp`, `log` and
`pow` come from the platform's math library — glibc, Apple's libm, the UCRT — and those are not
correctly rounded, do not agree in the final bits, and change between versions of the same
library. galata cannot avoid them: angle of attack is an `atan2`, the atmosphere's pressure
profile is a `pow`, every rotation is a `sin` and a `cos`.

Any claim of cross-platform bit-identity would therefore be either false or would require
shipping a correctly-rounded libm. ADR-0004 records that as a rejected alternative rather than
pretending the question does not arise.

## What holds tier 1 up

### The build flags, applied globally on purpose

`cmake/GalataDeterminism.cmake` uses `add_compile_options` at top-level directory scope — not an
interface target a numerical target links, because **there is no opt-out**. A new library cannot
forget to link it.

- **GCC / Clang / AppleClang:** exactly two flags, `-ffp-contract=off` and `-fno-fast-math`.
- **MSVC:** exactly one, `/fp:precise`.

`-fno-fast-math` is **passed explicitly rather than merely omitted**, so that a toolchain file or
a dependency's usage requirement cannot turn it on behind the build's back. `/fp:strict` was
considered and rejected: galata never manipulates the floating-point environment, so it buys
nothing over `/fp:precise` and costs measurable performance.

Contraction matters because fusing `a*b+c` into an FMA skips the rounding of the intermediate
product. Whether that happens depends on the target ISA, so a contracted expression is both more
accurate than the source says *and differently accurate on different machines*.

Eigen is pinned in the vcpkg manifest for the same reason: **Eigen's version is part of what
makes a numerical result reproducible.**

### Fixed iteration counts, never a tolerance exit

Newton runs a fixed number of iterations and *then* checks the residual, rather than exiting when
a tolerance is met. A tolerance-based exit makes the iteration count a function of floating-point
noise, and with it the output. Failure to converge is reported as failure, never as a silent best
effort — `trim_level` **throws**.

The same pattern appears in the frequency-domain code, with the reasoning written on the loop:

```cpp
// Bisect a sign-changing function on [low, high] for a FIXED number of steps.
// The count is fixed rather than tolerance-driven so that the answer does not
// depend on the order in which rounding happens (ADR-0004).
```

`MarginOptions` fixes `bisection_iterations = 80` (eighty bisections shrink a bracket by 2^80,
which reaches the floating-point floor from any starting width) and
`peak_refinement_iterations = 100`.

Three loops in the tree do carry an early `break`, and they are worth understanding rather than
copying blindly: Newton stops advancing on a non-finite step, `bisect` stops when the bracket is
down to adjacent representable numbers, and the golden-section refinement stops when its probes
cross. Each is a property of the number system reached at the floating-point floor, not a
convergence criterion — and each says so in a comment. **If you add one, it must be of that
kind, and the comment must say why it cannot depend on anything else.**

### Ordered containers wherever iteration order reaches output

No `unordered_map` iteration, no sorting by pointer value, no ordering that depends on where the
allocator happened to put something.

### No `long double` in the numerical core

It is 80-bit extended on x86-64 System V, 64-bit on MSVC and 128-bit quad on AArch64 Linux, so a
result that touches it is non-portable by construction.

Be clear-eyed about the enforcement: `Determinism.NoLongDoubleInTheNumericalCore` asserts
`sizeof(double) == 8` and `is_iec559` — *"the property the ban exists to protect rather than the
ban itself — a grep would be checking the letter of the rule and this checks that doubles behave
as doubles"*. The ban itself is on you and on the reviewer.

### Locale-independent formatting

Nothing in galata calls `setlocale` or `imbue`, so the program stays in the `"C"` locale and a
decimal comma in a user's environment cannot change a result file. A single `setlocale(LC_ALL, "")`
would put one there, and no test would catch it. Do not add one.

## What the workflow actually compares

`tools/determinism/` emits a **fingerprint**: one tab-separated `key\tvalue` line per value at
`%.17g`, with exactly one leading `#` comment naming the version. The build identification is
**deliberately not in it** — it names the compiler, and this file is compared across compilers.

The battery has four sections in fixed call order: atmosphere, rigid body, lateral modes, then
trim and linearisation. The rigid-body case integrates 15,000 fixed steps of 0.002 s with
attitude renormalisation as the projection.

**Tier 1** (`scripts/check-determinism.sh`) runs the same binary twice and `diff -u`s the two
outputs. When they differ it names the usual causes: *unordered container iteration reaching the
output, an unseeded PRNG, a tolerance-based loop exit, address-dependent ordering, or
uninitialised memory.* It also refuses a fingerprint of fewer than 50 values as "too thin to gate
anything".

**Tier 2** (`scripts/compare-determinism.sh`) is a **relative-deviation** comparison, not a byte
comparison: `|x−y| / max(|x|,|y|)`, gated at 1e-9. It fails on any key-set mismatch *before*
comparing a single value, and reports the total compared, how many were bit-identical, and the
worst deviation with the key it occurred at.

`.github/workflows/determinism.yml` builds a fingerprint on ubuntu-24.04/GCC, macos-14/AppleClang
and windows-2022/MSVC, then compares **every pair** rather than each against a nominated
reference. It also runs **nightly at 04:17 UTC**, so a libm update on a runner image shows up as
a change in the observed cross-platform deviation rather than as a surprise in somebody's PR.

### What is excluded from tier 2, and why

Keys prefixed `tier1.` are compared within a platform but not across. Today that is the
linearisation and the modes computed from it, because **a central difference divides by h**:

> With f of order 10 and h of 1e-6 that turns a 2e-15 disagreement into 2e-9 absolute, which on a
> small matrix entry is 2e-8 relative — past the 1e-9 cross-platform gate.

Trim outputs are **not** excluded, and the distinction is the useful one: *"Trim outputs are
roots. Solving to a residual of zero lands on the same point regardless of which libm got you
there."*

The split is counted by re-running the same battery with a counting emitter, so the report and
the comparison script cannot disagree about how many values were skipped. Both key on the literal
prefix `"tier1."`, in two places — if you ever rename it, rename both.

## The constructs that break this quietly

| Construct | What goes wrong |
|---|---|
| `for (auto& [k, v] : some_unordered_map)` where anything downstream reaches output | Iteration order is a function of hashing and insertion history. Use an ordered container. |
| `while (residual > tol)` | Trip count becomes a function of the last bits. Fix the count and check the residual after. |
| `-ffast-math`, `-Ofast`, `/fp:fast` in a *downstream* build | Reassociation is not value-preserving. Nothing can prevent a consumer doing this; ADR-0004 documents it instead. |
| FMA contraction, by any spelling | Skips an intermediate rounding, and whether it happens depends on the ISA. |
| `long double` | Three different widths across the supported platforms. |
| An uninitialised read | Reproducible under one allocator and not another; often invisible until a platform changes. |
| A parallel reduction with unspecified order | Floating-point addition is not associative. Any parallelism introduced later **must** use a fixed reduction tree, and that requirement is on the reviewer of the commit that introduces it. |
| `setlocale` / `imbue` | A decimal comma in a result file. Nothing tests for this. |
| An adaptive integrator in a gated path | The step sequence is a function of an error estimate, and therefore of the last bits of the state. Fixed-step RK4 is the only integrator used for determinism-gated output. |
| Sorting without a total order | Two equal keys can swap. Every comparator in the tree that could tie breaks the tie explicitly — see the modal classifier's `score, then mode_index, then signature_index`. |
| A new `find_package`d dependency, unpinned | A library version is part of the result. Eigen is pinned in `vcpkg.json` for exactly this reason. |

## Before you commit

```bash
cmake --build --preset dev
ctest --preset dev -L determinism
scripts/check-determinism.sh build/dev/tools/determinism/galata-determinism
```

If you added a numerical routine whose output reaches a file, add it to the fingerprint battery
in `tools/determinism/fingerprint.cpp`. A value that is not fingerprinted is not gated, and the
50-value floor in `check-determinism.sh` is a floor, not a target.

If the new value is downstream of a central difference — or of anything else that amplifies a
libm disagreement — give it the `tier1.` prefix and say in a comment why it cannot hold at 1e-9
across platforms. **Excluding a value from tier 2 is a decision that gets written down, not one
that gets made silently.**

## Never

- Loosen the tier-2 tolerance to make a comparison pass. Move the value to `tier1.` with a stated
  reason, or find out why it moved.
- Introduce threading into the numerical core without a fixed reduction order — and cite ADR-0004
  in that commit.
- Add a determinism-relevant compiler flag anywhere other than `cmake/GalataDeterminism.cmake`.
- Claim cross-platform bit-identity. It is not true, and ADR-0004 explains at length why saying
  so would be worse than the honest two-tier claim.
