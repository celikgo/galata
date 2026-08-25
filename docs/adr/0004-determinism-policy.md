# ADR-0004: Determinism policy, and what it does not cover

- **Status:** accepted
- **Date:** 2026-08-18
- **Deciders:** project authors

## Context

Charter rule 6 says determinism is a tested property, not an aspiration. A tool
whose output changes between runs cannot support a regression suite, cannot
support a Monte Carlo study whose worst case is meant to be re-examinable, and
cannot support the claim that a number is reproducible from the CLI.

"Deterministic" is also a word that gets over-claimed. It is worth stating
precisely what is guaranteed, what is not, and why — because a badge that
implies more than it delivers is worse than no badge.

## Decision

### The guarantee, in two tiers

**Tier 1 — same binary, same platform: bit-identical.** Running the same
computation twice with the same inputs on the same machine and binary produces
identical bits. This is gated two ways, both on Linux, macOS and Windows.
`tests/determinism/` re-runs each battery in process and compares the resulting
doubles with `EXPECT_EQ` — an exact comparison, not a tolerance.
`scripts/check-determinism.sh` runs `tools/determinism` twice and `diff -u`s the
two files, which is the byte comparison; the tool prints at `%.17g`, which
round-trips a double exactly, so byte-identical output means bit-identical values
rather than values that merely print the same.

There is no test that runs a *pipeline* twice and diffs its output files. That
would be the stronger check, because it would cover the report writers as well as
the numerics, and nothing currently gates those.

**Tier 2 — same source, different platform: agreement to a published bound.**
Output produced on Linux, macOS and Windows agrees to a documented tolerance —
1e-9 relative, between every pair of platforms — **with one carve-out, which
this record failed to state until now.**

Values downstream of the central-difference Jacobian are excluded from the
cross-platform comparison altogether: `scripts/compare-determinism.sh` drops
every fingerprint key prefixed `tier1.` before it compares anything. Dividing by
the perturbation h amplifies a libm disagreement by 1/h, which on a small matrix
entry reaches about 1e-8 relative through nobody's error — past this gate. Those
values are held byte-identical *within* a platform by tier 1 instead, which is
the stronger claim; the rest is what the 1e-9 bound covers. The split is 47
excluded of 145 fingerprinted values as this is written, and
`docs/VERIFICATION.md` reports both counts from the run rather than from here.

The *observed* deviation is printed by every comparison rather than merely
bounded, and it is read from the workflow log. It is deliberately **not**
restated in `docs/VERIFICATION.md`: `determinism.yml` holds `contents: read` and
writes nothing back to the repository, so a figure copied in by hand would be a
figure that drifts. That is the report's own reasoning and this record follows
it; an earlier version of this record claimed the opposite.

Tier 2 is not bit-identity, and the reason is worth being blunt about.

### Why Tier 2 is not bit-identity

`sqrt` is required by IEEE 754 to be correctly rounded, so it produces identical
bits everywhere. Nothing else transcendental does. `sin`, `cos`, `tan`, `asin`,
`atan2`, `exp`, `log` and `pow` come from the platform's math library — glibc on
Linux, Apple's libm on macOS, the UCRT on Windows — and those implementations
are not correctly rounded, do not agree with each other in the final bits, and
change between versions of the same library.

galata cannot avoid these functions. Angle of attack is an `atan2`. The
atmosphere's pressure profile is a `pow`. Every rotation is a `sin` and a `cos`.
Any claim of cross-platform bit-identity would therefore be either false or
would require shipping a correctly-rounded libm, which is a serious project in
its own right and is not one this tool is undertaking in 1.x.

So the honest claim is the two-tier one, and the badge says "determinism"
linked to the document that defines it.

### What produces Tier 1

- **`-ffp-contract=off` on GCC and Clang, `/fp:precise` on MSVC.** Contraction
  fuses `a*b+c` into an FMA, skipping the rounding of the intermediate product.
  Whether it happens depends on the target ISA, so a contracted expression is
  both more accurate than the source says and differently accurate on different
  machines.
- **`-fno-fast-math`, passed explicitly.** Not merely omitted — passed, so that
  a toolchain file or a dependency's usage requirement cannot enable it behind
  our back. Under `-ffast-math` the compiler may reassociate floating-point
  expressions, and reassociation is not value-preserving.
- **Fixed-step integrators for every gated result.** RK4 with a fixed step is
  the only integrator in the tree, and therefore the only one used for
  determinism-gated output. If an adaptive method is ever added — Dormand-Prince
  5(4) or another — its results may not be gated bit-identical, because its step
  sequence is a function of an error estimate and therefore of the last bits of
  the state. That is a requirement on the commit that adds one, not a
  description of today.
- **Fixed iteration counts in root-finders.** Newton runs a fixed number of
  iterations and *then* checks the residual, rather than exiting when a
  tolerance is met. A tolerance-based exit makes the iteration count a function
  of floating-point noise, and with it the output. Failure to converge is
  reported as failure, never as a silent best effort.
- **Seeded PRNGs, with the seed in the output — a requirement on the commit that
  first needs one.** Nothing under `src/` draws a random number today: there is
  no turbulence model and no Monte Carlo, and the only generator in the tree
  seeds the property-test inputs. When one arrives it draws from an explicitly
  seeded generator whose algorithm is fixed by galata rather than inherited from
  the standard library's implementation-defined engines. `std::mt19937_64` is
  specified bit-exactly by the standard and is therefore acceptable;
  `std::random_device`, `std::default_random_engine` and the distribution classes
  are not, since their outputs are implementation-defined, so distributions are
  implemented in-tree.
- **Ordered containers in any code path whose iteration order reaches output.**
  No `unordered_map` iteration, no pointer-value sorting, no
  address-of-allocation ordering.
- **No `long double` anywhere in the numerical core.** It is 80-bit extended on
  x86-64 System V, 64-bit on MSVC and 128-bit quad on AArch64 Linux. A result
  that touches it is non-portable by construction.
- **Locale-independent formatting, delivered by never changing the locale — and
  now gated.** Nothing in galata calls `setlocale` or `imbue`, so the process
  stays in the `"C"` locale and a decimal comma cannot reach a result file.
  Output goes through `std::printf` in `tools/determinism` and through iostreams
  with `std::setprecision` in the pipeline's report writers, and both take their
  decimal point from that locale.

  This rests on a convention rather than on a library, which is why it needed a
  gate more than any other item here: two runs on a German machine would be
  equally wrong and equally identical, so tier 1 alone would pass.
  `scripts/check-determinism.sh` therefore runs the fingerprint a **third** time
  with a comma-decimal locale in the environment and requires the bytes not to
  move. It passes because galata ignores the environment, which is exactly the
  claim; a `setlocale(LC_ALL, "")` introduced anywhere in the library turns every
  decimal point into a comma and fails it.

  An earlier version of this record credited `fmt` with delivering this. It never
  did — it was declared in `vcpkg.json` and linked by nothing — and the
  dependency has since been dropped rather than wired, because putting an
  unpinned library's undocumented float formatting underneath every committed
  artefact is the opposite of what pinning Eigen is for.

### What breaks the guarantee, stated so nobody is surprised

- Compiling galata into a build that enables `-ffast-math`, `-Ofast`, or
  `/fp:fast`. Nothing can prevent a downstream consumer from doing this. It is
  documented instead.
- Enabling FMA contraction, by any spelling.
- Linking a different libm, or the same libm at a different version — this moves
  results within the Tier 2 bound but breaks Tier 1 across the change.
- Parallel reduction over floating-point values in an unspecified order. Any
  parallelism introduced later must use a fixed reduction tree, and that
  requirement is on the reviewer of the commit that introduces it.
- **A consumer that changes the process locale in its own program.** galata's
  formatting is locale-*independent* only because galata leaves the locale
  alone; it is not locale-*insensitive*. An application that calls
  `setlocale(LC_ALL, "")` in its own `main` and then links galata as a library
  gets commas in galata's output on a machine configured for them. The gate above
  covers galata's own binaries and cannot cover somebody else's. Making the
  formatting insensitive rather than merely undisturbed would mean routing every
  numeric conversion through a locale-independent formatter, which is the `fmt`
  question above, and it was answered no.

## Alternatives considered

**Claim bit-identity across platforms and enforce it.** This is what the reader
wants to be true. It would require a correctly-rounded implementation of every
transcendental galata uses — the `crlibm` / RLIBM line of work shows it is
possible — and would make the tool substantially slower. Rejected for 1.x on
cost, and recorded here so the option is not lost. If it is ever done, Tier 2
collapses into Tier 1 and this ADR is superseded rather than amended.

**Gate cross-platform agreement on a hash of rounded output** — round every
number to, say, 9 significant figures and hash that. Attractive because it looks
like bit-identity and is easy to implement. Rejected: it converts a smooth
tolerance into a cliff, so a result sitting near a rounding boundary flaps
between pass and fail with no physical change. A tolerance with the observed
deviation published is honest; a hash of rounded values only looks stricter.

**Use `/fp:strict` on MSVC.** It additionally preserves exception semantics and
rounding-mode changes. galata never manipulates the floating-point environment,
so it buys nothing over `/fp:precise` and costs measurable performance.

## Consequences

- `cmake/GalataDeterminism.cmake` applies the flags globally rather than through
  an interface target that a new library could forget to link.
- Every result file records the seed, the integrator, the step size and the
  build identification, so a number can be re-derived. This is a requirement on
  every capability that emits a result, enforced at the point each one is
  written; `build_identification()` is the part of it that exists today.
- The determinism test suite is a first-class tier. `tests/determinism/` holds
  the tier 1 checks, and `.github/workflows/determinism.yml` gates tier 1 on
  Linux, macOS and Windows and measures tier 2 between every pair of them.
  `tools/determinism/` emits the fingerprint both tiers compare.
- Introducing threading into the numerical core requires revisiting the fixed
  reduction order, and any such commit cites this ADR.
- The README's determinism badge links here rather than to the workflow run, so
  that a reader who wants to know what is actually promised gets the two-tier
  answer rather than a green rectangle. The badge is added once the workflow has
  a run behind it — until then its URL does not resolve, and the documentation
  link gate correctly rejects it.

## Revisit when

A correctly-rounded math library becomes a practical dependency, or the project
introduces parallelism into any gated numerical path.
