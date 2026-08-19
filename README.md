# galata

Flight dynamics, control-law design and simulation — reproducibly, from a file
you can read, with every number traceable to the routine that produced it.

Today it takes a nonlinear aircraft model, **trims** it, **linearises** about
that trim, and produces a **labelled** modal table — short period, phugoid,
Dutch roll, roll subsidence and spiral, identified by eigenvector participation
rather than by frequency. From non-dimensional derivatives and geometry alone,
it reproduces NASA CR-2144's published dimensional derivatives to 0.26% and its
published modes to 1.0%.

It evaluates frequency response and reports all four margin types — gain,
phase, delay and disk — with every crossover and the frequency at which it
occurs, gated against closed-form transfer functions and a published worked
example. For multivariable loops it computes principal gains and the
sensitivity peaks M_S and M_T, which is what catches a design whose
per-channel margins look comfortable and whose loop is not.

Control synthesis and the nonlinear simulation loop are the point of the
project and are **not built yet**. The status table below is the authority on
what exists; the roadmap is the authority on what is intended.

C++20 core, strict SI units, deterministic by policy, Apache-2.0.

[![CI](https://github.com/celikgo/galata/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/celikgo/galata/actions/workflows/ci.yml)
[![Determinism](https://github.com/celikgo/galata/actions/workflows/determinism.yml/badge.svg?branch=main)](docs/adr/0004-determinism-policy.md)


---

## Status: 0.2.0 — trim, linearise, analyse

v0.1 was the spine: trim, linearise, and a classified modal table, validated
against a published NASA report and driven from a YAML file by a CLI. v0.2 adds
the frequency-domain tier on top of it — frequency response, all four margin
types, the sensitivity peaks M_S and M_T, and principal gains for multivariable
loops — each gated against a closed-form transfer function or a published
worked example.

The [charter](docs/CHARTER.md) requires that CI exist before the first feature
and that no capability be documented before it works, so the gates were built
first and the physics landed against them. What exists today:

| Surface | State |
|---|---|
| Build system (CMake + vcpkg, four platform/compiler combinations) | working |
| CI: format, build, test, version consistency, SI boundary, doc links | working |
| Version single-source-of-truth, with provenance in the build identification | working |
| ADRs 0001–0007: ABI relationship, conventions, units, determinism, versioning, EOM reference point, reference-value rights | written |
| Frames, quaternions, state vector, unit boundary (ADR-0002, ADR-0003) | implemented, property-tested |
| U.S. Standard Atmosphere 1976, −5000 m to 86 km | implemented and **validated** against the published tables |
| Fixed-step RK4, with Richardson step-size study | implemented, order verified against closed-form solutions |
| Nonlinear 6-DOF rigid body, general inertia tensor | implemented and **validated** against closed-form solutions of Euler's equations |
| YAML pipeline runner and the `galata` CLI | working |
| Runnable examples, executed by the test suite | working (five, four of them executed end to end) |
| Determinism: bit-identical repeat runs, cross-platform bound measured | working |
| Nonlinear aircraft model from a derivative buildup | implemented and **validated** |
| `trim.level` — Newton on a square residual, fixed iteration count | implemented and **validated** |
| `linearize.finitediff` — central differences with Richardson error estimates | implemented and **validated** |
| Frequency response, gain/phase/delay/disk margins, M_S and M_T, principal gains | implemented and **validated** |
| Everything else in §"What it will do" below | **not built** |

The table above is maintained by hand and checked in review. The capability
table below is not: it is generated from the registry the CLI dispatches
through, by `scripts/gen-status-table.sh`, and CI fails if the committed copy
disagrees. Run `galata capabilities` to get the same list from your own build.

### Capabilities in this build

<!-- BEGIN GENERATED CAPABILITY TABLE -->
| Capability | What it does | Produces | State |
|---|---|---|---|
| `analyze.diskmargin` | Disk margin of one loop — robustness to simultaneous gain and phase variation — with the guaranteed gain and phase range and a destabilising perturbation on the boundary | `disk_margin` | implemented and validated |
| `analyze.freqresp` | Frequency response of one loop of a linear model, evaluated by Hessenberg solves with the grid refined around the system's own lightly damped modes | `frequency_response` | implemented and validated |
| `analyze.margins` | Gain, phase and delay margins of one loop, with every crossover reported and the frequency at which each occurs | `stability_margins` | implemented and validated |
| `analyze.modes` | Eigenvalues, modal metrics and participation factors, with the classical aircraft modes classified by participation | `modal_table` | implemented and validated |
| `analyze.sensitivity` | Sensitivity and complementary sensitivity peaks M_S and M_T of a loop closed with negative unit feedback, and the frequencies at which they occur | `sensitivity_peaks` | implemented and validated |
| `analyze.sigma` | Singular values of a MIMO transfer matrix over frequency — the principal gains, their spread, and the peak gain | `singular_values` | implemented and validated |
| `linearize.finitediff` | Linearise about a trim point by central differences, with a Richardson truncation-error estimate per entry | `linear_system` | implemented and validated |
| `model.aircraft.derivatives` | Load a nonlinear aircraft model built from a non-dimensional derivative set | `aircraft` | implemented and validated |
| `model.linear.statespace` | Load a linear state-space model (A, B, state and input names) from a YAML file | `linear_system` | implemented, unvalidated |
| `report.markdown` | Write a Markdown report from upstream results | `report` | implemented, unvalidated |
| `trim.level` | Solve straight-line trim — wings level, no sideslip — for angle of attack, elevator and thrust, by Newton on a square residual | `trim_point` | implemented and validated |
<!-- END GENERATED CAPABILITY TABLE -->

*implemented and validated* means the output has been compared against a
published reference; see [`docs/VERIFICATION.md`](docs/VERIFICATION.md).
*implemented, unvalidated* means it works and is tested, but no published
reference has been compared against.

## What this is NOT

- **Not a certified tool.** Nothing here is DO-178C qualified. It must never be
  used as evidence in a certification package.
- **Not a MATLAB replacement.** It covers one workflow well, not a general
  numerical platform.
- **Not a flight simulator.** No scenery, no weather rendering, no cockpit, no
  X-Plane rivalry. The 3-D view is an engineering visualisation, not
  entertainment.
- **Not a CFD or panel code.** Aerodynamics come from tabulated or polynomial
  coefficient models supplied by the user or shipped as cited reference data.
  The tool never computes aerodynamic coefficients from geometry.
- **Not an autopilot.** It designs and analyses control laws and can export
  them; it does not fly real aircraft and ships no airworthy code.

## Who it is for

Flight control engineers, GNC engineers, controls graduate students and UAV
autopilot developers — people who currently reach for MATLAB, Simulink, Control
System Toolbox and Aerospace Blockset to go from trim to linearisation to
synthesis to analysis to nonlinear verification.

## Quickstart

Sixty seconds, and it works today. It builds the library, runs the test suite,
and then runs a real study.

```bash
git clone https://github.com/celikgo/galata.git
cd galata

# vcpkg in manifest mode fetches Eigen, fmt, yaml-cpp and GoogleTest.
export VCPKG_ROOT=/path/to/vcpkg

cmake --preset dev
cmake --build --preset dev
ctest --preset dev

# Then run the shipped study: it trims a nonlinear NT-33A, linearises it,
# and reports all five classical modes — every number checked against a
# published NASA report.
./build/dev/src/cli/galata run examples/nt33a-trim-and-linearise/study.yaml
```

That writes `trim-and-modes.md` next to the study: the trim with its residual
and Jacobian conditioning, the state matrices, and the modal tables with the
participation factors the labelling rests on. See
[`examples/nt33a-trim-and-linearise/`](examples/nt33a-trim-and-linearise/README.md).

`galata capabilities` lists what your build can do and how far each capability
has been checked.

Requires CMake 3.24+, Ninja, a C++20 compiler and a vcpkg checkout. Tested on
Linux (GCC and Clang), macOS (AppleClang) and Windows (MSVC) — see
[`.github/workflows/ci.yml`](.github/workflows/ci.yml) for the exact matrix.

## What it will do

This section describes the intended product. **Nothing named here is
implemented** — the two capabilities that used to be listed as intentions,
modal classification by eigenvector participation and the four margin types,
have been built and have moved up into the Status table. This section is kept
so that the decisions recorded in `docs/adr/` have a stated purpose. The Status
table above is what actually works.

The workflow is trim → linearise → analyse → synthesise → verify, expressed as a
YAML pipeline that the CLI and (later) the desktop application both execute, so
that a study is a file rather than a sequence of clicks. Still to come: a
nonlinear 6-DOF simulation with actuator rate limits in the loop, MIL-STD-1797A
handling-qualities assessment, LQR and Riccati synthesis, a stable C plugin ABI
for user-supplied aerodynamic and sensor models, and an AI layer that composes
and runs these pipelines without ever producing a number itself.

Milestones and their contents are in [`docs/ROADMAP.md`](docs/ROADMAP.md).

## Engineering rules

These are gates, not aspirations. A change that violates one does not merge.

1. **CI exists before the feature.** There is no commit that adds source without
   adding to the CI graph.
2. **Nothing is documented before it works.** A capability that is a stub says so
   in its own output and in the docs. A documentation claim that CI does not
   verify is a bug, and the Status table above is the contract.
3. **One source of version truth** — the `VERSION` file, checked by
   [`scripts/check-version-consistency.sh`](scripts/check-version-consistency.sh).
4. **Every URL in every document resolves**, checked by
   [`scripts/check-doc-links.sh`](scripts/check-doc-links.sh).
5. **Strict SI in the numerical core** — metres, seconds, kilograms, newtons,
   radians, kelvin, pascals. Degrees, feet and knots exist only at the UI and
   file-format boundary, converted by one documented set of functions and
   enforced by [`scripts/check-si-boundary.sh`](scripts/check-si-boundary.sh).
   See [ADR-0003](docs/adr/0003-strict-si-and-boundary-conversion.md).
6. **Determinism is tested, and its limits are published.** Same platform: bit
   identical. Across platforms: agreement to a published bound, because
   platform math libraries do not agree on `sin` in the last bits and claiming
   otherwise would be false. See
   [ADR-0004](docs/adr/0004-determinism-policy.md).
7. **Every physics and numerics source file cites its literature source** and
   states the model's validity envelope and the direction and magnitude of its
   known error.
8. **Reference values in tests come from published sources**, never from the
   implementation. See [`docs/TESTING.md`](docs/TESTING.md).
9. **No number reaches the user without provenance** — which capability produced
   it, from what inputs, at what version.

## Conventions

Getting these wrong is how flight software fails silently, so they are written
down once, in full, in
[ADR-0002](docs/adr/0002-state-and-frame-conventions.md):

- **NED** navigation frame, **FRD** body frame.
- Attitude as a **unit quaternion, Hamilton convention, scalar-first
  `[w, x, y, z]`, representing the body-to-NED rotation.** Euler angles (3-2-1)
  are derived output, never integrated state.
- Thirteen-component state vector `[p_n p_e p_d, u v w, q_w q_x q_y q_z, p q r]`
  in that order, which is the row and column order of every state-space matrix
  the tool produces.
- Full 6-DOF equations of motion with a **general inertia tensor** — `I_xz` is
  not assumed zero.

## Documentation

Everything below is also readable at
[celikgo.github.io/galata](https://celikgo.github.io/galata/), which is this
repository's own Markdown rendered — including the generated V&V report. It adds
no content that is not in the repository.

- [`docs/CHARTER.md`](docs/CHARTER.md) — the engineering rules, in full
- [`docs/adr/`](docs/adr/README.md) — architecture decision records
- [`docs/VERIFICATION.md`](docs/VERIFICATION.md) — the V&V report: what has been
  checked against a published document, the agreement measured, and what is
  explicitly unvalidated. Generated by CI from the code, not written by hand.
- [`docs/TESTING.md`](docs/TESTING.md) — the test tiers and what each proves
- [`docs/ROADMAP.md`](docs/ROADMAP.md) — milestones and their contents
- [`CONTRIBUTING.md`](CONTRIBUTING.md) — building, the pre-push gates, what review asks
- [`SECURITY.md`](SECURITY.md) — the threat model this tool actually has, and
  how to report a vulnerability privately
- [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md) — Contributor Covenant 2.1

### Checking the validation claim yourself

The agreement with NASA CR-2144 quoted at the top of this file is not a
sentence somebody typed. It is gated:

- the reference values live in
  [`tests/validation/reference/nt33a_fc1.csv`](tests/validation/reference/nt33a_fc1.csv),
  which carries the report number, its authors, its rights position, the
  SHA-256 of the scan the numbers were read from, and the method by which they
  were transcribed;
- [`tests/validation/test_nt33a_trim_linearize.cpp`](tests/validation/test_nt33a_trim_linearize.cpp)
  runs the whole chain from the non-dimensional derivative set and **fails** if
  any dimensional derivative deviates by more than 0.5%;
- [`tests/validation/test_nt33a_modes.cpp`](tests/validation/test_nt33a_modes.cpp)
  does the same for the five classical modes;
- [`docs/VERIFICATION.md`](docs/VERIFICATION.md) is regenerated from those runs
  by [`scripts/gen-verification.sh`](scripts/gen-verification.sh), and CI fails
  if the committed copy has drifted, so the report cannot describe a
  measurement the code no longer produces.

```bash
ctest --preset dev -L validation      # the whole validation tier
```

The tier carries the ctest **label** `validation`, so `-L` is the flag; the
tests are named after what they check, not after the tier.

## Licence

Apache-2.0. See [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

The name is the Galata Tower in Istanbul, from which — by an account first
printed in Evliya Çelebi's *Seyahatnâme* — Hezârfen Ahmed Çelebi is said to have
glided across the Bosphorus in the 1630s. The story is not evidence and this
project does not treat it as such; it is just where the name comes from.
