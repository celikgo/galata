# galata

Flight dynamics, control-law design and simulation: trim an aircraft, linearise
it, identify its modes, synthesise a control law, and verify the result in a
nonlinear simulation — reproducibly, from a file you can read, with every number
traceable to the routine that produced it.

C++20 core, strict SI units, deterministic by policy, Apache-2.0.

[![CI](https://github.com/celikgo/galata/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/celikgo/galata/actions/workflows/ci.yml)


---

## Status: 0.0.1 — skeleton

**This repository does not yet compute anything about an aircraft.** It contains
the build system, the CI gates, the engineering decisions that constrain
everything after it, and one library with one function in it.

That is deliberate. The [charter](docs/CHARTER.md) requires that CI exist before
the first feature and that no capability be documented before it works, so the
gates are built first and the physics lands against them. What exists today:

| Surface | State |
|---|---|
| Build system (CMake + vcpkg, four platform/compiler combinations) | working |
| CI: format, build, test, version consistency, SI boundary, doc links | working |
| Version single-source-of-truth, with provenance in the build identification | working |
| ADRs 0001–0005: ABI relationship, conventions, units, determinism, versioning | written |
| Frames, quaternions, state vector, unit boundary (ADR-0002, ADR-0003) | implemented, property-tested |
| U.S. Standard Atmosphere 1976, −5000 m to 86 km | implemented and **validated** against the published tables |
| Fixed-step RK4, with Richardson step-size study | implemented, order verified against closed-form solutions |
| Nonlinear 6-DOF rigid body, general inertia tensor | implemented and **validated** against closed-form solutions of Euler's equations |
| YAML pipeline runner and the `galata` CLI | working |
| One runnable example, checked by CI | working (two of them) |
| Determinism: bit-identical repeat runs, cross-platform bound measured | working |
| Everything else in §"What it will do" below | **not built** |

The table above is maintained by hand and checked in review. The capability
table below is not: it is generated from the registry the CLI dispatches
through, by `scripts/gen-status-table.sh`, and CI fails if the committed copy
disagrees. Run `galata capabilities` to get the same list from your own build.

### Capabilities in this build

<!-- BEGIN GENERATED CAPABILITY TABLE -->
| Capability | What it does | Produces | State |
|---|---|---|---|
| `analyze.modes` | Eigenvalues, modal metrics and participation factors, with the classical aircraft modes classified by participation | `modal_table` | implemented and validated |
| `model.linear.statespace` | Load a linear state-space model (A, B, state and input names) from a YAML file | `linear_system` | implemented, unvalidated |
| `report.markdown` | Write a Markdown report from upstream results | `report` | implemented, unvalidated |
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

Sixty seconds, and it works today. It builds the library and runs the test
suite; there is nothing else to run yet.

```bash
git clone https://github.com/celikgo/galata.git
cd galata

# vcpkg in manifest mode fetches Eigen, fmt, yaml-cpp and GoogleTest.
export VCPKG_ROOT=/path/to/vcpkg

cmake --preset dev
cmake --build --preset dev
ctest --preset dev

# Then run the shipped study: it turns a linear aircraft model into a
# labelled modal table, and every number in it is checked against a
# published NASA report.
./build/dev/src/cli/galata run examples/nt33a-lateral-modes/modal-study.yaml
```

That writes `lateral-modes.md` next to the study. It reports the NT-33A's
spiral, roll subsidence and Dutch roll — labelled, not just listed — with the
participation factors the labelling rests on. See
[`examples/nt33a-lateral-modes/`](examples/nt33a-lateral-modes/README.md).

`galata capabilities` lists what your build can do and how far each capability
has been checked.

Requires CMake 3.24+, Ninja, a C++20 compiler and a vcpkg checkout. Tested on
Linux (GCC and Clang), macOS (AppleClang) and Windows (MSVC) — see
[`.github/workflows/ci.yml`](.github/workflows/ci.yml) for the exact matrix.

## What it will do

This section describes the intended product and **none of it is implemented**.
It is here so that the decisions recorded in `docs/adr/` have a stated purpose.
Read the Status table above for what actually works.

The workflow is trim → linearise → analyse → synthesise → verify, expressed as a
YAML pipeline that the CLI and (later) the desktop application both execute, so
that a study is a file rather than a sequence of clicks. Planned surfaces include
a nonlinear 6-DOF simulation with actuator rate limits in the loop, automatic
modal classification by eigenvector participation, gain/phase/delay and disk
margins, MIL-STD-1797A handling-qualities assessment, a stable C plugin ABI for
user-supplied aerodynamic and sensor models, and an AI layer that composes and
runs these pipelines without ever producing a number itself.

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

- [`docs/CHARTER.md`](docs/CHARTER.md) — the engineering rules, in full
- [`docs/adr/`](docs/adr/README.md) — architecture decision records
- [`docs/VERIFICATION.md`](docs/VERIFICATION.md) — the V&V report: what has been
  checked against a published document, the agreement measured, and what is
  explicitly unvalidated. Generated by CI from the code, not written by hand.
- [`docs/TESTING.md`](docs/TESTING.md) — the test tiers and what each proves
- [`docs/ROADMAP.md`](docs/ROADMAP.md) — milestones and their contents
- [`CONTRIBUTING.md`](CONTRIBUTING.md) — building, the pre-push gates, what review asks

## Licence

Apache-2.0. See [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

The name is the Galata Tower in Istanbul, from which — by an account first
printed in Evliya Çelebi's *Seyahatnâme* — Hezârfen Ahmed Çelebi is said to have
glided across the Bosphorus in the 1630s. The story is not evidence and this
project does not treat it as such; it is just where the name comes from.
