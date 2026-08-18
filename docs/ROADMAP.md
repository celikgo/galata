# Roadmap

**Everything in this document is a plan.** Nothing here is a description of the
current repository except where it says so. The README's Status table is the
authority on what works today; this document is the authority on the order
things are intended to arrive in.

The rule for every milestone is the same: a stranger can clone the repository at
that tag and run something real.

## v0.1 — the honest spine

**In progress.** What it needs, and where each piece stands:

| Piece | State |
|---|---|
| Core types, frames, quaternions, units | done, property-tested |
| ISA atmosphere | done, validated against the published tables |
| Nonlinear 6-DOF equations of motion | done, validated against closed-form solutions of Euler's equations |
| Fixed-step RK4 | done, order verified |
| `analyze.modes` | done, validated against NASA CR-2144 |
| Pipeline runner and CLI | done |
| `ci.yml`, `determinism.yml` | done |
| A reference aircraft model | done as a **linear** model; the nonlinear one is not built |
| `trim.level` | **not built** |
| `linearize.finitediff` | **not built** |
| One example: YAML file to modal table | done, two of them, both checked by CI |

The two missing capabilities are the same missing thing: there is no nonlinear
aircraft model to trim or to linearise. `analyze.modes` is validated against a
published aircraft, but the state matrix it consumes is read from a file rather
than produced by trimming and linearising a nonlinear model. Closing that gap
means an aerodynamic coefficient model, a trim formulation as a constrained
root-find with a reported residual and Jacobian condition number, and
central-difference linearisation with a Richardson truncation-error estimate.

No AI. No GUI.

This is the credibility floor and nothing ships before it.

## v0.2 — analysis and design

Frequency response, gain/phase/delay/disk margins, `synth.pid`, `synth.lqr`,
CARE via the generalised Schur method validated against the CAREX benchmark
collection, root locus, and `sim.nonlinear` with actuator position and rate
limits in the loop. Markdown reports with embedded plots. A high-fidelity
aircraft model with its provenance, and the modal validation gate against
published values.

## v0.3 — the desktop application

Tauri shell, 3-D viewport, the plot suite (Bode, Nyquist with the disk-margin
disk drawn, Nichols with handling-quality boundaries, labelled pole-zero map,
root locus with a live gain slider), the pipeline editor. Signed installers for
three platforms.

No AI yet — the application has to be useful without it.

## v0.4 — the AI layer

Provider abstraction with Anthropic, OpenAI, the OpenAI-compatible generic
adapter and Ollama. Secrets in the OS keychain. The agent loop, the first twelve
tools, the audit log, budgets, and the eval suite. `galata-mcp` ships in the
same release.

The agent composes and runs pipelines. It never produces a number itself; tools
return handles to computed artefacts and the chat renders those artefacts.

## v0.5 — identification and hardware in the loop

Frequency sweeps, coherence, transfer-function fitting. A MAVLink SITL bridge.
The single-axis pitch-rig example with measured-versus-predicted step responses,
its raw data and its bill of materials.

## v1.0 — freeze

Plugin ABI v1 final, agent tool contract v1 final, the full provider set,
handling qualities, Monte Carlo, gain scheduling, the published verification
and validation report, and a documentation site that resolves.

Tagged only when all of that is true.

## Aircraft model data

Coefficient data ships in this repository only when it is transcribed from a US
Government work, which is in the public domain. Data traceable only to a
copyrighted source ships as a loader plus documented instructions for obtaining
the data — never as data. A dataset whose licensing position cannot be
established with confidence is not shipped at all.

Where no published reference value can be found for a validation case, that case
is marked unvalidated in `docs/VERIFICATION.md`.
