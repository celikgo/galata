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
| A reference aircraft model | done, nonlinear, validated |
| `trim.level` | done, validated |
| `linearize.finitediff` | done, validated |
| One example: YAML file to modal table | done, three of them, all checked by CI |

**v0.1 is complete.** The chain runs end to end: a nonlinear aircraft model
built from non-dimensional derivatives, trimmed by a constrained root-find that
reports its residual and Jacobian conditioning and refuses to return a
best-effort answer, linearised by central differences with a Richardson
truncation estimate per entry, and analysed into a labelled modal table. It
reproduces NASA CR-2144's published dimensional derivatives to 0.26% and its
published modes to 1.0%.

No AI. No GUI.

This is the credibility floor and nothing ships before it.

## v0.2 — analysis and design

**Done.** Frequency response (`analyze.freqresp`), evaluated by Hessenberg
solves with the grid refined around the system's own lightly damped modes, and
all four margin types: gain, phase and delay (`analyze.margins`) with every
crossover reported, and the disk margin (`analyze.diskmargin`) with its
guaranteed gain and phase range and a destabilising perturbation on the
boundary. Validated against closed-form transfer functions and against the
published worked example of Seiler, Packard & Gahinet (2020); see
`docs/VERIFICATION.md`.

**Remaining.** `synth.pid`, `synth.lqr`, CARE via the generalised Schur method
validated against the CAREX benchmark collection, root locus, and
`sim.nonlinear` with actuator position and rate limits in the loop. Markdown
reports with embedded plots. Singular values for MIMO loops and the sensitivity
and complementary-sensitivity peaks. A high-fidelity aircraft model with its
provenance, and the modal validation gate against published values.

Known gap carried forward: the disk margin's peak is found on a refined
frequency grid rather than by the exact Hamiltonian-eigenvalue method (Boyd &
Balakrishnan; Bruinsma & Steinbuch), so the reported margin is an upper bound on
the true one. The error is in the optimistic direction and is documented at
every point it surfaces.

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
