<!-- SPDX-License-Identifier: Apache-2.0 -->

# RFC-0002: Quadrotor programme support — native plant, hover trim, generic linearisation, measured-data import, identification, sampled control

- **Status:** partly delivered. WP1 through WP5 are implemented and their delivery records
  are at the foot of this document. Every capability is registered as
  implemented-unvalidated: none of them is anchored to a published reference, for the reason
  the acceptance section gives, and a completed run of any of them is not a validation.
  What is NOT delivered is named in each record rather than left to be inferred — chiefly
  discrete-time synthesis, the sampled loop's own robustness, and any evidence at all from real
  hardware, which does not exist. This line has been wrong three times: it said "nothing below
  is implemented" after WP1 landed, "WP3, WP4 and WP5 are not started" after all three had, and
  it named the Gramians as outstanding after `analyze.gramians` registered. The README's Status
  table, generated from the capability registry, remains the authority on what exists, and it
  is the authority precisely because a hand-maintained status line drifts; nothing here is a
  release commitment. The per-item evidence, and the four statuses kept apart, are in
  [the acceptance record](../reports/quadrotor-programme-acceptance.md).
- **Date:** 2026-09-08
- **Requested by:** the Souxmar forest-ISR quadrotor programme (GitLab `souxmar`, a Python repository; not the CAE project of the same name that ADR-0001 cites)
- **Affects:** `src/model/`, `src/trim/`, `src/linearize/`, `src/sim/`, `src/pipeline/capabilities.cpp`, `docs/product/FEATURES.md`, `docs/ROADMAP.md`, `docs/VERIFICATION.md`, ADR-0002, ADR-0006
- **Supersedes:** nothing. Refines feature rows F04, F06, F08, F10, F14, F16, F17, F21 and F26 of `docs/product/FEATURES.md` for one vehicle class.

## Why this document exists

Souxmar has no MATLAB. It has adopted galata as its Control System Toolbox replacement
and has verified that adoption: an independent nonlinear quadrotor plant in Python
(ENU/FLU frames, wind, first-order rotor lag, linear-plus-quadratic drag, battery-limited
commands) is trimmed at hover, linearised on a sixteen-coordinate local attitude-error
chart, transformed to this repository's NED/FRD conventions by an explicit similarity, and
exported as a named A/B/C/D file that `model.linear.statespace` reads unchanged. The
exported study runs `analyze.modes`, `sim.linear` and `report.*` end to end. Souxmar's
gated integration test then checks `sim.linear` against its own integration of the same
matrices and against the nonlinear plant under small perturbations. Both gates pass
against this repository's `main` at `f19c562` (Debug, AppleClang, Darwin/arm64, run
2026-09-08); the agreement with galata's integrator is at floating-point round-off, and the
linear-versus-nonlinear discrepancy is a few percent, bounded by the quadratic drag term
that a linearisation at zero airspeed cannot see. The measured values are retained in
Souxmar's run summary, not restated here.

That covers the part of the workflow that ends at linear analysis. The programme's
model-first path continues past it: build the aircraft, measure its components on the
bench, fly excitation manoeuvres from PX4, identify the model, and design and simulate a
sampled controller against it. None of that has a tool today. This RFC asks galata to grow
into it, in an order where each package is verifiable before the next starts, and it says
how each would be verified — which, in this repository, is what an RFC is for.

Souxmar's own account of the boundary lives in its repository, and the fixture it publishes
for this work is regenerable from seed and config:

```text
/Users/celikgo/souxmar/docs/adr/0014_galata_analysis_workbench.md     the decision
/Users/celikgo/souxmar/docs/phase5_galata_bridge.md                   the bridge, with its measured results
/Users/celikgo/souxmar/docs/research/galata_work_request.md           the original work request this RFC restates
/Users/celikgo/souxmar/outputs/galata_bridge/quad_hover_ned_frd.yaml  the exported model, in this repository's file form
/Users/celikgo/souxmar/outputs/galata_bridge/quad_hover_ned_frd.sidecar.json
/Users/celikgo/souxmar/outputs/galata_bridge/reference_trajectory.csv the open-loop cross-implementation fixture
/Users/celikgo/souxmar/outputs/galata_bridge/reference_trajectory.json
/Users/celikgo/souxmar/outputs/galata_bridge/bridge_check.json        the retained gate results
python -m apps.galata_bridge --output outputs/galata_bridge --galata <galata-cli>   # regenerates all of the above
```

## The aircraft, so the scope is concrete

A 650 mm X-configuration quadrotor, about 2.5 kg all-up, 6S battery, four T-Motor MN4006
KV380 motors on 15-inch propellers, Pixhawk 6C Mini running PX4. Flight logs are PX4 ULog.
The current Souxmar plant is a nominal 1.6 kg vehicle with diagonal inertia; the built
aircraft will carry a measured full inertia tensor and measured rotor positions, and the
model must accept those without a second plant.

## Conventions that bind every package

These are the repository's own rules; they are listed so no package needs to rediscover them.

- ADR-0002: one state layout. Rotor speeds and battery state are carried alongside the
  13-component rigid-body state, as `include/galata/core/state.hpp` already anticipates, not
  inside it.
- ADR-0003: strict SI, key-carries-the-unit. Conversions at transcription time only, recorded
  in the model's provenance file.
- ADR-0004 and ADR-0006: determinism, and equations about the CG through the existing
  `include/galata/sim/rigid_body.hpp` kernel. No second 6-DoF.
- ADR-0008 and ADR-0009: every capability's evidence separates "completed", "numerically
  accurate" and "engineering accepted", and lands in the run manifest.
- Every new header carries a `WHAT THIS IS NOT` block and a citation. Every capability
  registers with a strict schema; unknown keys are errors. Five tiers per `docs/TESTING.md`.
- No GUI, no onboard code, no qualification claim. A completed run is not a validation.

Frame boundary, stated once: Souxmar is ENU position and ground velocity, FLU body,
`wxyz` quaternion body→world. This repository is NED, FRD, body→NED. The transform is
ENU→NED `[n, e, d] = [y, x, −z]` on polar vectors and FLU→FRD `[f, r, d] = [x, −y, −z]`
as a basis change on the rotation; no axial vector passes through a reflection and no
quaternion component is reinterpreted. Souxmar yaw zero points east; heading zero here is
north. The sidecar file records both.

## WP1 — `model.quadrotor`

A parametrised nonlinear quadrotor as a first-class C++ model (feature rows F06, F21).

**Model.** Rigid body via the existing kernel with a full inertia tensor about the CG.
Four rotors at declared body positions with declared spin direction. Rotor-speed states
with first-order lag to a commanded speed in rad/s. Thrust `T_i = k_T ω_i²` along body −z,
reaction torque `Q_i = ±k_Q ω_i²`. Drag on the air-relative body velocity with linear and
quadratic coefficients per axis. Gravity via the existing `gravity_body`. Optional battery:
state of charge, open-circuit-voltage curve, internal resistance, voltage-dependent speed
limit; omit the block and the model is exactly the fixed-voltage plant. Inputs: four
rotor-speed commands. ESC-command mapping is out of scope and the header must say so.

**File.** A `models/<name>/` directory holding the YAML and its provenance, in the shape
the `adding-an-aircraft-model` skill prescribes. Keys, all unit-suffixed: mass; the six
inertia entries in the sign convention `MassProperties` documents; a rotor list with
position, spin, thrust and torque coefficients, time constant, speed limits; per-axis
linear and quadratic drag; optional battery. Reject nonfinite values, non-positive-definite
inertia, zero rotors, mixed rotor counts, negative coefficients.

**Verification, tier `validation`, each with a stated budget.**

1. Hover balance: `4 k_T ω_h² = m g` and zero moment for equal speeds.
2. Free fall: zero rotor speed gives zero body specific force and NED acceleration `[0, 0, g]`.
3. Torque signs: raising one diagonal pair accelerates the expected body axis with the
   expected sign; opposite-spin pairs yaw with the expected sign.
4. Torque-free conservation: rotors off, no drag, rotational energy and angular-momentum
   magnitude held to the kernel's existing tolerance.
5. Step refinement: dt and dt/2 over a 30 s manoeuvre, with a position and attitude budget.
6. Cross-implementation against Souxmar's fixture above: the same open-loop rotor-command
   history (pitch, roll, yaw and collective pulses, then a steady ENU wind), every column
   recorded in both frame conventions, dt 0.004 s, 8 s. State the tolerance, retain both
   trajectories. A disagreement is a finding, not a tolerance to widen.

## WP2 — `trim.hover` and a model-generic `linearize.finitediff`

Feature rows F08 and F10. `include/galata/trim/level.hpp` solves angle of attack, elevator
and thrust at positive airspeed and cannot express hover; `include/galata/linearize/finite_difference.hpp`
is bound to `model::Aircraft` and warns about its Euler-chart singularity.

- **`trim.hover`:** solve rotor speeds for equilibrium at declared altitude, mass, wind vector
  and optional steady ground velocity — still-air hover, crosswind hover, and cruise as a
  relative equilibrium with nonzero position rate. Report residual, Newton iterations and
  each rotor's margin to its speed limit. Refuse an infeasible trim rather than returning a
  best effort, as `trim_level` does. Freeze battery state of charge and document that a
  powered battery has no zero-energy-derivative equilibrium.
- **Generic linearisation:** accept any model exposing `f(x_ext, u)` where `x_ext` is the
  13-state plus the model's appended states, on a local attitude-error chart with three
  attitude perturbation coordinates, so the quadrotor linearises to `12 + n_rotor (+ battery)`
  states without the singularity. Keep the Richardson truncation estimate per entry.
- **Observation model:** C and D from a declared output list — body specific force, body
  rates, NED position, altitude, ground velocity — because the sensors do not measure every
  state and `C = I, D = 0` is not this programme's case. Specific force is `Rᵀ(a − g)`, so D
  carries the wind-to-specific-force drag feedthrough.
- **Disturbance columns:** a wind input as separately named columns of B and D, so
  `model.channels` can select or drop it.
- **File contract:** the existing `model.linear.statespace` YAML stays valid; Souxmar's
  exported file and sidecar are the shape to match or supersede, so that `analyze.*`,
  `synth.*` and `sim.linear` consume the quadrotor unchanged.

**Verification.** The hover-linearised quadrotor shows the expected structure — six
integrator eigenvalues from position and attitude, drag-only decay rates on the translational
and rotational axes, four rotor-lag poles at `−1/τ` — and the thrust-to-vertical-acceleration
gain equals `4·2·k_T·ω_h/m`. Small-perturbation nonlinear and linear responses agree to a
stated bound over a stated horizon, and the bound must name the quadratic-drag limit: the
nominal `drag_linear/drag_quadratic` ratio sets the airspeed below which the linear model is
meaningful. Souxmar's `analyze.modes` run correctly declined participation factors at hover
because the double-integrator chains are defective; that refusal is the right behaviour and
should be kept, and the fixed-wing mode labels must not be applied (`classify: false`).

## WP3 — measured-data import

Feature rows F04 and F26. Capabilities `data.import.csv` and `data.import.ulog`.

- ULog topics at minimum: `vehicle_attitude`, `vehicle_angular_velocity`,
  `vehicle_local_position`, `sensor_combined`, `actuator_motors` or `actuator_outputs`,
  `battery_status`. Resample to a declared fixed rate, align on one timebase, tag every
  channel with unit and frame, refuse unknown or unit-less channels.
- PX4 frames are NED and FRD, which match ADR-0002; say so, and test one sign per axis.
- Evidence: source-file digest, channel list, resampling method, dropped-sample count.

## WP4 — identification

Feature rows F23 and F26. This is the System Identification Toolbox slice and has no
counterpart in the repository today.

- **`identify.static_fit`:** linear least squares for bench maps — thrust and torque against
  ω², motor step response for τ, battery open-circuit voltage and internal resistance against
  current. Report coefficients, covariance, residual RMS and the valid input range of the fit.
- **`identify.greybox`:** prediction-error minimisation of a declared parameter subset of
  `model.quadrotor` against an imported flight record. Simulate the nonlinear model with the
  recorded inputs, minimise output error over declared outputs, bound parameters to declared
  ranges, report Jacobian-based covariance and confidence intervals. Refuse to report a fit
  whose Hessian is singular in a parameter direction — an unidentifiable parameter — rather
  than returning a number.
- **`identify.validate`:** run the identified model on a held-out record; report NRMSE fit
  percentage, residual whiteness and per-output error. A fit on the estimation record is
  never validation.

**Verification.** A synthetic self-test recovers known parameters from a simulated record
with declared noise to within the reported confidence intervals; a deliberately
unidentifiable parameter set is refused with the reason stated.

## WP5 — sampled control and time-varying inputs

Feature rows F14 and F17, after WP1–2. Two gaps confirmed in the tree today:

- `include/galata/sim/linear.hpp` accepts a constant input only. Souxmar needs the `lsim`
  equivalent: a declared piecewise-constant or sampled input history, with the sample
  hold stated, for both linear and nonlinear simulation.
- Nothing in the repository executes a controller at a declared rate. The PX4 rate and
  attitude cascade runs sampled, with zero-order hold and a known delay; so does
  Souxmar's own 250 Hz cascade. Discrete-time controller execution inside `sim.nonlinear`
  for the quadrotor, with `c2d`-style discretisation of a continuous design, DARE and
  sampled LQR, is the package.

A smaller companion, also missing: controllability and observability Gramians for a
declared input and output set, so an exported model's defective integrator chains and any
unobservable direction are reported rather than discovered from a failed synthesis.

## Housekeeping

- ADR-0001 cites a CAE project named `souxmar` on GitHub as the origin of the plugin-ABI
  pattern. The requesting programme's repository is also named Souxmar, on GitLab. Add one
  clarifying sentence to `docs/adr/0001-independent-c-abi.md`; do not rewrite the decision.
- Update the F-rows named above and `docs/ROADMAP.md`, keeping the rule that a roadmap entry
  is not a release commitment. Each package updates `docs/VERIFICATION.md` through the
  generator, never by hand.
- Test isolation: Souxmar's review of `1b403cf` saw one failure in a two-worker ctest run
  that passed in isolation and in a sequential rerun. The example helper in
  `tests/integration/test_example_studies.cpp` derives its output directory from the example
  name alone, so tests for the same example can race. Give each a unique scratch directory
  or serialise them; the retained logs are in Souxmar's outputs.

## Out of scope

General Simulink compatibility, extending the block language to vectors or nonlinear
blocks, the desktop editor, onboard deployment, ESC-level modelling, rotor inflow, flapping
and ground effect, and any statement that a completed workflow validates the aircraft. A
package that needs one of these stops and raises it here rather than absorbing it.

## Open questions for acceptance

1. Does the quadrotor's appended state (four rotor speeds, optional battery) live as a
   model-owned extension vector next to the 13-state, or does ADR-0002 need a successor
   describing an extensible state? The request prefers the former.
2. Which published quadrotor reference, if any, anchors WP1's validation tier beyond the
   analytic cases and the Souxmar cross-implementation fixture? Souxmar's plant is an
   independent implementation, not a published source, and ADR-0007 decides how it may be
   cited.
3. Whether WP5's discrete-time work is scoped to this vehicle or lands as the general F14/F17
   delivery; the request accepts either as long as the quadrotor case is the acceptance test.

## Acceptance

Answered 2026-09-08, before any package started, by reading the tree rather than the
RFC's summary of it. The status above is unchanged: these are answers to the three
questions, not an acceptance of the packages. Nothing here is implemented.

### 1. The appended state does not need a successor to ADR-0002

**Decision: no successor ADR. The four rotor-speed states and the optional battery
state live as a model-owned extension vector appended after index 12 of the
thirteen-component rigid-body state, as the request prefers. WP1 proceeds without a
stop.**

Four reasons, in the order they carry weight.

*ADR-0002's normative text does not reach the question.* What it fixes is the identity
and order of thirteen components — "the order in which the components are stored,
integrated, fingerprinted and serialised" — and the consequence it protects is that
changing that order "changes the meaning of every exported A, B, C, D matrix and is a
major-version event". Appending after index 12 permutes nothing and changes the meaning
of no matrix already exported. The record's "Revisit when: Never" guards the ordering as
a compatibility surface, and an appended vector does not touch it.

*The decision has already been taken, in the header ADR-0002 governs.*
`include/galata/core/state.hpp` names this exact case: the rigid-body state "carries no
structural modes, no fuel slosh, no rotor or propeller dynamics, and no engine state. A
model needing those carries them alongside, not inside." The quadrotor is the anticipated
case, not one argued by analogy. A successor ADR would restate a rule already written.

*The ADR index's own test excludes it.* `docs/adr/README.md` admits "a decision that a
stranger would have made differently, or that costs something real to reverse"; a
decision that follows from a rule already recorded does not get a record.

*There is a precedent for exactly this shape.* RFC-0001 names ADR-0004 in its Affects
line, resolves its open questions in an acceptance block written into the RFC, and merged
`src/synth/` without amending ADR-0004.

Three things the question did not ask, found while answering it. They are recorded here
because each falls due inside a package below.

**(a) `include/galata/core/state.hpp` contradicts ADR-0002 and must be corrected in WP1.**
The header says of the thirteen components: "This is the row and column order of every A
and B matrix galata produces". ADR-0002 says it is "**not** the row and column order of
the A and B matrices galata produces", because `linearize.finitediff` exports twelve Euler
coordinates and a reduced set of those. ADR-0002 is right and the header is wrong. No gate
catches it — `scripts/check-doc-references.sh` resolves names, not claims — and WP1 makes
the false sentence worse by adding a model whose exported width is neither thirteen nor
twelve. The one-line correction ships in WP1's first commit, not as a separate concern.

**(b) ADR-0013's channel cap, not ADR-0002, is what the battery variant runs into.**
`include/galata/modeling/linear_adapter.hpp` sets `kMaxLinearChannels = 16` and
`src/modeling/linear_adapter.cpp` refuses more: "linear graph supports at most 16 states,
inputs and outputs". Souxmar's already-working export declares exactly sixteen states, so
the fixed-voltage quadrotor sits precisely at the cap and one battery state puts it one
over. `model.linear.statespace` is unaffected — `src/model/linear_system.cpp` imposes no
state-count limit of any kind, only squareness, name-count agreement, shape agreement and
finiteness — so `analyze.*` and `sim.linear` read a seventeen-state model unchanged. What
the battery variant loses is the typed linear-graph path. That is an ADR-0013 question,
it falls due in WP2 and not in WP1, and it is raised here rather than absorbed.

**(c) WP2's attitude-error chart is an addition, not a replacement.** It needs no ADR on
the condition that it lands beside the existing twelve-coordinate Euler path rather than
changing what `linearize.finitediff` exports for the NT-33A. If implementation finds that
the two cannot coexist, that is a contract change and it stops for a record.

### 2. No published quadrotor reference anchors WP1

**Decision: none. Cases 1 to 5 are anchored to closed-form invariants, and case 6 is a
cross-implementation agreement, not a validation. `model.quadrotor` registers as
implemented-unvalidated, and the case registry carries case 6 as self-consistent.**

This is not a gap left open for want of looking. Charter rule 8 says plainly that where no
published value can be found, the V&V report marks the case unvalidated, and that "an
honest 'unvalidated' is worth more than a fabricated match". The repository already holds
the precedent for the honest form: `rigid_body.conservation` is a validated case whose
reference is exact mathematics rather than a document, and `analyze.freqresp` records that
"the reference is arithmetic, not a document". Hover balance, free fall, torque signs,
torque-free conservation and step refinement are all invariants of the equations, checkable
to round-off without an author. That is what cases 1 to 5 rest on, and it is a stronger
anchor than a transcribed table would be.

What none of them anchors is the *parameter set* — no published source is being claimed for
a 650 mm quadrotor's k_T, k_Q or drag. That is why `model.quadrotor` enters the registry
unvalidated rather than validated, and why the model's `WHAT THIS IS NOT` block must say
that a completed run of it is not evidence about any real aircraft.

Two consequences for how case 6 is built, both found by reading the fixture.

**The trajectory is not committed as data.** ADR-0007 draws its line between quoting a
scalar RESULT and shipping a DATASET, and keeps datasets to US Government primary sources,
routing everything else to "a loader plus fetch instructions, never as data". Souxmar's
sampled trajectory is a dataset by that test, and its rights position is not merely
unfavourable but unestablished — the source tree carries no licence file at all. So case 6
reads the fixture from its declared path, states in its own skip message why it did not run
when the path is absent, and the RFC's published regeneration command is the fetch
instruction. Both trajectories are retained under the run outputs, which are build
artefacts, not committed reference data. A decimated copy in `tests/` would be the same
dataset in a directory whose loader happens not to ask for a citation header, and that is
using a gate's boundary to escape a policy.

**Case 6's tolerance has a floor that is not model error, and it must be named before the
first number is read.** The two implementations differ in integration scheme, not only in
model: Souxmar's rotor lag is exact and sampled at the stages, galata's is an ODE state
carried through fixed-step RK4. The fixture's own retained gate results already show the
signature — a rotor-channel relative error near 6e-7 while attitude and body-rate errors
sit at round-off. The budget is stated as that floor plus the step-refinement bound, and it
is attributed to the scheme in the case note. It is not widened afterwards to fit a result.

### 3. WP5 lands under vehicle-neutral capability names, with the quadrotor as its acceptance test

**Decision: general. The discrete-time work registers under vehicle-neutral ids, and the
quadrotor is the case that must pass before any of them is called done.**

The registry is organised by verb, not by vehicle: every registered id carries a verb
prefix — `model`, `analyze`, `synth`, `sim`, `report`, `trim`, `linearize` — and none
names a programme. A `sim.quadrotor.discrete` would be the first, and the cost of undoing a
capability name is a compatibility break for every study file that dispatches through it,
whereas the cost of a general name that later needs narrowing is nothing. Where the tree
does admit a class segment is under `model.`: `model.aircraft.derivatives` is a registered
id today, which is the precedent `model.quadrotor` follows. The rule this fixes: a
model-class segment is allowed under `model.`, and is not used under `sim.`, `synth.`,
`analyze.`, `trim.` or `linearize.`.

One correction to how this must be written up. WP5 implements the proposed work of F14 and
part of F17; it does not deliver either row. `docs/product/FEATURES.md` states that an epic
"cannot pass acceptance while a required dependency is unresolved", and both rows carry
unresolved dependencies that no package here scopes. Saying WP5 "delivers F14" would be the
present tense charter rule 2 forbids. The F-rows stay open, and the roadmap and V&V notes
that currently record sampled execution as an open gap are amended through the generators
rather than left to drift.

### A correction to WP1's model, without which WP2 cannot pass its own criterion

WP1's model paragraph gives drag on the air-relative body velocity with linear and
quadratic coefficients per axis, and lists "per-axis linear and quadratic drag" among the
YAML keys. There is no angular drag term anywhere in it. WP2's verification then requires
the hover linearisation to show "drag-only decay rates on the translational **and
rotational** axes". Those two cannot both hold: with no angular drag the body-rate axes
have no decay, the hover A matrix has nine zero eigenvalues rather than six, and WP2's
stated acceptance criterion is unsatisfiable by WP1's stated model.

The fixture settles which side is wrong. Souxmar's plant carries an explicit per-axis
angular drag parameter, its exported hover A matrix carries the corresponding body-rate
diagonal terms, and its retained eigenvalues show six zeros and three separate rotational
decay rates — not nine zeros. Case 6 compares an open-loop trajectory against that plant,
so a WP1 model without angular drag would also disagree with the fixture by construction,
in a way no tolerance should be widened to accommodate.

**WP1's schema therefore carries a per-axis angular drag coefficient alongside the
translational ones, with the same rejection rules.** This is recorded as a correction to
the RFC rather than absorbed silently into the implementation, because it changes the model
the packages were requested against.

## Delivery record

### WP1 — `model.quadrotor`, 2026-09-08

Delivered in the order the request set: header, loader, model directory, pipeline
registration, validation cases. Nothing in WP2 to WP5 is started, and the status line above
is unchanged.

**What exists now.** `include/galata/model/quadrotor.hpp` and `src/model/quadrotor.cpp` carry
a nonlinear multirotor plant — rigid body through the existing
`include/galata/sim/rigid_body.hpp` kernel with no second six-degree-of-freedom
implementation, rotors with first-order speed lag, per-axis linear, quadratic and angular
drag, and an optional battery. The extended state is the thirteen ADR-0002 components
followed by one state per rotor and, when present, one for state of charge. `model.quadrotor`
is registered in `src/pipeline/capabilities.cpp` as implemented-unvalidated, and the README's
capability table now carries its row, regenerated through `scripts/gen-status-table.sh`.

The model file is `models/souxmar-quad/souxmar-quad.yaml` with its
`models/souxmar-quad/PROVENANCE.md`. The loader refuses unknown keys at every level of the
document, which `load_aircraft` does not, and the rejection list the request named is
enforced and tested.

**Verification.** Two cases are registered in `tools/validation/case_registry.cpp` and render
in `docs/VERIFICATION.md`, which was regenerated through `scripts/gen-verification.sh`:
`quadrotor.invariants` at validated, whose reference is mathematics rather than a document,
and `quadrotor.cross_implementation` at self-consistent. Every figure below is the one those
generated documents and the tests' own recorded properties carry; none is typed here.

The six cases the request asked for, by test name, all in the `validation` tier:

1. `QuadrotorHover.EqualSpeedsCarryTheWeightAndProduceNoMoment`
2. `QuadrotorFreeFall.ZeroRotorSpeedGivesZeroSpecificForceAndOneGeeDown`
3. `QuadrotorTorqueSigns.DifferentialThrustDrivesTheExpectedAxisAndOnlyThatAxis`
4. `QuadrotorTorqueFree.RotorsOffAndDragOffConservesEnergyAndAngularMomentum`
5. `QuadrotorStepRefinement.HalvingTheStepConvergesAtFourthOrderOverAManoeuvre`
6. `QuadrotorCrossImplementation.ReproducesTheSouxmarOpenLoopTrajectory`

Case 4 is held to the kernel's own existing bound rather than a new one, so it proves the
quadrotor adds no dissipation of its own. Case 5 states its budget as a convergence ORDER
rather than a magnitude, because the order is a property of the scheme and a magnitude is a
number somebody chose; it reports its absolute agreement alongside as
`position_difference_m_at_1ms` and `attitude_difference_at_1ms`. Case 6 records
`worst_position_m`, `worst_attitude`, `worst_body_rate_rad_s` and `worst_rotor_rad_s`, and
its rotor channel carries the integration-scheme floor the requesting programme quantified,
attributed in the case note rather than absorbed into the budget. Both trajectories are
retained side by side under the run outputs, at the path the case records as
`retained_trajectories`, so a reader can see where the two diverge rather than only how far —
in the build tree, because they are the output of a run and not committed data.

The loader's contract is held separately in the `unit` tier by `QuadrotorFile.*`,
`QuadrotorBattery.*` and
`QuadrotorModel.WrongLengthStateOrCommandIsRefusedRatherThanReinterpreted`. The battery block
is exercised there, with parameters that are the test's own, because the shipped model file
omits it.

**Two findings, raised rather than absorbed.**

*The wind step is a discontinuity in this repository's state and not in the fixture's.*
ADR-0002's velocity is air-relative; the requesting programme's is ground velocity. When the
wind changes, the ground velocity is continuous — no force acts at the instant the air mass
changes speed — so the air-relative velocity must jump by exactly minus the wind change.
`Quadrotor::derivative` takes the wind as steady and carries no `-R^T dw/dt` term, so a
caller that steps the wind without re-basing injects the entire wind increment as a
ground-velocity error, permanently and silently. This was found by case 6 failing, and the
first hypothesis — a different quadratic-drag law — was tested and refuted before the real
cause was isolated by a one-step comparison. The header's `WHAT THIS IS NOT` block now names
the limitation, and case 6 re-bases explicitly at the one wind step in its fixture. Gusts and
turbulence stay out of scope until a wind model owns that derivative term.

*The source appeared to publish two different rotor speed ceilings.* Raised as a finding,
and since resolved by the source programme: the ceiling is load-dependent, the nominal figure
being the speed at the pack's nominal voltage and the actual ceiling scaling by terminal
voltage over nominal, which separates the unloaded value from the one in force under hover
draw. `models/souxmar-quad/PROVENANCE.md` now records the mechanism and both figures, and
notes that the model's own battery block cannot reproduce the sag because it models no
current draw. No shipped case distinguishes them; the fixture's largest command is far below
all of them.

**Follow-ups after acceptance.** Case 6's rotor gate was nearly three orders above the
scheme bound while its note claimed one order; the note was right and the gate was wrong, so
the gate was tightened to one order above the bound the requesting programme published. It is
deliberately not set just above the measured agreement, which is far tighter: a budget drawn
from an observed value is a regression lock, and charter rule 8 requires a lock to be labelled
as one.

**The correction the acceptance section committed WP1 to making.**
`include/galata/core/state.hpp` claimed the thirteen-component order was "the row and column
order of every A and B matrix galata produces". ADR-0002 says it is not, and ADR-0002 is
right. The header now agrees with the record it is governed by.

### WP2 — `trim.hover` and `linearize.extended`, 2026-09-08

Delivered in the closure order the request set: contracts, trim, linearisation,
pipeline, acceptance, compatibility. WP3 to WP5 are not started and the status
line at the top of this document is unchanged.

**Three contracts were closed before any implementation was finished**, because
each decides what the code can be rather than how it is written.

*Wind perturbations are taken at fixed GROUND velocity.* ADR-0002's velocity is
air-relative and the plant's drag acts on it directly, so perturbing the wind
while holding the state fixed moves nothing: the wind columns of B reduce to the
position rows and D is exactly zero. That is self-consistent and it is a
linearisation of a vehicle nothing blows on, and it cannot produce the
wind-to-specific-force drag feedthrough this RFC asks D to carry. The physical
perturbation is the other one — no force acts at the instant the air mass
changes speed, so the ground velocity is continuous and the air-relative
velocity jumps by minus the wind change, which is the same re-basing WP1's first
finding identified and case 6 already performs. Read the chart's velocity
coordinate as the body-axis ground-velocity perturbation and the two halves add
up exactly: at fixed wind the two readings coincide, so A is untouched, and the
wind column contributes precisely the drag term A cannot see. A caller who
wants the other convention declines the wind offset and gets a zero D.

*Frozen state of charge is declared, not discovered.* A powered battery is
always discharging, so a point that is a perfect equilibrium in all six dynamic
coordinates still fails an equilibrium test that reads the battery row.
Excluding it silently would leave a constant term in that row which A cannot
represent and which nothing would report. `frozen_appended_states` names such
coordinates: the state is KEPT in the chart and its own derivative is declared
zero rather than measured. The rate that was declared away is reported so the
reader can divide it into the state's range and see the horizon over which the
freeze is defensible. The trim makes the same choice for the same reason, and
its exported evidence now carries its own `frozen_states` marker rather than
leaving the exclusion to be inferred from the presence of a battery block.

**What the freeze does and does not buy, corrected.** An earlier draft of this
section, and of `include/galata/linearize/extended.hpp`, said the declaration
"makes the linearisation exact rather than approximate". That was wrong and is
withdrawn. Declaring a rate zero changes WHICH SYSTEM IS LINEARISED — the
modified plant whose frozen rows vanish identically — and nothing else. Every
other row remains a finite-difference approximation carrying the truncation and
cancellation error the two findings below describe; the freeze removes a
constant term from one row and buys no accuracy anywhere. Nor does the result
reproduce the discharging plant: the real pack's charge falls, its speed ceiling
falls with terminal voltage, and the true trajectory departs from this model's.
The matrices are valid over a horizon short against that departure, which is
what the reported rate exists to let a reader compute.

*ADR-0013's channel cap is resolved, at 32.* The open question this RFC raised
was real: the fixed-voltage quadrotor's hover linearisation is exactly sixteen
states, so it sat on the cap, and the seventeen-state battery variant sat one
over and lost the typed linear-graph path for one state. The new figure is
derived rather than chosen — a lowered state row carries `n + m` terms and
`kMaxLinearTerms` caps a row at 64, so half of 64 is the largest cap under which
every admissible channel combination still produces a row the executor accepts.
Nothing else in ADR-0013 moves. The record carries the amendment, and the
boundary test now exercises 32 admitted and 33 refused; it also asserts the
constant against the record's own figure, so raising the cap without amending
the decision fails rather than passing quietly.

**One INTENTIONAL INTERFACE DEVIATION from the request, recorded rather than
left to be noticed.** This RFC asked for "a model-generic `linearize.finitediff`"
— that is the heading of the WP2 request above, and it asks for an existing
capability to be extended in place. What was delivered is a NEW capability
called `linearize.extended`, registered beside `linearize.finitediff`, which is
untouched. The deviation is deliberate and it is the same decision the
acceptance section already made under a different name: acceptance condition (c)
required the attitude-error chart to land beside the Euler path "rather than
changing what `linearize.finitediff` exports for the NT-33A". Extending
`linearize.finitediff` in place would have put two charts, two state orders and
two singularity stories behind one capability name, and a caller's meaning would
then have depended on which model it was handed — the failure mode being an
NT-33A study that silently changes its exported state order. Two names cost a
row in the capability table; one name would have cost the ability to say what a
study did. The condition and the interface follow from each other, and the
delivery satisfies the condition rather than the heading.

**How a user selects the new path.** By capability name in the pipeline stage —
there is no mode flag, no model sniffing and no default that changes under a
caller:

```yaml
- id: hover
  capability: trim.hover            # the multirotor equilibrium
  input: {quadrotor: {from: plant}}
- id: linear
  capability: linearize.extended    # the attitude-error chart
  input:
    trim: {from: hover}
    freeze_battery: true            # names the frozen coordinate explicitly
    evidence_path: operating-point.yaml
```

`linearize.finitediff` continues to mean exactly what it meant before this PR,
for exactly the models it accepted before it. A fixed-wing study is unaffected
by WP2 in every respect, which is checked rather than asserted: the NT-33A
generated artefacts all pass their `--check` mode unchanged, and
`QuadrotorWorkflow.TheExistingStateSpaceFilesStillLoadUnchanged` reloads every
state-space file already in the tree.

**What exists now.** `include/galata/trim/hover.hpp` and `src/trim/hover.cpp`
solve the multirotor equilibrium over roll, pitch and four rotor speeds against
the six dynamic accelerations, with yaw declared rather than solved because a
multirotor in still air is in equilibrium at every heading. The position rate is
not required to vanish, which is what admits cruise as a relative equilibrium.
`include/galata/linearize/extended.hpp` and `src/linearize/extended.cpp`
linearise any `f(x_ext, u)` on a multiplicative attitude-error chart, landing
BESIDE `include/galata/linearize/finite_difference.hpp` rather than replacing it: that routine is
untouched and every NT-33A generated artefact passes its `--check` mode
unchanged. `trim.hover`, `linearize.extended` and `model.linear.export` register
in the pipeline as implemented-unvalidated, for the reason `model.quadrotor`
carries, and the README's capability table is regenerated.
`model::serialize_linear_system` writes the same named-matrix YAML
`model.linear.statespace` reads, at `max_digits10` in the classic locale, so a
system galata computed and a system galata was given are the same kind of
object.

Every figure below is one a generated document or a test's own recorded property
carries. None is typed here.

**Acceptance, by test name, all in the `validation` tier unless marked.** Each
budget is stated in the test before the number it gates.

1. `QuadrotorHoverTrim.StillAirCrosswindCruiseAndUnequalRotorsSolveToTheirDeclaredBudget`
2. `QuadrotorHoverLinearisation.PoleStructureIsSixIntegratorsThreeDragPairsAndFourRotorLags`
3. `QuadrotorHoverLinearisation.CollectiveVerticalGainMatchesTheClosedFormAndActsUpward`
4. `QuadrotorHoverLinearisation.WindColumnsCarryTheDragFeedthroughAtFixedGroundVelocity`
5. `QuadrotorHoverLinearisation.LinearAndNonlinearAgreeWithinTheSecondOrderBoundOverOneSecond`

Case 2 finds the six integrators the request asked for — three because nothing
reads position, three because nothing reads attitude at a level hover — beside
three translational and three rotational drag rates and four rotor lags at
`-1/tau`. Case 3 gates the collective vertical gain on its SIGN as well as its
magnitude `8 k_T omega_h / m`, because a model with the sign inverted hovers,
trims and produces a plausible pole map while climbing when commanded to
descend; it also asserts the gain is NOT in B, since the command reaches the
airframe only through the rotor lag, and that the accelerometer row of C agrees
with the state row of A about the same slope. Case 5 declares its perturbations,
its horizon and its budget before it compares anything, and reports the
agreement separately rather than folding it into the gate.

**The strongest evidence is the sixth case, and it is a cross-check rather than a
validation.** `QuadrotorHoverLinearisation.MatricesAgreeWithTheIndependentSouxmarExport`
reads the requesting programme's own exported model — the one this document's
opening section says `model.linear.statespace` consumes unchanged — through the
shipped loader, and compares it entry by entry against what `linearize.extended`
computes. It compares `compared_entries` entries of A, B, C and D and reports
their `worst_relative_disagreement` against a `relative_budget` derived, before
any comparison, from the two implementations' independent finite-difference
errors. **Those figures are recorded properties of the run and are deliberately
not restated here** — an earlier draft typed two of them into this paragraph,
which is exactly the mistake charter rule 2 exists to prevent, since a number
copied out of a run is a number no later run can contradict. They are read from
the retained evidence named below, which carries them alongside the worst
absolute disagreement on the structural zeros, the fixture path and the
fixture's SHA-256.

Agreement between two implementations is not validation
and the registry records it as self-consistent, but this is the only case here
that could catch a shared mistake in galata's own reasoning about the chart,
because the other implementation trims and linearises in ENU/FLU with its own
code and reaches these conventions by an explicit similarity.

It is also the independent check on the coordinate contract, which is why it
matters more than its status suggests. The other programme names its velocity
states `ground_v_*` and its export carries the wind-to-specific-force
feedthrough in D, a zero D block against its own ground-velocity outputs, and
zero wind columns in its position rows — the same four blocks this document's
first contract predicts. Had galata taken the wind at fixed air-relative
velocity, three of them would be zero where the reference is not and one would
be nonzero where the reference is zero. The fixture is not committed: ADR-0007
routes it to a path plus regeneration instructions, and the case states why it
did not run when the path is absent.

**BOTH CROSS-CHECKS SKIPPED IN CI, AND A READER OF A GREEN RUN MUST NOT READ
THEM AS HAVING PASSED THERE.** `GALATA_SOUXMAR_FIXTURE_DIR` is set by no
workflow, so on every hosted job
`QuadrotorHoverLinearisation.MatricesAgreeWithTheIndependentSouxmarExport` and
`QuadrotorCrossImplementation.ReproducesTheSouxmarOpenLoopTrajectory` report
`Skipped`, not `Passed`. This is a consequence of ADR-0007 rather than an
oversight — the fixture's rights position is unestablished, so it is not
committed and CI has nothing to point at — but it means the single strongest
piece of WP2 evidence, and the altitude finding below, are established by a
CONFIGURED LOCAL RUN and by nothing else. The consequence for the two locks
below is concrete: a future change on either side of the altitude convention
will not turn any CI job red. Someone must run with the fixture configured.

The run that establishes them, recorded so it can be repeated and contradicted.
**The revision actually tested is `7dd36f7`** — the branch head, not the merge
commit — and the record says so rather than quietly attributing the run to a
commit that did not exist when it was made:

```text
revision  7dd36f7e1650af58551838e69e460730a2de0f71   (branch head; the tested tree)
platform  Darwin arm64, AppleClang 21.0.0.21000099, CMAKE_BUILD_TYPE=Debug
fixture   /Users/celikgo/souxmar/outputs/galata_bridge
          quad_hover_ned_frd.yaml
            sha256 ff99b38542b4308c6448c7b7cc3986f8eab0b1bc17c9ee5c4461c4b5563e2f4d
          reference_trajectory.csv
            sha256 b0329175e8ff1330ca7334827beff5f1225a3830a05f7cd60804d2fe72b75e85
result    both cases passed
evidence  build/dev/souxmar-cross-check/cross-check-7dd36f7.xml
            sha256 a21a0a6baa34dbb7d2be6f03bac2d09424abb915b6b177e047e07f91d2ae10dc
          gtest XML, retained in the build tree and NOT committed — it is the
          output of a run, and ADR-0007's reasoning about the fixture applies to
          it too. The digest is what makes it citable: the file is named after
          the revision it tested, and the digest says which bytes carry that name.

cmake -S . -B build/dev -DGALATA_SOUXMAR_FIXTURE_DIR=<fixture dir>
cmake --build build/dev --target galata_validation_tests
./build/dev/tests/validation/galata_validation_tests \
  --gtest_filter='*Souxmar*' \
  --gtest_output=xml:build/dev/souxmar-cross-check/cross-check-7dd36f7.xml
```

**The merge commit is recorded separately, and its equivalence to the tested
revision is verified rather than asserted:**

```text
merge     https://github.com/celikgo/galata/commit/7ad9ff6d095046881775254e0066d3ed3544dabb
tree      identical — `git diff --quiet 7ad9ff6 7dd36f7` is empty; both trees hash to 3dadaffde956c5e48f9d50df2c77af30d4916359
```

A squash merge rewrites the commit but need not rewrite the tree, and "need not"
is not "does not": a maintainer can amend during merge, and a conflict
resolution changes content while preserving the appearance of a fast-forward.
The tested revision is therefore stated first and on its own, and the merge
commit inherits its evidence only for as long as the diff above stays empty. If
it is ever non-empty, the merge commit is untested by this record and the run
must be repeated against it.

**An earlier draft of this block cited `7d424ab`, and that citation could not
have been sound.** `7d424ab` is an ancestor of `7dd36f7`, but six commits
followed it, and two of them changed what the retained XML would contain.
`cc18b0c` introduced `fixture_sha256` itself — so a run at `7d424ab` emitted no
such property, and this paragraph's own claim that the XML identifies its bytes
was describing an artefact that did not exist. `0586fc1` rewrote the
`altitude_row_disagreement` property string, so that run's XML would carry
superseded wording for the one entry the altitude lock exists to hold. The
digests were right and the fixture bytes never moved; the revision that consumed
them was stale, which is a different defect and a quieter one.

Nothing numerical moved between the two revisions, and that was checked rather
than assumed: the only change after `7d424ab` reaching the compared path is
`a631737`'s `trim.battery_state_of_charge_frozen = model.has_battery()`, a
metadata field assigned after the solve that enters no residual, no matrix and
no compared entry. The rest was prose, a failure message and a recorded-property
string. So the re-run confirms rather than revises. **It was still necessary**,
because an evidence record whose cited artefact cannot carry the properties the
prose quotes is not evidence, however right its numbers turn out to be — which
is the same argument, turned on this record, that `cc18b0c` made about paths.

Both digests are recorded by the cases themselves as `fixture_sha256`, so the
XML identifies the bytes it read rather than only the path it read them from —
an uncommitted fixture at a stable path is not an identification, and two runs
citing that path can have consumed different files. Every measured figure quoted
by name above is read from that XML.

**Hosted CI, and what it does and does not cover.** The branch merged green:

```text
run      https://github.com/celikgo/galata/actions/runs/34307091050
head     7dd36f7e1650af58551838e69e460730a2de0f71   (the revision CI tested)
merge    https://github.com/celikgo/galata/commit/7ad9ff6d095046881775254e0066d3ed3544dabb                        (tree equality verified above)
result   success — 11 checks, 0 failed
         Charter gates · Format · Engine (linux-gcc, linux-clang, macos)
         Determinism (Fingerprint linux, Fingerprint macos, Tier 2)
         Clang static analyzer · ASan/UBSan
tests    512/512 passed on linux-gcc and on linux-clang, 544/544 on macos
         (macos is higher only because ConnectionEditing, DiagramEditing and
         DiagramRouting build there and nowhere else)
```

CI and the fixture run tested the SAME revision, `7dd36f7`, by different means
and over disjoint sets of cases. That is what makes the two records comparable;
it is not what makes either cover the other.

**The green run does not cover the two cross-checks above, and the count says
so.** `512 passed` is `512 of 512 that RAN`; both Souxmar cases report `Skipped`
on every hosted job, for the reason recorded above, and a skip is not a pass.
The two bodies of evidence are disjoint: hosted CI establishes everything except
the two cases that carry the coordinate contract and the altitude lock, and the
configured local run at `7dd36f7` establishes exactly those two and nothing
else. Neither substitutes for the other. A reader who takes the green badge as
covering WP2 in full has read it wrong, and the concrete consequence is the one
already stated: a future change on either side of the altitude convention turns
no CI job red.

Five generated artefacts are regenerated and diffed by that run, all on
`Engine (linux-gcc)`, which is why they read `skipped` on the other two legs —
a matrix condition, not an absent check: `docs/VERIFICATION.md`, the README
capability table, `docs/assets/modal-map.json`, `docs/assets/nt33a-fc1-run.json`
and `docs/reports/nt33a-fc1.html`. A sixth committed generated artefact,
`docs/assets/social-preview.svg`/`.png`, is gated by nothing; that gap predates
this work and is tracked at issue #14.

The contracts are held in the `unit` tier by `HoverTrim.*` and
`ExtendedLinearize.*`, which cover the refusals — an over-actuated vehicle, an
infeasible trim, a non-equilibrium point, a malformed name list, a frozen index
nobody has — the five declared observations, and the one claim the chart exists
to make, that it is regular at ninety degrees of pitch where the Euler chart is
not. `QuadrotorWorkflow.*` in the `integration` tier runs the chain end to end,
reads the export back through the existing loader and finds the same matrices
bit for bit, lowers the seventeen-state battery variant through the typed graph
adapter, and confirms every state-space file already in the tree still loads and
still round-trips. `Determinism.HoverTrimAndItsLinearisationAreBitIdenticalAcrossRuns`
holds ADR-0004 tier 1 over both new routines and over the exported bytes.

**Four findings, raised rather than absorbed.**

*The hover translational entries carry a FIRST-order finite-difference error,
not a second-order one, and the Richardson estimate cannot see it.* The
quadratic drag term `c_q v |v|` is continuous and once differentiable at zero
airspeed and not twice, so at hover the central difference straddles a kink.
Working the quotient out by hand gives `-c_l - c_q h` exactly, so the relative
error is `h / (drag_linear / drag_quadratic)` — the step divided by the very
quadratic-drag airspeed the model publishes. The pole gate is derived from that
mechanism rather than from the truncation estimate, which looks healthy. This is
the case `include/galata/numerics/jacobian.hpp` warns about in the abstract, met in the
concrete.

*The chart destroyed the shared Jacobian's relative-step rule, and it cost eight
digits.* Every chart coordinate is zero at the nominal by construction, so
`max(relative_step * |x_i|, absolute_step)` collapses to the one absolute floor
for every column. That floor is sized for a component of order one; a rotor
speed is several hundred radians per second, and perturbing it by six parts in a
million and subtracting leaves about half the mantissa. The rotor-lag entries
were wrong in the eighth figure until the floors were derived from the magnitude
of the state each coordinate perturbs. It was a pole gate that caught this, not
the truncation estimate, and the fix improved the collective-gain agreement by
two orders as well.

*The first attempt at case 5 named only the quadratic drag and gated on it, and
the vertical channel failed at 42 percent — correctly.* Quadratic drag is not
the largest thing a hover linearisation drops: the gravity and thrust projection
are exact in the attitude while the linearisation keeps only the first-order
tilt, and the Coriolis term is a product of two perturbed quantities and
therefore entirely second order. The budget now sums all three, doubled once and
for a stated reason. The case also normalises each channel by its coordinate
group's perturbation scale rather than by its own peak excursion, because a
channel whose response is small — vertical velocity at hover is driven only by
drag decay — would otherwise be held to a bound thousands of times tighter than
the mechanism that limits it.

*The reference's altitude channel reports the down coordinate, not altitude.*
Its tenth output is named `altitude_down_m` and its C row selects `+1` on the NED
down state; galata's `OutputKind::Altitude` is documented as positive up, which
is `-1`. Both are internally consistent — they are different quantities under
similar names, and the other programme's name has `down` in it.

**NEITHER SIDE CHANGES.** galata does not: altitude positive up is the ordinary
meaning of the word, the observation model's header states it, and ADR-0002's
down axis points down. Absorbing a factor of minus one into a numerical budget
would be absorbing a sign error, which is the one thing a budget must never hide.

And the reference is not asked to change either, which corrects how an earlier
draft of this section read. It described a future sign flip on the Souxmar side
as a "fix" that would let this exclusion be deleted. It would not be a fix. That
row's sign is not a defect: `altitude_down_m` reports what its name declares.
Flipping it would leave the file loading exactly as it does today — same schema,
same shape, same channel name, same round-trip through
`model.linear.statespace` — while reversing what the channel MEANS for every
consumer already reading it. **That is a semantic compatibility break, and the
preserved loadability is what makes it dangerous**, because no loader, schema
check or round-trip test in either programme would report anything. Should the
two ever want a single convention, it is a coordinated migration under a renamed
channel, planned separately from this work and from this PR.

So the mapping is DOCUMENTED rather than reconciled, and it is the mapping the
bridge runs on today:

| position | Souxmar | galata |
|---|---|---|
| output 10 (index 9) | `altitude_down_m`, C row `+1` on the NED down state — down position, positive downward | `altitude_m`, C row `-1` on the same state — altitude, positive upward |

Charter rule 3 applies: the row is excluded from the bulk comparison and held by
a two-sided check instead. It fails if the two rows stop being exact negatives,
and it fails if the reference's entry stops being `+1` — the second side now
reading as a compatibility alarm rather than as a fix detector. A change on
either side is loud rather than silent, subject to the CI-skip caveat recorded
above: loud in a configured local run, silent in CI. This is the one entry that
disagrees, and the count and its budget are read from the retained evidence.

**What WP2 deliberately did not do.** `linearize.finitediff` is unchanged and
the NT-33A export is unchanged; the two charts coexist, which is the condition
this RFC's acceptance section set. The fixed-wing mode labels are not applied to
a multirotor — the integration study runs `analyze.modes` with `classify: false`
— and `analyze.modes` correctly continues to decline participation factors at
hover, because the double-integrator chains are defective. `model.channels` can
select or drop the wind columns because they are named, but no case here
exercises that path. Gusts and turbulence stay out of scope until a wind model
owns the `-R^T dw/dt` term, exactly as WP1 left them.

### WP3 — measured-data import, 2026-09-10

`data.import.csv` and `data.import.ulog` register as implemented-unvalidated, with
`data.window` beside them. `include/galata/data/record.hpp` is the contract every fitting
capability reads: a channel carries the unit and the frame a human wrote down, plus what the
source held before conversion and the scale and offset applied, so a coefficient can be traced
back to the exact bytes that produced it. The readers refuse rather than repair — an unmapped
and unignored column, a missing unit, a missing frame, a non-monotonic timestamp, a ragged row
— and what they had to drop is counted and reported rather than absorbed. ADR-0016 records why
ULog is parsed in-repo rather than by a dependency.

**One capability the request did not ask for.** `data.window` cuts a record to a half-open
interval of itself, keeping the file's identity and adding the interval. It exists because of a
defect WP4 found in its own held-out semantics, described below; it is not a convenience.

**Acceptance** is held in the `unit` tier by `CsvImport.*` and `UlogImport.*`, and the ULog
fixture is generated from `tests/data/make_ulog_fixture.py` rather than committed, because a
binary blob nobody can read is a fixture nobody can check.

**What WP3 does not deliver.** No frame conversion is performed anywhere: PX4's NED and FRD
already match ADR-0002, and a record in FLU stays in FLU and says so. There is no smoothing,
no interpolation on demand and no gap repair, by design.

### WP4 — identification, 2026-09-10

`identify.static_fit`, `identify.greybox` and `identify.validate` register as
implemented-unvalidated, with `model.quadrotor.export` beside them, and
`examples/quadrotor-identification/` runs the whole path through public capabilities.

**The grey box is grey.** `identify.greybox` fits a DECLARED subset of `model::Quadrotor`'s
parameters by simulating that same plant through the same `Quadrotor::derivative` and
`numerics::integrate_fixed_step` pair `sim.plant` uses, so a fit and a simulation cannot
disagree about what the model does. A parameter the study did not name is held at the value the
model file gave it, and the exported provenance lists every such parameter BY NAME so that
"unfitted parameters are preserved" is a checkable statement rather than a promise.

**Three things kept apart, because ADR-0008 requires them to be separable.** The optimiser
finished; the parameters are identifiable; the fit is acceptable. The first is nearly
worthless on its own — the iteration count is declared, so it is true even of a run that moved
nothing — which is why `objective_improved` and `accepted_steps` are reported beside it and
carried in the headline. The second is a REFUSAL: a parameter direction the Jacobian cannot see
is a parameter this record did not measure, and the run stops rather than reporting where the
optimiser happened to stop. The third is not made here and is not implied.

**ADR-0018 records the fitted-model artefact decision**, which the request did not anticipate
and without which the fit produced numbers nothing downstream could consume. A fit emits the
same `quadrotor` artefact kind `model.quadrotor` emits, so `trim.hover`, `linearize.extended`
and `sim.plant` take it with no adapter and no special case; `model.quadrotor.export` writes it
as a model file the loader reads back, with a required evidence file carrying the base-model
identity, the estimation-record identity, every fitted parameter with its unit, bounds and
standard error, the objective in words, and the optimiser diagnostics. The base model is never
rewritten, and that is enforced by the executor rather than promised by the capability: the
model file is a recorded run input and `RunFiles::check_input_collision` refuses any output
that would overwrite one.

**A DEFECT IN THE FIRST HELD-OUT SEMANTICS, FOUND AND CORRECTED HERE.** The first version of
`identify.validate` set a boolean `is_held_out` to `validation_digest != estimation_digest`.
The negative half of that is sound — the same bytes are the same observations — and the
positive half is not, because a digest identifies BYTES and held-out is a claim about DATA. A
file reformatted, exported twice, or written to a different number of decimal places has a
different digest and the same observations in it; a segment copied between two files shares
every sample it copied; two exports of one flight at two sample rates describe the same seconds
of the same aircraft. Every one of those would have passed as validation, and each is a model
being scored on its own training data under a label saying otherwise.

Independence is now classified, and the classification says on what grounds.
`VerifiedDisjoint` is reserved for the one case this code can prove — two windows of ONE
imported file over intervals that do not meet, which is why `data.window` exists — and is
confirmed by scanning the shared channels for a sample instant occurring in both.
`CallerDeclared` is what an honest study gets when it knows two files are different flights and
galata cannot check that: recorded as the caller's claim, in the caller's name, and downgraded
the moment a check contradicts it. `Unknown` is what silence gets. `NotHeldOut` beats every
positive branch, so a caller cannot declare away a proof of overlap. And `identify.validate`
now reads the estimation record's identity out of the fitted model's own provenance rather than
off a string the study typed, refusing a study that contradicts it, and refusing a record wired
in as the training data whose lineage is not the lineage the fit recorded — which a digest
comparison could not have caught, since a window keeps its parent's digest.

**Acceptance, by test name.** `unit`: `StaticFit.*`, `Greybox.*`, `Validate.*` and
`Independence.*` — the last being the eight cases the old boolean could not have had, each one
a pair of records whose digest comparison gives the wrong answer. `QuadrotorSerialisation.*`
holds the model file round trip bit for bit, and `RecordWindow.*` the half-open interval and
the surviving source identity. `integration`: `IdentifyWorkflow.*` holds the strict schemas,
the durable export, the refused overwrite of the source model and the provenance-driven
identity; `ExampleQuadrotorIdentification.*` runs the shipped example and compares the
recovered parameters against the committed truth model. `determinism`:
`Determinism.AGreyboxFitIsBitIdenticalAcrossRuns` holds ADR-0004 tier 1 over the longest
floating-point chain in the tree, and over the exported bytes as well as the parameters.

**What WP4 does not deliver, stated plainly.** The synthetic self-test is noiseless to
round-off, so it demonstrates that the machinery recovers a parameter it can SEE and nothing
about behaviour against a sensor. No record here was measured; no bench produced any
coefficient; there is no aircraft. The reported standard errors on that record are correct and
useless in magnitude — they say the record demonstrates no scatter. The request's ULog topic
list is imported but no PX4 log from the built aircraft exists to import, so the physical
half of WP3's and WP4's verification is outstanding and is blocked on hardware rather than on
software.

**One gate was silently skipping this whole vertical.** `scripts/check-si-boundary.sh` listed
`src/ident` and `include/galata/ident` among the numerical-core directories; the code landed at
`src/identify` and `include/galata/identify`, so the gate reported them as "not yet present,
skipped" while they existed, and passed. The list now names the real directories. A gate that
skips is not a gate that passes, and this one said so in its own output without anybody reading
it.

### WP5 — sampled control and time-varying inputs, 2026-09-10

`sim.plant`, `sim.sampled` and the declared input histories register as
implemented-unvalidated, under vehicle-neutral names as the acceptance section decided, with
the quadrotor as the case that had to pass.

**`sim.plant` closed a gap the audit called the most basic one.** Before it, the nonlinear
multirotor was reachable only from C++: this repository could integrate a quadrotor in its own
tests and a user could not integrate one at all. `sim.nonlinear` is the fixed-wing path — it
takes a `trim.level` point and actuators named elevator, aileron, rudder and thrust — and is
untouched.

**Declared input histories, for both the linear and the nonlinear path.** A command or wind
history is a declared sample list with a stated hold and a stated extrapolation rule, and the
recorded trajectory carries the command and wind AT EACH SAMPLE rather than the constants the
run was configured with, because a schedule makes those constants a half-truth. A discontinuity
strictly inside an integration step is refused rather than rounded to the nearest step, which
would move the event and say nothing about having done so.

**`sim.sampled` executes a state-feedback law at a declared rate**, with zero-order hold, a
whole number of periods of delay, and per-rotor saturation, feeding back on the attitude-error
chart coordinates ADR-0017 made publicly invertible. Requested and applied commands are both
recorded: a run reported only through its applied commands hides a controller that spent the
whole run against its limits, and one reported only through its requested commands describes a
vehicle that was never flown. A controller period that is not a whole number of integration
steps is refused rather than rounded, because unlike a wind step it cannot be split around —
the tick is where the command is DEFINED to change.

**Acceptance, by test name.** `unit`: `InputSchedule.*` and `ChartMapping.*`. `integration`:
`QuadrotorWorkflow.*` re-derives the sampled logic — the law, the saturation and the delay line
— from the states the run recorded, so a loop that sampled at the wrong instant or shifted its
delay by one tick fails there rather than passing on a plausible trajectory;
`ExampleQuadrotorSampledControl.*` runs the shipped example and requires the closed loop to
remove at least nine tenths of the displacement it was started from.

**The companion the request asked for, delivered.** `analyze.gramians` reports the reachable
and observable subspaces of a linear model for a declared input and output set, and names the
directions that fall outside them by state — which is what the request asked for in one
sentence: "so that an exported model's defective integrator chains and any unobservable
direction are reported rather than discovered from a failed synthesis". On the hover
linearisation it finds the heading unobservable, because the observation model carries body
rates, position, altitude, ground velocity and specific force and none of them measures an
absolute yaw angle.

Two things about it are worth recording rather than leaving in the header.
*The infinite-horizon Gramians do not exist for this class of model at all.* Six eigenvalues
sit at the origin, so the T-to-infinity limit diverges and the matrix a Lyapunov solve would
return for it is not a Gramian of anything. The capability integrates over a DECLARED finite
horizon instead, refuses to default that horizon, and says in its own summary and report which
quantity it computed. *The rank is taken from an orthogonal staircase and not from a Krylov
matrix.* `[B, AB, ..., A^15 B]` on a model with rotor-lag poles near -1/tau and integrators at
zero spans the fifteenth power of that spread, which no floating-point rank test can resolve;
re-orthonormalising at each step forms no power of A at all. `Gramians.*` in the `unit` tier
holds the arithmetic against closed forms — a first-order system's exponential integral, a
double integrator's `[T^3/3, T^2/2; T^2/2, T]` — rather than against a previous run.

**And the frequency-domain margins the audit found unavailable are now available, without
widening anything.** The audit recorded `analyze.margins` and `analyze.diskmargin` refusing on
this plant with "internal stability unresolved: eigenvectors are ill-conditioned", and the
refusal was CORRECT. Handing one channel of the MIMO return ratio `L(s) = K(sI-A)^-1 B` to a
SISO margin routine breaks that channel and leaves the other three OPEN — a vehicle flying with
most of its controller disconnected. A multirotor at hover needs all four, so that closure
leaves four modes at the origin, its Nyquist encirclement count means nothing, and refusing was
the only honest answer. Nothing about the check was wrong and nothing in it was relaxed.

What was missing was the OTHER reading. `model.control_system` gains `use: single_loop` with a
required `channel`, which builds the loop seen at one plant input with the other loops still
CLOSED: `A_k = A - BK + b_k k_k^T`, so that closing unit negative feedback around it returns
exactly `A - BK`, the design's own closed loop. Internal stability of the Nyquist test is then
the stability of the design, which an LQR solution guarantees, and the margin is well posed.
`synth::single_loop_others_closed` checks that identity to round-off rather than asserting it,
because if it ever stopped holding then every margin read from that system would be a margin of
a different aircraft. `channel` is required and not defaulted: there is one such loop per input,
they are different loops with different margins, and picking one for the caller would be
choosing which number to report.

`QuadrotorWorkflow.SingleLoopMarginsAreAvailableWhereTheBrokenLoopIsRefused` holds both halves —
the other-loops-open reading refused with its cause named, and the loop-at-a-time reading
yielding a margin — and `examples/quadrotor-sampled-control` now runs both analyses in the
shipped study.

**What WP5 still does not deliver.** A set of loop-at-a-time margins does NOT bound simultaneous
variation: each can be generous while a small perturbation applied to two channels at once
destabilises the loop, and no loop-at-a-time figure sees it. `analyze.diskmargin` is the
capability for that question and the headers say so where the confusion would occur.
`c2d`-style discretisation of a continuous design and a sampled DARE remain absent:
`sim.sampled` executes a continuously-designed law at a discrete rate, which is the honest
description of what it does and is not the same thing as designing in discrete time. And the
SAMPLED loop's own robustness is unanswered — every margin in this repository describes the
continuous loop, and no claim of sampled-loop robustness is made anywhere from one. Per
`docs/product/FEATURES.md`, this implements the proposed work of F14 and part of F17 and
delivers neither row; both stay open.
