<!-- SPDX-License-Identifier: Apache-2.0 -->

# RFC-0002: Quadrotor programme support — native plant, hover trim, generic linearisation, measured-data import, identification, sampled control

- **Status:** proposed. Nothing below is implemented; the README's Status table remains the authority.
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
