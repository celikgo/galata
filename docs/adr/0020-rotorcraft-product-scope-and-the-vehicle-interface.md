# ADR-0020: Galata is a rotorcraft flight-dynamics tool, and the vehicle is data

<!-- SPDX-License-Identifier: Apache-2.0 -->

- **Status:** accepted
- **Date:** 2026-09-16
- **Supersedes:** nothing. Narrows the product direction `docs/PRODUCT_PLAN.md`
  left open, and does not alter ADR-0002, ADR-0003, ADR-0004 or ADR-0006.

## Context

The rotorcraft readiness audit of 16 September 2026 found that galata could not
represent a helicopter, and that the obstruction was structural rather than a
matter of missing refinement. Rotor thrust was the fixed body-axis vector
`(0, 0, −k_T ω²)` at `src/model/quadrotor.cpp:355`, so a tail rotor produced a
downward force and a pitching moment instead of a side force and a yaw moment,
and a main rotor with cyclic produced no in-plane force at all. Neither was an
approximation that was missing; both were inexpressible.

The audit also found that the vehicle class was encoded in the **capability
name** and propagated outwards: `sim.nonlinear` against `sim.plant`,
`trim.level` against `trim.hover`, `linearize.finitediff` against
`linearize.extended`, and a chart routine that switched on artifact kind and
named elevator, aileron and rudder in the plotting code. Adding a helicopter
along that grain meant a third parallel family and a third set of hard-coded
charts, with the cost landing again on every downstream capability.

Two product directions were open. One was a general MATLAB-like numerical
environment. The other was a rotorcraft flight-dynamics and control-design tool
with MATLAB-like *usability*. They imply different architectures, different
teams and different competitors, and the codebase committed to neither.

## Decision

**Galata is a deterministic, evidence-driven rotorcraft flight-dynamics and
control-design tool with MATLAB-like engineering usability.**

Full MATLAB syntax, `.m` script execution, `.mat` I/O and Simulink model
compatibility are **out of scope** and will not be built. The usability tier that
*is* in scope is named parameters, parameter sweeps, reusable model
configuration, on-demand plotting and a Python binding over the existing
capability registry — none of which requires MATLAB syntax.

The differentiators below are the product and are not to be traded for
compatibility with anything:

- Deterministic results, bit-identical on a platform (ADR-0004).
- Strict SI in the numerical core, converted only at the boundary (ADR-0003).
- Content-addressed provenance on every run.
- **Refusal** in place of an unsupported best-effort answer.
- Validation budgets derived from the source's printed precision *before* the
  comparison is made.
- Explicit validity envelopes, and a `WHAT THIS IS NOT` block on every physics
  file naming the direction of its known error.
- Reproducible reports, regenerated and diffed by CI.

**And the vehicle becomes data.** `galata::model::VehicleModel` is the interface
that trim, linearisation, simulation, analysis and reporting dispatch through. A
model supplies a wrench, an auxiliary-state rate, mass properties, an envelope
report and its own vocabulary of state, control and output names. It does **not**
supply the composition: `VehicleModel::derivative` is deliberately non-virtual, so
the ADR-0002 state ordering, the gravity resolution and the wind treatment happen
in exactly one place and a model cannot permute the state or drop the wind term.

## Consequences

A new vehicle class costs a model, not a capability family. `linearize.vehicle`,
`analyze.nonlinear_agreement` and the declared trim problem of ADR-0022 work on
any `VehicleModel` without knowing what it is.

`galata::model::Environment` carries `wind_rate_ned_m_s2`, the d*w*/d*t* term
`include/galata/model/quadrotor.hpp` documents as missing and makes the caller own. Carrying it here
removes a whole class of silent error — a caller that steps the wind without
re-basing the state injects the entire wind increment as a ground-velocity error
— and it is the prerequisite for gusts and turbulence.

**Existing models are not yet routed through the interface.** `model::Aircraft`
and `model::Quadrotor` keep their own capability families, and their outputs are
therefore unchanged by construction rather than by verification. That refactor is
outstanding work and is recorded as such rather than claimed.

## Alternatives rejected

**A general MATLAB-like environment.** Rejected on the evidence of the audit: the
assets galata already has — determinism, provenance, refusal semantics,
budget-before-answer validation — are worth a great deal in aviation and defence
engineering and almost nothing in a general numerical computing environment,
where an established free alternative already exists for every one of them.

**Freezing the plugin C ABI of ADR-0001 around `VehicleModel` now.** Rejected: the
interface's shape is still moving. An ABI fixed around it today would fix the
wrong shape. ADR-0001 stands; the ABI wraps this interface once the vehicle set
has stabilised.

**Adding a third capability family for the helicopter.** Rejected as the audit
described: it multiplies the cost across every downstream capability, and it was
already three families' worth of duplication before a helicopter existed.

**A `Quantity<Length>` unit type.** Still rejected, for the cost reason ADR-0003
records. Unit safety continues to rest on the CI boundary gate and on every field
carrying its unit in its name.
