# ADR-0021: A fixed-step implicit integrator does not weaken the determinism policy

<!-- SPDX-License-Identifier: Apache-2.0 -->

- **Status:** accepted
- **Date:** 2026-09-16
- **Relates to:** ADR-0004, which it clarifies and does not amend.

## Context

ADR-0004 makes fixed-step RK4 the only integrator used for gated results, and
gives the reason: an **adaptive** method chooses its step sequence from an error
estimate, which is a function of the last bits of the state, so two runs
differing by one ulp can take different numbers of steps and diverge visibly.

`include/galata/numerics/integrator.hpp` also records the cost of that choice. RK4's stability
region on the negative real axis reaches `hλ ≈ −2.78`, so a mode at 100 rad/s
needs `h < 28 ms` merely to remain stable, before accuracy is considered, and
"a system with modes separated by more than about three decades will be
impractical rather than merely slow."

A Level-1 helicopter is such a system. It carries rigid-body modes at
O(0.1–1) rad/s, rotor speed and governor at O(1–10), dynamic inflow and
actuators at O(10–100) and blade flapping at O(100). The readiness audit
measured the consequence directly: at λ = −1000, RK4 at `h = 5 ms` returned
−2.49 × 10²²⁷ instead of 0.54, **and returned it without complaint**, because the
value was finite.

## Decision

Two things, separately.

**1. The prohibition in ADR-0004 is on adaptive stepping, not on implicit
methods.** A method with a **fixed step** and a **fixed iteration count** performs
exactly the same arithmetic in the same order on every run. It is not adaptive,
and it is admitted. `include/galata/numerics/integration_method.hpp` adds implicit Euler
(first order, A- and L-stable) and the trapezoidal rule (second order, A-stable),
each solved by a simplified Newton iteration with a **count, not a tolerance**.
There is no residual test and no early exit anywhere in that file.

Fixed-step RK4 remains the **default** and the only method used for gated
results unless a study declares otherwise. `IntegrationMethod::Rk4Fixed`
delegates to `rk4_step` unchanged, so a study that does not choose another method
gets the same bits it got before this ADR, and a test asserts that equality
exactly rather than to a tolerance.

**2. A finite state is not a valid state, and simulation now says so.**
`numerics::StateBounds` carries a declared magnitude per state, checked after
every completed step, and terminates the run with the **offending state named**.
A default-constructed `StateBounds` checks finiteness only, which is exactly the
old behaviour, so no existing caller changes. `TerminationReason` reaches the
capability summary, the Markdown report and the run manifest, so a diverged run
cannot be read as a completed one from any of the three.

## Consequences

A study declares its integrator. A rotorcraft model whose flap and inflow lags
put it outside RK4's practical range can select an A-stable method and choose its
step for accuracy instead of for stability.

The cost is visible rather than hidden: an implicit step forms a numerical
Jacobian, costing *n* extra derivative evaluations for an *n*-state model, and
`IntegrationResult::newton_iterations_performed` reports the total so the price of
the method choice is in the result.

Neither implicit method estimates its own error. `step_size_study()` remains the
only honest error information on offer, and it works on these methods too.

The trapezoidal rule is A-stable but **not** L-stable: its stability function
tends to −1 as `hλ → −∞`, so a very stiff mode rings at the step frequency
instead of decaying. Implicit Euler damps it at the cost of first-order accuracy.
Both are offered because the right answer depends on whether the stiff mode
carries information or only stability, and only the model's author knows that.

## Alternatives rejected

**An adaptive Dormand–Prince or Runge–Kutta–Fehlberg method.** Rejected for
exactly the reason ADR-0004 gives. Nothing in this ADR reopens it.

**A tolerance-based exit on the implicit stage iteration.** Rejected: an exit
condition read off the residual is an exit condition read off the last bits of
the state, which is the same defect as an adaptive step in a smaller place.

**A Rosenbrock method.** Deferred rather than rejected. It would avoid the
Newton iteration entirely at the cost of needing an accurate Jacobian every step,
and the two methods here cover the helicopter's range. Worth revisiting if a
model with a genuinely stiff *nonlinear* stage appears.

**Leaving the divergence detection to the capability layer.** Rejected: the
integrator is the only place that sees every step, and a bound checked once per
reported sample would miss a divergence that grows and returns between samples.
