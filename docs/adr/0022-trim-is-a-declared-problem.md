# ADR-0022: Trim is a declared problem, not a hard-coded unknown set

<!-- SPDX-License-Identifier: Apache-2.0 -->

- **Status:** accepted
- **Date:** 2026-09-16
- **Relates to:** ADR-0020. `trim.level` and `trim.hover` are unchanged and
  remain supported.

## Context

Galata had two trim solvers, each with a hard-coded unknown vector.
`include/galata/trim/level.hpp` solves angle of attack, elevator and thrust; `include/galata/trim/hover.hpp`
solves two attitude angles and *N* rotor speeds. The readiness audit's benchmark
attempt stopped on the second one's refusal:

> trim_hover: this solve needs exactly 4 rotors, and this model has 2.

That refusal is sound reasoning about the solver it guards — six equations need
six unknowns, and an over-actuated vehicle is an allocation problem with a null
space rather than a root. It is not a statement about helicopters. A helicopter's
trim unknowns are roll, pitch, collective, longitudinal cyclic, lateral cyclic
and pedal: six for six, a perfectly well-posed problem that neither existing
solver can express at any effort, because neither can express a different unknown
set.

The same rigidity blocked climb, descent, coordinated-turn and autorotation trim,
each of which would otherwise have needed its own solver.

## Decision

A trim is a **declared problem**: a list of `TrimUnknown` (name, initial guess,
bounds, scale), a list of `TrimResidual` (which accelerations or auxiliary rates
must vanish, or a caller-supplied scalar), solved by one Newton on a square
system. Unknowns and residuals are resolved **by name** against the vehicle's own
vocabulary, so a problem declaration can be written once and applied to any model
carrying those names.

Four properties are part of the decision, not implementation detail.

**Rank is checked before the answer is believed.** A square system is not
necessarily a solvable one. The Jacobian's rank and condition number are computed
by SVD and reported whether or not the solve converged, and a rank-deficient
problem is **refused** with the unknowns the residuals do not constrain named —
read off the null-space basis, following the diagnostic pattern
`identify.greybox` already uses for a parameter the data does not constrain.

**Bounds are checked after the solve and a violation is refused, not projected.**
A best effort reported as a trim gets linearised, and a linearisation about a
non-equilibrium carries a constant term the A matrix cannot represent. This is the
same choice `trim_level` and `trim_hover` already make, for the same reason.

**The Jacobian is taken in scaled unknowns.** Each unknown declares a scale and
the derivative is taken with respect to `x/scale`, so a problem mixing an engine
torque of order 10⁴ N·m with attitude angles of order 10⁻¹ rad is not conditioned
by its units. Measured: the Souxmar hover conditioned at 7.4 × 10⁶ before this and
1.9 × 10² after, with an identical answer.

**The step is bounded by a fixed trust region, not a line search.** Newton's first
step from a poor guess can be large enough to carry the iterate through a
saturation, and a saturated unknown has a flat Jacobian column, so the *next*
iteration is rank deficient and a feasible problem is refused. Capping the step's
infinity norm in scaled coordinates removes that. A line search would need a
residual comparison, which is the tolerance-driven control flow ADR-0004 forbids.

## Consequences

Hover, forward flight, climb, descent and coordinated turn become declarations
rather than solvers. The helicopter problem `trim::helicopter_trim_problem` is
nine unknowns: the six pilot controls plus the two rotor inflow states and the
engine torque, each of which is a state with its own equilibrium.

That last point was learned rather than designed. The first version solved the six
force-and-moment equations alone, converged to a residual of 10⁻¹⁷, and was **not
an equilibrium**: the inflow and engine-torque states were left wherever the
initial guess put them. It was invisible in the forces and surfaced only when
`linearize_vehicle` measured the point's full dynamic residual and refused it with
`tail_inflow_ratio` named at 0.66. A declared problem is what made the fix a
three-line addition instead of a new solver.

`trim.level` and `trim.hover` keep their identifiers, their behaviour and their
byte-identical outputs. They are not yet reimplemented on top of the declared
solver, which remains outstanding work rather than a completed migration.

## Alternatives rejected

**A constrained optimiser.** Rejected. This finds a root of a square residual
system; an optimiser would return a best fit and the caller could not tell a
converged trim from a nearly-feasible one. The refusal is the value.

**A pseudo-inverse step on a rank-deficient Jacobian.** Rejected: it returns
whichever point the null-space component happened to reach and reports it as a
trim.

**Solving for yaw in the hover.** Rejected as genuinely singular — a helicopter in
still air is in equilibrium at any heading — and a test asserts that declaring it
is refused *by name* rather than answered.
