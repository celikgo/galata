# Souxmar F1 nonlinear-versus-linear agreement, in cruise and in exact hover

<!-- SPDX-License-Identifier: Apache-2.0 -->

```bash
galata run examples/heli-nonlinear-vs-linear/study.yaml --output-dir build/heli-nvl
```

## Why an order and not a tolerance

A linearisation can be wrong in ways no eigenvalue shows: a sign error, a missing
term, a point that is not quite an equilibrium. **No single tolerance at a single
perturbation size separates any of those from a correct linearisation** — pick the
tolerance loosely and it passes everything, tightly and it fails correct work.

What separates them is how the error *scales*. A correct linearisation's error is
the Taylor remainder, so it falls as the **square** of the perturbation: halve the
perturbation and the discrepancy quarters. A first-order error falls only
linearly. So the measurement is an order, and an order needs at least two points.

## What the run measures

| condition | observed orders |
|---|---|
| 30 m/s cruise | 1.9984, 1.9992, 1.9996, **1.9998** |
| exact hover | 1.1529, 0.7612, −0.0076, −0.1536 |

Cruise converges on 2 from below, which is what a correct second-order expansion
does once the perturbation is small enough that the third-order term stops
contributing.

**Hover does not converge at all**, and the cause is localised rather than
absorbed. The advance ratio is `mu = |V_inplane| / (Omega R)`, and `|V|` has no
derivative at `V = 0`: the one-sided slopes are +1 and −1. A central difference
therefore returns **zero** for `d(mu)/dV`, so the linearisation carries no
flap-back response to a speed perturbation while the nonlinear model has one
proportional to `|V|`. A second term compounds it: the empennage's incidence is
`atan2(w, u)`, evaluated at a tail sitting in about 4 m/s of rotor downwash with
no free stream, so the angle swings through a right angle for an arbitrarily small
perturbation in `u`.

Removing the empennage isolates the first term: the floor drops away and the order
becomes a clean 1.00.

## This is a property of the physics, not a defect

`mu` genuinely is `|V|/(Omega R)`. Smoothing it would make the Jacobian look
better and the **model** worse, and the hover linearisation would then be a good
approximation to the wrong aircraft.

So the behaviour is gated in both directions, the way the two load-bearing
regression locks in [`docs/VERIFICATION.md`](../../docs/VERIFICATION.md) are:
`HelicopterLinearisation.HasANonVanishingErrorFloorInExactHoverAndThatIsThePhysics`
fails if the floor **grows** beyond the two known kinks, and it also fails if the
floor **vanishes**. A future change that smooths the advance ratio will be loud
rather than silent.

## What this means for using the hover linearisation

It remains usable for what it is for — stability analysis, modal
characteristics, control-law synthesis — because those read the matrix, not a
predicted trajectory. It is **not** fit for trajectory prediction from the hover:
there the error does not shrink with the perturbation, so there is no
perturbation small enough to make it negligible. Take the linearisation at a few
metres per second instead, where it is second order.
