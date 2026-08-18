# ADR-0006: The equations of motion are written about the centre of gravity

- **Status:** accepted
- **Date:** 2026-08-18
- **Deciders:** project authors
- **Refines:** [ADR-0002](0002-state-and-frame-conventions.md)

## Context

[ADR-0002](0002-state-and-frame-conventions.md) says the body frame's origin is
"a documented geometric reference point fixed to the airframe, *not* the centre
of gravity", and that the CG is carried as an explicit offset so that CG
position can be swept.

That statement fixes where *geometry and aerodynamic data* are referenced. It
does not, on its own, say which point the equations of motion are written about,
and those are different questions. Writing them about an origin offset from the
CG is legitimate and some tools do it, but it introduces additional terms —
`omega_dot x r` and `omega x (omega x r)` in the force equation — and it makes
`(u, v, w)` the velocity of the reference point rather than of the CG, so `V`,
`alpha` and `beta` acquire a dependence on where the origin happens to sit.

ADR-0002 left this ambiguous. This record removes the ambiguity.

## Decision

**The rigid-body equations of motion are written about the instantaneous centre
of gravity.** `(u, v, w)` in the state vector is the air-relative velocity of
the CG, and `V`, `alpha` and `beta` derived from it are the CG's.

The airframe reference point remains as ADR-0002 describes it: a fixed
geometric datum, the point aerodynamic coefficient data is referenced to.

The CG offset is therefore not a term in the equations of motion. It appears in
exactly one place: **the transfer of the aerodynamic wrench from the reference
point to the CG**, `M_cg = M_ref + r_ref_to_cg x F`. That transfer is where a CG
sweep does its work, and it is the mechanism by which moving the CG aft reduces
static margin and drives the short period unstable — which is the phenomenon
this tool exists to let a user find.

The inertia tensor supplied in `MassProperties` is about the CG, in body axes,
and is not assumed diagonal.

## Alternatives considered

**Write the equations about the airframe reference point.** The honest case:
it is what ADR-0002's wording most naturally implies; it removes the need to
know where the CG is before integrating; and it means a CG change does not
change the meaning of the state vector, which makes two runs at different CG
directly comparable component by component.

Rejected on three counts. The extra `omega_dot x r` term makes the force
equation implicitly coupled to the moment equation — `v_dot` depends on
`omega_dot`, which depends on the moments — so each derivative evaluation needs
either a 6x6 solve or an algebraic rearrangement, which is a real cost paid on
every RK4 stage of every simulation. `alpha` and `beta` become properties of an
arbitrary datum rather than of the aircraft, so a model whose author moved the
reference point produces different aerodynamic angles for identical motion.
And it is not what the flight-dynamics literature this project validates
against does: Stevens/Lewis/Johnson and Etkin/Reid both write CG-referenced
equations, so a reader checking a sign against a textbook would be checking
against a different set of equations.

**Carry both formulations.** Rejected: two sets of equations of motion is two
places for a sign error, and the second exists only to serve a preference.

## Consequences

- `MassProperties` carries mass and the CG inertia tensor. The CG *position*
  lives in the aircraft model, next to the reference point it is measured from,
  because that is the only place it is used.
- A time-varying CG (fuel burn) moves the transfer point and changes the
  inertia tensor. It does not add terms to the equations of motion. It does
  make the CG-referenced frame non-inertial in a way this formulation ignores:
  the CG accelerating relative to the airframe contributes a term proportional
  to the rate of mass movement, which is negligible for fuel burn on the
  timescales this tool simulates and is **not** modelled. Stated here so that
  nobody discovers it by surprise in a fuel-transfer study.
- Any model supplying aerodynamic moments must state which point they are about.
  The transfer function is explicit and takes the offset as an argument rather
  than defaulting it, so this cannot be forgotten silently.
- ADR-0002's frame definition is unchanged. Only the ambiguity about which point
  the dynamics are written at is resolved.

## Revisit when

A vertical requires the reference-point formulation — a rotorcraft with a large
offset rotor hub, or a launch vehicle with substantial CG travel — at which
point the extra terms earn their cost and this becomes a per-model choice rather
than a project-wide one.
