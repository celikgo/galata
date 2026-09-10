# ADR-0017: The attitude-error chart has a public inverse

- **Status:** accepted
- **Date:** 2026-09-10
- **Deciders:** project maintainer

## Context

RFC-0002's WP5 asks for a controller executed at a declared rate against the
nonlinear multirotor plant. A controller has to close the gap between where the
aircraft is and where it should be, and that gap has to be expressed in the
coordinates the control law was designed in.

`synth.lqr` designs against the model `linearize.extended` produces, whose state
is the local attitude-error chart: twelve rigid coordinates plus one per
appended state, with attitude carried as a three-component rotation vector
rather than as a quaternion. `sim.plant` integrates the plant, whose extended
state is ADR-0002's thirteen components plus the same appended block, with
attitude carried as a **quaternion**.

Those are different spaces. Seventeen numbers on one side, sixteen on the other,
and the attitude coordinates are not the same quantity in different units — one
is a member of a group, the other a member of its tangent space at a declared
reference.

**Only one direction of the map exists, and it is private.** `unpack` in
`src/linearize/extended.cpp` goes chart → full: additive on position, velocity,
body rates and the appended block, and multiplicative on attitude, `q = q0 *
exp(e/2)`. It is a local lambda inside the linearisation, reachable from nothing
else.

**The direction a controller needs does not exist anywhere.** Going full → chart
requires the inverse of that composition — `e = 2 log(q0^-1 * q)` — and
`include/galata/core/quaternion.hpp` today has no quaternion product, no
inverse and no logarithm. It has `angular_distance`, which returns the magnitude
of the rotation between two attitudes and discards its axis. A magnitude cannot
close a loop.

## Decision

**Make the chart a named, public, two-way map**, in `galata::linearize`, with the
reference attitude an explicit argument in both directions.

Two functions, proposed as `chart_from_extended` and `extended_from_chart`, plus
the quaternion primitives the first needs — a product, an inverse and a
logarithm — added to `galata::core` where the rest of the attitude algebra
already lives. The existing `unpack` becomes a caller of the public forward map
rather than a second copy of it.

The reference is an argument, never an ambient default. A chart coordinate is
meaningless without the attitude it is measured from, and a function that
supplied its own would let a caller take a difference against a reference they
did not choose and get a number that looks like an error.

## Why not the alternatives

**Feed back on the quaternion directly, with a gain of seventeen columns.**
Rejected. The gain came from a sixteen-state design and there is no
seventeen-column matrix that means the same thing. Widening it would require
inventing a column for the redundant quaternion component, and the redundancy is
exactly what the chart exists to remove — a four-component attitude with a norm
constraint is not four independent coordinates, and a linear law that treats it
as such drives the state off the unit sphere.

**Let the sampled-control capability compute the chart internally.** Rejected on
the rule that a second implementation of anything is one too many. The map would
then exist twice, privately, in two places that must agree about a half-angle
convention and a sign; when they drift, the symptom is a controller that works
in simulation and not in analysis, or the reverse, with no test between them
able to see it.

**Ship the inverse without the quaternion primitives**, computing the logarithm
inline. Rejected. `q0^-1 * q` is attitude algebra and belongs beside the rest of
it. Written inline it would be the third place in this repository that composes
quaternions by hand.

## What this must get right, stated because it is easy to get wrong quietly

**The double cover.** `q` and `-q` are the same attitude, and their naive
logarithms differ by a full turn. The inverse must canonicalise before taking
the logarithm, or a controller crosses a sign boundary and commands a rotation
the long way round at full authority. Souxmar's own cascade carries the same
guard — `2*error_q[1:]*(1 if error_q[0] >= 0 else -1)` — for the same reason.

**The small-angle limit.** `log` divides by `sin(half_angle)`, which vanishes at
zero error, the point a controller spends most of its time near. The series
expansion is required, not optional, and the boundary between the two branches
is a declared constant rather than whatever happened to be numerically quiet.

**Round-trip, both ways, as a gate.** `chart_from_extended(extended_from_chart(d))
== d` and its converse must hold to a stated bound, including at the reference
where the chart is exactly zero, at small angles across the series boundary, and
at large angles approaching a half turn where the parameterisation itself stops
being unique.

**Agreement with the linearisation.** The public forward map has to reproduce
what `linearize.extended` already does, or the matrices and the controller are
describing different vehicles. The existing extended-linearisation cases are the
gate on that: they must pass unchanged, bit for bit.

## Consequences

- `linearize.extended`, `sim.plant` and the sampled-control capability to come
  all speak one chart, and the ADR-0002 state and the chart stop being connected
  only by code nobody outside one file can call.
- `galata::core` gains quaternion product, inverse and logarithm. They are
  attitude algebra, not new dynamics, and no second rigid-body implementation
  follows from them.
- ADR-0004 is unaffected in kind: the map is closed-form, has no iteration and no
  tolerance-based branch, and its one conditional — the small-angle series — is
  taken on a declared constant.
- The functions named here are **proposed** until they exist. They needed no
  entry in `scripts/doc-references-allow.txt`, because the gate checks file
  paths and gtest names and a proposed function name is neither — which is worth
  saying rather than leaving as an apparent omission. Nothing mechanical will
  notice if they are never written; this record is the only thing that says they
  were intended.
