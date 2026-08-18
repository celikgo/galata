# ADR-0003: Strict SI internally, conversion only at the boundary

- **Status:** accepted
- **Date:** 2026-08-18
- **Deciders:** project authors

## Context

Aerospace is a mixed-unit discipline by tradition and will remain one. Altitudes
are quoted in feet, speeds in knots, temperatures in Celsius, angles in degrees,
and a control engineer who is shown 3048 m instead of 10,000 ft has been given a
worse answer even though the number is right.

Meanwhile every unit conversion inside a numerical routine is a latent defect. A
derivative computed per radian and consumed per degree is wrong by a factor of
57.3 and looks entirely plausible on a Bode plot. Mars Climate Orbiter is the
canonical example and it was not a stupid mistake; it was a boundary that nobody
had written down.

## Decision

**The numerical core is strictly SI.** Metres, seconds, kilograms, newtons,
newton-metres, radians, radians per second, kelvin, pascals, kilograms per cubic
metre. Nothing else crosses a function signature inside `src/core`, `src/model`,
`src/numerics`, `src/trim`, `src/linearize`, `src/synth`, `src/analyze`,
`src/sim` or `src/ident`.

**Every public struct field carries its unit in a comment**, on the same line,
in the form `// m/s`. A field without one does not pass review.

**Conversions exist only in two places:** the user interface layer, and file
format adapters. Both call the same single set of `constexpr` functions in
`galata/core/units.hpp`, and there is no other conversion code anywhere.

The conversion factors are exact by definition, and each carries the definition
that makes it exact:

| From | To | Factor | Basis |
|---|---|---|---|
| international foot | metre | 0.3048 | exact, by international agreement (1959) |
| nautical mile | metre | 1852 | exact, by definition (CGPM/BIPM) |
| knot | metre per second | 1852 / 3600 | exact, from the nautical mile |
| degree | radian | pi / 180 | exact |
| degree Celsius | kelvin | + 273.15 | exact, by definition of the kelvin |
| pound-force | newton | 4.4482216152605 | exact, from the pound and standard gravity |
| slug | kilogram | 14.5939029372064 | exact, from the pound-force and foot |

Non-exact conversions are not permitted: if a factor cannot be written exactly,
the underlying unit is not one this project converts.

**This is gated, not merely stated.** `scripts/check-si-boundary.sh` fails CI if
a file under the numerical core calls a conversion function, or contains a
numeric literal matching a known conversion factor. It reports by name the core
directories that do not exist yet, so its coverage is visible in the CI log
instead of being assumed — as of this record it scans `src/core` and skips the
rest, because the rest have not been written. The literal check is the one
that catches the real failure mode, which is not calling `feet_to_metres` in the
solver — nobody does that — but writing `alt * 0.3048` inline because it seemed
obvious at the time.

## Alternatives considered

**A compile-time dimensional-analysis type system** (`mp-units`, `au`, or a
hand-rolled `Quantity<Length>`). This is the strictly better engineering answer:
it makes the entire class of error a compile error rather than a review
question, and it costs nothing at runtime.

It is rejected for the 1.x series on interface cost, not on merit. galata's
numerical core is built on Eigen, and its central objects are `Eigen::VectorXd`
state vectors and `Eigen::MatrixXd` Jacobians whose entries have *different*
dimensions from each other — the A matrix of a linearised aircraft mixes 1/s,
rad/s, m/s and dimensionless entries in the same matrix. A dimensioned scalar
type does not compose with a matrix library whose element type must be uniform,
and the wrapper needed to make it compose would be a larger and less inspectable
artefact than the rule it replaces. The C plugin ABI compounds this: a C ABI
cannot carry a C++ type system, so the boundary would be undimensioned anyway
and the guarantee would stop exactly where plugins begin.

The rule plus the CI gate covers the same ground for the cases that actually
occur, at a fraction of the interface cost. If Eigen ever gains first-class
support for dimensioned scalars this should be reconsidered.

**Allow degrees in the aerodynamic model** because published coefficient tables
are almost always tabulated against degrees. Rejected, and this is the specific
case the rule exists for: the table *adapter* converts breakpoints to radians on
load, once, and records that it did so. A derivative that is per-degree inside
the solver is exactly the 57.3× error described above.

**Carry a unit tag alongside each value at runtime.** Rejected: it moves a class
of compile-time-preventable error to run time and adds a branch to every
arithmetic operation, which the determinism policy would then have to reason
about.

## Consequences

- The UI displays feet, knots, degrees and Celsius by default and converts at
  the edge. Pipeline YAML likewise names its units explicitly in the key —
  `altitude_m`, not `altitude`.
- Reading a published coefficient table means converting its breakpoints at load
  time, and recording in the model's provenance that the conversion happened.
- The CI gate has a false-positive mode: a legitimate constant that happens to
  equal a conversion factor. There is an explicit allowlist with a required
  justifying comment; entries are rare and are reviewed.
- Nothing prevents a plugin author from doing arithmetic in feet inside their
  own plugin. The ABI documents the units of every value it passes, and the
  conformance suite checks a plugin against known inputs, which is as far as a C
  boundary can enforce this.

## Revisit when

Eigen supports dimensioned scalar types without a wrapper, or the project drops
the C plugin ABI. Neither is expected within 1.x.
