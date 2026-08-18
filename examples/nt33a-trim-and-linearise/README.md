# NT-33A — trim, linearise, and read off the modes

The workflow this project exists for, end to end. **Nothing in the input is a
matrix**: it is a set of non-dimensional aerodynamic derivatives and some
geometry. Everything downstream — the trim, the state-space model, the modes —
is computed.

## Run it

```bash
galata run examples/nt33a-trim-and-linearise/study.yaml
```

Seven stages: load the model, trim it, linearise it twice (longitudinal and
lateral), analyse both, write the report.

## What it produces, and what checks it

Trim, at sea level and 69.4944 m/s (228 ft/s):

| | galata | NASA CR-2144 |
|---|---|---|
| Angle of attack | 2.1481° | 2.20° |
| Dynamic pressure | 61.78 psf | 61.7 psf |
| Mach | 0.2042 | 0.204 |

Modes, from linearising about that trim:

| Mode | galata | published | error |
|---|---|---|---|
| Phugoid | ζ 0.0949, ω_n 0.1714 | 0.0948, 0.172 | 0.1%, 0.3% |
| Short period | ζ 0.6219, ω_n 1.5950 | 0.622, 1.59 | 0.02%, 0.3% |
| Spiral | 1/T 0.0319 | 0.0318 | 0.3% |
| Roll subsidence | 1/T 2.1992 | 2.20 | 0.04% |
| Dutch roll | ζ 0.0603, ω_n 1.1293 | 0.0609, 1.13 | 1.0%, 0.06% |

And, more sharply than the modes, the linearised **dimensional derivatives**
match the report's Table II-7 to within 0.26% across all seven — numbers the
report computed from the same non-dimensional set by a completely different
route.

## Three things worth noticing in the output

**The trim angle of attack is 0.05° below the published 2.20°, on purpose.** The
published trim relates α to C_L = 0.813 through the conventional level-flight
relation C_L = W/(q̄S), which neglects the vertical component of drag in body
axes. galata solves the exact balance, which needs C_L = 0.8084. The difference
is exactly `D·tan α/(q̄S)/C_Lα`, and a validation test asserts that it is that
term and nothing else.

**The trim's Jacobian condition number is about 3.5e5, and that is not a
warning.** It reflects the units of the unknowns, not the physics: an angle of
about 0.04 rad and a thrust of about 8700 N sit in the same vector, so the
Jacobian's columns differ in scale by five orders before any aircraft is
involved. The residual is exactly zero. A condition number worth worrying about
would come with a residual that would not come down.

**The truncation estimates are around 1e-12 and 1e-15.** That is the Richardson
estimate of the finite-difference error, per entry, and it says the
linearisation is limited by nothing that matters here. It would *not* catch a
discontinuity inside the perturbation window — a rate limit, a table breakpoint
— and there are none in this model because it is a smooth polynomial.

## What this example does not show

No control synthesis and no nonlinear simulation: neither exists yet. The model
also has no stall, so do not ask it about one — see
[`models/nt33a/PROVENANCE.md`](../../models/nt33a/PROVENANCE.md).
