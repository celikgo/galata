# A quadrotor whose four rotors are not the same

Nobody measures four identical motors. A bench produces four thrust
coefficients, four torque coefficients and four time constants, and they differ
— by a few percent if the parts are well matched, by more if they are not. This
example trims and linearises such a vehicle.

## Run it

```bash
galata run examples/quadrotor-heterogeneous-rotors/study.yaml
```

Six stages: load the model, trim it at hover, linearise on the attitude-error
chart, export the matrices, read off the poles, write the report.

## What it demonstrates

**The rotors carry unequal shares at unequal speeds, and the vehicle stays
level.** Still air and a symmetric rotor *layout* mean the trimmed attitude is
level whatever the coefficients do; only the speeds move. The direction is
forced rather than chosen: rotor 0 has the smallest thrust coefficient, so it
must spin fastest to carry its share, and rotor 3 the largest, so it must spin
slowest. The diagonal pairs move together, because the moments they balance are
diagonal.

**The numbers are in the run, not in this file.** `operating-point.yaml` carries
the residual, the per-rotor speeds, each rotor's margin to its own ceiling, the
Jacobian condition number and the finite-difference step sizes. Read them from
there; a figure typed into a README is a figure no later run can contradict.

## What was actually wrong before

The equilibrium equations always carried one unknown per rotor. What refused a
model like this one was the *starting point*: the solver seeded every rotor from
a single vehicle-wide hover speed, `sqrt(m g / (n k_T))`, which is undefined
when the coefficients differ — and said so, correctly, by throwing.

The fix asks each rotor for its share of the **force** rather than for a shared
speed: `sqrt(m g / (n k_T_i))`. For a vehicle whose rotors match, that is the
same expression evaluated on the same numbers, so a homogeneous model starts
from bit-identical values and ADR-0004's determinism guarantee is untouched.

It is a guess, not an equilibrium — equal thrust per rotor balances the force and
leaves a residual couple whenever the rotors differ. Newton removes it.

## What checks this

`QuadrotorHoverTrim.FourDistinctThrustCoefficientsTrimToARealEquilibrium` in the
`validation` tier. It does not trust the solver's own residual: it **rebuilds**
the force and moment balance from the model file's coefficients — thrust
`k_T ω²` along body −z, moment `r × F`, reaction `∓k_Q ω²` about body z, weight
rotated into the body frame — and requires both to close. A solver that
converged to the wrong root passes a residual gate and fails that one.

`QuadrotorHoverTrim.StillAirCrosswindCruiseAndUnequalRotorsSolveToTheirDeclaredBudget`
covers the homogeneous conditions the change had to leave alone, and
`QuadrotorHoverTrim.ANonPositiveThrustCoefficientIsRefusedByName` covers the
rotor that has no hover speed at all.

## What this model is not

`quad-heterogeneous.yaml` is **synthetic**. Its four thrust coefficients are the
nominal 1.0e-5 scaled by 0.92, 1.00, 1.05 and 1.08 — a spread chosen by hand so
the rotors differ by a visible, physically ordinary amount. No aircraft was
weighed and no motor was run up.

It is not the Souxmar nominal vehicle (`models/souxmar-quad`, whose rotors are
identical), and it is not the proposed 2.475 kg hardware airframe, which has no
model here and whose coefficients will come from a bench when one exists.
Nothing computed from this file is evidence about any aircraft.
