<!-- GENERATED FILE — DO NOT EDIT.
     Produced by tools/validation/report_main.cpp via scripts/gen-verification.sh.
     CI regenerates this file and fails if it differs from the committed copy,
     so every number below is a number the code actually produced. -->

# Verification and validation

What galata has been checked against, what agreement was measured, and what has
not been checked at all.

The last column is the one to read. "Unvalidated" is not a placeholder here — it
is a statement that no published reference value has been found for that case,
and it is preferred to a number invented to fill the gap.

## Summary

The table below is GENERATED from `tools/validation/case_registry.cpp` and is
reconciled against the capability registry the CLI dispatches through. It used
to be typed by hand and it drifted three times, most memorably by still saying
"there is no aerodynamic model yet" in the commit that added one.

Four checks stand behind it, each a test rather than a convention:

* every case claiming a comparison names a reference and names its evidence;
* every piece of evidence names a test that is **actually registered** in the
  binary the case says it lives in, so a renamed or deleted test breaks the
  build rather than leaving a fictional citation;
* every capability declaring *implemented and validated* is backed by at least
  one validating case, which makes that declaration unfalsifiable by hand;
* a case marked *not implemented* may not name a capability that exists, so
  the row cannot outlive the thing being built.

| Case | Reference | Status |
|---|---|---|
| U.S. Standard Atmosphere 1976 — temperature, pressure, density, speed of sound | COESA, *U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 / NASA-TM-X-74335, Tables I and III | **validated** — Temperature and speed of sound round to the printed value everywhere; pressure and density do not, at 3 of 32 cells, by at most 0.961 units in the last printed place. Every deviation is listed below. |
| U.S. Standard Atmosphere 1976 — derived layer base temperatures and the pressure recurrence | COESA, *U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 / NASA-TM-X-74335, Table I at each breakpoint | **validated** — Table 4 has no base-temperature column, so these are derived rather than transcribed. The pressure recurrence is checked at the top of the seven-layer chain, where any per-layer error would have accumulated. |
| U.S. Standard Atmosphere 1976 — dynamic viscosity | COESA, *U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 / NASA-TM-X-74335, equation (51) | unvalidated — Implemented, but no tabulated viscosity values were transcribed, so there is nothing to compare against. It also inherits the source's own S = 110 K versus 110.4 K ambiguity, worth 0.10%. |
| Quaternion, frame and state conventions | ADR-0002, cross-checked against Eigen's independent implementation | self-consistent, not externally validated — The rotation matrix is written out by hand from ADR-0002 and compared against Eigen over the whole rotation group. That checks the documented convention against the implemented one; it is not a comparison against a document. |
| Numerical Jacobians against analytically known Jacobians | Charter validation case 4; analytic derivatives of closed-form functions | **validated** — Agreement to the cancellation limit, eps \|f\| / h, which is the floor a central difference has even on a linear function. The Richardson estimate is checked to bound the actual error rather than understate it. |
| Fixed-step RK4 — method order | Hairer, Norsett & Wanner (1993); exact solutions of closed-form problems | **validated** — Exact on cubics, as Simpson's rule must be; error falls by 16 per halving on the exponential. |
| Newton's method — convergence and failure reporting | Nocedal & Wright (2006); systems with closed-form roots | **validated** — Quadratic convergence on a smooth system; a system with no real root is reported as unconverged rather than returned as a least-bad point. |
| Torque-free precession of a symmetric top | Closed-form solutions of Euler's equations (Goldstein; Landau & Lifshitz) | **validated** — Checked for fourth-order CONVERGENCE to the closed form, not proximity to it. A solution converging to the wrong closed form sits at a small constant error and passes an absolute check. |
| Intermediate-axis instability (the Dzhanibekov effect) | Closed-form solutions of Euler's equations (Goldstein; Landau & Lifshitz) | **validated** — Asserted against the cosh/sinh closed form pointwise, including the sign the (I2 - I3) < 0 factor forces. Fitting a log-slope instead measures 0.699 sigma and looks like a defect in the dynamics. |
| Energy and angular-momentum conservation, general inertia tensor | Exact invariants of torque-free motion | **validated** — The angular-momentum figure is the VECTOR resolved in NED, not its body-axis magnitude. A transposed direction-cosine matrix conserves the magnitude and fails this. Drift measured below. |
| Six-degree-of-freedom equations with aerodynamic forces | Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972), Tables II-1 and II-7 | **validated**, with a caveat — Validated INDIRECTLY: the linearised derivatives that match Table II-7 to 0.26% run through these equations, the coefficient buildup and the wind-to-body rotation. There is no case comparing the equations in isolation. |
| Nonlinear simulation with aerodynamic forces, over time | — | not implemented — There is a state derivative, not a loop flying an aircraft through time. |
| Aircraft lateral modes from a hand-assembled matrix — spiral, roll subsidence, Dutch roll | Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972), Table II-8 | **validated** — Tolerance measured, not chosen: each input is perturbed by half a unit in its own last printed digit and the published value's own rounding is added. |
| Aircraft longitudinal modes from a hand-assembled matrix — phugoid frequency, short-period frequency and damping | Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972), Table II-4 | **validated** — Three of the four longitudinal quantities. The fourth is the row below. |
| Aircraft longitudinal modes — phugoid DAMPING RATIO, from a hand-assembled matrix | Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972), Table II-4 | **known discrepancy** — 0.0929 against a published 0.0948, out by 2.04%, about three times what the inputs' rounding allows. Localised to the hand assembly — the full chain reproduces it. Held by a labelled regression lock; see below. |
| Modal classification into the five classical modes | Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972), labels checked against the report's own identification | **validated** — By eigenvector participation, not by frequency. A unit test builds a system whose phugoid block is deliberately faster than its short-period block; a frequency-based classifier gets both labels backwards on it. |
| Trim of a nonlinear model against the published flight condition | Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972), Table II-2 | **validated** — Dynamic pressure 61.78 psf against a published 61.7; Mach 0.2042 against 0.204. The trimmed alpha is 0.0519 deg below the published 2.2, and a test asserts that difference is exactly the drag-inclination term the conventional C_L = W/(qS) relation neglects. |
| Linearised dimensional derivatives from a nonlinear model | Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972), Table II-7 | **validated** — Seven numbers the report computed from the same non-dimensional set by a different route, reproduced to 0.26%. The sharpest comparison in the suite. |
| All five classical modes from trim and linearisation of a nonlinear model | Heffley & Jewell, *Aircraft Handling Qualities Data*, NASA CR-2144 (1972), Tables II-4 and II-8 | **validated** — To 1.05%, worst case Dutch roll zeta. The input is a non-dimensional derivative set and some geometry; there is no matrix anywhere in it. |
| Determinism tier 1 — same platform, byte-identical | ADR-0004 | **validated** — Gated on Linux, macOS and Windows over 145 fingerprinted values. The strongest of these is splitting: 4000 steps must equal 1500 then 2500, bit for bit. |
| Determinism tier 2 — cross-platform, bounded | ADR-0004 | **validated**, with a caveat — Bounded at 1e-9 relative between every pair of platforms, not bit-identical, because platform math libraries disagree on sin in the last bits. Values downstream of a finite difference are excluded from this tier and held byte-identical in tier 1 instead — 47 of the 145 values — because dividing by h amplifies a libm disagreement by 1/h. |
| Riccati solvers against the CAREX and DAREX benchmark collections | — | not implemented — Named in the v0.2 milestone. |
| Gain, phase, delay and disk margins | — | not implemented — Named in the v0.2 milestone. |

### Evidence

Every case above, and the tests that stand behind it. These names are checked
against the tests actually registered in each binary, so they can be run:

```
ctest --preset dev -R '<test name>'
```

| Case | Evidence |
|---|---|
| U.S. Standard Atmosphere 1976 — temperature, pressure, density, speed of sound | `Ussa1976.TemperatureMatchesThePublishedTable` (validation)<br>`Ussa1976.PressureMatchesThePublishedTable` (validation)<br>`Ussa1976.DensityMatchesThePublishedTable` (validation)<br>`Ussa1976.SpeedOfSoundMatchesThePublishedTable` (validation) |
| U.S. Standard Atmosphere 1976 — derived layer base temperatures and the pressure recurrence | `Ussa1976.DerivedBaseTemperaturesMatchTheStandardsOwnTabulation` (validation)<br>`Ussa1976.PressureAtTheTopOfTheModelMatchesAfterSevenLayers` (validation) |
| U.S. Standard Atmosphere 1976 — dynamic viscosity | — |
| Quaternion, frame and state conventions | `Quaternion.DcmMatchesEigensOwnRotationMatrix` (unit)<br>`QuaternionProperties.HandWrittenDcmAlwaysAgreesWithEigen` (property) |
| Numerical Jacobians against analytically known Jacobians | `Jacobian.QuadraticFunctionMatchesItsAnalyticJacobian` (unit)<br>`Jacobian.LinearFunctionIsDifferentiatedToTheCancellationLimit` (unit)<br>`Jacobian.TruncationEstimateBoundsTheActualError` (unit) |
| Fixed-step RK4 — method order | `Rk4.IntegratesCubicsInTimeExactly` (unit)<br>`Rk4.IsFourthOrderOnTheExponential` (unit)<br>`Rk4.StepSizeStudyRecoversTheMethodOrder` (unit) |
| Newton's method — convergence and failure reporting | `Newton.SolvesALinearSystemInOneStep` (unit)<br>`Newton.ConvergesQuadraticallyOnASmoothNonlinearSystem` (unit)<br>`Newton.ReportsFailureRatherThanReturningAWrongRoot` (unit) |
| Torque-free precession of a symmetric top | `Shapes/SymmetricTop.ConvergesToTheClosedFormPrecessionAtFourthOrder/oblate` (validation)<br>`Shapes/SymmetricTop.ConvergesToTheClosedFormPrecessionAtFourthOrder/prolate` (validation) |
| Intermediate-axis instability (the Dzhanibekov effect) | `IntermediateAxis.PerturbationFollowsTheClosedFormHyperbolicGrowth` (validation)<br>`IntermediateAxis.RotationAboutTheMajorAndMinorAxesIsStable` (validation) |
| Energy and angular-momentum conservation, general inertia tensor | `TorqueFreeConservation.EnergyAndAngularMomentumDriftIsBounded` (validation)<br>`TorqueFreeConservation.AngularMomentumRotatesInBodyAxesButNotInNed` (validation) |
| Six-degree-of-freedom equations with aerodynamic forces | `Nt33aChain.LateralDimensionalDerivativesMatchThePublishedTable` (validation) |
| Nonlinear simulation with aerodynamic forces, over time | — |
| Aircraft lateral modes from a hand-assembled matrix — spiral, roll subsidence, Dutch roll | `Nt33aHandAssembled.LateralModesMatchThePublishedValuesWithinTheSourcesOwnPrecision` (validation)<br>`Nt33aHandAssembled.TheDutchRollPeriodAgreesWithThePublishedPeriod` (validation) |
| Aircraft longitudinal modes from a hand-assembled matrix — phugoid frequency, short-period frequency and damping | `Nt33aHandAssembled.LongitudinalModesMatchThePublishedValuesWithinTheSourcesOwnPrecision` (validation) |
| Aircraft longitudinal modes — phugoid DAMPING RATIO, from a hand-assembled matrix | `Nt33aHandAssembled.PhugoidDampingDiscrepancyDoesNotGrow` (validation) |
| Modal classification into the five classical modes | `Nt33aHandAssembled.AllThreeLateralModesAreFoundAndCorrectlyLabelled` (validation)<br>`Nt33aHandAssembled.BothLongitudinalModesAreFoundAndCorrectlyLabelled` (validation)<br>`Nt33aChain.ModesAreLabelledCorrectlyFromParticipationAlone` (validation)<br>`Modes.ClassifiesLongitudinalModesByParticipationNotByFrequency` (unit) |
| Trim of a nonlinear model against the published flight condition | `Nt33aChain.TrimConvergesToMachinePrecision` (validation)<br>`Nt33aChain.TrimSatisfiesTheClosedFormForceBalanceExactly` (validation)<br>`Nt33aChain.DynamicPressureAndMachMatchThePublishedFlightCondition` (validation)<br>`Nt33aChain.TrimAlphaDiffersFromThePublishedValueByExactlyTheDragInclinationTerm` (validation) |
| Linearised dimensional derivatives from a nonlinear model | `Nt33aChain.LateralDimensionalDerivativesMatchThePublishedTable` (validation)<br>`Nt33aChain.TruncationErrorIsNegligible` (validation)<br>`Nt33aChain.TheLongitudinalAndLateralAxesDecoupleAtThisTrim` (validation) |
| All five classical modes from trim and linearisation of a nonlinear model | `Nt33aChain.AllFiveClassicalModesMatchThePublishedValues` (validation)<br>`Nt33aChain.ThePhugoidDampingThatTheHandAssembledMatrixMissedIsRecovered` (validation) |
| Determinism tier 1 — same platform, byte-identical | `Determinism.LongIntegrationIsBitIdenticalAcrossRuns` (determinism)<br>`Determinism.SplittingAnIntegrationInTwoGivesTheSameResult` (determinism)<br>`Determinism.ModalDecompositionIsBitIdenticalAndOrderStable` (determinism)<br>`Determinism.AtmosphereDoesNotDependOnQueryOrder` (determinism) |
| Determinism tier 2 — cross-platform, bounded | `Determinism.TheFingerprintTrajectoryIsNotChaotic` (determinism) |
| Riccati solvers against the CAREX and DAREX benchmark collections | — |
| Gain, phase, delay and disk margins | — |

### Capabilities, and the cases that validate them

Read straight out of the registry the CLI dispatches through, so a capability
cannot appear here without existing, and cannot claim validation without a case
backing it. A capability may legitimately be *implemented, unvalidated* — that
is an honest state, and it means no published reference has been compared
against.

| Capability | Declared state | Validated by |
|---|---|---|
| `analyze.modes` | implemented and validated | `nt33a.lateral_modes_hand`, `nt33a.longitudinal_modes_hand`, `nt33a.phugoid_damping_hand`, `analyze.classification`, `nt33a.chain_modes` |
| `linearize.finitediff` | implemented and validated | `nt33a.linearised_derivatives`, `nt33a.chain_modes` |
| `model.aircraft.derivatives` | implemented and validated | `nt33a.trim`, `nt33a.linearised_derivatives`, `nt33a.chain_modes` |
| `model.linear.statespace` | implemented, unvalidated | — |
| `report.markdown` | implemented, unvalidated | — |
| `trim.level` | implemented and validated | `nt33a.trim`, `nt33a.linearised_derivatives`, `nt33a.chain_modes` |

## U.S. Standard Atmosphere, 1976

**Reference.** U.S. Committee on Extension to the Standard Atmosphere (COESA),
*U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 / NASA-TM-X-74335,
U.S. Government Printing Office, October 1976.
NTRS document 19770009539, <https://ntrs.nasa.gov/citations/19770009539>.
Rights: *Work of the US Gov. Public Use Permitted.*

**Transcription.** The NTRS scan's OCR layer is unusable for numeric tables —
roughly 95,800 extractable characters across 243 pages, with digits rendered as
underscores and letters. No value came from it. The page images were extracted and
read visually, with load-bearing digits re-cropped at native resolution, and a
second independent pass re-read twelve values without reference to the first.
The full method is in the header of `tests/validation/reference/ussa1976.csv`.

**Agreement measured.** Deviation is given in units of the last printed
significant figure. Below 0.5 the computed value rounds to exactly what the
document prints.

| Quantity | Worst deviation (units in last printed place) | Rounds to the printed value everywhere |
|---|---|---|
| Temperature | 0.487 | yes |
| Pressure | 0.961 | **no** |
| Density | 0.85 | **no** |
| Speed of sound | 0.499 | yes |

**Gate.** The validation suite requires every cell to agree within 1.0 units of
the last printed place. That bound is chosen to be the tightest one the 1976
tables actually support, not the loosest one that passes: the measured
disagreements are around one part in 10^5, while a wrong lapse-rate sign, a
geopotential/geometric mix-up or the wrong Earth radius each move these values by
one part in 10^2 or worse.

### Cells that do not round to the printed value

There are 3 of them, out of 32 tabulated cells.

| Geometric altitude (m) | Quantity | Published | Computed | Units in last place |
|---|---|---|---|---|
| 11000 | pressure | 226.99 | 227 | 0.961 |
| 71000 | pressure | 0.044795 | 0.044796 | 0.632 |
| 71000 | density | 7.1966e-05 | 7.1965e-05 | 0.85 |

### Known model-versus-table differences that are not errors

**Molecular-scale versus kinetic temperature above 80 km.** The seven-layer
system is defined in terms of molecular-scale temperature `T_M`, which equals
kinetic temperature `T` only where the mean molecular weight equals its
sea-level value. Above roughly 80 km the ratio `M/M_0` falls away from 1 — the
document's Table 8 gives 0.9995788 at 86 km — so galata's temperature runs high
relative to the tabulated kinetic temperature by up to about 0.08 K in the top
6 km. At 86 km the document tabulates `T = 186.87` K and `T_M = 186.95` K;
galata computes 186.946 K, which is the `T_M` value.

**The upper bound is stated twice and the two statements differ.** The standard
gives the ceiling as both 84.8520 km' geopotential and 86 km geometric.
Equation (18) maps 86000 m to 84852.046 m', so the geopotential figure is the
document's own rounding. galata bounds the envelope in geometric altitude, so
that a query at exactly the standard's stated ceiling is accepted.

### Ambiguities in the source document

These are inconsistencies in the 1976 publication itself, found during
transcription. Each is recorded so that it is not later mistaken for a
transcription error.

| Quantity | Printed as | And also as | galata uses |
|---|---|---|---|
| Sutherland's constant `S` | 110 K (Table 2B, printed page 4) | 110.4 K (text at equation (51)) | 110.4 K, the value stated with the equation it parameterises |
| `R*` | 8.31432e3 N m/(kmol K) (body text, printed page 3) | 8.31432e-3 (Table 2A) | 8.31432e3; only this makes equations (33a)/(33b) dimensionally consistent |
| Sutherland's `beta` | 1.458e-6 (Table 2B, printed page 19) | 1.458e6 (printed page 4, minus sign absent) | 1.458e-6, corroborated twice |
| `r_0` | 6,356,766 m (printed page 8) | 6356.766 km (printed page 4) | 6356766 m; Table 2B's exponent glyph is illegible in the available scans and was not used |

### What is not validated here

**Dynamic viscosity.** galata implements equation (51), but no tabulated
viscosity values were transcribed from the document, so there is nothing to
compare against. The implementation is unvalidated. It additionally inherits the
`S = 110` versus `S = 110.4` ambiguity above, which moves the result by
0.10%.

**Altitudes between the tabulated points.** Eight altitudes are checked against
the tables. Continuity, monotonicity and the absence of steps at layer
boundaries are checked on a 100 m grid across the whole envelope, which
constrains the space between the sampled points but is not the same as checking
it against published values.

**Everything above 86 km.** Out of scope: galata refuses the query rather than
extrapolating.

## Determinism

ADR-0004 defines two tiers, and both are gated by
`.github/workflows/determinism.yml` on Linux, macOS and Windows.

**Tier 1 — same binary, same platform, byte-identical.** `tools/determinism`
emits 92 values at `%.17g`, which round-trips a double exactly, so byte-identical
output means bit-identical values rather than values that merely print the same.
Gated absolutely on every platform.

**Tier 2 — cross-platform, bounded.** Every platform PAIR is compared, not each
against a nominated reference: with a reference, which platform holds that role
is an arbitrary choice that then shows up in the published numbers. The gate is
1e-9 relative — far above the roughly 1e-16 that one math-library call costs,
far below the roughly 1e-5 that any real divergence in the physics would
produce, so it discriminates between "different libm" and "different answer".

The observed deviation is printed by every run rather than merely bounded; read
it from the workflow log. It is not restated here, because a measured figure
copied into a hand-maintained document is a figure that drifts.

**Why tier 2 is not bit-identity.** `sqrt` is required by IEEE 754 to be
correctly rounded and agrees everywhere. `sin`, `cos`, `tan`, `asin`, `atan2`,
`exp`, `log` and `pow` are not, and come from the platform's math library.
galata cannot avoid them — angle of attack is an `atan2`, the atmosphere's
pressure profile is a `pow` — so the honest claim is the two-tier one rather
than a bit-identity claim that would be false.

**A constraint on what may be fingerprinted.** A cross-platform bound is only
meaningful on a computation that does not amplify small differences; on a
chaotic trajectory the gate would be measuring chaos rather than agreement. The
fingerprint's rigid-body case was measured and amplifies a perturbation by a
factor between 0.06 and 1.0 over 60 s. A test asserts this, so a change that
makes the battery chaotic fails there rather than as an intermittently red
workflow.

## Trim and linearisation

**Reference.** Heffley and Jewell, NASA CR-2144, Tables II-1, II-2 and II-7.

This is the end-to-end case, and it is a much stronger statement than the modal
comparison above. There, the state matrix came from the report's own
dimensional derivatives, so only the eigen-analysis was under test. Here the
input is the NON-dimensional derivative set and the geometry; galata builds a
nonlinear model, finds its trim, linearises about it, and both the dimensional
derivatives and the modes fall out. Everything in between is under test: the
atmosphere, every unit conversion, the coefficient buildup, the wind-to-body
rotation, the equations of motion, the root-find and the finite differences.

**Trim.** Converges to a residual of 0, with quadratic
convergence visible in the reported history. It is checked two ways: against
the closed-form force balance `L = mg - D tan(a)` and `T = D/cos(a)`, which it
satisfies to a part in 10^6; and against the published flight condition.

| Quantity | galata | published |
|---|---|---|
| Dynamic pressure | 61.78 psf | 61.7 psf |
| Mach | 0.2042 | 0.204 |
| Trim lift coefficient | 0.8084 | 0.813 |
| Angle of attack | 2.1481 deg | 2.2 deg |

The trimmed angle of attack sits 0.0519 deg below the published
value, and the difference is understood rather than tolerated. The published
pair (alpha and C_L) is related by the conventional level-flight relation
C_L = W/(qS), which neglects the vertical component of drag in body axes.
galata solves the exact balance, which needs a slightly smaller C_L. The shift
is exactly `D tan(a)/(qS)/C_L_alpha`, and a test asserts it is that term and
nothing else.

**Linearised dimensional derivatives** against Table II-7 — seven numbers the
report computed from the same non-dimensional set by a different route:

| Derivative | galata | published | difference |
|---|---|---|---|
| Y_v | -0.124902 | -0.125 | 0.08% |
| L_beta' | -5.49695 | -5.49 | 0.13% |
| N_beta' | 0.667796 | 0.667 | 0.12% |
| L_p' | -2.0353 | -2.03 | 0.26% |
| N_p' | -0.115922 | -0.116 | 0.07% |
| L_r' | 0.64184 | 0.641 | 0.13% |
| N_r' | -0.207034 | -0.207 | 0.02% |

The source prints its inputs and its outputs to three significant figures, so
each carries up to about 0.5% of its own rounding and several combine in every
one of these. The gate is 0.5%; the worst observed is 0.26%.

**All five modes**, from the same linearisation:

| Mode | galata | published | difference |
|---|---|---|---|
| Phugoid zeta | 0.094852 | 0.0948 | 0.05% |
| Phugoid omega_n | 0.1714 | 0.172 | 0.35% |
| Short period zeta | 0.62193 | 0.622 | 0.01% |
| Short period omega_n | 1.595 | 1.59 | 0.32% |
| Spiral 1/T | 0.031902 | 0.0318 | 0.32% |
| Roll subsidence 1/T | 2.1992 | 2.2 | 0.04% |
| Dutch roll zeta | 0.060259 | 0.0609 | 1.05% |
| Dutch roll omega_n | 1.1293 | 1.13 | 0.06% |

The worst is Dutch roll zeta at 1.05%.

**An error this comparison caught.** The first version of the model treated the
report's lateral derivatives as body-axis when the report gives them in
stability axes. At a trim angle of attack of about two degrees that looks like a
0.07% effect, since cos(2.2 deg) = 0.9993. It is not: the rotation MIXES the
rolling and yawing moments, and C_l_beta is 2.6 times C_n_beta, so the cross
term dominates and C_n_beta moves by 10%. The Dutch roll damping came out 35%
high. It was the derivative-by-derivative comparison above that localised it —
the modes alone said only that something was wrong.

A second error was found the same way: the force assembly rotated the
lift/drag/side triple through the full wind-axis rotation, which adds a
`-D sin(beta)` term to the body y force that a body-axis C_Y_beta already
contains. Double-counting it inflated the side-force derivative by 19%.

Both are the reason `models/nt33a/nt33a-fc1.yaml` must DECLARE which axes its
lateral derivatives are in, rather than defaulting.

## Rigid-body dynamics

**Reference.** These cases carry no transcribed numbers: the reference is an
analytic solution of Euler's equations, with the derivation written out in
`tests/validation/test_rigid_body_dynamics.cpp` so a reader can check it against
the equations rather than against a table. That makes them the strongest
validation in the suite — there is no transcription step to get wrong, and the
expected values are exact rather than rounded.

H. Goldstein, C. P. Poole and J. L. Safko, *Classical Mechanics*, 3rd ed.,
Addison Wesley, 2002; L. D. Landau and E. M. Lifshitz, *Mechanics*, 3rd ed.,
Butterworth-Heinemann, 1976.

**Conservation under torque-free motion.** Asymmetric body with a non-zero
product of inertia, 60 s of integration at a 1 ms fixed step — 60,000 RK4 steps.

| Invariant | Worst relative drift over 60 s |
|---|---|
| Rotational kinetic energy | below 1e-13 |
| Angular momentum vector, resolved in NED | below 1e-13 |

The angular-momentum figure is the vector in the NAVIGATION frame, not its
magnitude in body axes. That is deliberate and it is the stronger claim: the
magnitude is conserved by the rotational dynamics alone, whereas the vector
being fixed in NED requires the attitude kinematics and the rotational dynamics
to agree with each other. A transposed direction-cosine matrix conserves the
magnitude and fails this.

RK4 is not symplectic, so this drift is secular rather than oscillatory — it
grows with integration length rather than staying bounded. Over the tens of
seconds a flight simulation runs it is far below every other error in the
model; over an orbit it would not be, and this is the wrong integrator for that.

**Torque-free precession.** For a body symmetric about its z-axis the
transverse angular-velocity vector rotates in the body frame at
`lambda = (Ia - It) n / It` with constant magnitude. Both an oblate case
(`lambda > 0`) and a prolate case (`lambda < 0`, where the precession runs the
other way round the body) are checked, and the check is that the error falls
like `h^4` as the step halves — not merely that it is small at one step. A
solution converging to the *wrong* closed form would sit at a small constant
error and pass an absolute check while failing this one outright.

**Intermediate-axis instability.** Rotation about the intermediate principal
axis is unstable with growth rate
`sigma = W sqrt((I3 - I2)(I2 - I1) / (I1 I3))`. Starting from a perturbation in
`e1` with `e3 = 0` gives `e1(t) = a cosh(sigma t)` and
`e3(t) = a sigma I1 sinh(sigma t) / ((I2 - I3) W)` — a cosh, not an exponential.
Both are asserted pointwise to a relative tolerance of 1e-4 over four
e-foldings. Rotation about the major and minor axes is checked to remain
bounded.

**Mass-property guard rails.** `MassProperties::validate()` rejects a
non-positive mass, an asymmetric inertia tensor, an indefinite one, and one
whose principal moments violate the triangle inequality `Ia + Ib >= Ic`. The
last of these is the one that catches a moment quoted about the wrong axis,
which passes every other check.

## Aircraft modal characteristics

**Reference.** R. K. Heffley and W. F. Jewell, *Aircraft Handling Qualities
Data*, NASA CR-2144, Systems Technology Inc., December 1972.
NTRS 19730003312, <https://ntrs.nasa.gov/citations/19730003312>. The report's
own documentation page prints "Distribution Statement: Unclassified -
Unlimited", with no copyright notice and no limited-rights legend.

Aircraft NT-33A at flight condition 1 — sea level, M = 0.204, power approach.
The derivatives, the flight condition and the published modal results are
transcribed in `tests/validation/reference/nt33a_fc1.csv`; the state matrices
are assembled from the report's own equations (Appendix C pages C-1 and C-3),
because that is the only way to compare against the report's own answers.

**What is being validated.** The eigenvalue decomposition, the modal metrics,
the participation factors and the classification. Not a trim-and-linearise
chain — neither capability exists yet.

**How the tolerance is set.** Both sides of the comparison are rounded: the
report prints its derivatives to three significant figures and its modal
results to three. So the test perturbs each input by half a unit in its own
last printed digit, accumulates the resulting spread in each modal quantity,
and adds the published value's own rounding. The gate is that the disagreement
falls inside that band. The tolerance is therefore a property of the source
document rather than of the author's patience.

One subtlety that is easy to get wrong and does change the answer: the step
must be taken on the value AS PRINTED and then carried through whatever unit
conversion the value went through. `M_u*` is printed per second-foot; taking
half a unit in the last digit of its per-second-metre form understates its
uncertainty by the conversion factor of 3.28.

**Result.** Six of the seven published modal quantities reproduce within that
band: the spiral root, the roll-subsidence root, the Dutch-roll frequency and
damping, the phugoid frequency, and the short-period frequency and damping. All
five modes are also labelled correctly by participation factor alone, with
scores between 0.72 and 0.99. The Dutch-roll period, published separately in
Table II-10, agrees to within 1%.

### The phugoid damping ratio: localised, and no longer open

**Status changed.** This was recorded as an unexplained discrepancy. It is now
localised to the hand-assembled state matrix, and it does not appear in the
full chain.

Two independent routes to the same published number:

| Route | Phugoid zeta | vs published |
|---|---|---|
| State matrix assembled by hand from the report's Table II-3 dimensional derivatives | 0.0929 | 2.04% |
| Nonlinear model, trimmed, then linearised by central differences | 0.0949 | 0.05% |

The second route takes the report's NON-dimensional derivatives, builds a
nonlinear aircraft, finds its trim, and perturbs it. It shares no arithmetic
with the first beyond the source data, and it reproduces the published value.

So the discrepancy is in the hand assembly, not in the eigen-analysis and not
in the published value. What has NOT been established is which term the hand
assembly omits. The strongest remaining candidate is unchanged: the report
leaves `X_udot`, `X_wdot`, `X_q`, `Z_udot` and `M_udot` blank for this aircraft
and the hand assembly reads those blanks as zeros, while the nonlinear model
never needs them because it differentiates the forces directly.

The regression lock on the hand-assembled route stays, and is now anchored to
the trim-and-linearise result rather than to a bare measurement.

### Detail of the hand-assembled discrepancy

The phugoid damping ratio does **not** reproduce within the source's precision.
It disagrees by about 2% relative, which is roughly three times the band the
inputs' own rounding allows.

The mode itself is in very nearly the right place. Published, the phugoid
eigenvalue is -0.016306 ± 0.171225j; the hand
assembly gives -0.015965 ± 0.171168j. The imaginary parts
agree to a few parts in ten thousand. The disagreement is concentrated in a
derived quantity: ζ = |Re| / |λ| divides a small real part by a small natural
frequency, so a residual of a few times 1e-4 in the real part becomes
2.04% in the ratio — the entire discrepancy, accounted for but
not explained.

Two candidate explanations, neither confirmed:

1. **Terms read as zero that may mean "not supplied".** `X_udot`, `X_wdot`,
   `X_q`, `Z_udot` and `M_udot` are blank in the report's table for this
   aircraft and are taken as zero. `X_q` enters the phugoid directly through
   the `(−X_q + W_o)s` term of Appendix C's first row.
2. **Untranscribed coefficients.** The report's own longitudinal quartic is
   written in terms of `M_alpha` and `M_alpha_dot`, and its `D` and `E`
   coefficients were not transcribed. The phugoid roots are set by the
   low-order coefficients, so an omitted term would show there first.

Until this is resolved the phugoid damping ratio is listed as an open
discrepancy rather than as validated, and a **regression lock** — labelled as
one, per charter rule 8 — holds the gap at its measured size so it cannot grow
unnoticed, and fails if it shrinks, since that would mean the cause has been
found and the lock should become a validation.

---

Generated from galata 0.1.0 by `tools/validation/report_main.cpp`.
