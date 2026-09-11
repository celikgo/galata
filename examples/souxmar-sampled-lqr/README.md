# A sampled LQR on the native Souxmar plant

`examples/quadrotor-sampled-control` flies a **continuous** design at a discrete
rate. That is emulation, and the example says so. This one **designs in discrete
time**. The plant and the cost are discretised under the same hold at the period
the controller will run at, the discrete Riccati equation is solved for that
discretised problem, and the law is flown against the nonlinear plant at exactly
that period. The run is then compared with what the design itself predicted.

The plant is the native Souxmar model, `models/souxmar-quad/souxmar-quad.yaml`,
read in place rather than copied, so this example reads the same model file as
every other Souxmar check.

## Run it

```bash
galata run examples/souxmar-sampled-lqr/study.yaml
```

Nine stages: load the plant, trim it at hover, linearise it on the attitude-error
chart, select the rotor commands, discretise, design the sampled LQR, fly it
against the nonlinear plant, then write the time history and the report.

| Stage | Capability | What it adds |
|---|---|---|
| `plant` | `model.quadrotor` | The native model. Its rotors carry a first-order speed lag. |
| `hover` | `trim.hover` | The equilibrium at 120 m. |
| `linear` | `linearize.extended` | Sixteen chart states. Four of them are the rotor speeds, so the **rotor lag is in the model the design discretises**. |
| `rotors` | `model.channels` | The four rotor commands. The wind columns are dropped, because a law fed back on a disturbance it cannot measure is not one anybody can fly. |
| `discrete` | `model.discretize` | The explicit adapter from continuous to discrete time: a zero-order hold at 4 ms, both declared. |
| `sampled` | `synth.sampled_lqr` | The design, from the **continuous** plant and the **continuous** weights. |
| `closed` | `sim.sampled` | The law flown against the nonlinear plant at 250 Hz, with a declared hold, a declared one-period delay and per-rotor speed limits. |
| `history`, `report` | `report.csv`, `report.markdown` | Every tick, and the account of it. |

## What it demonstrates

**Two time domains, and one door between them.** A continuous model is a
`linear_system` and a discrete one is a `discrete_linear_system`. These are two
artefact kinds, not one kind with a flag, so a stage wired to the wrong one is
refused by name before any number is computed:

- `analyze.modes` handed a discrete model is refused.
- `synth.dare` handed a continuous model is refused, and the refusal names
  `model.discretize` as the adapter.
- `synth.sampled_lqr` handed a discrete model is refused, because the cost
  discretisation needs the continuous plant.

`model.discretize` is the only adapter, and it runs in one direction only.
Neither the hold nor the sample time has a default.

**The cost is discretised, not reused.** Holding the input across an interval
makes the state move *through* the interval, so the exact cost of that interval
has a state-input **cross term**, even though the continuous cost declared in
`study.yaml` has none. `synth.sampled_lqr` computes that term exactly and solves
with it. The report shows it, and `sampled-lqr.yaml` records it next to the
declared continuous weights. Dropping it would give a gain that is optimal for
some other objective.

**The conventions are stated where the numbers are.** The cost is the sum over
ticks of `x' Q x + 2 x' N u + u' R u`, with weights per sample. The gain is
applied as `u[k] = -K x[k]`, with `K = (R + B'XB)^-1 (B'XA + N')`. The Riccati
residual is the residual of the equation *as posed*, cross term included. A
solution over its budget is refused, never returned, and no caller can widen
that budget. The closed loop must have every eigenvalue strictly inside the unit
circle, and the symplectic spectrum must stay off the circle; failing either is
also a refusal. The report and the evidence file both carry these statements.

**The period is the design's.** A discrete gain is optimal for the period its
plant and cost were discretised at, and for no other. `sim.sampled` refuses to
run a `sampled_control_law` at any other period, faster or slower. Galata has no
way to convert a discrete gain from one rate to another, so the only remedy is to
redesign at the new period, and the refusal says so. For the same reason, a
discrete design flown by `sim.sampled` must declare its `hold` and its
`delay_periods`. Defaults written for continuous laws are not inherited.

**The comparison is against the design's own prediction.** With a discrete law,
`sim.sampled` also computes the loop the design predicts: the same discrete
model, gain, delay and hold, started from the same chart state. The one-period
delay is represented exactly, as one extra period of input memory. The only
things the prediction lacks are the nonlinearity and the actuator limits, so the
difference between prediction and run is attributable to them. The run did not
saturate, which the tests require; saturation would be outside the prediction's
premise.

The chart coordinates mix metres, radians and radians per second, so no plain
vector norm of the difference has a unit. The headline figure uses the design's
own cost-to-go norm, `sqrt(e' X e)`, and the report also tabulates each
coordinate in its own unit. `sampled-run.csv` has the measured and predicted
chart coordinates side by side at every tick.

## The small-perturbation budget, and what the first run found

The budget was set in `study.yaml` before the study had ever run: five percent
of the prediction's peak, for the declared perturbation of decimetres of position
and a hundredth of a radian of attitude. The derivation behind it is written
beside it and has not been edited since. That order of events is the author's
account, and the repository cannot confirm it: the study, this finding and its
tests all arrived in one commit. It compared each of the plant's
nonlinearities with its own linear part: the ω² thrust law, the quadratic drag,
and the attitude kinematics.

**The run falls just outside that budget.** The budget was not moved, and the
perturbation was not shrunk to fit. The discrepancy was localised instead:

- **It is second order in the perturbation.** Halving the perturbation divides
  the miss by about four. So the prediction is right to first order, and what is
  being measured is the linearisation's own error. A structurally wrong
  prediction would only halve the miss. The negative control below shows that
  happening for a prediction made at the wrong period.
- **It is not the drag.** The quadratic drag is the term a hover linearisation
  cannot see, and the obvious suspect. Setting its coefficients to zero leaves
  the discrepancy essentially where it was, and still outside the budget.
- **It is the rigid-body kinematics the hover linearisation drops.** While the
  loop removes the horizontal offset, the vehicle pitches and rolls while it
  carries forward and sideways speed. Three second-order products follow, and
  the linear model has none of them:
  - the transport of velocity by the rotating body axes, `−ω × v`, which puts
    `q·u − p·v` into the body vertical velocity. This is the largest single
    term.
  - gravity projected onto a tilted body, whose component along the body's own
    vertical axis falls short of `g` by about half the square of the tilt. This
    partly offsets the first.
  - the rotation of body velocity into NED position, `(R − I) v`.

  Nearly all of the discrepancy, in the cost-to-go norm, sits in the body
  vertical velocity and the down position. The comparison is made in the chart,
  whose velocities are in body axes. So part of that vertical miss is a tilted
  vehicle's forward speed appearing along its own vertical axis, rather than a
  miss in how fast it climbs. The metric counts both, and this account is of
  the metric.
- **It is not the rotor-speed curvature.** An earlier version of this section
  said it was: that squaring a differential rotor command produced a collective
  thrust the derivation had not counted. Traced term by term, that curvature
  carries almost none of the discrepancy, and with the opposite sign. The rotor
  speeds do miss their prediction almost equally at the worst tick, but that is
  the feedback answering the vertical miss through the gain. The rotor equations
  themselves are forced by nothing the linearisation drops.
- **Nothing is broken.** The discretisation, the Riccati gain, the delay line
  and the prediction's recursion were each re-derived independently, and each
  agrees to round-off. Carried through the delayed loop, the named terms account
  for the whole discrepancy.

`ExampleSouxmarSampledLqr.TheOverBudgetDiscrepancyIsTracedToTheKinematicsTheHoverLinearisationDrops`
holds that account. It evaluates each term along the recorded run and carries
it through the design's own delayed loop. It then requires three things:

- the terms together close the discrepancy to within a tenth;
- the kinematic group carries the discrepancy, and the transport term is the
  largest single contributor;
- the rotor curvature and the drag each carry under a twentieth.

The finding is held by a two-sided labelled lock,
`ExampleSouxmarSampledLqr.TheOverBudgetDiscrepancyIsHeldByATwoSidedLock`. It
fails if the discrepancy grows. It also fails if the discrepancy comes back
inside the budget, because then this section is stale and must be revisited.

**And one thing the comparison cannot see.** The negative control was first
written as a prediction that leaves out the one-period delay, and that control
*passed* the order gate when it should have failed it. On a loop this slow, at
250 Hz, one tick of delay changes the prediction by less than the
linearisation's own second-order error at this perturbation. So the comparison
does not certify the delay line, and nothing here claims it does. The delay is
certified by exact re-derivation instead:

- `QuadrotorWorkflow.TheDiscreteLawIsExecutedAsDesigned` checks every applied
  command against the one computed a period earlier.
- `DiscretePrediction.AMultivariableDelayedLoopMatchesTheAugmentedStateMatrix`
  checks the prediction's delay queue against powers of the augmented-state
  matrix.

The negative control is now a prediction made at *twice* the design period.
That is the mismatch `sim.sampled` refuses to run, and the gate catches it. The
delay test was kept, inverted, as the record of this limit.

## The valid envelope, and a proposed change to the acceptance case

**The agreed case is `study.yaml`, and its result is outside the budget.**
Nothing below changes that.

The absolute miss is second order and the prediction's peak is first order, so
the relative discrepancy grows in proportion to the perturbation. Along the
declared direction there is therefore one scale below which the budget holds and
above which it does not. The declared perturbation sits just past it:

- `ExampleSouxmarSampledLqr.TheBudgetHoldsAtNineTenthsOfTheDeclaredPerturbation`
  runs the study at nine tenths of the declared perturbation, and it passes.
- `ExampleSouxmarSampledLqr.TheOverBudgetDiscrepancyIsHeldByATwoSidedLock` holds
  the declared perturbation itself outside the budget.

The boundary lies between the two. The horizontal part of the offset sets it. A
vertical offset is followed almost exactly by the linear model, and adding one
*lowers* the metric: it raises the prediction's peak without adding much miss.
**So a perturbation can be made to pass by adding excitation the linearisation
handles well.** A pass is worth something only if it also holds on the
demanding part of the excitation alone.

**Proposed, not adopted.** `proposed-acceptance.yaml` is this study with one
change: the initial perturbation is halved. Everything else is unchanged: the
budget, the metric, the law, the period, the hold, the delay and the limits. A
test requires the file to equal `study.yaml` apart from that one vector. At half
the excitation the run passes, and so does its most demanding constituent, the
horizontal offset alone:
`ExampleSouxmarSampledLqr.TheProposedCaseAtHalfTheExcitationPassesTheUnchangedBudget`.

The proposal has a cost. The miss is second order, so halving the perturbation
quarters the miss the comparison can see, and its resolution against a
structural error falls with it. The one-tick-delay limit above applies to the
proposed case more strongly, not less. Adopting it is a decision for whoever
agreed the acceptance case, not for this repository.

```bash
galata run examples/souxmar-sampled-lqr/proposed-acceptance.yaml --output-dir <out>/proposed
```

## What this is not

**Not a margin of the sampled loop.** None of the stages computes a gain,
phase, delay or disk margin of this loop, and none is implied. A Riccati
solution whose closed loop lies inside the unit circle says the *nominal* sampled
loop converges. A run that converged is one trajectory from one initial state.
Neither says how much gain, phase or delay the loop tolerates. That question
needs its own machinery and a choice between several inequivalent definitions
of a sampled margin, and galata has neither. The continuous margin path does not
fill the gap either: `model.control_system` refuses a `sampled_control_law` by
its artefact kind.

**Not a delay-aware design.** The design modelled no delay. The one period this
run applies is part of a plant the design did not see. The prediction includes
the delay; the gain does not account for it.

**Not a constrained design.** Actuator limits were not modelled. The run
clamps each rotor to its own speed range and records whether it had to. The
comparison is meaningful only because it did not.

**Not a model of the inter-sample response.** The discrete model and the
prediction describe the state at the ticks. The nonlinear run integrates between
them, but the comparison is made at the ticks only.

**Not evidence about the aircraft.** The Souxmar coefficients are an
independent implementation's nominal set, not measured data, and
`models/souxmar-quad/PROVENANCE.md` says so. The weights are this study's
choice. They are declared rather than defaulted, and they are not tuned against
any flying-qualities requirement.

## What checks this

`ExampleSouxmarSampledLqr` in the `integration` tier runs this exact study
file:

- `ExampleSouxmarSampledLqr.RunsEndToEndOnTheNativePlant` checks that the model
  is the native one, and that the adapter and the design discretise
  identically. It checks that the hold's cross term reaches the evidence file,
  and that the law is flown at its own period with the declared hold and delay
  and without saturating. It re-derives the headline discrepancy from the
  recorded vectors, and checks that the report does not let the comparison be
  read as a margin.
- `ExampleSouxmarSampledLqr.TheDisagreementIsSecondOrderInThePerturbation`
  runs the study at the declared perturbation and at half of it, and requires
  the observed order of the miss to clear the midpoint between first and second
  order.
- `ExampleSouxmarSampledLqr.APredictionAtTheWrongPeriodFailsTheSameOrderTest`
  is the negative control. The same two runs, compared with a prediction made
  at twice the design period, must *fail* that gate. Without this test the gate
  would show nothing.
- `ExampleSouxmarSampledLqr.AOneTickDelayErrorIsBelowThisComparisonsResolution`
  records the limit described above, and fails if the comparison ever starts
  resolving a one-tick delay error.
- `ExampleSouxmarSampledLqr.TheOverBudgetDiscrepancyIsHeldByATwoSidedLock`
  holds the finding above.
- `ExampleSouxmarSampledLqr.TheOverBudgetDiscrepancyIsTracedToTheKinematicsTheHoverLinearisationDrops`
  holds the trace of that finding. It requires the named terms to close the
  discrepancy, and requires the kinematic group, not the rotor curvature, to
  carry it.
- `ExampleSouxmarSampledLqr.TheBudgetHoldsAtNineTenthsOfTheDeclaredPerturbation`
  holds the inside edge of the envelope.
- `ExampleSouxmarSampledLqr.TheProposedCaseAtHalfTheExcitationPassesTheUnchangedBudget`
  holds the proposal: that it is `study.yaml` with only the perturbation
  halved, that it passes, and that its horizontal offset alone passes too.

`DiscreteWorkflow` in the same tier holds the three capabilities' contracts:
the scalar DARE's closed form and Riccati value iteration through the pipeline,
the refusals between time domains, the undeclared hold and sample time, unknown
keys, and the unstabilisable, undetectable and invalid-cost cases. It also
checks that a fixed-wing linearisation's evidence survives discretisation and
design as the same object. The `QuadrotorWorkflow` tests on discrete laws hold
the period refusal and the declared timing. They re-derive the executed
commands from the discrete gain, and refuse a law whose inputs are not the rotor
commands in the model's order.

The numbers themselves are in the run, not in this file. `sampled-lqr.yaml`
carries every matrix and every check the design passed. `sampled-lqr.md` carries
the comparison and its verdict. `sampled-run.csv` carries every tick.
