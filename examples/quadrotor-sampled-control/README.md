# A sampled law, flown against the plant it was designed from

A controller designed on a linearisation and then checked on that same
linearisation has been checked against its own assumptions. This example closes
the loop the other way: it designs on the linearisation and then flies the law
against the **nonlinear** plant the linearisation was taken from, executing at a
declared rate, with a declared delay, and with the rotors free to run out of
authority.

The vehicle's four rotors are not identical, so the chain also exercises the
heterogeneous trim path end to end.

## Run it

```bash
galata run examples/quadrotor-sampled-control/study.yaml
```

Twelve stages: load the plant, trim it at hover, linearise on the attitude-error
chart, select the channels the design may see, solve the LQR, report what the
model can reach and what the sensors can see, read the margins one loop at a
time, run the law against the nonlinear plant at 250 Hz with two periods of
delay, write the time history, write the report.

## What it demonstrates

**The design and the run answer different questions, and both are asked.**
`synth.lqr` designs against a 16-state linearisation with the wind columns
dropped — a law fed back on a disturbance it cannot measure is not a law anybody
can fly. `sim.sampled` then executes that law at a fixed period against
`model::Quadrotor`, holding each command over the period and applying it two
periods late. Nothing about the design guarantees the run, which is why the run
is here.

**Requested and applied commands are both recorded.** `sampled-run.csv` carries
both at every tick. A file with only the applied command hides a controller that
spent the run against its limits; a file with only the requested command
describes a vehicle that was never flown. The report gives the worst
single-channel saturation residual over the run, which is the honest measure of
how much authority the law asked for and did not get.

**What the model can reach, and what the sensors can see, before the design is
trusted.** `analyze.gramians` reports the reachable and observable subspaces of
the chart coordinates for this input and output set, and names the directions
that fall outside them. In **the observation model this study selects**, the
heading is unobservable: `linearize.extended` is asked for body rates, position,
altitude, ground velocity and specific force, and none of those is a heading
reference. The report says so by name. Without it, that fact arrives later as a
Riccati diagnostic from a synthesis that failed, and the reader has to work
backwards to which coordinate it was.

That is a statement about **this declared output set**, not about any vehicle's
sensors. A real airframe may well carry a heading reference — a magnetometer,
say — and if it does, the model that omits one is still unobservable in yaw
while the aircraft is not. Making galata see such a reference would mean adding
a magnetic observation to the observation model, which is a **model extension**
and not a change to anything here.

The Gramians are integrals over a **declared** horizon, and the capability
refuses to default it. The infinite-horizon Gramians do not exist for a hover
linearisation at all: six eigenvalues sit at the origin, so the limit diverges
and the matrix a Lyapunov solve would return for it is not a Gramian of
anything. The report states that rather than leaving a reader to assume the
textbook quantity was computed.

**Frequency-domain margins, read one loop at a time with the others closed.**
Handing the MIMO return ratio to a SISO margin routine breaks one channel and
leaves the other three *open* — a vehicle flying with most of its controller
disconnected. That closure is not internally stable, and `analyze.margins`
refuses it and names the cause. The loop-at-a-time reading closes back to the
design's own closed loop, so the Nyquist test is well posed and the margin
exists.

**Three robustness questions, and this study answers two of them.** They are
easy to conflate and the difference matters:

| Question | Answered by | Scope |
|---|---|---|
| Tolerable *pure* gain change, or *pure* phase change | `analyze.margins` | one channel, others at nominal |
| Tolerable *simultaneous gain and phase* variation | `analyze.diskmargin` | **still one channel** — a SISO condition |
| Tolerable *simultaneous variation across channels* | **nothing here** | the multi-loop case needs a structured singular value, which galata does not have |

Running the disk margin on each single loop in turn does **not** add up to the
third row. The classic counterexample perturbs two channels together while every
loop-at-a-time figure, disk margins included, stays comfortable.
`analyze.sensitivity` and `analyze.sigma` give MIMO *peaks*, which qualify a
design but are not a structured robustness margin either. And none of the three
says anything about the sampled loop.

**One continuous figure is worth comparing against the sampled implementation:
the delay margin.** The loop executes at 250 Hz with two periods of transport
delay, and the hold contributes roughly half a period more. This study's four
channels are comfortably clear of that equivalent lag — and that is not
automatic: a design on the same plant at the same rate with unit state weights,
penalising the rotor-speed states as hard as position, falls the *wrong* side of
it. This study weights those states at a thousandth, which is one of the reasons
its weights are declared in `study.yaml` rather than defaulted.

**It is a comparison, not a stability condition — in either direction.** Three
reasons, and they are why this README does not call it necessary or sufficient:

- **The hold is not a delay.** A zero-order hold's low-frequency phase lag is
  *approximately* that of a half-period delay, and only well below the sample
  rate; it also reshapes the loop's magnitude. Adding half a period is an
  approximation of the hold, not a model of it.
- **A continuous delay margin bounds a continuous perturbation.** Applying it to
  a sampled loop compares a figure computed for one system against a lag
  appearing in a different one.
- **So a sampled loop can be stable past the margin, or unstable inside it.**

What the comparison gives is a warning sign in one direction: a design whose
continuous delay margin is only a small multiple of its transport delay is one
to examine with discrete-time tools before flying, and this repository has none.

Every figure here is a property of **this** LQR design and **this** loop
construction — not of the plant, not of the sample rate, and not of any other
controller that happens to run at the same rate.

**The trim is a real equilibrium of a vehicle whose rotors differ.** Four
different thrust coefficients mean four different equilibrium speeds and no
vehicle-wide hover speed to seed the solve from; `trim.hover` seeds per rotor
and Newton closes the residual. See `examples/quadrotor-heterogeneous-rotors`
for that path on its own.

**The numbers are in the run, not in this file.** `operating-point.yaml` carries
the equilibrium residual, the per-rotor margins and the finite-difference step
sizes; `sampled-control.md` carries the CARE residual, the tick count, the
saturation record and the final state; `sampled-run.csv` carries the whole
history. A figure typed into a README is a figure no later run can contradict.

## What this is not

**Not a statement about the sampled loop's robustness.** Gain, phase and disk
margins computed from the continuous linearisation describe the *continuous*
loop. This loop samples, holds and delays, and its own margins are a separate
question no capability in this repository answers yet. The figures in
`sampled-control.md` are the continuous design's, and the report says so where a
reader will meet them.

**Not a MIMO robustness measure.** Both margin figures in this study are per
channel — the disk margin included, which bounds simultaneous gain *and phase*
in one channel and not simultaneous variation across channels. Nothing here
bounds the across-channel case.

**Not a controllability guarantee under actuator limits.** Every actuator is
unbounded in the Gramian analysis. A direction it reports as reachable may be
reachable only through a rotor speed no motor can produce, and nothing in those
figures says so — the trim's per-rotor margins and the sampled run's saturation
record are where that question is answered.

**Not an ESC model.** Commands are rotor speeds in rad/s. The map from a rotor
speed to an electrical command is outside RFC-0002's scope and is not modelled.

**Not evidence about any aircraft.** `quad-heterogeneous.yaml` is synthetic —
its four thrust coefficients are the nominal 1.0e-5 scaled by hand — and its own
header says so. The LQR weights are the study's choice, declared in `study.yaml`
rather than defaulted, and they are not tuned against any flight-quality
requirement.

## What checks this

`ExampleQuadrotorSampledControl.RunsEndToEnd`,
`ExampleQuadrotorSampledControl.TheLawDrivesTheDisplacementOut` and
`ExampleQuadrotorSampledControl.ReportsWhatTheModelCanReachAndWhatTheLoopTolerates`
in the `integration` tier run this exact study file: they require the closed loop
to remove the declared initial displacement, require the heading to be reported
as unobservable by name, and require the report to say that none of its margins
is a statement about the sampled loop.
`QuadrotorWorkflow.SingleLoopMarginsAreAvailableWhereTheBrokenLoopIsRefused`
holds both halves of the margin argument — the other-loops-open reading refused
with its cause named, and the loop-at-a-time reading yielding a margin — and
`Gramians.*` in the `unit` tier holds the Gramian arithmetic against closed
forms rather than against a previous run.

The delay comparison is held by a pair, because one half alone would measure
nothing.
`ExampleQuadrotorSampledControl.TheTransportDelayIsWellInsideTheContinuousDelayMargin`
requires this study's four channels to clear the equivalent lag by a factor of
five, and
`QuadrotorWorkflow.AFasterDesignRunsOutOfDelayMarginAtTheSameSampleRate`
requires a unit-weighted design on the same plant to fall the wrong side of it —
so that clearing it stays a measurement rather than becoming something nothing
can fail. Neither test claims a sampled-stability result. The sampled logic itself — the law,
the saturation and the delay line — is re-derived independently from the
recorded states by `QuadrotorWorkflow` in the same tier, so a loop that sampled
at the wrong instant or shifted its delay by one tick fails there rather than
here.
