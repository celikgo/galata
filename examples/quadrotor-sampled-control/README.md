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
that fall outside them. On this vehicle the heading is unobservable — the
observation model carries body rates, position, altitude, ground velocity and
specific force, and none of them measures an absolute yaw angle — and the report
says so by name. Without it, that fact arrives later as a Riccati diagnostic
from a synthesis that failed, and the reader has to work backwards to which
coordinate it was.

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

A set of loop-at-a-time margins does **not** bound simultaneous variation: each
can be generous while a small perturbation applied to two channels at once
destabilises the loop. `analyze.diskmargin` is in the study for that reason, and
neither figure says anything about the sampled loop.

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

**Not a MIMO robustness measure.** The loop-at-a-time margins are per channel.
Simultaneous variation is `analyze.diskmargin`'s question, and the MIMO peaks
are `analyze.sensitivity`'s and `analyze.sigma`'s.

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
forms rather than against a previous run. The sampled logic itself — the law,
the saturation and the delay line — is re-derived independently from the
recorded states by `QuadrotorWorkflow` in the same tier, so a loop that sampled
at the wrong instant or shifted its delay by one tick fails there rather than
here.
