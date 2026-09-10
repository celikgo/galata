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

Eight stages: load the plant, trim it at hover, linearise on the attitude-error
chart, select the channels the design may see, solve the LQR, run the law
against the nonlinear plant at 250 Hz with two periods of delay, write the time
history, write the report.

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
question no capability in this repository answers yet.

**Not an ESC model.** Commands are rotor speeds in rad/s. The map from a rotor
speed to an electrical command is outside RFC-0002's scope and is not modelled.

**Not evidence about any aircraft.** `quad-heterogeneous.yaml` is synthetic —
its four thrust coefficients are the nominal 1.0e-5 scaled by hand — and its own
header says so. The LQR weights are the study's choice, declared in `study.yaml`
rather than defaulted, and they are not tuned against any flight-quality
requirement.

## What checks this

`ExampleQuadrotorSampledControl.RunsEndToEnd` and
`ExampleQuadrotorSampledControl.TheLawDrivesTheDisplacementOut` in the
`integration` tier run this exact study file and require the closed loop to
remove the declared initial displacement. The sampled logic itself — the law,
the saturation and the delay line — is re-derived independently from the
recorded states by `QuadrotorWorkflow` in the same tier, so a loop that sampled
at the wrong instant or shifted its delay by one tick fails there rather than
here.
