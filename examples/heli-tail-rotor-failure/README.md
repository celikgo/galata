# Souxmar F1 hover hold, as the control case for a tail-rotor failure

<!-- SPDX-License-Identifier: Apache-2.0 -->

```bash
galata run examples/heli-tail-rotor-failure/study.yaml --output-dir build/heli-tr-failure
```

## Why this example exists

It is the demo that was **structurally impossible** to write against the previous
model.

Rotor thrust used to be the fixed body-axis vector `(0, 0, −k_T ω²)` — one line,
`src/model/quadrotor.cpp:355`. A tail rotor built from it produced a *downward*
force and a *pitching* moment: not a side force, and not a yaw moment. There was
no anti-torque in the model, so there was nothing to lose. Being able to pose this
question at all is the evidence that the readiness audit's first blocking
capability is closed.

## What the run contains

A trimmed hover, held with the controls fixed for four seconds. This is the
**control case**: it must stay put, and a trimmed aircraft that drifted here would
invalidate any failure comparison made against it.

The failure itself is applied through `HelicopterFailures::tail_rotor_effectiveness`,
which is a multiplier rather than a special case in the physics, so a failed run
takes exactly the same code path as a healthy one. Two unit tests exercise it
directly:

- `Helicopter.TailRotorFailureRemovesTheAntiTorqueAndLeavesAYawMoment` — with the
  tail rotor gone, a yaw moment of over 1 kN·m remains where the trimmed aircraft
  had none, and it acts in the sense the main rotor's torque does.
- `Helicopter.EngineFailureRemovesTheSuppliedTorqueSoTheRotorDecays`.

The failure is not yet reachable from a study file: no capability input exposes
it. That is a gap, and it is listed as one rather than worked around here.

## What this is not

**Not a survivability assessment, and not a recovery procedure.** A real pilot's
response — collective down to cut the torque, forward cyclic to gain the airspeed
that gives the fin its authority — is not modelled. The controls are held at their
trim values, so a failure run shows the *uncorrected* divergence and nothing more.

**Not valid into autorotation.** The rotor decays under its own drag with the
engine failed, which is the right first-order behaviour, but the autorotative
energy exchange that keeps a real rotor turning is not modelled at all. A decay
computed here is **faster** than the real aircraft's and must not be read as a
descent-rate prediction. The helicopter header's `WHAT THIS IS NOT` block says so
in full.

**No ground contact.** The model will fly through the ground without noticing.
