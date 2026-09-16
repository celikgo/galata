# Souxmar F1 rotor-speed droop — recovered, and limited by the drive rating

<!-- SPDX-License-Identifier: Apache-2.0 -->

```bash
galata run examples/heli-governor-droop/study.yaml --output-dir build/heli-droop
```

## Why this is the case that proves the drivetrain is real

Rotor speed is a **state**, driven by rotor inertia against the torque the rotors
demand and the torque the engine supplies. Engine torque is itself a **state**,
relaxing towards the governor's request through the governor's own lag. So a
collective step raises the demand faster than the engine can answer.

**One degree — droop, then recovery.** The governor sees the speed error and
torque rises until the rotor is back at its reference:

| t (s) | rotor speed (rad/s) | droop | engine torque (N·m) |
|---|---|---|---|
| 0.0 | 32.500 | — | 15 730 |
| 0.2 | 32.343 | −0.48% | 16 517 |
| 0.5 | 32.218 | **−0.87%** | 17 653 |
| 1.0 | 32.331 | −0.52% | 18 165 |
| 2.0 | 32.502 | +0.01% | 17 933 |

**Three degrees — the drive rating binds and the rotor stays low.** The demand
exceeds what the drivetrain can pass, the engine torque saturates, and no amount
of governor error can recover the speed:

| t (s) | rotor speed (rad/s) | droop | engine torque (N·m) | |
|---|---|---|---|---|
| 0.0 | 32.500 | — | 15 730 | |
| 1.0 | 30.862 | −5.04% | 20 260 | |
| 2.0 | 30.695 | **−5.56%** | 20 454 | |
| 3.0 | 30.731 | −5.44% | 20 461 | **at the limit** |
| 8.0 | 31.086 | −4.35% | 20 461 | **at the limit** |

The speed recovers only slowly, and only because the aircraft's climb gradually
reduces the demand — not because the governor has any authority left.

The limit that binds is **20 461.5 N·m**, and it is the *drive* rating rather than
the engine's. Two PW207D1 engines could supply about 26 600 N·m at this shaft; the
design package declines that credit explicitly
(`drive.no_credit_for_full_910kw_on_existing_rotor_drive`) and caps the rotor-drive
input at 700 kW, so that is the limit the model carries.
[`PROVENANCE.md`](../../models/souxmar-heli/PROVENANCE.md) §2 records the
derivation.

**Running only the one-degree case would teach the wrong lesson** — that the
governor always recovers. Both are in the study for that reason.

**A model with rotor speed held constant shows none of that** — and this model
held it constant in a first draft. `governor_time_constant_s` was declared in the
YAML and had no effect, because engine torque was computed algebraically from the
demand rather than carried as a state. The rotor speed then came out at exactly
its reference through the whole step, to every printed digit. That silence is
what this example exists to break, and it is why the parameter is now a state
with a rate rather than a number nothing reads.

## What this is not

**Not an engine model.** There is no fuel flow, no compressor map, no spool
dynamics and no temperature limit. The governor is a proportional gain on the
speed error plus a feed-forward of the demand, through one first-order lag.

**Not a droop prediction.** The magnitude above depends on the governor gain, the
governor lag and the rotor's polar inertia, and all three are *assumptions* in
[`PROVENANCE.md`](../../models/souxmar-heli/PROVENANCE.md) §4 rather than
measurements. What the example demonstrates is that the mechanism is present and
behaves the right way round; the number is not a claim about a PW207D1.

**Not a transient limit check.** Real rotor-speed limits are enforced by an
engine control unit with authority this model does not have. The advisory
`minimum_rotor_speed_rad_s` and `maximum_rotor_speed_rad_s` are reported by
`envelope()` and enforced by nothing.
