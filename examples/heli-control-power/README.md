# Souxmar F1 control power — collective, cyclic and pedal steps

<!-- SPDX-License-Identifier: Apache-2.0 -->

```bash
galata run examples/heli-control-power/study.yaml --output-dir build/heli-control-power
```

Three step responses against the **nonlinear** helicopter, from a hover trim,
with each actuator's position limit, rate limit and first-order lag active.

## What each step shows

**Collective, +1 degree.** A heave response — the aircraft climbs, and altitude
rises from 30 m to about 40 m over five seconds. Two other things happen in the
same run: the rotor's torque demand rises, so rotor speed **droops** before the
governor recovers it, and the extra torque is momentarily unopposed, so the
aircraft **yaws**. All three are in one trajectory because they are one event.

**Longitudinal cyclic, +0.5 degree.** A pitch response, and then a forward
acceleration as the tilted thrust vector pulls the aircraft along. That forward
force is the one that was *identically zero* before the rotor had an orientation:
the previous model's rotor thrust was the fixed body-axis vector
`(0, 0, −k_T ω²)`, which has no in-plane component at all.

**Pedal, +1 degree.** A yaw response through the tail rotor's thrust at its
6.7 m moment arm.

## Why the steps are small

Each is a degree or less at the stick. A larger step saturates the actuator's
rate limit, and a response dominated by a rate limit measures the actuator rather
than the aircraft. `sim.helicopter` reports how many samples fell outside the
declared envelope, which is how the study shows the step stayed inside the
model's validity instead of testing its extrapolation.

## What this is not

**Not a handling-qualities assessment.** ADS-33E bandwidth, phase delay and
attitude quickness are read off a frequency sweep, not a step, and galata has no
capability for them.

**Not evidence about any aircraft.** The model is a design study and no published
rotorcraft reference has been compared against. The actuator rate limits and lags
in particular are *assumed* — see
[`PROVENANCE.md`](../../models/souxmar-heli/PROVENANCE.md) §4 — and they are the
parameters a control-law result is most sensitive to.

**Not a high-bandwidth control study.** The rotor's flap dynamics are
quasi-static here, so a law with bandwidth approaching the flap frequency will
look **more** stable in this model than in flight.
