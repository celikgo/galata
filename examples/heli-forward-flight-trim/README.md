# Souxmar F1 forward-flight trim at 40 m/s

<!-- SPDX-License-Identifier: Apache-2.0 -->

```bash
galata run examples/heli-forward-flight-trim/study.yaml --output-dir build/heli-cruise
```

## Why this exists separately from the hover case

**Forward flight is where the linearisation is second order.** At exactly zero
airspeed the advance ratio `mu = |V| / (Omega R)` has no derivative — approach it
from either side and the one-sided slope is ±1 — so a central difference returns
zero for it and the hover linearisation carries no flap-back response to a speed
perturbation. The measured consequence: order **2.004** here against a
non-vanishing error floor of about 3.1e-3 in the hover. Both are gated, and the
hover gate is two-sided so that smoothing the kink away would fail it. The rotor
header's `NOT DIFFERENTIABLE AT EXACTLY ZERO AIRSPEED` block has the full
argument.

The truncation estimate says the same thing from the other side: the worst
column-relative Richardson estimate is **2e-10** here and **3.3e-1** in the hover.

**And the trim is different in kind.** Compared with the hover:

| | hover | 40 m/s |
|---|---|---|
| collective | 15.16 deg | 12.18 deg |
| longitudinal cyclic | −1.38 deg | +5.02 deg |
| pedal | 10.22 deg | 3.49 deg |
| pitch attitude | +1.65 deg | −0.68 deg |
| rotor power | 465.7 kW | 277.7 kW |

Every one of those moves in the direction it should. Collective falls because
translational lift is doing part of the work; forward cyclic rises to overcome
fuselage drag and the rotor's own flap-back; pedal falls because there is less
main-rotor torque to oppose and the fin now carries some of it; the fuselage
pitches nose-down; and power falls because 40 m/s is nearer the minimum-power
speed than the hover is.

## Validity at this condition

`mu = 40 / 195 = 0.205`, inside the model's declared advance-ratio limit of 0.35,
and `C_T/sigma` is inside the design package's own 0.11 screen. Above either,
`envelope()` reports the departure and the thrust and control power become
**optimistic**, because neither retreating-blade stall nor compressibility is
modelled.

Altitude is declared but the atmosphere is **not** yet altitude-dependent in this
capability: `trim.helicopter` builds a sea-level environment regardless. That is a
known limitation, not a modelling choice, and it is listed in the remaining-gaps
table of the rotorcraft work.
