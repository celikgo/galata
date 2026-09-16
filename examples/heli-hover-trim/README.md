# Souxmar F1 hover equilibrium and its linearisation

<!-- SPDX-License-Identifier: Apache-2.0 -->

```bash
galata run examples/heli-hover-trim/study.yaml --output-dir build/heli-hover
```

Trims the helicopter in the hover, linearises about that point, and classifies
the modes.

## What the run answers

Where the four pilot controls sit, what attitude the aircraft holds, what the
rotors demand, and what the aircraft does when it is disturbed.

Nine unknowns are solved against nine residuals: roll, pitch, collective, both
cyclics and pedal against the six body force-and-moment equations, plus the two
rotor inflow states and the engine torque against their own rates. The last
three are states with their own equilibria, and leaving them out produces a point
that balances the airframe's forces while the rotor's wake and the drivetrain are
still accelerating — a mistake this project made once and caught only when
`linearize.vehicle` refused the point.

## Two results worth reading twice

**The aircraft banks.** About 3.5 degrees of right roll in a still-air hover. That
is not an error: the tail rotor pushes sideways, so the lateral forces balance
only at a small bank angle. Every single-main-rotor helicopter does it.

**The hover is unstable.** The modal table carries an oscillation with negative
damping — about 0.155 rad/s, a 41-second period, doubling in 26 seconds. That is
correct and it is the point: a hovering helicopter is unstable, which is why a
pilot is continuously correcting it and why a stability augmentation system
exists. A model that hovered stably would be wrong in a way no tolerance would
catch.

## What it does not answer

Anything about a real helicopter. The model is a preliminary **design study** —
no Souxmar aircraft has been built, flown or measured — and
[`models/souxmar-heli/PROVENANCE.md`](../../models/souxmar-heli/PROVENANCE.md)
separates the thirteen parameters taken from that package from the six derived
and the twenty assumed.

The inertia tensor is the one to be most careful with: it is a
radius-of-gyration estimate, not a mass-properties statement, because the design
package carries no lateral offsets. Every modal frequency in the table inherits
its uncertainty, and §3 of the provenance file states the direction and rough
size of the error.

The run also reports that the rotor is within one diameter of the ground, which
this model does not model: there is no ground effect, so the collective and power
it reports at zero altitude are **too high**.
