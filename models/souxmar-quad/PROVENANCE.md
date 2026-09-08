# souxmar-quad — provenance

## Source

The Souxmar forest-ISR quadrotor programme's nonlinear plant, the module
fcs/plant/quadrotor.py in the GitLab repository `souxmar`. That path is in the source
programme's tree, not in this one. This is the Python programme that requested RFC-0002.
It is **not** the CAE project of the same name that [ADR-0001](../../docs/adr/0001-independent-c-abi.md)
cites as the origin of the plugin-ABI pattern; the two are unrelated and share only a name.

The numbers below were read from the parameter block the programme publishes with its
open-loop cross-implementation fixture:

```text
outputs/galata_bridge/reference_trajectory.json    vehicle_parameters
outputs/galata_bridge/quad_hover_ned_frd.sidecar.json    vehicle_parameters, operating_point, validity
```

Both files are regenerated from seed and configuration by

```text
python -m apps.galata_bridge --output outputs/galata_bridge --galata <galata-cli>
```

so the parameter set is reproducible from the source programme rather than transcribed from
a document. The fixture's own `csv_sha256` field records the digest of the trajectory the
parameters produced.

## Rights

**This is not a published source and this model is not validated against one.**

The parameter set is an independent implementation's nominal coefficients, contributed by
the requesting programme for the purpose of cross-checking two implementations of the same
equations. It is not measured aircraft data, and no published quadrotor reference anchors
it. [ADR-0007](../../docs/adr/0007-reference-values-from-copyrighted-sources.md) governs
what may ship in-tree: a small parameter set quoted to make an independent implementation
runnable is not the dataset case that ADR forbids, and it is not a scalar result quoted
from a copyrighted worked example either. It is the requesting programme's own
configuration, published by them for this use.

The **trajectory** that these parameters generate is a different matter and does not ship.
It is a 2001-sample dataset, its rights position is unestablished — the source tree carries
no licence file — and ADR-0007 routes a dataset to a loader plus fetch instructions rather
than into the tree. RFC-0002's acceptance section records that decision. The
cross-implementation validation case reads the trajectory from its declared path and states
why it did not run when the path is absent.

## Transcription

Every value was read directly from the JSON files named above; none was read from a
rendered document, and no OCR was involved. There was no unit conversion to perform: the
source publishes SI throughout, in keys that carry their units, which is why the values
here are exact copies rather than rounded transcriptions.

The transcription was checked against a quantity the source computes independently. The
published hover rotor speed is `hover_rotor_speed_rad_s`, and the same quantity formed from
the transcribed mass, thrust coefficient and rotor count through `sqrt(m g / (n k_T))`
reproduces it to the last printed digit. A transcription error in mass, in the thrust
coefficient or in the rotor count would break that identity.

## The aircraft and the condition

A nominal 1.6 kg quadrotor in X configuration on a 0.23 m arm, with a diagonal inertia
tensor. This is the programme's simulation vehicle, not the aircraft it is building: the
built airframe is a 650 mm, roughly 2.5 kg machine that will carry a measured full inertia
tensor and measured hub positions when it exists. Nothing in this directory describes that
aircraft, and the model accepts a full tensor and arbitrary hub positions when it does.

The model is a full nonlinear plant rather than an expansion about one flight condition, so
there is no condition suffix on the file name. Its validity envelope is the header's
`WHAT THIS IS NOT` block in
[`include/galata/model/quadrotor.hpp`](../../include/galata/model/quadrotor.hpp), and the
airspeed scale below which a hover linearisation means anything is the per-axis ratio
`drag_linear / drag_quadratic`, which for this parameter set is 4.8 m/s on the horizontal
axes and about 5.14 m/s on the vertical.

## Conversions

**None were applied.** The source publishes metres, kilograms, seconds, newtons,
newton-metres and radians per second, which is what this file carries. Every value is an
exact copy.

One conversion would be needed if the optional battery block were shipped, and it is
recorded here because the block's keys are defined in joules while the source publishes
watt-hours: 1 W h = 3600 J exactly, so the source's 120 W h is 432000 J exactly. That
conversion is not applied anywhere in the numerical core; it would happen here, once, at
transcription time, per [ADR-0003](../../docs/adr/0003-strict-si-and-boundary-conversion.md).

## Choices made here that the source does not make

**The hub positions are derived, not published.** The source publishes `arm_length_m: 0.23`
and states an X configuration; it does not publish the four hub coordinates. This file
places them at `0.23 / sqrt(2)` in each of body x and y, in the rotor order the fixture's
columns use — front-left, rear-left, rear-right, front-right — with the rotor plane through
the centre of gravity. That derivation was confirmed rather than assumed: the roll, pitch
and yaw rows of the source's own exported hover matrix are reproduced by these coordinates
together with the transcribed coefficients, and a different arm interpretation does not
reproduce them.

**The rotor spin signs are derived the same way**, from the yaw row of that matrix:
front-left and rear-right spin one way, rear-left and front-right the other. The source
publishes the configuration but not the sign convention, and the two possible assignments
differ by an inverted yaw axis.

**The rotor speed ceiling is the one that applies under hover load.** The source publishes
two numbers that look like contradictory ceilings — `max_rotor_speed_rad_s: 1000.0` among the
vehicle parameters, and 1102.4200493562662 rad/s in the sidecar's validity block, which is
also what its published hover margin is computed against. They are not in conflict, and the
source programme resolved it: **the ceiling is load-dependent.** The 1000 rad/s figure is the
speed at the pack's 22.2 V nominal voltage, and the actual ceiling scales by terminal voltage
over nominal. That gives 1133.99 rad/s unloaded and 1102.42 rad/s under hover load, the
terminal voltage falling with the current the hover draw pulls through the internal
resistance.

This file carries the hover-load figure, because that is the ceiling in force over the
fixture the cross-implementation case replays. **No shipped case distinguishes any of the
three:** the fixture's largest command is hover plus three per cent, far below all of them.

The model's own battery block computes a ceiling differently, and the difference is recorded
here rather than hidden. `Quadrotor::speed_ceiling_rad_s` scales `speed_at_full_voltage_rad_s`
by open-circuit voltage over FULL voltage, and it does not model the current draw, so it
cannot reproduce the load-dependent sag that separates 1133.99 from 1102.42. The header says
so in its envelope: the internal resistance enters only through `terminal_voltage_v`, which
the derivative does not call. A plant that needs the sag needs a current model, and that is
not in this package.

**The battery block is omitted.** The source's export freezes state of charge at 1.0 and
the fixture never approaches a speed limit, so nothing in the shipped cases would exercise
a battery, and the two published ceilings above leave its limit semantics ambiguous.
Omitting the block gives exactly the fixed-voltage plant, which is what the fixture ran.
The model's battery support is exercised by tests that supply their own parameters inline,
where the values are the test's own choice and are not claimed to describe this vehicle.

## What this model is not

Not a model of any real aircraft, and not validated against one. The parameters are one
implementation's nominal choices; agreement between this model and that implementation is
evidence that two codebases solve the same equations the same way, and is not evidence
about an airframe. `model.quadrotor` is registered implemented-unvalidated for exactly
this reason, and RFC-0002's acceptance section records why no published reference anchors it.

Not valid where the header's envelope says it is not: no rotor inflow, so thrust is
over-predicted in climb and in fast forward flight and the model has no vortex-ring state
at all; no ground effect, so thrust is under-predicted near the ground; no blade flapping,
no rotor-to-rotor interaction, no gyroscopic term from the rotor discs; and a per-axis drag
fit with no lift, no side force and no cross-axis coupling.

Not a description of the built aircraft. When the 650 mm airframe exists and its inertia
tensor and hub positions are measured, that is a second model directory with its own
provenance, not an edit to this one.
