# F-4C power-approach provenance

## Source

Robert K. Heffley and Wayne F. Jewell, *Aircraft Handling Qualities Data*,
NASA CR-2144, Systems Technology, Inc. for NASA, December 1972. The public
NASA technical report is available from the NASA Technical Reports Server:

<https://ntrs.nasa.gov/citations/19730003312>

The F-4C data are in Section IV. Figure IV-1 (printed page 63) gives the
power-approach configuration, weight and body-axis inertia. Table IV-1
(printed page 70) gives the local non-dimensional derivative set. It states
sea level, 230 ft/s (136 kt), 11.7 degrees angle of attack and stabilator
deflection `delta_s = -9.1 degrees`; the configuration is two aft AIM-7
missiles, 20% internal fuel, full flaps with boundary-layer control and gear
down.

The report prints the power-approach values as:

```
W = 33196 lb                 Ix = 23668 slug-ft^2
cg = 0.291 c-bar, W.L. 25.2 Iy = 117500 slug-ft^2
                                  Iz = 133723 slug-ft^2
                                  Ixz = 2177 slug-ft^2

CL = .915, CD = .242         CLa = 2.8/rad, CDa = .555/rad
Cm_a = -.098/rad              Cm_adot = -.95/rad, Cmq = -2.0/rad
CL_ds = .24/rad, Cm_ds = -.322/rad, CD_ds = -.14/rad

CYb = -.655/rad, Cnb = .199/rad, Clb = -.156/rad
Clp = -.272/rad, Cnp = -.013/rad, Clr = .205/rad, Cnr = -.320/rad
CYda = -.0355/rad, Cnda = -.0041/rad, Clda = .057/rad
CYdr = .124/rad, Cndr = -.072/rad, Cldr = -.0009/rad
```

The source uses `delta_s` for the stabilator. Galata's generic fixed-wing
schema has an elevator perturbation input, so the YAML maps the stabilator
derivative rows to that perturbation slot; it does not claim a full F-4C
control-system or stabilator schedule.

The standard F-4C reference geometry used with this data set is `S = 530
ft^2`, `b = 38.7 ft`, `c-bar = 16 ft`. These values are also shown in the
NASA-published F-4C MASCOT example used as the independent geometry reference:

<https://ntrs.nasa.gov/api/citations/19950004810/downloads/19950004810.pdf>

## Conversion and model convention

The YAML stores SI geometry, mass and inertia. The conversions are exact
definitions used at the source boundary:

- `1 ft = 0.3048 m`;
- `1 slug = 14.593902937206362 kg`;
- `1 lbf = 4.4482216152605 N`; and
- standard gravity is `9.80665 m/s^2`, so mass is weight divided by standard
  gravity.

The lateral rows are declared `lateral_axes: stability`; the loader applies
Galata's documented stability-to-body moment rotation at 11.7 degrees. The
source does not print a standalone `C_m` intercept, so
`pitching_moment_ref: 0` is an explicit model convention. The source control
setting is not used as a full trim control because Galata's generic schema
stores perturbation derivatives, not a configuration control schedule.

## Validation scope

The committed fixture preserves the printed F-4C condition and all printed
non-dimensional derivative values. The C++ validation test checks the
independent transcription, the unit conversion, the lateral-axis conversion,
and the complete nonlinear trim -> finite-difference linearisation path.

This is one local F-4C power-approach derivative condition, not a global F-4C
model: it has no Mach schedule, engine or thrust model, flap/gear schedule,
structural model, validated flight envelope, flight-test campaign,
airworthiness evidence or certification status.
