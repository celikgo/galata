# A-7A flight-condition 1 provenance

## Source

Gary L. Teper, *Aircraft Stability and Control Data*, NASA CR-96008,
Systems Technology, Inc. for NASA Ames Research Center, April 1969,
N69-31783. The public NASA technical report is available from the NASA
Technical Reports Server:

<https://ntrs.nasa.gov/citations/19690022405>

The A-7A data are Section II, Tables II-A through II-F. Table II-A gives the
geometry, mass properties and nine flight conditions. Tables II-B and II-C give
the longitudinal and lateral dimensional derivatives. Tables II-D, II-E and
II-F give the published longitudinal and lateral transfer-function factors.

The source report identifies this set as a clean flexible airplane about the
body-fixed centerline axes. This model is flight condition 1: sea level,
Mach 0.25, true airspeed 279 ft/s, dynamic pressure 91.5 lb/ft², weight
21,889 lb, and reference angle of attack 11.2 degrees. The source's printed
trim elevator is -7.4 degrees; this model treats all controls as perturbations
about that source trim, because the generic Galata aircraft model has no
separate control-trim field.

## Source values and conversion

The source uses feet, slugs, pounds-force and dimensional equations-of-motion
derivatives. YAML stores SI geometry, mass and inertia, but the aerodynamic
coefficients are dimensionless, so the conversion is independent of the unit
system. The conversions below are the same equations used by the validated
NT-33A model.

Let `qbar = 91.5 lb/ft²`, `S = 375 ft²`, `b = 38.7 ft`, `c = 10.8 ft`,
`m = 680 slug`, `V = 279 ft/s`, `alpha = 11.2 deg`, and `Iy = 58,966
slug-ft²`. The body-force coefficients are

```
C_X = -C_D cos(alpha) + C_L sin(alpha)
C_Z = -C_D sin(alpha) - C_L cos(alpha)
```

and each dimensional force derivative is reconstructed from

```
F_x = (qbar S / m) C_X
F_z = (qbar S / m) C_Z
dV/du = cos(alpha),  dV/dw = sin(alpha)
dalpha/du = -sin(alpha)/V,  dalpha/dw = cos(alpha)/V.
```

Inverting the four printed values `X_u = 0.0162`, `X_w = -0.0145`,
`Z_u = -0.0814`, `Z_w = -0.779` gives the local trim values and alpha
slopes stored in YAML: `C_D = 0.0886682071`, `C_L = 0.6339082607`,
`C_D_alpha = 1.5318226182`, and `C_L_alpha = 3.9516541329`.

For the pitch derivatives, `K_m = qbar S c / Iy` and

```
M_u = K_m (2 cos(alpha) C_m / V - sin(alpha) C_m_alpha / V)
M_w = K_m (2 sin(alpha) C_m / V + cos(alpha) C_m_alpha / V)
M_q = K_m C_m_q c / (2 V)
M_wdot = K_m C_m_alphadot c / (2 V²)
M_delta_e = K_m C_m_delta_e.
```

The source values are `M_u = 0.00201`, `M_w = -0.00982`, `M_q = -0.466`,
`M_wdot = -0.000286`, and `M_delta_e = -5.44`. The source does not print a
standalone trim pitching-moment intercept. The model stores the 0.00143 value
reconstructed from the printed `M_u`/`M_w` pair; it is a rounded
derivative-consistency value, not an independent source measurement.
The source control-force values `X_delta_e = 5.75` and `Z_delta_e = -29.0`
give `C_D_delta_e = -0.0001525173` and `C_L_delta_e = 0.5859057771`.

For lateral derivatives, the source's primes are the angular-acceleration
derivatives after eliminating the `I_xz` coupling. With the quoted positive
product `I_xz = 2,933 slug-ft²`, raw moment derivatives are recovered by

```
L = I_xx L' - I_xz N'
N = -I_xz L' + I_zz N'.
```

These are divided by `qbar S b`; rate derivatives additionally divide by
`b/(2V)`. Side-force derivatives use `C_Y_beta = Y_v m V/(qbar S)` and
`C_Y_delta = Y_delta m/(qbar S)`. Table II-C says these derivatives are for
the body-fixed centerline axes, so the model declares `lateral_axes: body` and
does not apply a stability-axis rotation.

## Validation scope

The model is a reproducible local first-order derivative model. It is not a
global A-7A aerodynamic database: there is no stall, Mach schedule, engine,
configuration change, structural flexibility model, or validated flight
envelope. The validation test compares the complete trim -> linearisation ->
modal path against the source's rounded condition-1 transfer-function factors
with a predeclared 5% budget. That is evidence for this local transcription,
not airworthiness evidence.
