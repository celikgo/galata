# A-4D flight-condition 1 provenance

## Source

Gary L. Teper, *Aircraft Stability and Control Data*, NASA CR-96008,
Systems Technology, Inc. for NASA Ames Research Center, April 1969,
N69-31783. The public NASA technical report is available from the NASA
Technical Reports Server:

<https://ntrs.nasa.gov/citations/19690022405>

The A-4D data are Section III, Tables III-A through III-C. Table III-A gives
the geometry, mass properties and eight flight conditions. Table III-B gives
longitudinal dimensional derivatives. Table III-C gives lateral dimensional
derivatives. The source identifies this set as a clean flexible airplane about
the body-fixed centerline axes.

This model is flight condition 1: sea level, Mach 0.4, true airspeed 447 ft/s,
dynamic pressure 237 lb/ft², weight 17,578 lb, and reference angle of attack
4.7 degrees. The source's trim controls are not part of the generic Galata
aircraft schema; elevator and thrust are therefore solved as perturbation
inputs by `trim.level`.

## Conversion

The YAML stores SI geometry, mass and inertia. The aerodynamic coefficients are
dimensionless, so their conversion is independent of the final SI unit system.
For the source condition, `qbar = 237 lb/ft²`, `S = 260 ft²`, `b = 27.5 ft`,
`c = 10.8 ft`, `m = 546 slug`, `V = 447 ft/s`, `alpha = 4.7 deg`,
`Ix = 8780 slug-ft²`, `Iy = 25900 slug-ft²`, `Iz = 28500 slug-ft²`, and the
quoted product is `Ixz = -4070 slug-ft²`.

The body-force coefficients are

```
C_X = -C_D cos(alpha) + C_L sin(alpha)
C_Z = -C_D sin(alpha) - C_L cos(alpha)
```

and the source force derivatives are inverted using

```
X_u = (q S / m) (2 cos(alpha) C_X / V - sin(alpha) C_X_alpha / V)
X_w = (q S / m) (2 sin(alpha) C_X / V + cos(alpha) C_X_alpha / V)
Z_u = (q S / m) (2 cos(alpha) C_Z / V - sin(alpha) C_Z_alpha / V)
Z_w = (q S / m) (2 sin(alpha) C_Z / V + cos(alpha) C_Z_alpha / V).
```

The printed `M_u`, `M_w`, `M_q`, `M_w_dot` and `M_delta_e` values are mapped
to `C_m`, `C_m_alpha`, `C_m_q`, `C_m_alphadot` and `C_m_delta_e` with the
same `q S c / Iy` scaling. The printed `X_delta_e` and `Z_delta_e` values are
inverted directly into `C_D_delta_e` and `C_L_delta_e`. Both `C_L_ref` and
`C_D_ref` are the local intercepts reconstructed by the force-derivative
inversion. The unconstrained thrust input supplies the source condition's
streamwise force balance; the source weight remains an independent condition
check rather than being used to overwrite the derivative-consistent intercept.

For the lateral data, the source's primed moment rows are the
`I_xz`-eliminated angular-acceleration derivatives. Raw moments are recovered
with

```
L = I_xx L' - I_xz N'
N = -I_xz L' + I_zz N'
```

Static derivatives are divided by `q S b`; rate derivatives additionally use
`2 V / (q S b²)`. Side-force derivatives use
`C_Y_beta = Y_v m V / (q S)` and
`C_Y_delta = Y_delta m / (q S)`. The YAML therefore declares
`lateral_axes: body` and does not apply a stability-axis rotation. The local
`C_L_ref` is the derivative-consistent intercept from the same rounded
force-derivative inversion; it is not silently replaced by the conventional
`W/(q S)` estimate, because doing so would make the published `X_w` and `Z_u`
rows irreproducible. The source's weight remains an independent condition check.

## Validation scope

The committed validation fixture preserves the printed dimensional values, and
the C++ validation test runs the loaded nonlinear model through trim,
finite-difference linearisation and lateral derivative comparison. This is a
reproducible local first-order derivative model, not a global A-4D database:
there is no stall, Mach schedule, propulsion map, configuration change,
structural flexibility model, validated operating envelope, flight-test
campaign or airworthiness evidence.
