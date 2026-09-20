# A-4D — trim, linearisation and modes at NASA condition 1

## trim

| Quantity | Value |
|---|---|
| Altitude | 0 ft |
| Airspeed | 136.246 m/s |
| Mach | 0.4004 |
| Dynamic pressure | 11369.7 Pa |
| Angle of attack | 4.4197 deg |
| Flight-path angle | 0.0000 deg |
| Elevator | 0.6233 deg |
| Thrust | 8172.2 N |
| Trim lift coefficient | 0.28224 |

**Evidence.** Residual norm 0.000000000000 (m/s^2 and rad/s^2); Jacobian condition number 1001414.1.

A trim is only as good as its residual, so the residual is reported rather
than asserted. The solver refuses to return an answer at all when it is above
tolerance: a linearisation about a point that is not an equilibrium produces a
state-space model that is plausible and wrong.

_Produced by `trim.level`._

## longitudinal

**Model.** A-4D, flight condition 1 — sea level, M = 0.4, clean flexible airplane, 17,578 lb, 4.7 degree reference angle of attack — linearised about 0 ft, 4.41967 deg alpha

**Units.** linearised about 136.246 m/s at 0 m; velocities m/s, angles rad, rates rad/s

State matrix A, rows and columns in the order `u`, `w`, `q`, `theta`:

```
  -0.008703  0.061193  -10.499275  -9.777488
  -0.074397  -0.899951  135.840453  -0.755714
  0.005989  -0.072836  -1.494970  0.000000
  0.000000  0.000000  1.000000  0.000000
```

**Upstream linearization evidence — longitudinal.** These diagnostics describe the source Jacobian, not an error bound for this downstream result. Full step vectors and truncation matrices are retained in the run manifest.

| Source diagnostic | Value |
| --- | ---: |
| Euler chart conditioning | 9.970263e-01 |
| Minimum supported chart conditioning | 1.000000e-01 |
| Recomputed equilibrium residual (m/s^2, rad/s^2 norm) | 1.776357e-15 |
| Equilibrium acceptance budget (same norm) | 1.000000e-10 |
| Neglected coupling ratio | 6.910061e-06 |
| Worst relative Richardson truncation estimate | 4.358929e-12 |

_Produced by `linearize.finitediff`._

## longitudinal_modes

**Model.** A-4D, flight condition 1 — sea level, M = 0.4, clean flexible airplane, 17,578 lb, 4.7 degree reference angle of attack — linearised about 0 ft, 4.41967 deg alpha

**Source.** Gary L. Teper, "Aircraft Stability and Control Data", NASA CR-96008, Systems Technology, Inc. for NASA Ames Research Center, April 1969, Tables III-A through III-C, N69-31783.

| Mode | Eigenvalue (1/s) | omega_n (rad/s) | zeta | Period (s) | T to half (s) | T to double (s) | Evidence |
|---|---|---|---|---|---|---|---|
| phugoid | -0.0066 ± 0.0964j | 0.0967 | 0.0685 | 65.149 | 104.663 | — | 0.976 of participation in u and theta |
| short period | -1.1952 ± 3.1398j | 3.3596 | 0.3558 | 2.001 | 0.580 | — | 0.997 of participation in w and q |

A dash means the quantity is not defined for that mode: a real root has no
period, and a mode has either a time to half amplitude or a time to double,
never both.

### Participation factors

| Mode | u | w | q | theta |
|---|---|---|---|---|
| phugoid | 0.488 | 0.020 | 0.004 | 0.488 |
| short period | 0.003 | 0.497 | 0.500 | 0.000 |

Participation is normalised to sum to one across states. It is the measure
the classification rests on, so a label whose evidence looks thin here is a
label to distrust.

Eigenvector matrix condition number: 102.23.

**Upstream linearization evidence — longitudinal.** These diagnostics describe the source Jacobian, not an error bound for this downstream result. Full step vectors and truncation matrices are retained in the run manifest.

| Source diagnostic | Value |
| --- | ---: |
| Euler chart conditioning | 9.970263e-01 |
| Minimum supported chart conditioning | 1.000000e-01 |
| Recomputed equilibrium residual (m/s^2, rad/s^2 norm) | 1.776357e-15 |
| Equilibrium acceptance budget (same norm) | 1.000000e-10 |
| Neglected coupling ratio | 6.910061e-06 |
| Worst relative Richardson truncation estimate | 4.358929e-12 |

_Produced by `analyze.modes`._

## lateral

**Model.** A-4D, flight condition 1 — sea level, M = 0.4, clean flexible airplane, 17,578 lb, 4.7 degree reference angle of attack — linearised about 0 ft, 4.41967 deg alpha

**Units.** linearised about 136.246 m/s at 0 m; velocities m/s, angles rad, rates rad/s

State matrix A, rows and columns in the order `v`, `p`, `r`, `phi`:

```
  -0.248884  10.499275  -135.840453  9.777488
  -0.218487  -1.816535  0.874802  0.000000
  0.097095  -0.029057  -0.577223  0.000000
  0.000000  1.000000  0.077291  0.000000
```

**Upstream linearization evidence — lateral.** These diagnostics describe the source Jacobian, not an error bound for this downstream result. Full step vectors and truncation matrices are retained in the run manifest.

| Source diagnostic | Value |
| --- | ---: |
| Euler chart conditioning | 9.970263e-01 |
| Minimum supported chart conditioning | 1.000000e-01 |
| Recomputed equilibrium residual (m/s^2, rad/s^2 norm) | 1.776357e-15 |
| Equilibrium acceptance budget (same norm) | 1.000000e-10 |
| Neglected coupling ratio | 4.777293e-19 |
| Worst relative Richardson truncation estimate | 3.003302e-15 |

_Produced by `linearize.finitediff`._

## lateral_modes

**Model.** A-4D, flight condition 1 — sea level, M = 0.4, clean flexible airplane, 17,578 lb, 4.7 degree reference angle of attack — linearised about 0 ft, 4.41967 deg alpha

**Source.** Gary L. Teper, "Aircraft Stability and Control Data", NASA CR-96008, Systems Technology, Inc. for NASA Ames Research Center, April 1969, Tables III-A through III-C, N69-31783.

| Mode | Eigenvalue (1/s) | omega_n (rad/s) | zeta | Period (s) | T to half (s) | T to double (s) | Evidence |
|---|---|---|---|---|---|---|---|
| spiral | -0.0096 | 0.0096 | 1.0000 | — | 71.847 | — | 0.916 of participation in phi |
| roll subsidence | -1.7587 | 1.7587 | 1.0000 | — | 0.394 | — | 0.876 of participation in p |
| Dutch roll | -0.4371 ± 3.9242j | 3.9484 | 0.1107 | 1.601 | 1.586 | — | 0.917 of participation in v and r |

A dash means the quantity is not defined for that mode: a real root has no
period, and a mode has either a time to half amplitude or a time to double,
never both.

### Participation factors

| Mode | v | p | r | phi |
|---|---|---|---|---|
| spiral | 0.000 | 0.007 | 0.076 | 0.916 |
| roll subsidence | 0.003 | 0.876 | 0.058 | 0.063 |
| Dutch roll | 0.492 | 0.068 | 0.424 | 0.016 |

Participation is normalised to sum to one across states. It is the measure
the classification rests on, so a label whose evidence looks thin here is a
label to distrust.

Eigenvector matrix condition number: 38.05.

**Upstream linearization evidence — lateral.** These diagnostics describe the source Jacobian, not an error bound for this downstream result. Full step vectors and truncation matrices are retained in the run manifest.

| Source diagnostic | Value |
| --- | ---: |
| Euler chart conditioning | 9.970263e-01 |
| Minimum supported chart conditioning | 1.000000e-01 |
| Recomputed equilibrium residual (m/s^2, rad/s^2 norm) | 1.776357e-15 |
| Equilibrium acceptance budget (same norm) | 1.000000e-10 |
| Neglected coupling ratio | 4.777293e-19 |
| Worst relative Richardson truncation estimate | 3.003302e-15 |

_Produced by `analyze.modes`._

---

Generated by `galata 0.3.0 (AppleClang 21.0.0.21000099, RelWithDebInfo, Darwin/arm64)`.
