# NASA GTM T2 nominal derivative slice — provenance

## Source

The source is NASA Langley Research Center's public
[`GTM_DesignSim`](https://github.com/nasa/GTM_DesignSim) release, revision
`9717143270144aca1f5d38d7c24c0fce678d1589`. NASA describes it as a nonlinear
flight-dynamics simulation of a 5.5% scale generic transport aircraft. The
associated research paper is Cunningham, Cox, Murri and Riddick,
*A Piloted Evaluation of Damage Accommodating Flight Control Using a Remotely
Piloted Vehicle*, AIAA-2011-6451. The public NASA technical summary is
[NASA/TM-2021-220642](https://ntrs.nasa.gov/citations/20210020347).

The exact source files used were:

| Source file | SHA-256 |
|---|---|
| `gtm_design/config/AC_baseparams_T2.m` | `e7d4c93447085f023e83863aa9b49b3e964fc850ec0d62beafc06bc818541ba0` |
| `gtm_design/config/T2_polynomial_aerodatabase.mat` | `7f34845a54bcecdd788f91226b6be6b386612b3cb691a42538854b4c49466f5c` |

The NASA repository carries NASA Open Source Agreement 1.3 for
`GTM_DesignSim, LAR-17625-1`. This repository does not redistribute NASA's
MATLAB/Simulink source or the `.mat` database. It distributes only the
Galata YAML transcription/derivation below, with this source notice retained.
The NASA release and its agreement remain the authority for reuse terms.

## Derivation

This file is deliberately a derivative *slice*, not a claim that the full
NASA nonlinear model has been ported.

1. Geometry, gross weight and inertia are transcribed from
   `AC_baseparams_T2.m`, converting ft, ft², lbf and slug-ft² to SI.
2. The coefficient reference point is `C6_bas(alpha=4, beta=0)`, where 4° is
   an exact database grid point. `CX` and `CZ` are rotated to stability-axis
   `CD` and `CL` using the declared alpha.
3. `CL_alpha`, `CD_alpha` and `Cm_alpha` are central differences between the
   2° and 6° database rows, per radian. The lateral beta derivatives use the
   -2° and +2° rows.
4. Elevator, aileron and rudder derivatives use the zero-centred control
   increments available in the database. The rudder table contains only the
   negative-deflection branch, so its slope is taken between -10° and 0°.
5. Rate derivatives use the near-zero symmetric entries from `dC3_q`,
   `dC3_p` and `dC3_r`, expressed in Galata's ADR-0002 rate coordinates.
   The database has no explicit alpha-dot force/moment row suitable for this
   adapter, so `pitching_moment_alpha_dot: 0` is an explicit boundary choice.

The resulting YAML is loaded by the same first-order derivative model as the
other fixed-wing references. It is not a global GTM model, does not include
the nonlinear alpha/beta/control schedules, propulsion, sensors, damage
models, or actuator dynamics, and does not establish flight validation.

## What this does and does not prove

The test proves that this controlled coefficient transcription loads, trims,
and produces finite linearised dynamics. It does not prove agreement with a
physical GTM vehicle, certify the NASA simulation, or close Galata's separate
independent-aircraft-validation readiness gate.
