<!-- GENERATED FILE — DO NOT EDIT.
     Produced by tools/validation/report_main.cpp via scripts/gen-verification.sh.
     CI regenerates this file and fails if it differs from the committed copy,
     so every number below is a number the code actually produced. -->

# Verification and validation

What galata has been checked against, what agreement was measured, and what has
not been checked at all.

The last column is the one to read. "Unvalidated" is not a placeholder here — it
is a statement that no published reference value has been found for that case,
and it is preferred to a number invented to fill the gap.

## Summary

| Case | Reference | Status |
|---|---|---|
| U.S. Standard Atmosphere 1976 — temperature, pressure, density, speed of sound | COESA, NOAA-S/T 76-1562 / NASA-TM-X-74335 (1976), Tables I and III | **validated** |
| U.S. Standard Atmosphere 1976 — derived layer base temperatures | same, Table I at each breakpoint | **validated** |
| U.S. Standard Atmosphere 1976 — dynamic viscosity | same, equation (51) | **unvalidated** — no tabulated viscosity values were transcribed |
| Quaternion, frame and state conventions | ADR-0002; cross-checked against Eigen's independent implementation | **self-consistent, not externally validated** |
| Rigid-body dynamics | — | **not implemented** |
| Riccati solvers | — | **not implemented** |
| Aircraft modal characteristics | — | **not implemented** |
| Determinism, cross-platform | — | **not implemented** |

## U.S. Standard Atmosphere, 1976

**Reference.** U.S. Committee on Extension to the Standard Atmosphere (COESA),
*U.S. Standard Atmosphere, 1976*, NOAA-S/T 76-1562 / NASA-TM-X-74335,
U.S. Government Printing Office, October 1976.
NTRS document 19770009539, <https://ntrs.nasa.gov/citations/19770009539>.
Rights: *Work of the US Gov. Public Use Permitted.*

**Transcription.** The NTRS scan's OCR layer is unusable for numeric tables —
roughly 95,800 extractable characters across 243 pages, with digits rendered as
underscores and letters. No value came from it. The page images were extracted and
read visually, with load-bearing digits re-cropped at native resolution, and a
second independent pass re-read twelve values without reference to the first.
The full method is in the header of `tests/validation/reference/ussa1976.csv`.

**Agreement measured.** Deviation is given in units of the last printed
significant figure. Below 0.5 the computed value rounds to exactly what the
document prints.

| Quantity | Worst deviation (units in last printed place) | Rounds to the printed value everywhere |
|---|---|---|
| Temperature | 0.487296 | yes |
| Pressure | 0.960739 | **no** |
| Density | 0.849645 | **no** |
| Speed of sound | 0.499227 | yes |

**Gate.** The validation suite requires every cell to agree within 1.0 units of
the last printed place. That bound is chosen to be the tightest one the 1976
tables actually support, not the loosest one that passes: the measured
disagreements are around one part in 10^5, while a wrong lapse-rate sign, a
geopotential/geometric mix-up or the wrong Earth radius each move these values by
one part in 10^2 or worse.

### Cells that do not round to the printed value

There are 3 of them, out of 32 tabulated cells.

| Geometric altitude (m) | Quantity | Published | Computed | Units in last place |
|---|---|---|---|---|
| 11000 | pressure | 226.99 | 227 | 0.960739 |
| 71000 | pressure | 0.044795 | 0.0447956 | 0.632462 |
| 71000 | density | 7.1966e-05 | 7.19652e-05 | 0.849645 |

### Known model-versus-table differences that are not errors

**Molecular-scale versus kinetic temperature above 80 km.** The seven-layer
system is defined in terms of molecular-scale temperature `T_M`, which equals
kinetic temperature `T` only where the mean molecular weight equals its
sea-level value. Above roughly 80 km the ratio `M/M_0` falls away from 1 — the
document's Table 8 gives 0.9995788 at 86 km — so galata's temperature runs high
relative to the tabulated kinetic temperature by up to about 0.08 K in the top
6 km. At 86 km the document tabulates `T = 186.87` K and `T_M = 186.95` K;
galata computes 186.946 K, which is the `T_M` value.

**The upper bound is stated twice and the two statements differ.** The standard
gives the ceiling as both 84.8520 km' geopotential and 86 km geometric.
Equation (18) maps 86000 m to 84852.046 m', so the geopotential figure is the
document's own rounding. galata bounds the envelope in geometric altitude, so
that a query at exactly the standard's stated ceiling is accepted.

### Ambiguities in the source document

These are inconsistencies in the 1976 publication itself, found during
transcription. Each is recorded so that it is not later mistaken for a
transcription error.

| Quantity | Printed as | And also as | galata uses |
|---|---|---|---|
| Sutherland's constant `S` | 110 K (Table 2B, printed page 4) | 110.4 K (text at equation (51)) | 110.4 K, the value stated with the equation it parameterises |
| `R*` | 8.31432e3 N m/(kmol K) (body text, printed page 3) | 8.31432e-3 (Table 2A) | 8.31432e3; only this makes equations (33a)/(33b) dimensionally consistent |
| Sutherland's `beta` | 1.458e-6 (Table 2B, printed page 19) | 1.458e6 (printed page 4, minus sign absent) | 1.458e-6, corroborated twice |
| `r_0` | 6,356,766 m (printed page 8) | 6356.766 km (printed page 4) | 6356766 m; Table 2B's exponent glyph is illegible in the available scans and was not used |

### What is not validated here

**Dynamic viscosity.** galata implements equation (51), but no tabulated
viscosity values were transcribed from the document, so there is nothing to
compare against. The implementation is unvalidated. It additionally inherits the
`S = 110` versus `S = 110.4` ambiguity above, which moves the result by about
0.1%.

**Altitudes between the tabulated points.** Eight altitudes are checked against
the tables. Continuity, monotonicity and the absence of steps at layer
boundaries are checked on a 100 m grid across the whole envelope, which
constrains the space between the sampled points but is not the same as checking
it against published values.

**Everything above 86 km.** Out of scope: galata refuses the query rather than
extrapolating.

---

Generated from galata 0.0.1 by `tools/validation/report_main.cpp`.
