# F-16 nominal derivative slice — provenance

## Source

This controlled slice is derived from NASA's public
[`simupy-flight`](https://github.com/nasa/simupy-flight) repository, revision
`70754e6916afc206e8c0abb386d1a9c98bf8f561`. The source identifies the model as
an F-16 and uses it in the NESC Case 11 simulation example. The repository's
license is the NASA Open Source Agreement Version 1.3.

The exact source files used were:

| Source file | SHA-256 |
|---|---|
| [nesc_test_cases/F16_aero.py](https://github.com/nasa/simupy-flight/blob/70754e6916afc206e8c0abb386d1a9c98bf8f561/nesc_test_cases/F16_aero.py) | `493cbd62017dd1af4275b504e5369e88711604f5cfd0d27a003f2b67254209a7` |
| [nesc_test_cases/F16_inertia.py](https://github.com/nasa/simupy-flight/blob/70754e6916afc206e8c0abb386d1a9c98bf8f561/nesc_test_cases/F16_inertia.py) | `60ae5e441d28eb15ac1f49b9a000cb1a39da1be98fc61684a9bb3f45f1979834` |
| `LICENSE` | `36091f81ae96b024f56aeb46080774a6674d2dff2b48599079c669ab8fde9552` |

Galata does not redistribute the NASA Python model, spline tables or source
code. It distributes only the SI geometry/inertia transcription and a local
first-order derivative slice derived by evaluating the public source at the
declared condition. The NASA repository and its agreement remain the
authority for reuse terms.

## Derivation

The declared source condition is clean, beta zero, zero controls and zero
body rates at alpha = 5 degrees and V = 300 ft/s. The source geometry is
`S = 300 ft^2`, `b = 30 ft` and `c-bar = 11.32 ft`. The source mass is
`637.26 slug`, with the source inertia evaluated at `CG_PCT_MAC = 25`.
These are converted to SI using the source helper's foot, slug and pound-force
conversion constants.

The YAML coefficients are the source's body-axis coefficients transformed to
the Galata stability-axis longitudinal convention at the reference alpha.
Static alpha, beta and control derivatives are central differences of the
source functions. Rate derivatives use the source's nondimensional p, q and r
coordinates, matching Galata's ADR-0002 convention. The source aero function
has no alpha-dot row, so `pitching_moment_alpha_dot: 0` is an explicit adapter
boundary choice. Propulsion is not copied: Galata's generic model accepts a
direct thrust input, not an F-16 engine map.

The committed CSV in `tests/validation/reference/f16_nominal.csv` records the
numeric source boundary and the exact condition used to derive it. It is a
traceability fixture, not independent flight data.

## What this does and does not prove

The validation test proves that the controlled F-16 derivative slice loads,
matches its committed derivation boundary, trims and produces finite local
dynamics. The example exercises the ordinary fixed-wing workflow.

This is not a global nonlinear F-16 model. It has no Mach/alpha/control
schedule, propulsion, actuator, sensor, structural, stores or flight-test
model. It does not establish agreement with a physical F-16, qualification,
airworthiness, certification or the separate independent-aircraft flight
validation gate.
