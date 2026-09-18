# Rotorcraft feature-closure register

This is the executable handoff register for the Galata rotorcraft increment. It
keeps software completion, numerical evidence, and aircraft-model validity
separate. A passing Galata test can establish that the implementation obeys its
declared contract; it cannot validate the Souxmar F1 assumptions.

## Baseline and delivery scope

The implementation baseline was the clean checkout on `feat/rotorcraft-level1`,
HEAD `1a7df6d`, after the historical report head `280a650`. The report's
historical references `3de599e`, `280a650`, and its test totals were not treated
as current evidence. The pre-delivery executable exposed 845 CTest cases; no
clean baseline pass claim is made because that run was superseded while the
implementation was being rebuilt.

The first delivered increments close the altitude/environment contract (H6) and
make the existing helicopter failure hooks reachable from studies (H5). The
remaining rows are intentionally not promoted to complete by proximity or by a
passing unrelated test.

## Feature-closure register

| ID | User-visible capability | Current implementation | Remaining work / dependency | Acceptance evidence | Status |
|---|---|---|---|---|---|
| C1 | Fixed-wing, multirotor, and helicopter use one vehicle execution architecture | `VehicleModel` is used by the helicopter vertical; the fixed-wing and multirotor paths still retain vehicle-specific capability routes and adapters | Route model evaluation, trim, simulation, linearisation, metadata, environment, envelope, and reporting through one generic path without changing valid study artifacts | Shared-path examples for all three vehicle types plus direct before/after artifact comparison | Partially implemented |
| C2 | Independent published rotorcraft case | No independent published rotorcraft comparison has been selected or executed | Acquire a source with reproducible configuration, condition, quantity, precision, rights, and uncertainty; add a separate reference-aircraft workflow | A named executable case and generated comparison report; flight-test agreement and published-simulation agreement labelled separately | Missing |
| H1 | Time-varying wind and gusts for the helicopter | The shared environment carries wind and wind rate; scheduled wind support exists on the multirotor paths, not in `sim.helicopter` | Add a `WindField` policy and helicopter study inputs, including discontinuity rebase and a supported turbulence model | Ground-velocity continuity, air-relative response, refinement, and seeded statistics | Partially implemented |
| H2 | Deterministic sensors and seeded randomness | Existing deterministic helpers are not a production sensor/noise module and helicopter closed loop has no sampled sensor path | Version the generator, derive independent streams, add bias/noise/sample/latency/quantisation/dropout primitives and manifests | Golden sequences, solver-call invariance, statistical budgets, noisy closed-loop example | Missing |
| H3 | Sweeps, ensembles, and Monte Carlo provenance | No first-class helicopter sweep/ensemble capability | Add deterministic member ordering, continuation policy, independent manifests, aggregate failure accounting, and optional parallel execution | Serial/parallel equivalence and independently reproducible members | Missing |
| H4 | Closed-loop helicopter simulation | Helicopter trim, linearisation, and open-loop nonlinear simulation are available; controller execution, sampled measurements, anti-windup, and closed-loop helicopter studies are not | Reuse shared sampled-control semantics after H2 sensor primitives and expose helicopter controller metadata | Rate damping, attitude hold, altitude hold, LQR, saturation recovery, step/sample studies | Partially implemented |
| H5 | Study-scheduled tail-rotor loss, engine degradation, and actuator jam | `sim.helicopter.failure_events` applies tail-rotor and engine fractions and actuator jams at deterministic fixed-step boundaries; events are logged in the summary and `<trajectory>.events.txt` | Add broader failure-study examples and measured engine/jam acceptance cases; keep autorotation outside the declared model | Tail-rotor example runs healthy and failed cases; off-lattice events refuse; engine/jam studies prove their semantics | Implemented and verified for the delivered tail-rotor case |
| H6 | Altitude-consistent atmosphere and trim/simulation transfer | `Environment::at_geometric_altitude` uses `core::isa`; trim accepts altitude and `delta_isa_k`, carries atmospheric metadata, and downstream simulation freezes the trim environment explicitly | If future studies vary atmosphere with position, add a separate environment-field policy; do not silently change the frozen policy | Atmosphere agreement at 3,000 m, impossible-altitude refusal, measurable trim-power change, 500 m example report metadata | Implemented and verified |
| M1 | Python/CLI engineering workflow and general plotting | CLI pipeline, Markdown/HTML/CSV reporting, and the desktop trajectory view exist; no general Python model/trim/linearise/controller/sim/plot API exists | Add a documented Python orchestration layer, named-channel plots, SVG/PNG export, and 3-D playback with frame metadata | One script runs load → configure → trim → linearise → design → simulate → inspect → plot → export and agrees with CLI | Partially implemented |
| M2 | Advanced blade/flap/inflow physics | Level-1 momentum/quasi-static model only; the exact-hover derivative limitation is documented and tested | Add only with independent equations and suitable evidence | Dynamic flapping/inflow evidence and revised hover linearisation budget | Deferred — separate physics increment |
| M3 | High-fidelity propulsion, failures, and engine/rotor coupling | Governor lag, torque state, static failure multipliers, and actuator limits exist | Add validated drivetrain/engine maps and declared validity limits | Published or measured propulsion evidence | Deferred — model-data dependency |
| M4 | Higher-fidelity aeroelastic/flight-envelope prediction | No dynamic flapping, lead-lag, stall, compressibility, ground effect, or autorotation model | Select scope and source data before implementation | Evidence for each claimed envelope extension | Deferred — outside Level-1 validity |
| Q1 | Requirements, coverage, performance, generated evidence | Existing charter gates and generated reports exist; this register is the first rotorcraft-specific traceability artifact | Add explicit requirement IDs, new-code coverage report, reproducible benchmarks, and a performance regression policy | CI artefacts link requirement → capability → test → evidence | Partially implemented |

## Traceability for this delivery

| Requirement | Capability/code | Test | Evidence and limit |
|---|---|---|---|
| H6-ENV-01: declared altitude selects atmosphere | `Environment::at_geometric_altitude`, `trim.helicopter` | `HelicopterEnvironment.GeometricAltitudeUsesTheAtmosphereModule` | Uses the repository's independently checked COESA implementation; this validates environment plumbing, not Souxmar parameters |
| H6-ENV-02: impossible altitude is diagnosed | `trim.helicopter` atmosphere construction | `HelicopterEnvironment.ImpossibleAtmosphericAltitudeIsRefused` | Refusal names the declared altitude and atmosphere-envelope error |
| H6-TRIM-01: altitude changes relevant rotor quantities | helicopter trim and breakdown | `HelicopterTrim.AltitudeChangesAirDensityAndTheTrimPower` | Compares sea level and 3,000 m; no monotonic claim beyond this case |
| H6-PIPE-01: study report records altitude and density | `trim.helicopter`, `report.markdown` | `ExampleHeliForwardFlightTrim.ProducesTheTableItsReadmeQuotes` | 500 m case checks the summary metadata and updated altitude-aware values |
| H5-FAIL-01: tail-rotor event is reachable from a study | `sim.helicopter.failure_events` | `ExampleHeliTailRotorFailure.HealthyAndFailedRunsAreBothRecorded` | Healthy baseline, failed run, applied event summary, and event sidecar are checked |
| H5-FAIL-02: actuator/engine event grammar is explicit | `sim.helicopter.failure_events` parser | Targeted parser/negative tests remain to be added | Current implementation refuses unknown components, invalid fractions, invalid actuator names, out-of-travel jam positions, and off-lattice times; broader executable examples remain open |

## Readiness decisions at this handoff

| Decision | Result | Reason |
|---|---|---|
| Exploratory Level-1 modelling | Ready with envelope warnings | The helicopter model, trim, linearisation, open-loop simulation, reports, and failure/atmosphere plumbing run through the CLI |
| Hover and forward-flight trim | Ready for exploratory use | Sea-level and altitude-aware trim cases run; the Souxmar physical parameters remain unvalidated |
| Closed-loop simulation | Not ready | No helicopter sampled-controller/sensor workflow yet |
| Failure and gust studies | Failure case ready for the delivered tail-rotor scenario; gust studies not ready | Tail-rotor event is executable; engine/jam acceptance examples and helicopter gust fields remain open |
| Interactive engineering analysis | Partially ready | Existing report/desktop paths are usable; the requested general Python plotting workflow is open |
| Agreement with a published reference | Not ready | C2 is missing |
| Prediction of the real Souxmar aircraft | Not ready and not implied | The model is a preliminary design study with assumed inertia and no flight-test validation |

## Handoff record

Fill this section only from executable evidence at the final integrated head:

- Baseline commit: `1a7df6d`.
- Final commit: this delivery commit, `feat(rotorcraft): close altitude and failure study paths`.
- Baseline CTest result: 845-case inventory recorded; no clean pass claim made (see above).
- Final CTest result: 849/849 passed, 0 failed, no skipped cases reported, 620.19 s real time.
- Final repository gates: SI boundary, documentation references, version consistency,
  generated capability table, generated validation report, and documentation links all passed.
- Platform scope: this delivery was executed on macOS arm64; no Linux result is claimed here.
- Published-reference result: none; C2 remains unvalidated.
