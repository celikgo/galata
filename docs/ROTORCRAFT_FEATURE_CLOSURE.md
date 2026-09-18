# Rotorcraft feature-closure register

This is the executable handoff register for the Galata rotorcraft increment. It
keeps software completion, numerical evidence, and aircraft-model validity
separate. A passing Galata test can establish that the implementation obeys its
declared contract; it cannot validate the Souxmar F1 assumptions.

## Baseline and delivery scope

The implementation baseline was the clean checkout on `feat/rotorcraft-level1`,
HEAD `08e1d76`, after the historical report head `280a650`. The report's
historical references `3de599e`, `280a650`, and its test totals were not treated
as current evidence. The pre-delivery executable exposed 849 CTest cases; the
baseline helicopter, sampled-multirotor and example-workflow gates were green
before this increment was started.

The first delivered increments closed the altitude/environment contract (H6) and
made the existing helicopter failure hooks reachable from studies (H5). This
increment adds the reusable sampled VehicleModel path, deterministic sensor
primitive, helicopter closed-loop capability, six executable studies, and direct
plus integration acceptance evidence. The remaining rows are intentionally not
promoted to complete by proximity or by a passing unrelated test.

## Feature-closure register

| ID | User-visible capability | Current implementation | Remaining work / dependency | Acceptance evidence | Status |
|---|---|---|---|---|---|
| C1 | Fixed-wing, multirotor, and helicopter use one vehicle execution architecture | `VehicleModel` is used by the helicopter vertical; the fixed-wing and multirotor paths still retain vehicle-specific capability routes and adapters | Route model evaluation, trim, simulation, linearisation, metadata, environment, envelope, and reporting through one generic path without changing valid study artifacts | Shared-path examples for all three vehicle types plus direct before/after artifact comparison | Partially implemented |
| C2 | Independent published rotorcraft case | No independent published rotorcraft comparison has been selected or executed | Acquire a source with reproducible configuration, condition, quantity, precision, rights, and uncertainty; add a separate reference-aircraft workflow | A named executable case and generated comparison report; flight-test agreement and published-simulation agreement labelled separately | Missing |
| H1 | Time-varying wind and gusts for the helicopter | The shared environment carries wind and wind rate; scheduled wind support exists on the multirotor paths, not in `sim.helicopter` | Add a `WindField` policy and helicopter study inputs, including discontinuity rebase and a supported turbulence model | Ground-velocity continuity, air-relative response, refinement, and seeded statistics | Partially implemented |
| H2 | Deterministic sensors and seeded randomness | `sim::DeterministicSensor` provides versioned `mt19937_64`/Box–Muller streams, named-channel bias/noise, sample lattice, latency, quantisation, saturation, dropout and explicit unavailable/hold-last policy; provenance is written into the helicopter report | Statistical budgets and a larger independent golden-sequence corpus remain future work; this is a reproducible impairment primitive, not an estimator or aircraft sensor model | `DeterministicSensor.LatencyDropoutAndQuantisationAreDeclared`, `ExampleHeliClosedLoop.SensorRunIsBitStableAndRecordsItsProvenance`, `examples/heli-noisy-feedback` | Implemented, self-consistent and unvalidated |
| H3 | Sweeps, ensembles, and Monte Carlo provenance | No first-class helicopter sweep/ensemble capability | Add deterministic member ordering, continuation policy, independent manifests, aggregate failure accounting, and optional parallel execution | Serial/parallel equivalence and independently reproducible members | Missing |
| H4 | Closed-loop helicopter simulation | `sim.helicopter.closed_loop` executes a generic nonlinear `VehicleModel` with fixed-step RK4, named perfect or impaired measurements, PID or continuous/sampled LQR state feedback, saturation, whole-period delay, zero-order hold, controller-state evidence and failure callbacks | No sampled-loop margin claim, estimator, gust model, or aircraft-validity claim; nonlinear control performance remains a design-study result | `SampledLoop.AppliesBoundaryTickSaturationDelayAndHoldInOrder`, `ExampleHeliClosedLoop.AllRequestedWorkflowsProduceControllerEvidence`, `examples/heli-sas-design`, `heli-attitude-hold`, `heli-altitude-hold` | Implemented, unvalidated |
| H5 | Study-scheduled tail-rotor loss, engine degradation, and actuator jam | `sim.helicopter.failure_events` and the closed-loop path apply tail-rotor/engine fractions and actuator jams at deterministic fixed-step boundaries; events are logged in summaries and `<trajectory>.events.txt` | Autorotation remains outside the declared Level-1 model; no measured engine/jam aircraft evidence is implied | `ExampleHeliTailRotorFailure.HealthyAndFailedRunsAreBothRecorded`, `ExampleHeliFailures.EngineLossAndActuatorJamHaveStableSidecars`, `ExampleHeliFailures.InvalidFailureInputsAreRefusedByName`, `examples/heli-engine-degradation`, `heli-actuator-jam` | Implemented and verified for scheduled-study semantics |
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
| H2-SENSOR-01: named impairments are reproducible and observable | `sim::DeterministicSensor`, `sim.helicopter.closed_loop`, `report.helicopter_csv` | `DeterministicSensor.LatencyDropoutAndQuantisationAreDeclared`, `ExampleHeliClosedLoop.SensorRunIsBitStableAndRecordsItsProvenance` | The test fixes the algorithm version, stream naming, latency/dropout semantics and repeated controller CSV bytes; no statistical claim is made |
| H4-LOOP-01: shared sampled timing is explicit | `sim::run_sampled_loop` | `SampledLoop.AppliesBoundaryTickSaturationDelayAndHoldInOrder` | Boundary callback, tick, saturation, delay release and held derivative ordering are asserted on a scalar plant |
| H4-LOOP-02: nonlinear helicopter controller studies are executable | `sim.helicopter.closed_loop` | `ExampleHeliClosedLoop.AllRequestedWorkflowsProduceControllerEvidence`, `ExampleHeliClosedLoop.SensorRunIsBitStableAndRecordsItsProvenance` | SAS, continuous LQR attitude hold, PID altitude hold, and noisy delayed feedback emit controller sidecars; LQR and nonlinear performance remain unvalidated |
| H5-FAIL-02: engine/jam event grammar and application are explicit | `sim.helicopter.failure_events` parser | `ExampleHeliFailures.EngineLossAndActuatorJamHaveStableSidecars`, `ExampleHeliFailures.InvalidFailureInputsAreRefusedByName` | Engine degradation/loss order, actuator jam sidecar, unknown keys, invalid fractions, off-lattice times and out-of-travel positions are checked |

## Readiness decisions at this handoff

| Decision | Result | Reason |
|---|---|---|
| Exploratory Level-1 modelling | Ready with envelope warnings | The helicopter model, trim, linearisation, open-loop simulation, reports, and failure/atmosphere plumbing run through the CLI |
| Hover and forward-flight trim | Ready for exploratory use | Sea-level and altitude-aware trim cases run; the Souxmar physical parameters remain unvalidated |
| Closed-loop simulation | Ready for exploratory Level-1 studies, not validation | Shared sampled timing, PID/LQR execution, sensor impairment and controller sidecars are executable; no sampled-loop margin or aircraft-validity claim is made |
| Failure and gust studies | Scheduled failure studies ready; gust studies not ready | Tail-rotor, engine degradation/loss and actuator-jam events are executable with refusal tests; helicopter wind/gust fields remain open |
| Interactive engineering analysis | Partially ready | Existing report/desktop paths are usable; the requested general Python plotting workflow is open |
| Agreement with a published reference | Not ready | C2 is missing |
| Prediction of the real Souxmar aircraft | Not ready and not implied | The model is a preliminary design study with assumed inertia and no flight-test validation |

## Handoff record

Fill this section only from executable evidence at the final integrated head:

- Baseline commit: `08e1d76`.
- Final implementation commit: `a72fe30`; the handoff-record update is committed
  separately so this implementation hash remains unambiguous.
- Baseline CTest result: 849-case inventory recorded; focused helicopter, sampled-multirotor and example gates passed before this increment.
- Final CTest result: `855/855` tests passed, `0` failed; full-suite real time was `696.41 s` on this macOS arm64 host.
- Final repository gates: `cmake --build --preset dev -j 4`, the six new sampled-loop/helicopter acceptance tests, `scripts/gen-status-table.sh build/dev/src/cli/galata --check`, `scripts/check-doc-links.sh`, and `git diff --check` passed. The generated capability table, six examples, unit tests, and integration tests are part of this increment.
- Platform scope: this delivery was executed on macOS arm64; no Linux result is claimed here.
- Published-reference result: none; C2 remains unvalidated.
