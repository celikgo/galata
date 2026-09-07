# Proposed end-product features

**Planning status: PROPOSED.** This document defines a candidate end product;
it does not change the implemented capability or validation status of galata
v0.3.0. The current contract remains the [README](../../README.md),
[operating guide](../WORKBENCH.md), [roadmap](../ROADMAP.md) and
[verification report](../VERIFICATION.md). Completing a feature would not, by
itself, establish aircraft validity, handling-quality compliance or tool
qualification.

The proposed target is an offline desktop workbench for fixed-wing flight
dynamics and control engineering, with a Simulink-style executable block-model
editor and the same computations available through the CLI and C++ library.
This does not propose universal Simulink compatibility or full feature parity;
the [Simulink-style feature plan](SIMULINK_FEATURES.md) sets out the selected
semantics and staged scope. An engineer would be able to import a model, establish
its applicable conditions, trim and linearise it, design continuous or sampled
control, simulate nominal and adverse cases, compare evidence and deliver a
reproducible review package. SITL and HIL would be later integrations. Onboard
software and approval of flight hardware would remain separate products and
activities.

This is a scope proposal, not a delivery estimate. **Must v1** denotes a release
requirement for that target, **Next** denotes an optional subsequent increment,
and **Research** denotes work without a product implementation commitment.
Milestones indicate the earliest planned acceptance point, not calendar dates:

| Milestone | Proposed outcome |
| --- | --- |
| M0 | Close the audit findings that can produce misleading or unsafe conclusions. |
| M1 | Establish numerical benchmarks, data contracts and explicit evidence requirements. |
| M2 | Deliver a desktop vertical slice, including basic executable block models, using the existing local aircraft workflow. |
| M3 | Add the selected aircraft-model, control and simulation depth. |
| M4 | Complete campaigns, comparisons and engineering review. |
| M5 | Accept, package and support the product on its declared platforms. |

Dependencies below refer to feature IDs. An epic can begin before its
dependencies finish, but cannot pass acceptance while a required dependency is
unresolved. Every numerical acceptance comparison would name an independent
reference and a justified error budget before results are accepted. Proposed
acceptance requirements below are targets, not measurements of current code.

## Numerical trust and engineering contracts

### F01 — Trustworthy result states and numerical failure handling

**Must v1 · M0 · Dependencies: none**

- **Baseline and gap:** The existing code has useful residual and conditioning
  evidence in CARE and Hamiltonian norm calculations. The audit found that
  sampled sensitivity estimates can nevertheless become false guaranteed
  margins, sign-only eigenvalue checks can misclassify an ill-conditioned
  unstable realization, and a nonfinite loop evaluator can become an unlimited
  margin.
- **Proposed work:** Define consistent result states for established results,
  estimates, unresolved numerics and invalid inputs. Repair the identified
  paths; qualify internal stability numerically; validate channels and finite
  intermediate results before downstream calculations. Reserve mathematical
  bound language for computations whose direction and assumptions support it.
- **Acceptance:** Each audit reproducer has a regression test at its public
  entry point. The CLI and reports neither label the false-margin case a
  guarantee nor label the ill-conditioned unstable case established stable.
  NaN, infinity and invalid indices produce explicit failures. Explicit
  exploratory execution may retain unresolved results, but neither computation
  completion nor exploratory output can be reported as engineering acceptance
  or a passed criterion. Failed execution remains distinct from completion.

### F02 — Independent numerical benchmark and regression programme

**Must v1 · M1 · Dependencies: F01**

- **Baseline and gap:** Analytic tests, published scalar comparisons,
  determinism tests and small SLICOT CARE examples exist; broad solver
  comparisons and independent aircraft-controller evidence remain limited.
- **Proposed work:** Maintain benchmark matrices and analytic cases for every
  supported solver family, including nonnormal, poorly scaled, repeated-pole,
  near-singular and marginal cases. Compare relevant algorithms with an
  independent implementation; distinguish agreement, expected refusal and
  known discrepancy. Gate supported compiler and platform configurations.
- **Acceptance:** The benchmark register maps every supported numerical family
  to independent evidence, domain boundaries and justified budgets. All release
  configurations satisfy their declared budgets and expected refusals. A
  regression lock is never counted as independent validation. Unsupported
  families remain explicitly outside the release contract.

### F03 — Typed states, units, frames and interconnection identity

**Must v1 · M1 · Dependencies: F01**

- **Baseline and gap:** Strict SI, documented frames and named states exist.
  Linear connections still rely on the caller to match physical meaning,
  channel order and units; names alone do not establish compatibility.
- **Proposed work:** Carry dimensions, coordinate frames, reference points,
  perturbation conventions, state identity and continuous/discrete time domain
  with model and controller artifacts. Make explicit adapters reviewable.
- **Acceptance:** Permuted channels, degrees supplied as radians, mismatched
  reference conditions, body/NED confusion and incompatible sample times are
  rejected before execution unless an explicit compatible adapter is present.
  Supported reordering and unit/frame conversions pass round-trip and
  independently defined physical equivalence cases.

## Aircraft and source data

### F04 — Model ingestion, source data and import diagnostics

**Must v1 · M1 · Dependencies: F03**

- **Baseline and gap:** Strict YAML loading supports local derivative aircraft
  and linear state-space models. General tabulated aerodynamics and measured
  data import are absent.
- **Proposed work:** Define a versioned native model schema and declared CSV
  table adapter, with source citation, redistribution position, units, axes,
  reference condition and configuration. Preview conversions and report errors
  at their source cells; retain original bytes and import decisions.
- **Acceptance:** A published fixture imports identically through the CLI and
  desktop. Unknown units, duplicate coordinates, nonfinite entries and missing
  mandatory provenance fail with actionable locations. Export/reimport retains
  all numerical values and required metadata. Import never silently repairs
  or extrapolates source data.

### F05 — Scheduled aerodynamic tables and explicit validity domains

**Must v1 · M3 · Dependencies: F02, F04**

- **Baseline and gap:** The aircraft model uses a first-order coefficient
  expansion about one condition; its angle-of-attack/Mach guard is advisory.
- **Proposed work:** Preserve that local model as an explicit model type and
  add table-based coefficient schedules over declared independent variables.
  Record interpolation method, derivative behaviour, bounds, configurations
  and holes in the evidence domain. Track sideslip, rate and control validity
  wherever those variables enter the model.
- **Acceptance:** Table nodes are reproduced within the prescribed arithmetic
  budget; interpolation is checked against withheld data and analytic tables.
  Every evaluation declares its domain status, including intermediate solver
  stages. Domain holes and unsupported configuration transitions cannot be
  silently bridged. An advisory range is never presented as a validated flight
  envelope.

### F06 — Mass properties, propulsion and configuration models

**Must v1 · M3 · Dependencies: F03, F04, F05**

- **Baseline and gap:** Fixed mass/inertia and commanded thrust are supported.
  Generic thrust lag is not an available-thrust or fuel model.
- **Proposed work:** Add cited configuration-specific mass, centre of gravity,
  inertia, thrust availability and propulsion response contracts. Represent
  fuel/payload effects only when data and evolution rules are supplied.
  Transform forces and moments about declared reference points.
- **Acceptance:** Reference-point changes preserve equivalent rigid-body
  motion; inertia validity and mass balance are checked. Trim cannot claim
  feasibility when required thrust or control exceeds declared availability.
  Configuration and fuel changes reproduce their independently specified
  reference cases, with unsupported transitions rejected.

### F07 — Aircraft evidence packs and model adequacy review

**Must v1 · M3 · Dependencies: F02, F04, F05, F06**

- **Baseline and gap:** Published NT-33A comparisons address a documented local
  condition. They do not validate another aircraft or an expanded envelope.
- **Proposed work:** Ship evidence packs that bind model revisions to source
  conditions, uncertainty, comparison cases, discrepancies and intended uses.
  Establish the specific fixed-wing reference models and data rights during
  M1; an unavailable independent dataset is a scope blocker, not permission to
  manufacture a passing reference.
- **Acceptance:** Every shipped model claim identifies conditions and outputs
  actually compared with independent data. The proposed table-model workflow
  has evidence at distinct conditions sufficient to exercise its scheduling
  variables. A reviewer can separate calibration data from independent
  assessment data and identify every unsupported region.

## Trim, linearisation and analysis

### F08 — Constrained equilibrium and manoeuvre trim

**Must v1 · M3 · Dependencies: F02, F03, F06**

- **Baseline and gap:** Straight-line, wings-level trim checks six dynamic
  residuals. Turning, climbing and asymmetric equilibrium are unsupported.
- **Proposed work:** Add explicitly formulated level, climb/descent and steady
  coordinated-turn problems, with selectable free variables and constraints.
  Support asymmetric trim only for declared aircraft/configuration models.
  Report scaled residuals, feasibility, active limits and solver diagnostics.
- **Acceptance:** Each supported trim formulation has an independent reference
  case and infeasible counterexample. Returned equilibria satisfy all declared
  constraints and the full dynamics within predeclared budgets. No nearest
  unsuccessful iterate is accepted as trim.

### F09 — Trim continuation and envelope maps

**Must v1 · M3 · Dependencies: F05, F07, F08**

- **Baseline and gap:** Individual trim conditions can be studied; there is no
  managed map of feasible equilibria or branch history.
- **Proposed work:** Sweep altitude, speed, configuration and other supported
  parameters using deterministic continuation and explicit initial guesses.
  Preserve branch changes, failures, active constraints and data-domain status.
- **Acceptance:** Reordered or resumed execution produces the same identified
  cases and declared repeatability results. Every map cell links to its inputs,
  residuals and status. A discontinuity, failed solve or missing model data is
  visible and cannot be rendered as a continuous feasible region.

### F10 — Linearisation and local model reduction

**Must v1 · M3 · Dependencies: F02, F03, F05, F08**

- **Baseline and gap:** Central differences and Richardson estimates exist
  about the supported trim, with named Euler perturbation coordinates.
- **Proposed work:** Extend linearisation to the accepted equilibrium types
  and augmented actuator/sensor states. Preserve scaling, coordinate transforms
  and operating-point identity; qualify derivative discontinuities. Make any
  state removal or reduction explicit and report its relevant response error.
- **Acceptance:** Analytic derivatives and shrinking-disturbance comparisons
  verify each supported path. Step refinement exposes cancellation or table
  discontinuities. Hidden unstable states cannot disappear through ordinary
  channel selection, and any reduction carries a checked use-specific error
  budget rather than an unconditional equivalence claim.

### F11 — Modes, root locus and classification confidence

**Must v1 · M3 · Dependencies: F01, F02, F10**

- **Baseline and gap:** Eigenvalues, modal metrics and participation-based
  classical labels exist. Root locus and tracked mode branches are absent.
- **Proposed work:** Add pole/zero and root-locus views, condition-aware modal
  diagnostics, continuity-aware branch tracking and reviewable label
  confidence. Preserve ambiguous or unclassified modes.
- **Acceptance:** Analytic pole trajectories, repeated/near-defective cases
  and supported aircraft examples have reference checks. Labels are not
  assigned merely to fill a classical-mode list. A branch exchange or
  numerically unresolved pole is visible in both plots and exported data.

### F12 — Frequency analysis and defensible robustness assessment

**Must v1 · M3 · Dependencies: F01, F02, F03, F10**

- **Baseline and gap:** Classical crossovers, sampled S/T peaks, SISO disk
  estimates and numerical Hamiltonian bounds coexist. Their evidential
  meaning differs; simultaneous structured MIMO robustness is absent.
- **Proposed work:** Unify loop-break definitions, scaling, nominal-stability
  qualification, DC/feedthrough treatment and display of bounds versus
  estimates. Extend only the explicitly selected analyses to discrete time.
  Use conservative norm endpoints for guaranteed inequalities.
- **Acceptance:** Missed-peak, multiple-crossover, unstable-hidden-mode and
  direct-feedthrough cases receive the correct result status. Independent
  continuous and discrete reference cases satisfy declared budgets. No finite
  grid or per-channel MIMO margin is presented as a global robustness proof.

## Controller design and execution semantics

### F13 — Continuous control-design workspaces

**Must v1 · M2 · Dependencies: F02, F03**

- **Baseline and gap:** CARE/LQR, explicit filtered PID and state-space
  interconnections exist in the CLI and C++ library. Costs and actuator limits
  in the shipped aircraft example are illustrative.
- **Proposed work:** Provide editable, versioned design inputs with physical
  state scaling, explicit feedback sign, channel selection and loop break.
  Compare candidate gains, costs, stability evidence and actuator demand.
- **Acceptance:** The desktop and CLI execute the same serialized design and
  produce equivalent result artifacts under the determinism contract. A
  controller cannot attach to a different state basis or operating point
  silently. Solver success and aircraft-controller validation remain separate
  fields in every design review.

### F14 — Discrete models and sampled LQR

**Must v1 · M3 · Dependencies: F02, F03, F10, F13**

- **Baseline and gap:** The product supports continuous control only; discrete
  models, DARE and sampled LQR are not implemented.
- **Proposed work:** Add explicit hold/discretisation methods, sample time,
  discrete state-space contracts and a bounded stabilising DARE/LQR path.
  Preserve the cost discretisation and input-hold assumptions in the design.
- **Acceptance:** Analytic and independent DARE examples cover stabilisable,
  unstabilisable, detectable and invalid-cost cases. Stable sampled poles are
  tested against the unit circle with numerical qualification. Discretisation
  and held-input trajectories agree with independently computed references;
  mixed time domains are rejected without an explicit adapter.

### F15 — Practical PID design, tracking and anti-windup

**Must v1 · M3 · Dependencies: F03, F13, F14, F20**

- **Baseline and gap:** Filtered PID accepts explicit gains. Tuning, saturation
  recovery, anti-windup and bumpless transfer are absent.
- **Proposed work:** Define continuous and sampled PID forms, derivative
  filtering, setpoint weighting, output limits, tracking and anti-windup.
  Offer reviewable tuning against specified objectives for a bounded set of
  methods; retain manual gains as the transparent baseline.
- **Acceptance:** Independent step/frequency cases verify each PID form.
  Saturation-release and manual/automatic transfer scenarios meet the declared
  recovery and continuity criteria. Every tuning result retains its objective,
  constraints, initialization and failure status; tuning success is not a
  handling-quality approval.

### F16 — Measurement models and linear state estimation

**Must v1 · M3 · Dependencies: F02, F03, F14, F20**

- **Baseline and gap:** LQR uses exact full-state feedback at every integration
  stage. There is no sensor or estimator contract.
- **Proposed work:** Add named measurement models and bounded continuous/
  discrete linear observers and Kalman filtering, with declared covariance,
  initialization, observability/detectability and update timing. Nonlinear
  filtering would require a separate scope decision.
- **Acceptance:** Analytic covariance cases and independent filter references
  verify the selected methods. Unobservable unstable modes, invalid covariance
  and incompatible measurement frames are rejected. Seeded noisy simulations
  report estimation error and covariance-consistency diagnostics separately
  from closed-loop aircraft performance.

### F17 — Sampling, delay and executable controller timing

**Must v1 · M3 · Dependencies: F03, F14, F16, F19**

- **Baseline and gap:** Existing feedback is continuous; transport delay,
  held commands, multirate execution and jitter are absent.
- **Proposed work:** Define controller ticks, sample/hold order, sensor
  timestamps, bounded delay queues and declared multirate schedules. Treat
  rational delay approximations separately from an executed delay model.
- **Acceptance:** Boundary-time fixtures verify the precise ordering of
  sampling, estimation, control and actuator updates. Changing integration
  step without changing the sample schedule preserves controller event times.
  Exact-delay and approximation comparisons expose their valid frequency or
  time range; dropped, late and duplicate samples have explicit policies.

### F18 — Scheduled controllers and transition assessment

**Must v1 · M3 · Dependencies: F09, F13, F14, F17**

- **Baseline and gap:** A design targets one local linear model. Automatic gain
  scheduling and transition assessment are absent.
- **Proposed work:** Bind controller grids to trim branches and valid model
  regions, with explicit interpolation, scheduling variables, initialization
  and transition logic. Preserve both local analysis and nonlinear transition
  evidence.
- **Acceptance:** Each schedule node has accepted local evidence; tested
  transitions cover declared scheduling rates, saturation and edge conditions.
  Missing nodes and out-of-domain scheduling fail explicitly. A collection of
  locally stable designs is never reported as proof of global scheduled
  stability.

## Time-domain studies and hardware effects

### F19 — Scenario commands, events and reference trajectories

**Must v1 · M3 · Dependencies: F03, F08**

- **Baseline and gap:** Linear/nonlinear runs accept initial perturbations and
  constant commands around a level translating reference.
- **Proposed work:** Add time-stamped steps, ramps, pulses and supplied
  reference trajectories; state/time-triggered events; stop conditions; and
  explicit equilibrium/reference ownership. Define reproducible event ordering.
- **Acceptance:** Scenario replay reproduces event times and declared numerical
  results. Coincident events have a tested order, invalid commands are caught
  before execution, and event records distinguish a requested transition from
  an achieved aircraft response.

### F20 — Actuator, sensor and failure-effect components

**Must v1 · M3 · Dependencies: F02, F03, F19**

- **Baseline and gap:** Four actuators have position bounds, rates and positive
  first-order lags. Sensor dynamics and explicit failure modes are absent.
- **Proposed work:** Add composable named actuator/sensor models with declared
  lag, rate/position limits, sampling, bias, noise and optional deadband.
  Define bounded injected failures such as stuck, loss, bias or delayed data.
- **Acceptance:** Each component has an analytic or measured reference and
  explicit applicability. Saturation and failure onset/recovery are checked at
  event boundaries. The same component can augment linear and nonlinear
  studies consistently. Generic components cannot inherit a real hardware
  part number's validation status.

### F21 — Wind, gust and atmospheric disturbance scenarios

**Must v1 · M3 · Dependencies: F02, F03, F05, F19**

- **Baseline and gap:** Atmospheric properties exist; operational wind/gust
  and turbulence scenarios are not part of the current simulation contract.
- **Proposed work:** Add deterministic wind/gust inputs and selected stochastic
  disturbance models with explicit frame, altitude dependence, units, bandwidth
  and seed. Separate weather data import from the adopted disturbance model.
- **Acceptance:** Deterministic force/relative-airflow cases and reference
  spectra verify the selected implementations. Repeat runs retain the same
  random stream under the declared execution policy. A statistical spectrum
  check uses a predeclared ensemble and confidence criterion, and does not
  imply that a particular aircraft response is validated.

### F22 — Numerical convergence and simulation evidence

**Must v1 · M3 · Dependencies: F02, F10, F17, F19, F20, F21**

- **Baseline and gap:** Fixed-step RK4, step guards and selected convergence
  tests exist. Users still assemble study-specific convergence evidence
  manually; flight time histories are externally unvalidated.
- **Proposed work:** Automate step-refinement studies for declared engineering
  quantities, matching actuator/sensor and controller timing across compared
  runs. Include event times, peak responses and domain exits, not only final
  states. Preserve fixed-step repeatability as the baseline method.
- **Acceptance:** Smooth analytic cases show the expected method order within
  their resolved range; nonsmooth cases are judged against declared quantities
  and budgets. A study cannot be marked converged when only endpoints agree or
  when event schedules differ. Reports separate integration convergence,
  local linearisation agreement and independent flight-data evidence.

## Campaigns and engineering assessment

### F23 — Parameter uncertainty and experiment design

**Must v1 · M4 · Dependencies: F04, F07, F09, F22**

- **Baseline and gap:** Individual study inputs are reproducible; uncertainty
  distributions, correlations and designed parameter sets are absent.
- **Proposed work:** Define named bounded variations, covariance/correlation,
  deterministic sweeps and seeded sampling. Carry the evidence behind each
  uncertainty assumption and distinguish numerical, parameter and model-form
  uncertainty.
- **Acceptance:** Generated cases reproduce from their manifest, obey declared
  bounds/correlation contracts and fail invalid covariance inputs. Results
  identify exactly which uncertainty sources were exercised. A finite ensemble
  is never described as an exhaustive worst-case proof.

### F24 — Campaign execution, resume and resource control

**Must v1 · M4 · Dependencies: F23, F27, F32, F35**

- **Baseline and gap:** The runner executes one stage graph; there is no
  managed campaign scheduler, resume ledger or resource budget.
- **Proposed work:** Execute case sets locally with bounded concurrency,
  cancellation, checkpointed case status and failure retention. Key reusable
  results by complete declared inputs and computation identity.
- **Acceptance:** Serial, parallel and interrupted/resumed runs produce the
  same case membership and results under the determinism contract. Corrupt or
  incompatible cached outputs are rejected. Failed cases remain in summary
  denominators, and the user can stop a campaign without losing accepted prior
  case records.

### F25 — Handling-quality and performance criteria

**Must v1 · M4 · Dependencies: F07, F11, F12, F18, F22, F24**

- **Baseline and gap:** Modal metrics and response outputs exist; there are no
  implemented handling-quality classifications or application acceptance rules.
- **Proposed work:** Implement an explicitly selected, source-pinned subset of
  fixed-wing criteria, including applicability conditions, measurement method,
  configuration and flight-phase assumptions. Also allow versioned project
  limits without presenting them as published-standard criteria.
- **Acceptance:** Every claimed criterion has an independent reference example,
  boundary cases and an unsupported-case result. Reviews expose the measured
  quantity, threshold, source edition, applicability and numerical uncertainty.
  A partial criteria set cannot yield a blanket aircraft-compliance label.

### F26 — Measured-data alignment and model assessment

**Must v1 · M4 · Dependencies: F04, F07, F22, F31**

- **Baseline and gap:** Reference tables are transcribed for tests; engineers
  cannot yet align measured time histories with study results in a supported
  review workflow.
- **Proposed work:** Import measurements with calibration, units, frames,
  timestamps and uncertainty; explicitly align/resample for comparison. Report
  residuals and validity by condition. Fitting or parameter identification
  would be a later feature, with calibration and holdout data kept distinct.
- **Acceptance:** Known shifts, sampling differences, missing data and frame
  transformations are exercised by fixtures. Every processing operation is
  retained; original measurements remain unchanged. A comparison can be
  reproduced from the review package without hidden interactive adjustments.

## Desktop workflow, projects and review

### F27 — Portable project workspaces and schema migration

**Must v1 · M2 · Dependencies: F03, F04, F35**

- **Baseline and gap:** Studies and models live in YAML files with relative
  paths; there is no managed desktop project or long-term migration contract.
- **Proposed work:** Organize model revisions, studies, scenarios, requirements,
  results and review annotations in portable local projects. Provide explicit
  migrations and read-only opening of unsupported versions where feasible.
- **Acceptance:** A project moved to another declared platform retains its
  dependencies and can run without the original checkout. Migration preserves
  original content and provides a reviewable change record. External missing
  files, incompatible versions and path escapes are detected before execution.

### F28 — Desktop study editor and end-to-end vertical slice

**Must v1 · M2 · Dependencies: F01, F13, F27, F32, F39, F42, F46**

- **Baseline and gap:** The CLI/C++ workflow is usable; a desktop shell and
  interactive study editor do not exist.
- **Proposed work:** Provide a local project browser, form/graph study editor,
  capability discovery, validation messages, run progress, cancellation and
  result navigation. Keep the study-stage DAG distinct from the executable
  signal-flow editor in F39: one orchestrates studies, the other describes
  dynamics. The desktop would call the same capability contracts as the CLI,
  without a second numerical implementation.
- **Acceptance:** An engineer completes import, trim, linearisation, LQR,
  analysis, simulation and export of the existing local study through the UI.
  Its serialized study also runs headlessly with equivalent artifacts. A
  background run leaves navigation responsive and displays failures at the
  affected stage rather than presenting partial output as complete.

### F29 — Interactive engineering plots and linked inspection

**Must v1 · M2 · Dependencies: F01, F28, F35**

- **Baseline and gap:** Reports contain selected static plots and tables;
  general interactive linked views are absent.
- **Proposed work:** Add pole maps, Bode/Nyquist/singular-value plots, time
  histories and cross-run overlays with cursors, legends, units, view presets
  and accessible non-colour distinctions. Mark searched bands, bounds,
  unresolved regions, limits and events directly in context. M2 covers existing
  modal/frequency outputs; later F11/F12 analyses use this same view contract.
- **Acceptance:** Cursor values identify the underlying run and sample or
  interpolation rule; exported plots and tables agree with source artifacts.
  Display decimation preserves or explicitly reports extrema and never changes
  computed metrics. Changing display units leaves stored SI results unchanged.

### F30 — Model, controller and result comparison

**Must v1 · M4 · Dependencies: F03, F18, F27, F29, F35**

- **Baseline and gap:** Reports preserve model descriptions and controller
  evidence, but comparison is largely manual.
- **Proposed work:** Compare structured model/controller revisions, operating
  points, schedules and results side by side. Surface state/channel mapping,
  units, costs, hardware assumptions and evidence differences before overlays
  or numerical deltas are accepted.
- **Acceptance:** Compatible permutations compare equivalently after an
  explicit mapping. Incompatible conditions or physical dimensions block an
  unlabeled numerical comparison. Every delta links to both immutable inputs
  and the computation that produced it; uncertainty and failures remain visible.

### F31 — Engineering review packages and acceptance records

**Must v1 · M4 · Dependencies: F24, F25, F29, F30, F35**

- **Baseline and gap:** Markdown, CSV, self-contained HTML and run manifests
  exist. Formal review decisions, case coverage and application acceptance
  records are not a supported workflow.
- **Proposed work:** Assemble self-contained review packages with objectives,
  input assumptions, model evidence, requirement-to-case links, results,
  discrepancies and reviewer decisions. Separate author assertions from
  independently recorded review outcomes.
- **Acceptance:** A reviewer can reconstruct every reported decision from
  included cases and declared criteria, including excluded and failed cases.
  Edited inputs invalidate the affected acceptance record instead of silently
  inheriting approval. Exported packages work offline and visibly identify
  incomplete, superseded and unvalidated evidence.

## Automation, delivery and sustainment

### F32 — One automation contract across desktop, CLI and C++

**Must v1 · M2 · Dependencies: F01, F03, F04**

- **Baseline and gap:** The capability registry and installed C++/CLI package
  exist; the pre-1.0 C++ interface has no frozen ABI.
- **Proposed work:** Define stable serialized study/result schemas, structured
  diagnostics, cancellation and progress, with a documented source/ABI policy
  for each public surface. Supply headless examples and machine-readable output
  suitable for project automation.
- **Acceptance:** The same fixture executed through each supported surface
  yields equivalent accepted results and failure categories. Compatibility
  tests exercise the promised migration window. Automation needs neither screen
  scraping nor parsing human prose, and cannot bypass numerical result states.

### F33 — Offline operation, data boundaries and import containment

**Must v1 · M5 · Dependencies: F04, F27, F28, F32**

- **Baseline and gap:** Runtime reports are self-contained and the pipeline
  contains output writes. A desktop product requires a declared policy for
  untrusted projects, confidential inputs, network activity and diagnostics.
- **Proposed work:** Make offline operation complete, network integrations
  explicitly enabled, imported paths contained and diagnostic export deliberate.
  Treat manifests as model-bearing data. Integrate with organizational storage
  and access policies without inventing a classification authority inside the
  application.
- **Acceptance:** The accepted workflow runs on a disconnected machine after
  installation; a network-denied test observes no required external service.
  Malformed imports and path traversal fixtures are contained. Diagnostic
  bundles expose their contents before export, and do not automatically upload
  models, measurements or credentials.

### F34 — Installers, upgrades and release acceptance

**Must v1 · M5 · Dependencies: F02, F28, F31, F33, F36**

- **Baseline and gap:** Relocatable package and archive checks exist; a signed
  desktop installer and full product release acceptance are absent.
- **Proposed work:** Freeze supported operating systems, architectures and
  compiler/runtime combinations. Produce verified installers, dependency and
  licence inventories, upgrade/rollback procedures and disconnected-install
  instructions. Add signing appropriate to each chosen distribution channel.
- **Acceptance:** A clean machine per declared platform installs and completes
  the release workflow without development tools. Upgrade and rollback preserve
  user projects, and archive/installer contents match the release manifest.
  Release gates consume required CI results; missing evidence is not a pass.

### F35 — Provenance, replay and immutable result lifecycle

**Must v1 · M1 · Dependencies: F01, F04**

- **Baseline and gap:** Content-addressed manifests retain exact input bytes,
  output hashes and build/dependency identity. Hash identity is not authorship;
  publication is atomic per file, not across a whole study.
- **Proposed work:** Define states for staged, failed, completed, superseded
  and reviewed runs. Verify imported run contents; preserve build recipes,
  binary identity and environment assumptions needed for replay. Distinguish
  content integrity, trusted origin and reviewer identity.
- **Acceptance:** Tampered snapshots and outputs fail integrity verification.
  A failed run is distinguishable from a completed run even when partial files
  remain. Replaying a retained case satisfies its same-platform or bounded
  cross-platform contract, with excluded quantities explicitly identified.

### F36 — Operating guidance, diagnostics and maintenance ownership

**Must v1 · M5 · Dependencies: F02, F31, F32, F35**

- **Baseline and gap:** Operating instructions, validation notes and examples
  exist. Product support periods, migration commitments and incident ownership
  need definition.
- **Proposed work:** Maintain task-based guidance, complete reference studies,
  diagnostic explanations, supported-use boundaries and a release support
  policy. Assign owners for dependency updates, benchmark drift, numerical
  defects, data corrections and user-reported incidents.
- **Acceptance:** Acceptance participants complete the declared workflows using
  shipped guidance; unresolved usability failures are dispositioned before
  release. Every supported capability links to assumptions and evidence. A
  release drill traces a reported wrong result to its inputs/version and
  demonstrates correction notice, regression coverage and affected-user guidance.

## Subsequent integrations

### F37 — SITL and recorded external-controller execution

**Next · M5 or later · Dependencies: F17, F19, F20, F22, F32, F33**

- **Baseline and gap:** There is no external controller runtime or supported
  SITL transport.
- **Proposed work:** Add a bounded transport-neutral execution contract and a
  selected adapter, with explicit clocks, channel mapping, timeouts and replay.
  Preserve the external executable/configuration identity and input/output log.
- **Acceptance:** A deterministic reference controller matches its in-process
  counterpart within declared timing/numerical budgets. Disconnects, malformed
  messages, stale samples and restart behaviour are tested. Offline replay
  reconstructs the observed exchange and distinguishes simulator time from
  wall-clock performance; success does not qualify onboard software.

### F38 — HIL interface feasibility and measured hardware comparison

**Research · M5 or later · Dependencies: F20, F26, F33, F37**

- **Baseline and gap:** There are no real-time guarantees, hardware drivers or
  validated hardware-in-loop responses.
- **Proposed work:** First establish the intended bench, safe hardware states,
  timing/jitter budget, synchronization, calibration and operator procedures.
  Promote a particular interface to a delivery epic only after a measured
  feasibility study and a separate hardware risk review.
- **Acceptance for the research stage:** A bench specification identifies each
  missing platform guarantee and interface owner; recorded measurements bound
  latency/jitter under the proposed load and demonstrate timeout/disconnect
  behaviour. A documented go/no-go decision states whether Galata, an external
  real-time runner or a different architecture would execute each function.

## Executable visual block modelling

The following epics extend the proposed v1 scope in response to the request for
Simulink-style features. They describe specific engineering workflows, not a
claim to implement another product's complete block catalog, solver semantics,
file formats or generated-code toolchain. A visual model would compile to a
reviewable execution representation shared by desktop and headless runs.

### F39 — Typed signal-flow canvas and model compilation

**Must v1 · M2 · Dependencies: F03, F32**

- **Baseline and gap:** The existing pipeline is a DAG of studies and analysis
  stages. It is not an executable block model and cannot represent dynamic
  feedback merely by permitting cyclic stage references.
- **Proposed work:** Add an executable signal-flow model with named input/output
  ports, continuous/discrete state ownership, direct-feedthrough declarations
  and stable block/port identity. Compile dimension, type, physical unit, frame
  and timing constraints; show errors at blocks and connections. Causal
  feedback through state or delay is a supported model structure.
- **Acceptance:** An aircraft/PID feedback model compiles, executes and matches
  its independently assembled state-space reference. Wrong port dimensions,
  units, frames, unresolved timing and invalid connections fail before running.
  Repositioning canvas elements does not change the compiled execution or
  result. The study DAG and signal graph have separate schemas and validators.

### F40 — Hierarchical subsystems and reusable model libraries

**Must v1 · M3 · Dependencies: F39, F41, F42**

- **Baseline and gap:** C++ state-space interconnections exist; user-defined
  hierarchical executable subsystems and managed block libraries do not.
- **Proposed work:** Support nested subsystems, named interface ports, exposed
  parameters and versioned library references for aircraft, controllers,
  sensors and actuators. Preserve parameter scope and stable identity through
  flattening; make updates to referenced models explicit review actions.
- **Acceptance:** A flattened model and its hierarchical equivalent agree under
  the declared execution contract. Separate instances cannot share mutable
  state accidentally. Recursive references, interface changes and unresolved
  library revisions are diagnosed. Updating a library revision produces an
  inspectable project change and invalidates affected accepted results.

### F41 — Data dictionaries, configurations and model variants

**Must v1 · M2 · Dependencies: F27, F39**

- **Baseline and gap:** YAML values configure individual study stages; there is
  no shared typed parameter dictionary or compiled variant configuration.
- **Proposed work:** Add named parameters with units, bounds, source, scope and
  mutability, plus selected configurations for airframe/hardware/controller
  variants. Define override precedence explicitly and materialize resolved
  parameter values into the run input. Separate compile-time structure from
  run-time tuning.
- **Acceptance:** The same configuration resolves identically from every
  execution surface. Cyclic parameter expressions, ambiguous overrides,
  inactive-variant references and incompatible parameter types fail with
  source locations. Every result identifies the active variants and exact
  resolved values; changing a variant cannot reuse incompatible cached results.

### F42 — Bounded standard block palette and block contracts

**Must v1 · M2 · Dependencies: F02, F39**

- **Baseline and gap:** Several useful primitives exist as numerical routines,
  not as a documented visual block catalog.
- **Proposed work:** Start with constants and scenario inputs; sum/gain/product
  and selected elementary math; routing; integrators; continuous state-space;
  filtered PID; saturation; aircraft and actuator wrappers; and scopes/recorders.
  Add supported sampled delays, holds, filters, discrete state-space and sensor
  blocks with F43. Each block declares equations, port contracts, parameters,
  state initialization, direct feedthrough and supported time domains.
- **Acceptance:** Every shipped block has an independent analytic or referenced
  test plus invalid-parameter and initialization cases. The M2 palette closes
  and simulates the existing continuous aircraft-control study; F43 is required
  before sampled variants are released. A block's UI and headless declaration
  expose the same defaults and limitations.

### F43 — Continuous, discrete and multirate block execution

**Must v1 · M3 · Dependencies: F14, F17, F39, F42, F46**

- **Baseline and gap:** The numerical drivers support continuous integration;
  no compiled mixed-time block scheduler exists.
- **Proposed work:** Propagate sample times and offsets through the model;
  define zero-order holds, rate transitions, update order and state commits.
  Support a bounded deterministic fixed-step/multirate policy first. Treat
  incompatible clocks and unspecified cross-rate semantics as model errors.
- **Acceptance:** Reference diagrams combining continuous plants and sampled
  controllers match independently computed held-input solutions. Multirate
  event ordering, initial samples and coincident ticks pass exact schedule
  tests. Rate transitions never depend on thread timing. Unsupported clock
  ratios or required events between supported solver ticks are rejected.

### F44 — Execution inspection, checkpoints and replayable tuning

**Must v1 · M4 · Dependencies: F19, F22, F29, F39, F41, F43**

- **Baseline and gap:** Runs currently execute to completion or failure; visual
  stepping, model-state checkpoints and runtime tuning are absent.
- **Proposed work:** Add start/pause/step/stop, linked scopes, event inspection
  and checkpoints containing the solver, block, delay, controller and random
  generator states. Allow declared tunable parameters to change only at
  defined boundaries; record every change as part of execution history.
- **Acceptance:** An uninterrupted run and a paused/checkpointed/resumed run
  match under the repeatability contract. Replaying a tuning log reproduces
  its results; changing structural parameters requires recompilation. Dropped
  display frames or scope decimation cannot alter model execution or recorded
  acceptance metrics.

### F45 — Model test harnesses, assertions and scenario coverage

**Must v1 · M4 · Dependencies: F24, F25, F39, F43, F44**

- **Baseline and gap:** Source-level unit/integration tests exist; engineers
  cannot create reusable executable model/subsystem test harnesses.
- **Proposed work:** Define harness inputs, initial states, stubs, scenarios and
  assertion windows around a model or subsystem. Support versioned signal,
  event-order and project-criterion assertions, with explicit vacuity and
  missing-data outcomes. Execute harness sets through the campaign runner.
- **Acceptance:** A deliberately broken component fails a targeted harness;
  absent triggers and missing observations cannot produce a silent pass.
  Tests bind to the model/configuration revision they exercised and report
  executed, failed, skipped and inconclusive cases. Claimed scenario coverage
  is traceable to the selected requirement set, not inferred from a run count.

### F46 — Algebraic-loop detection and fail-closed execution

**Must v1 · M2 · Dependencies: F01, F39, F42**

- **Baseline and gap:** Linear feedback can solve a well-posed feedthrough
  interconnection; a general signal graph has no algebraic-loop contract.
- **Proposed work:** Detect strongly connected direct-feedthrough dependencies,
  distinguish them from causal feedback through state, and identify the exact
  participating blocks. Refuse general algebraic loops in v1. Explicit supported
  linear interconnection blocks may retain their separately checked solve.
  Do not insert an artificial delay or change equations automatically.
- **Acceptance:** Direct-loop fixtures identify the correct cycle; nested-loop
  fixtures extend this gate with F40. Integrator/delay feedback is accepted in
  its supported time domain. Unsupported loops fail at compile
  time on every surface. Supported explicit linear solves retain their
  conditioning/ill-posedness checks. A general implicit algebraic/DAE solver
  requires a separate research and acceptance programme.

### F47 — Bounded MATLAB/Simulink data and model interchange

**Next · M5 or later · Dependencies: F04, F39, F41, F43, F45**

- **Baseline and gap:** There is no supported MAT or SLX importer, Simulink
  model translator or universal block compatibility layer.
- **Proposed work:** First choose specific interchange workflows, source-format
  versions and a supported subset, such as numeric MAT data or explicitly
  exported state-space/parameter/scenario data. Any later SLX adapter would
  translate only declared block/solver/configuration semantics and emit a
  complete incompatibility report.
- **Acceptance:** Reference fixtures enumerate supported and rejected constructs.
  Imported data retains units/identity through explicit mappings; unsupported
  blocks, scripts, callbacks and implicit solver assumptions cannot be silently
  approximated. A translated model needs source-versus-Galata execution
  evidence, with the source tool/version and comparison budget recorded.

### F48 — Explicit state machines and supervisory logic

**Next · M5 or later · Dependencies: F17, F39, F43, F45**

- **Baseline and gap:** There is no Stateflow-like state-machine language or
  supervisory-logic execution engine.
- **Proposed work:** Select a bounded state-machine subset for controller modes,
  guards, events, entry/exit actions and timeouts. Define transition priority,
  initialization and interaction with the block scheduler before adding an
  editor. Do not claim Stateflow language compatibility.
- **Acceptance:** Simultaneous guards, event ordering, zero-time cycles,
  unreachable states and reset/timeout cases have explicit outcomes and tests.
  The machine's transition trace replays exactly and appears in model harness
  evidence. Unsupported concurrency or action semantics are compile errors.

### F49 — Restricted C/C++ generation and SIL equivalence

**Next · M5 or later · Dependencies: F02, F32, F37, F43, F45**

- **Baseline and gap:** Galata is implemented in C++; it does not generate
  controller source from visual models or qualify generated software.
- **Proposed work:** Define a restricted discrete-block subset, numeric types,
  initialization and step API for generated C/C++. Reject dynamic or unsupported
  constructs explicitly. Preserve generator, model and compiler identity and
  compare compiled software-in-loop execution with the accepted model semantics.
- **Acceptance:** Accepted subset models pass SIL trajectory, event and state
  comparisons under declared budgets, including reset, saturation and failure
  cases. Generated code has documented memory/execution behaviour for the
  chosen target profile. HIL and onboard use require their own hardware,
  timing, assurance and qualification gates; generation success grants none of
  those approvals.

## Deliberate deferrals and promotion criteria

These capabilities could extend the end product. They are excluded from the
Must v1 acceptance above; listing them does not imply an implementation plan.

| Candidate | Proposed disposition and reason | Evidence needed before promotion |
| --- | --- | --- |
| H-infinity synthesis, structured MIMO robustness and mu analysis | Research after F12. Existing norm analysis does not provide a robust synthesis solver or structured-uncertainty certificate. | A selected formulation, independent benchmark collection, conditioning/failure policy and engineer-reviewed application case. |
| General implicit algebraic-loop and descriptor/DAE solvers | Research after F46. Detecting an unsupported loop is a bounded v1 feature; solving arbitrary implicit dynamics changes the numerical contract. | A selected equation class, consistent-initialization strategy, independent difficult benchmarks and unresolved/failure semantics. |
| Model predictive control and nonlinear optimisation-based control | Research after F14/F17/F20. Constraint feasibility, numerical failure recovery and execution budgets create a separate solver/runtime scope. | An explicit offline or real-time use case, independent solver references, infeasibility policy and constrained closed-loop evidence. |
| Nonlinear estimators and automated system identification | Next after F16/F26. Linear estimation and transparent measured-data comparison establish the necessary contracts first. | Identifiable/observable cases, calibration and holdout partitions, uncertainty assessment, independent references and failure diagnostics. |
| Rotorcraft, VTOL transition, aeroelasticity and high-angle-of-attack flight | Separate model programmes after F07. These require new states, physics, data and applicability evidence; the fixed-wing local model cannot inherit them by adding interface fields. | Rights-cleared datasets, model adequacy criteria, independent response evidence and specialist review for the selected vehicle class. |
| C/plugin ABI and third-party model extensions | Next after F03/F32/F33. Public contracts and failure containment should stabilize before freezing an extension boundary. | A concrete integration need, version/ownership policy, conformance tests and untrusted-extension execution design. |
| Python bindings or MCP automation | Next after F32. Additional clients should use a single stable contract rather than create another implementation. | A prioritized user workflow, packaging/support ownership and parity tests for results, failures, provenance and cancellation. |
| AI-assisted study generation or interpretation | Research after F31/F32/F33. Suggestions must not silently alter physics, evidence or review decisions. Offline data policy and evaluation obligations need definition. | A bounded assistance task, evaluation set containing unsafe suggestions, explicit review semantics and proof that generated studies obey the same validators. |
| 3-D aircraft and scene visualization | Next after F29. Linked numerical plots and review are sufficient for the first complete engineering workflow; a scene cannot serve as numerical validation. | A specific diagnostic task improved by 3-D, verified frame/time mapping and an accepted rendering/performance budget. |
| Cloud execution and multi-user hosted collaboration | Separate deployment profile after F24/F31/F33. The baseline must remain usable on disconnected systems. | Customer authorization, data/storage architecture, identity/access controls and operational ownership for the intended environment. |
| Onboard executable generation and flight deployment | Separate product and assurance programme. An engineering design artifact is not a flight-qualified controller implementation. | Explicit target hardware, execution and safety requirements, configuration-controlled implementation and the applicable assurance/approval programme. |

The release decision would be based on completed Must v1 epics and their
accepted evidence, not a count of implemented algorithms or attractive screens.
Any reduced v1 scope would need a revised end-to-end workflow and explicit
removal of the affected claims before milestone acceptance.
