# Product discovery and decision agenda

**Status: proposed investigation plan, not implemented capability or a delivery
commitment.** Planning target: an offline desktop workbench for fixed-wing
aviation and defence engineering, with the CLI and C++ library as the first
delivery surfaces. SITL/HIL is an optional later extension. Onboard software is
a separate product and assurance effort.

The [operating guide](../WORKBENCH.md), [verification report](../VERIFICATION.md)
and [roadmap](../ROADMAP.md) define the current baseline. This agenda resolves
the decisions needed to extend it. It does not presume that a desktop shell,
new aircraft data or a commercial dependency has already been selected.

## How to use the agenda

Each package ends with a reviewable decision and retained evidence. An owner
role identifies accountability; it does not imply that a person has been hired.
At kickoff, assign a named owner, reviewer, effort cap and decision date. Those
caps are planning inputs to be agreed against available staff, not estimates
invented here. Close a package as **proceed**, **narrow**, **defer** or **stop**.
An unresolved question must retain its consequences and next evidence needed.

Numerical acceptance budgets must be agreed before evaluating candidate
implementations. Record their physical quantity, units, envelope, source,
uncertainty and consequence of failure. Keep algorithm verification, aircraft
model validation, controller assessment and product usability evidence distinct.
Two programs agreeing is useful evidence, but shared algorithms, libraries or
source models can preserve the same error.

The prototypes below are bounded experiments. They are not permission to build
all proposed capabilities. Fix reliability defects affecting an experiment
before treating its results as decision evidence; preserve the failing case.

## Ordered decision gates

| Gate | Required decisions | Proceed when | Stop or narrow when |
|---|---|---|---|
| A — Worth doing | D01, D02, D03 and an initial D16 cost assessment | A customer role owns a concrete review task, acceptance contract and usable evidence/data route | The customer, intended reliance, data access or funding owner cannot be identified |
| B — Credible engineering scope | D04–D09 for the selected initial workflow | Numerical comparisons, model semantics and an independent aircraft validation plan justify the proposed claims | Only self-consistency is available for a claim requiring external validation, or interpolation/timing semantics remain ambiguous |
| C — Product architecture | D10, D12–D14, then D11 | A bounded interoperability profile, recovery model, deployment boundary and measured desktop prototype satisfy the chosen environment | A stack needs unavailable runtime services, cannot preserve evidence, or fails the agreed workflow/performance/accessibility contract |
| D — Fund a delivery slice | D16 updated with A–C decisions | Named owners, data rights, verification work, support cost and release evidence are funded together | A feature-only estimate omits aircraft evidence, independent review, packaging or sustainment |
| H — Optional SITL/HIL entry | D15 after its dependencies | A specific bench, timing budget, interface owner and failure policy justify a separate extension | Offline speed or an FMI interface is the only evidence offered for real-time suitability |

Independent packages may run in parallel once their inputs exist. Gate C need
not wait for every future aircraft or controller. It does need a representative
workload from Gate A and the relevant semantics from Gate B. Gate H does not
block the offline product.

## Discovery packages

### D01 — Customer decisions and accepted engineering artifacts

- **Question:** Who uses the product, what decision do they make, and what
  evidence does their reviewer accept? Candidate tasks are model import/review,
  trim and mode comparison, controller change assessment, and a repeatable
  operating-envelope campaign; these are hypotheses to rank. Include a
  Simulink-style block-model workflow in the ranking and identify the specific
  blocks, hierarchy and automation that make it useful. Full product parity is
  not the default scope.
- **Method/prototype:** Observe a customer engineer completing a recent study
  with authorized or sanitized inputs. Reconstruct one priority task with the
  existing CLI, including a deliberately invalid model and a failed run. Review
  its tables, plots, assumptions, exceptions and run record with the recipient.
- **Output/decision:** A task map from input owner to engineering decision;
  required artifacts; current effort and error sources; and a customer-approved
  acceptance script with numerical, usability and review requirements.
- **Owner:** Product lead with a flight-dynamics engineer and customer reviewer.
- **Dependencies:** None; do not contact customers without authorization.
- **Stop/go:** Proceed with a named user and evidence recipient who accept the
  task contract. Narrow if the only request is a broad feature list or visual
  demonstration without a downstream decision.

### D02 — Intended reliance and assurance scope

- **Question:** Is the product supporting exploration, independently checked
  engineering analysis, a contractual verification activity, or a qualification
  effort? What errors could escape the customer's existing review process?
- **Method/prototype:** Walk the D01 task through a representative incorrect
  model, optimistic numerical result and incomplete report. Have the customer's
  assurance authority identify required independence, review records, applicable
  standards/editions, configuration controls and retention obligations.
- **Output/decision:** Intended-use statement, prohibited claims, responsibility
  split, evidence matrix and a scoped assurance work estimate. Record separately
  any request for tool qualification; do not infer a qualification level or
  aviation approval from the industry label or from passing tests.
- **Owner:** Assurance lead with the customer engineering authority.
- **Dependencies:** D01; an initial D16 funding boundary.
- **Stop/go:** Proceed when the customer accepts what the tool establishes and
  what requires independent evidence. Defer qualified use if its lifecycle,
  independence or retained evidence cannot be funded; preserve offline scope.

### D03 — Aircraft data access, rights and pedigree

- **Question:** Which aircraft/configuration has usable aerodynamic, mass,
  propulsion, actuator and measured-response evidence for the initial workflow?
- **Method/prototype:** Build a field-level inventory identifying origin,
  transcription, calibration, reference conditions, uncertainty, known missing
  terms and permitted use. Inspect an authorized sample through the proposed
  ingestion path. Separate redistribution rights from permission for a customer
  to use private data locally. Record where originals and derived artifacts may
  be retained or shared, including model bytes embedded in run manifests.
- **Output/decision:** Selected reference case, pedigree register, access and
  rights decision, missing-data plan and a safe public example strategy. Follow
  [ADR-0007](../adr/0007-reference-values-from-copyrighted-sources.md); do not copy
  proprietary aerodynamic tables or assume a hosted report is redistributable.
- **Owner:** Aircraft-model lead with the data owner and licensing reviewer.
- **Dependencies:** D01 and D02.
- **Stop/go:** Proceed only with an explicit use route and sufficient evidence
  for the selected claims. Use synthetic/public fixtures for infrastructure
  work while restricted access is unresolved; do not relabel them validated.

### D04 — Independent numerical benchmarks and solver reuse

- **Question:** Which algorithms should Galata retain, replace, wrap or use only
  as cross-checks, and what constitutes an independent numerical oracle?
- **Method/prototype:** Select published and analytic cases covering scaling,
  ill-conditioning, nonnormal dynamics, repeated/near-axis poles and expected
  refusal. Compare Riccati solutions/gains, closed-loop stability, residuals and
  conditioning, plus norm brackets and adverse peak cases. Evaluate SLICOT and
  SciPy candidates with pinned versions and documented algorithm dependencies;
  obtain commercial-solver results only through an authorized installation.
- **Output/decision:** A benchmark catalog with source/rights records, failure
  taxonomy, prespecified error budgets and build/reuse decision per algorithm.
  Retain disagreements for investigation rather than widening a passing band.
- **Owner:** Numerical-analysis lead with an independent reviewer.
- **Dependencies:** D02; D03 for aircraft-derived cases.
- **Stop/go:** Proceed only when unresolved discrepancies are outside the
  promised scope or cause an explicit refusal. Agreement of residuals alone is
  insufficient evidence for a correct stabilizing solution or conservative bound.

SLICOT publishes the [CAREX benchmark collection](https://www.slicot.org/working-notes/wgs-niconet-reports/65-carex-a-collection-of-benchmark-examples-for-continuous-time-algebraic-riccati-equations-version-2-0)
and a [benchmark generator interface](https://www.slicot.org/objects/software/shared/doc/BB01AD.html).
SciPy documents a [QZ-based CARE implementation and its failure conditions](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_continuous_are.html).
These are candidates for a verified comparison harness, not blanket accuracy or
redistribution guarantees.

### D05 — Scheduled aerodynamic model contract

- **Question:** What is the smallest model representation that supports the
  chosen conditions without silently changing coefficient meanings?
- **Method/prototype:** Import an authorized table and a synthetic table with
  analytically known derivatives. Exercise grid knots, boundary faces, missing
  cells and invalid extrapolation. Specify units, frames, coefficient/rate
  normalization, moment reference point, configuration, scheduling variables,
  interpolation, continuity and derivative treatment at breakpoints. Distinguish
  local validity metadata from demonstrated aerodynamic validity. For block
  models, investigate a shared typed data dictionary for parameters, units,
  signal dimensions and named structures, with explicit ownership and overrides.
- **Output/decision:** A versioned schema proposal, canonical SI conversion
  rules, capability-based validity contract and interpolant choice with error
  evidence. Decide whether sparse tables or scattered samples are initially
  supported; do not make missing coefficients silently zero by convention.
- **Owner:** Aerodynamics lead with the numerical and data engineers.
- **Dependencies:** D03 and D04.
- **Stop/go:** Proceed when independent calculations recover values and slopes
  under documented conventions and all unsupported/extrapolated queries are
  explicit. Narrow the envelope if the table cannot support the needed physics.

### D06 — Propulsion, configuration and physically feasible trim

- **Question:** Which trim and maneuver requests must be physically achievable,
  and what hardware/configuration information is needed to establish that?
- **Method/prototype:** Replay a selected trim family using documented surface
  stops, thrust availability/lapse, actuator limits, CG/inertia and configuration
  data. Add cases demanding unavailable thrust, an impossible control deflection
  and asymmetric moments. Compare a bounded constrained solve with the current
  unconstrained result and independent force/moment calculations.
- **Output/decision:** Supported trim constraints and maneuvers, propulsion
  adapter boundary, feasible/infeasible/unknown states and conditioning/refusal
  diagnostics. Decide whether configuration changes are discrete selections or
  modeled transitions; retain their effects on mass and reference points.
- **Owner:** Flight-mechanics lead with propulsion/actuation expertise.
- **Dependencies:** D01, D03–D05.
- **Stop/go:** Proceed when an accepted trim is both an equilibrium and feasible
  under the declared hardware assumptions. If availability data is missing,
  report unknown feasibility and narrow the intended-use claim.

### D07 — Signal-flow semantics, discrete clocks and events

- **Question:** How do typed block signals, feedback cycles, sampled control,
  sensor updates, integration steps, delays and limit events interact
  reproducibly? The study's acyclic stage graph must remain distinct from a
  cyclic signal-flow model evaluated inside a simulation stage.
- **Method/prototype:** Implement a disposable scheduler and typed block-model
  representation around an analytic plant and a stateful sampled controller.
  Use a bounded block set selected by D01. Exercise simultaneous events,
  different rates, start/reset, sample-and-hold, delayed messages, saturation
  and sampling periods that do not divide the integration step. Compare traces
  to an independently derived discrete system and refine integration steps.
  Add a delayed feedback cycle and an algebraic loop; decide sample-time
  propagation, direct-feedthrough detection and whether unsupported loops are
  rejected or resolved by a separately verified solver.
- **Output/decision:** A clock/event-order contract, explicit delay and hold
  semantics, typed graph validation rules, supported period representation and
  determinism/error budgets. Define hierarchical subsystem/library instance
  semantics independently of their diagram rendering.
  Define when random samples are drawn and how seeds, streams and resets are
  recorded. Stateful controllers must not advance at every RK stage by accident.
- **Owner:** Simulation lead with control and numerical reviewers.
- **Dependencies:** D01, D02 and D04.
- **Stop/go:** Proceed when event ordering and state updates match the independent
  trace and repeat after restart. Reject ambiguous schedules instead of silently
  rounding sample periods; keep offline virtual time separate from wall time.

### D08 — Controller, gain-schedule and estimator lifecycle

- **Question:** Which controller family and measurement assumptions are needed
  for the first task, and how are gains, estimator states and schedules reviewed?
- **Method/prototype:** Build a bounded sampled-control example using the D07
  semantics. Exercise bias/noise, measurement units, observable/unobservable
  channels, saturation/anti-windup, initialization and reset. If scheduling is
  required, compare design points and transitions, interpolation, switching and
  out-of-domain behavior using an independent analysis route. For a requested
  Stateflow-style feature, first select a small state-machine subset and define
  transition priority, guards, actions and event timing; explicitly decide which
  hierarchy/history/parallel-state features to defer. Compare event traces to an
  independently authored reference rather than just matching diagram appearance.
- **Output/decision:** Versioned controller/estimator inputs and states, gain
  lineage, schedule validity, measurement/plant mapping and transition criteria.
  Select design versus import-only support. Frozen-point stability must not be
  reported as proof of stability throughout a varying schedule. Keep state-machine
  and block semantics suitable for later code-generation review, without making
  generated or onboard code part of this package's acceptance claim.
- **Owner:** Control-design lead with an estimation specialist as needed.
- **Dependencies:** D04–D07 and measurement data from D03.
- **Stop/go:** Proceed when state ordering, sample timing, limits and reset
  behavior are unambiguous and the selected acceptance cases pass. Defer automatic
  scheduling or estimation when only ideal full-state evidence exists.

### D09 — Aircraft validation and decision-level uncertainty

- **Question:** What independent evidence bounds the error in the quantities
  that drive the customer's decision across the promised envelope?
- **Method/prototype:** Reserve measured or independently produced cases from
  calibration. Compare relevant trim, derivatives, modes and time histories with
  their uncertainty, instrumentation bandwidth, alignment and configuration.
  Separate model-form, parameter, measurement and numerical uncertainty; test
  correlated parameter perturbations where the evidence supports them.
- **Output/decision:** An aircraft validation matrix, discrepancy disposition,
  supported envelope and reference report showing uncertainties and acceptance
  budgets. For handling-quality assessments, obtain the customer's applicable
  criteria and definitions before implementing automated pass/fail classifications.
- **Owner:** Verification/validation lead independent of model calibration.
- **Dependencies:** D02–D06; D07/D08 for sampled closed-loop claims.
- **Stop/go:** Proceed only for quantities and conditions supported by evidence.
  Missing flight evidence remains a visible gap; agreement with a simulator using
  the same data is a consistency comparison, not independent aircraft validation.

### D10 — Interoperability profile and adapter reuse

- **Question:** Which exchanges remove real customer effort, and which meanings
  are lost when moving between tools?
- **Method/prototype:** Round-trip one named state-space model and one time
  history through the highest-priority interfaces. Compare a JSBSim model/run
  mapping, an explicitly selected FMI interface, MAT/CSV exchange and a Python
  driver. Test units, axes, channel order, timestamps, sample times, metadata,
  missing values and incompatible versions. A runnable FMU is executable code;
  include its loading boundary in D13. Run separate bounded subspikes for
  customer-required Simulink exchanges: investigate officially supported
  APIs/export routes for an explicit SLX subset through an authorized tool
  installation, and report every unsupported block, parameter or semantic
  conversion. Do not assume native SLX compatibility from parsing its container.
- **Output/decision:** A narrow supported import/export profile and loss ledger;
  choose native implementation, external adapter or defer per interface. Decide
  whether Python initially invokes the CLI or needs a versioned native binding.
  If C code generation is requested, select a small controller/block subset,
  explicit numeric types and runtime contract, then compare generated-code SIL
  traces against the interpreter across resets and limit events. Record compiler,
  undefined-behavior/overflow assumptions and semantic limitations. This is a
  future code-generation decision, not a flight-software or full Simulink/Stateflow
  compatibility claim.
- **Owner:** Integration lead with customer toolchain owners.
- **Dependencies:** D01, D03, D05 and D07 where dynamic coupling is required.
- **Stop/go:** Proceed when the receiving tool reconstructs the intended
  quantities and records provenance. Refuse ambiguous mappings; do not claim full
  model conversion from a successful matrix or trajectory exchange.

The [JSBSim manual](https://jsbsim-team.github.io/jsbsim-reference-manual/)
provides a flight-dynamics integration starting point; its software license does
not establish every aircraft dataset's pedigree. The
[FMI specification](https://fmi-standard.org/docs/3.0.2/)
distinguishes Model Exchange, Co-Simulation and Scheduled Execution; selecting
one is part of this decision. For MAT files,
[SciPy documents support through MATLAB 7.2 and a separate HDF5 requirement for 7.3](https://docs.scipy.org/doc/scipy/reference/generated/scipy.io.loadmat.html).
Version, shape and metadata choices must therefore be explicit in the profile.

### D11 — Desktop shell: Qt versus a local web interface

- **Question:** Which stack completes the accepted engineering workflow with
  the lowest support burden on the actual customer machines?
- **Method/prototype:** Build the same thin workflow in Qt and a local web shell:
  open a study, validate inputs, run/cancel an isolated numerical worker, compare
  plots, inspect a failed result and export its evidence. Include keyboard-only
  operation, screen-reader labels, high DPI, large tables, offline documentation,
  worker crash recovery and an installation on a clean target machine. Shortlist
  Electron versus an OS-webview shell before building the web candidate. If D01
  selects block authoring, include typed ports, a nested subsystem, a reusable
  library instance, data-dictionary editing and compile-time diagnostic navigation
  using the same D07 semantics; diagram editing must not bypass graph validation.
- **Output/decision:** Measured comparison, module-level license/redistribution
  review, runtime inventory, accessibility results and a desktop ADR. Keep
  numerical logic in the same CLI/library-backed worker used by automation.
- **Owner:** Desktop lead with UX, release and security reviewers.
- **Dependencies:** D01, D10 and D12–D14.
- **Stop/go:** Select only a candidate satisfying the offline, deployment and
  review contracts. If neither does, continue the CLI product and narrow the
  desktop slice; do not select on screenshots or assumed package size.

Investigate actual Qt modules and terms: Qt describes
[LGPL/commercial options and module-specific obligations](https://www.qt.io/development/open-source-lgpl-obligations)
and [deployment dependencies, including accessibility plugins](https://doc.qt.io/qt-6/deployment.html).
A web candidate must follow the relevant isolation model, such as
[Electron's context isolation and sandbox guidance](https://www.electronjs.org/docs/latest/tutorial/security).
[Tauri uses platform webviews](https://v2.tauri.app/concept/process-model/), so
runtime availability and rendering differences belong in the test matrix;
Microsoft documents [WebView2 distribution choices](https://learn.microsoft.com/en-us/microsoft-edge/webview2/concepts/distribution).
None of these sources selects the stack for Galata.

**Prototype status.** A third candidate now exists and is partly measured: a
native AppKit/Objective-C++ macOS shell, built under
[ADR-0012](../adr/0012-project-worker-preview.md) and recorded in the
[M2 implementation guide](M2_IMPLEMENTATION.md). It runs the workflow this
packet asks for — open a project, edit a typed block diagram, launch and cancel
an isolated CLI worker, inspect a failed result, review and restore a saved
revision, and read the retained evidence — against the same worker headless
callers use. Keyboard-selectable block and sample tables, diagnostic navigation
and offline packaging exist and are checked by
`scripts/check-desktop-package.py`.

This does not answer D11. The packet's decision rests on a *comparison*, and
the Qt and web-shell candidates have not been built, so there is nothing to
compare against. Nor is the native candidate itself complete on this packet's
terms: it has no Developer ID signing or notarization, no clean-target-machine
installation, no full accessibility acceptance and no Linux GUI story — and a
macOS-only shell cannot satisfy the macOS-first-then-Linux delivery order on its
own. The stop/go rule stands unchanged: do not select on a working prototype
any more than on screenshots.

### D12 — Campaign storage, compatibility and recovery

- **Question:** What is authoritative when studies, runs, plots, cached arrays
  and application versions change, and how is an interrupted campaign recovered?
- **Method/prototype:** Compare a portable directory/package with immutable
  artifacts against a local indexed store using the same campaign. Exercise
  worker termination, disk full, duplicate jobs, concurrent readers, interrupted
  publication, damaged cache, migration and reopening by an older supported
  release. Restore a backup and verify input/output identities independently.
- **Output/decision:** Ownership/transaction model, cache invalidation rules,
  schema migration policy, compatibility window and archive/restore contract.
  Distinguish study specification, mutable working state and immutable run
  evidence; retain partial failure without presenting it as completed analysis.
- **Owner:** Data/platform lead with verification and release reviewers.
- **Dependencies:** D01, D03, D07 and workload sizes from D14.
- **Stop/go:** Proceed when interrupted work recovers without silent result
  substitution and supported migrations preserve provenance. Defer shared-drive
  editing if its locking and durability behavior is not established.

[SQLite documents that WAL mode does not work across network filesystems](https://www.sqlite.org/wal.html).
A local SQLite index is therefore a candidate to test, not a general shared
campaign-store answer. Pin and review the actual selected database release.

**Prototype status.** The portable-directory half of this comparison is built.
[ADR-0012](../adr/0012-project-worker-preview.md) implements content-addressed
immutable revisions, run-owned artifacts, OS locking and atomic head
publication; [ADR-0014](../adr/0014-project-revision-recovery.md) adds bounded
history listing and explicit restore. The three-way ownership split this packet
asks for is realised as draft, submitted request and terminal record, and
interrupted work is distinguishable from completed work by a verified terminal
record rather than by the presence of output — the
[project-file contract](../PROJECT_FILES.md) states the status meanings.

The indexed-store arm is unbuilt, so the comparison is still open, and the
preview deliberately claims no migration policy, compatibility window,
shared-drive editing or power-loss durability. Its recovery evidence is local
to one filesystem on macOS arm64.

### D13 — Offline deployment and execution trust boundary

- **Question:** What can a study, imported model, report, plugin or update cause
  the application to read, write or execute in the customer's environment?
- **Method/prototype:** Trace those boundaries through a packaged thin slice on
  a disconnected machine. Test malformed/oversized files, crafted paths and
  reports, untrusted plugin/FMU loading, worker resource exhaustion, cancellation
  and update rollback. Verify that required installation, licensing,
  documentation and normal execution work under the agreed network restrictions.
- **Output/decision:** Threat model, worker/IPC and filesystem policy, dependency
  inventory, artifact-signing and offline-update design, data-sharing defaults
  and vulnerability-response ownership. Treat model bytes in manifests and
  diagnostics as customer data; hashes alone do not establish authorship.
- **Owner:** Security/release lead with customer IT.
- **Dependencies:** D01–D03; D10 provides adapter candidates.
- **Stop/go:** Proceed when imported content has no unintended execution or data
  access route and the supported offline lifecycle is demonstrated. Disable or
  defer extension loading when its containment and support cost are unresolved.

### D14 — Supported platforms and measured performance envelope

- **Question:** Which operating systems, architectures and workloads must the
  first supported release handle, within what resource and response budgets?
- **Method/prototype:** Obtain representative customer machines and measure a
  trim/mode grid, a sampled nonlinear run, a parameter campaign and interactive
  review of its saved outputs. Record state/table sizes, conditions, duration,
  event rates, sample volume and concurrency. Compare solver time, memory, disk,
  worker startup, cancellation and plot response separately; verify deterministic
  results when parallel execution changes job completion order.
- **Output/decision:** Supported OS/architecture matrix, workload definitions,
  measured baseline, numerical/UX resource budgets and optimization priorities.
  Decide whether sparse methods or distributed campaigns are actually required.
- **Owner:** Performance/platform lead with the D01 users.
- **Dependencies:** D01; D04–D08 supply only the selected representative models.
- **Stop/go:** Commit only measured combinations meeting agreed budgets. Do not
  infer real-time capability from a favorable average offline throughput figure.

### D15 — Optional SITL/HIL boundary

- **Question:** Does a specific customer bench require external controller code
  or hardware, and can its timing and failure behavior be supported separately?
- **Method/prototype:** First connect a software controller stub through an
  explicit adapter. Exercise clocks, timestamps, overruns, stale/out-of-order
  messages, disconnects, reset and replay under imposed delays. Only after that
  contract is accepted, define a bench experiment with an authorized hardware
  owner, interfaces, timing budget, containment and stop procedure.
- **Output/decision:** Separate SITL/HIL architecture, clock ownership,
  deadline/overrun policy, replay limits, target support matrix and verification
  cost. Determine whether hardware access needs a dedicated process or host.
- **Owner:** Simulation-integration lead with the bench owner.
- **Dependencies:** D02, D07, D08, D10, D13 and D14.
- **Stop/go:** Fund HIL only when a real bench and measured timing contract
  justify it. Keep offline virtual-time execution valid if the extension is
  deferred. An FMI wrapper or desktop timer does not establish hard real-time
  behavior, and this package does not authorize onboard deployment.

### D16 — Build, buy, staffing and sustainment decision

- **Question:** Can the chosen product scope be delivered and maintained at the
  customer's required evidence level with the available funding and expertise?
- **Method/prototype:** Use an initial cost envelope at Gate A, then replace it
  with a work breakdown informed by completed spikes. Compare retaining Galata
  algorithms, wrapping approved libraries and buying integrations/tools using
  total lifecycle effort: licensing, data access, model validation, independent
  review, platform packaging, training, support, updates and migration. Identify
  single-person dependencies and a realistic defect/escalation path.
- **Output/decision:** Funded scope, role allocation, dependencies, uncertainty
  ranges, make/buy/defer choices, pilot contract and support/retirement policy.
  Set calendar milestones only after staffing, procurement and data lead times
  are known; tie payment/acceptance to evidence-bearing deliverables.
- **Owner:** Product owner with engineering, assurance and commercial leads.
- **Dependencies:** Initial D01–D03; final decision uses D04–D14. Price D15 as an
  optional extension so that it does not silently expand the offline baseline.
- **Stop/go:** Proceed when development and sustainment are funded together and
  reviewers/data owners are available. Narrow scope when validation or support
  is unfunded, even if a feature prototype already works.

## Inputs that must remain unknown until supplied

The project does not establish the following customer facts. Record answers and
their owner at Gate A instead of substituting convenient defaults.

| Unknown | Decision it blocks |
|---|---|
| First user, purchasing sponsor and receiving engineering authority | Task priority, acceptance and intended reliance |
| Aircraft class, configurations, operating envelope and control architecture | Model fidelity, data selection, trim and simulation scope |
| Available aerodynamic/hardware/flight evidence and its permitted use | Validation claims, redistribution and sharing defaults |
| Whether results support a qualification or contractual approval activity | Evidence independence, lifecycle and assurance budget |
| Customer OS/architecture, disconnected-site policy and permitted runtimes | Desktop stack, installers, updates and interoperability |
| Existing MATLAB/Python/simulator/FMI workflows and tool licenses | Adapter priority and build/buy choices |
| Team skills/capacity, funding, procurement lead times and desired horizon | Ownership, feasible scope and delivery dates |
| Required maintenance period, compatibility expectations and support owner | Dependency policy, migrations and lifecycle cost |
| Specific SITL/HIL bench and deadline/failure requirements | Whether the optional extension should begin |

Official ecosystem sources above were checked on 2026-09-07. At each spike's
start, record the precise release, license, platform dependencies and retrieved
documentation revision used. A later upstream change requires revisiting the
affected decision; these links are discovery inputs, not a frozen dependency
approval list.
