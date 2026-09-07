# Proposed product assurance and acceptance plan

**Status: proposed work, 7 September 2026.** This document defines evidence to
build and acceptance gates to satisfy. It does not declare that the controls,
campaigns, approvals or desktop features below already exist. The [product
definition](../PRODUCT_PLAN.md), [delivery plan](DELIVERY.md) and [Simulink-style
scope](SIMULINK_FEATURES.md) define the coordinated milestone boundaries. The current
[verification report](../VERIFICATION.md), [operating guide](../WORKBENCH.md)
and [security policy](../../SECURITY.md) remain the statements of delivered scope.

The target is a complete **offline fixed-wing engineering desktop workbench**,
with the same engineering capabilities available through the CLI and C++ API.
Its users would develop models, author cyclic signal-flow diagrams and
subsystems, trim and linearise models, design and assess controllers, simulate
hybrid/multirate cases, compare evidence and review a study for an intended
engineering decision. Graphical modeling is part of the proposed v1 product;
its assurance must cover the compiled semantics as well as its appearance.
Hardware-in-the-loop work is an optional later program. State machines, wider
model interchange and restricted C code generation belong to a subsequent
track. Onboard execution and aircraft approval are outside this product plan.

## Intended use decides the assurance obligation

Before expanding features, the product owner, domain lead and prospective
customer should approve an intended-use record containing the aircraft class,
flight regimes, decisions supported, expected users, model sources, operating
systems, deployment environment and independent checks required of the user.
It should identify the consequence of each plausible wrong result: an incorrect
design choice, missed instability, invalid load estimate, lost study evidence or
disclosure of restricted model data. Assign accountable owners and an acceptance
authority; record likelihood as unknown where there is no defensible evidence.

Maintain three separate decisions:

| Decision | Proposed decision maker | Evidence needed | Meaning of completion |
|---|---|---|---|
| Product acceptance for v1 | Product owner with independent verification, domain and security leads | Supported-feature contracts, complete verification campaigns, supported installations, operator training and disposition of product risks | The named release is accepted for its declared offline use |
| Aircraft/program acceptance of a study | Customer's responsible engineering and safety authorities | Aircraft/configuration-specific data, validation domain, uncertainty, acceptance requirements and independent review | The study is acceptable for the particular decision recorded |
| Tool qualification, if required by the use | Applicant/program and relevant certification authority | A use-specific qualification determination and the applicable lifecycle evidence for the selected configuration | The identified tool use and environment satisfy the agreed qualification basis |

FAA AC 20-115D concerns airborne software assurance and recognizes DO-330 as a
separate tool-qualification framework. The need and level depend on tool use and
its impact in the software lifecycle. Calling this an aviation workbench does
not automatically apply every DO-178C objective or establish a qualification
level. Record whether the intended use replaces or reduces a verification
activity, what detects tool errors, and the authority's agreed determination.
If qualification is pursued, obtain the applicable standards and plan its
evidence explicitly. [FAA AC 20-115D, sections 1, 2 and 10](https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_20-115D.pdf).

For model credibility, NASA-STD-7009B provides a useful reference framework for
model/simulation development and use with program-defined acceptance criteria.
Adopting selected practices would not imply NASA acceptance. [NASA standard
record](https://standards.nasa.gov/standard/nasa/nasa-std-7009).
For defence programs, record the actual contractual system-safety basis and
assigned risk authority. MIL-STD-882 covers hardware/software hazards through
the lifecycle when applicable; this document does not assign it to every
customer. [Official MIL-STD-882 record](https://quicksearch.dla.mil/qaDocDetails.aspx?ident_number=36027).

The first policy correction should reconcile the present security policy's
blanket certification-evidence prohibition with the operating guide's
application-specific review language. The proposed policy is that no generic
qualification or aircraft approval is claimed, and every certification-related
use needs its own explicit determination. Approving this plan would not itself
change that policy or authorize a customer's use.

## A result must carry four distinct kinds of evidence

The desktop, CLI, C++ artifacts, CSV exports and human-readable reports should
share one versioned result contract. A green completion indicator must never
mean that an aircraft design is acceptable.

| Evidence axis | Proposed states | Required supporting information |
|---|---|---|
| Execution | Completed, failed, cancelled, interrupted | Stage and run identities; expected and produced artifacts; failure time/reason; atomic completion record; partial-output status |
| Numerical reliability | Resolved within budget, estimate only, unresolved, invalid input | Method/options; physical scaling; residuals; conditioning; discretisation/refinement evidence; search coverage; bounds and their direction; diagnostics |
| Model validity | Validated for the stated domain, supported by limited evidence, outside domain, unknown | Model/data revision; validation cases; condition/configuration; interpolation or extrapolation status; missing physics; uncertainty and limitations |
| Engineering acceptance | Not evaluated, requirements met, requirements not met, review required | Named requirement set and revision; measured margins to criteria; treatment of uncertainty; reviewer identity and decision; scope of acceptance |

An estimate cannot silently become a bound. A sampled norm peak should remain a
lower estimate unless a stronger argument establishes an upper bound; derived
robustness guarantees need the mathematically conservative endpoint and a
stated numerical assurance level. A missing crossover is a statement about the
search, not proof of infinite physical tolerance. Unresolved eigenvalue signs,
Euler-chart conditioning or equilibrium evidence should propagate downstream
and prevent a stronger conclusion from being issued.

The proposed evidence record should retain:

- Exact study and model bytes, units, frames, channel identities, configuration,
  source/licensing pedigree and transformations applied at import.
- Capability/method versions, effective defaults, physical and numerical
  assumptions, solver options, uncertainty definitions, seeds, stopping reasons
  and all diagnostics used to accept or reject the result.
- Exact source identity, effective build configuration, dependency inventory,
  executable and loaded-library identities, platform/math-runtime information
  and the signed installation inventory when available.
- Full-resolution numerical output and integrity hashes. Decimation for plots
  must preserve the original record and disclose its method; plot limits must
  not clip a violation without notice.
- Requirements evaluated, bounds used, warnings/overrides, links to independent
  checks, and a separately signed or otherwise controlled review record bound
  to the result digest. A review must not rewrite the underlying evidence.

Changing a model, controller, solver setting, runtime or acceptance criterion
should invalidate the associated acceptance decision and trigger the relevant
re-evaluation. A report copied away from its workspace should still identify
its evidence axes, input/result digest, limitations and means of verification.

## Requirements and traceability

Use stable requirement IDs, with an owner, rationale, source, intended-use
scope, verification method and predeclared acceptance criterion. Keep revisions
and supersession history. Trace in both directions through requirement,
hazard/risk, design decision, implementation, benchmark/test/review result and
release. Generated reports should detect missing links, orphan claims and
evidence from the wrong revision. A registered test name alone does not prove
that a requirement was exercised or passed.

The following are proposed assurance requirements to decompose at M1. These IDs
identify acceptance work; they are not claims of existing compliance.

| ID | Proposed requirement | Minimum verification evidence |
|---|---|---|
| ASR-001 | Every supported engineering conclusion preserves the four evidence axes through every interface | Contract tests, exported-artifact checks and representative operator review |
| ASR-002 | Every public numerical boundary rejects invalid values/dimensions/options or returns an explicit unresolved outcome | Invalid-input, conditioning and resource-bound campaigns for CLI and direct C++ use |
| ASR-003 | Reported bounds and guarantees retain their valid direction and assumptions | Independent analytic counterexamples and a preapproved benchmark collection |
| ASR-004 | Trim, linearisation and simulation use evidence tied to the actual model, condition, state and controls | Stale-artifact/mutated-model tests and independently recomputed equilibrium checks |
| ASR-005 | Model validity and uncertainty are explicit for each requested study condition | Versioned model dossier, boundary tests and independent validation data |
| ASR-006 | Every completed study can be attributed to its exact inputs and complete supported runtime configuration | Replay, changed-dependency, changed-source and provenance-integrity tests |
| ASR-007 | Report publication, cancellation, recovery and migration preserve study evidence and identify partial results | Fault injection, power/process interruption and restore/migration exercises |
| ASR-008 | Data movement and extension execution follow the selected offline deployment policy | Threat-model review, network observation, isolation tests and export inspection |
| ASR-009 | Released artifacts bind one immutable source revision to complete passing evidence and authentic distributables | Negative release-gate tests, signed inventory verification and offline install/update/rollback exercises |
| ASR-010 | Accepted results and product claims have explicit scope, independent review and supported lifecycle ownership | Acceptance audit, operator trials, support drills and versioned decision records |
| ASR-011 | Visual models compile to one explicit typed, dimensioned, unit-aware, timed execution model shared with headless execution | Block/connection-to-IR traceability, compiler rejection tests, execution equivalence and preserved source mappings |
| ASR-012 | Hybrid/multirate execution and tuning have deterministic, replayable timing and state semantics | Independent event/sample-time cases, schedule/conflict tests, state initialization/reset evidence and tuning revision replay |

For each error-sensitive requirement, agree independence before implementation.
The person or team preparing the expected result should not derive it from
Galata's output. A second numerical package is useful only after checking that
it does not share the relevant implementation and assumptions. Review residuals,
conditioning and physical plausibility as well as numerical agreement.

Measure structural coverage to find unexercised failure paths and code; review
uncovered decisions in error-sensitive routines. Set coverage obligations from
the risk/qualification determination, not a universal percentage or a
test-to-source line ratio. Use mutation testing selectively to check whether
critical acceptance predicates can fail. Preserve explanations for justified
exclusions and rerun affected evidence after changes.

## Initial risk register and audit closure

The observed defects below come from the September 2026 working-tree review.
They remain open in this plan. M0 should preserve the exact audited source
snapshot, minimal fixtures and independent expected results in maintained
evidence, so closure does not rely on temporary build output. The source links
identify the affected surfaces; they do not imply that a later line change
closes the finding. Priorities describe offline engineering impact, not an
airworthiness level. Likelihood and residual risk require assessment in A02.

| Risk | Basis and consequence | Proposed treatment and closure evidence | Work/gate |
|---|---|---|---|
| AR-01: false robustness guarantees | Observed: sampled sensitivity underestimates a controlling peak, while the report asserts guaranteed classical margins. See [sensitivity implementation](../../src/analyze/sensitivity.cpp) and [report capabilities](../../src/pipeline/capabilities.cpp). | Separate estimates from bounds; independently establish the conservative conversion; preserve out-of-band and narrow-peak counterexamples; audit disk-margin wording and all downstream decisions | A01, A03, A04 / M0 |
| AR-02: unreliable stability sign | Observed under extreme conditioning: an exactly unstable finite matrix passes the classical/disk nominal-stability gates. See [margins](../../src/analyze/margins.cpp) and [disk margins](../../src/analyze/disk_margin.cpp). | Add finite, scaling and conditioning-aware reliability checks with an unresolved outcome; preserve an exact-arithmetic reference and equivalent-realisation cases; never fix solely by enlarging a sign tolerance | A01, A04 / M0 |
| AR-03: discarded linearisation limitation | Observed: Euler-chart conditioning is computed but lost in pipeline conversion, allowing a misleading successful report. See [linearisation contract](../../include/galata/linearize/finite_difference.hpp) and [capabilities](../../src/pipeline/capabilities.cpp). | Carry every validity diagnostic through artifacts/reports; test singular-chart approach and refusal before controller design | A01, A03 / M0 |
| AR-04: stale equilibrium evidence | Observed in direct C++ use: altered controls can retain an old trim residual when linearised. See [finite differences](../../src/linearize/finite_difference.cpp). | Bind artifacts to model/state/control identities and re-establish equilibrium; test mutated trim and model inputs | A01, A03 / M0 |
| AR-05: invalid C++ inputs become misleading outputs | Observed boundary gaps include nonfinite mass/aerodynamic data, integration/Newton options and callback responses; disk channel bounds also need explicit validation. See [rigid body](../../src/sim/rigid_body.cpp), [aircraft](../../src/model/aircraft.cpp), [numerics](../../src/numerics/) and [disk margins](../../src/analyze/disk_margin.cpp). | Define one public-boundary policy; exercise invalid, unconverged and unresolved paths through every supported API; sanitizer-backed negative tests | A01, A08 / M0 |
| AR-06: release without complete evidence | Observed workflow gap: release publication does not require all CI checks and resolves a mutable tag separately in matrix jobs. See [release workflow](../../.github/workflows/release.yml). | Resolve one commit SHA, require all applicable CI checks for it, verify package/source identity and test rejected/skipped/mismatched evidence | A01, A09 / M0 |
| AR-07: incomplete runtime identity | Observed: run provenance hashes the executable but not loaded libraries; a dirty flag does not identify dirty source bytes. See [provenance](../../src/pipeline/provenance.cpp) and [build stamp](../../cmake/GalataProvenance.cmake). | Bind a complete runtime inventory and source/build identity; detect compatible-library substitution and retain a replayable installation | A01, A09 / M0 core closure; M4 campaign |
| AR-08: untested minimum and stale claims | Observed: preset schema and advertised CMake minimum disagree; assurance prose contains conflicting/outdated claims. See [presets](../../CMakePresets.json), [security policy](../../SECURITY.md) and [determinism ADR](../adr/0004-determinism-policy.md). | Test the actual minimum, reconcile policies, generate measurable claims and gate evidence-site publication with the release/CI baseline | A01, A09 / M0 |
| AR-09: model use beyond supporting evidence | Known scope limit: one aircraft reference condition and advisory alpha/Mach guards do not establish a full flight envelope or flight-data agreement. See [operating guide](../WORKBENCH.md). | Model dossiers, multidimensional domain checks, uncertainty and independent flight/simulator comparisons; refuse unsupported acceptance claims | A05, A06 / M3–M5 |
| AR-10: hostile or oversized imported data/extensions | Future desktop importers and optional extensions enlarge today's local file attack surface | Parser/asset fuzzing, resource budgets, contained workers, explicit trust boundaries and extension conformance/security review | A07, A08, A11 / M2–M4 |
| AR-11: restricted model disclosure | Existing manifests include model contents; future caches, crash reports, previews and exports create more copies | Data-flow inventory, controlled workspace/export behavior, no automatic external transfer, verified retention/cleanup and operator training | A07, A10 / M2–M5 |
| AR-12: unsupported deployed baseline | Existing policy has a single maintainer, no older-version backports and response targets without an SLA | Named primary/backup maintainers, funded support window, vulnerability response, dependency maintenance and recovery exercises | A12 / M5 |
| AR-13: ambiguous graphical/hybrid execution | Proposed visual blocks, feedback cycles, rate transitions and live tuning could execute with unintended causality, units or event order | Typed intermediate representation, explicit timing/state semantics, direct-feedthrough/algebraic-loop checks, independent hybrid benchmarks and versioned tuning replay | A03, A04, A13 / M1–M4 |

Do not silently widen a tolerance, remove a difficult case, or relabel a failure
as passed. Resolve a defect, narrow the supported contract so the tool refuses
the case, or retain an explicit limitation with a reviewed rationale. A known
case that produces a false accepted conclusion blocks release within that
supported use. Preserve the existing labelled reference-discrepancy locks until
their separately reviewed scientific resolution.

## Independent numerical and model campaigns

Approve the benchmark protocol and thresholds **before implementing a new
method or inspecting its acceptance-run output**. A04 should specify the source,
rights, transcription check, independent oracle, physical scaling, numerical
budget, comparison metric and failure interpretation for each case. Existing
methods need the same protocol before rerunning the acceptance campaign.

| Campaign | Proposed coverage | Acceptance argument to establish in advance |
|---|---|---|
| Primitive numerics and frames | Analytic derivatives/integration, rotations, general inertia, extreme scales, invalid input and invariant preservation | Floating-point/conditioning and truncation budgets; dimensional quantities checked in declared units; no unexplained invariant drift |
| Trim, Jacobians and modal analysis | Supported flight conditions and equilibria, steps spanning the truncation/cancellation region, coordinate singularities, modal coalescence and nonnormal realizations | Independently recomputed force/moment residuals; reliable eigenvalue/classification status; sensitivity to perturbation and representation |
| Frequency response and robustness | DC/feedthrough, out-of-band/narrow peaks, multiple/tangent crossings, unstable hidden modes, continuous/discrete domains as introduced | Independent rational or trusted-reference solutions; coverage of frequency limits; justified bound direction; unresolved cases never become guarantees |
| Control synthesis/interconnections | CARE and subsequently supported discrete/generalised cases, scaling, rank limits, feedthrough algebra, channel order, controller/observer assumptions | Independent benchmark families with residual and stability evidence; positive/negative cases; controller feasibility checked separately from solver success |
| Time-domain execution | Smooth and saturated motion, actuators, sensors and sampled controllers, multirate boundaries, zero crossings, simultaneous events, disturbances, failure events, step and event convergence | Matched physics on comparison paths; state/peak/event-time budgets; stability and convergence assessments; explicit event ordering and no hidden interpolation of failed intervals |
| Visual compiler and signal flow | Typed/unit/dimension/sample-time edges, subsystem boundaries, valid dynamic feedback, direct-feedthrough cycles, initialization/reset, rate transitions and unsupported imported constructs | Independently stated block semantics; deterministic compiled schedule; traceability to graph elements; rejected ambiguity; equivalent flattened/subsystem and graphical/headless execution |
| Whole-study and UI parity | Same saved study through desktop, CLI and C++; graph edits, tuning revisions, scopes/harnesses, cancellation, recovery, migration, batch execution and report export | Same canonical inputs/methods and compatible results within the declared contract; warnings, units, labels, limits and decision states retained |

Do not propose one universal percentage for every output. Allocate the error
budget from the engineering decision: source measurement/transcription error,
model-form discrepancy, parameter uncertainty, numerical solve/integration
error, interpolation and presentation error. Keep these components separate;
combine them only with a documented rationale, accounting for correlation.
Where no defensible budget or independent truth exists, the output remains
exploratory and cannot satisfy a requirement that depends on that evidence.

A05 should require a versioned dossier for each supported aircraft/model family:
source and permitted use; geometry/mass/inertia; axes, units and moment reference;
aerodynamic, propulsion, configuration, actuator and sensor assumptions;
validation conditions; parameter ranges; correlations; interpolation rules;
unvalidated extrapolation; missing physics; and known direction of model error.
When the error direction is unknown, record unknown and forbid a conservative
claim. Validation should include quantities used by the customer, not only
matrix entries or modal roots.

Broaden the validation domain deliberately across speed/Mach, altitude, angle
of attack, sideslip, rates, control deflection, mass/CG and configuration as the
model supports them. Multi-variable combinations matter; independent bounds on
each coordinate do not automatically validate their Cartesian product.
Use independent holdout flight, wind-tunnel or suitably established simulator
data where available; keep calibration and validation datasets separate. Without
appropriate data, narrow the declared product/model scope at M3 rather than
claiming a complete aircraft envelope.

A06 should define uncertainty studies before running them: distributions or
bounded sets, provenance, dependencies, seeds, sample/coverage rationale and
convergence of the decision quantity. Include adverse combinations and model
discrepancy. Monte Carlo success is not proof over an uncertainty set. Acceptance
must account for uncertainty relative to each requirement; an unresolved
overlap with a limit should yield review required or requirements not met,
according to the preapproved rule.

## Assurance for graphical and hybrid models

The visual editor should author a versioned intermediate representation (IR)
that the same headless compiler and numerical runtime execute. It must not
introduce a second set of equations in UI code. Each block type needs a reviewed
contract for its parameters, ports, units, dimensions, direct-feedthrough
dependencies, continuous/discrete states, sample times, initialization, reset
and failure behavior. Each instance, connection and subsystem needs a stable
identity preserved through compilation, diagnostics, traces and reports.

Before a run, the compiler should validate type/dimension/unit compatibility,
parameter bounds, sample-time consistency, state initialization and explicit
rate transitions. Silent physical-unit conversion or implicit reinterpretation
of a channel is unacceptable; approved conversions should be visible model
operations with traceable semantics. Validate feedback by its direct-feedthrough
dependencies: a cycle with a state/delay can be well-defined, while an algebraic
loop requires a separately supported solution method. Detect and refuse
algebraic loops until that method has a defined convergence/failure contract and
independent verification. Highlight the actual cycle to the operator.

A13 should specify the order of evaluation, sampling, zero-crossing detection,
event handling, output publication and state updates. Simultaneous events need
a deterministic priority rule; rate transitions need explicit hold/interpolation
and latency semantics; zero-crossing chattering needs a bounded diagnostic or a
defined resolution policy. Benchmark event times and discontinuous responses
against independent cases. Numerical step convergence, event tolerance and
scheduler timing error must have separate budgets. Never treat a finite-step
simulation as a hard-real-time or target-scheduler guarantee.

Subsystem encapsulation, graph flattening, block reduction and other compiler
optimizations need semantics-preserving tests, including state ordering and
source mapping. Preserve traceability from graph element through IR and
scheduled kernel execution to recorded signal/state values and requirement
results. Record the compiler version, IR digest, execution schedule/options and
any transformations in the run evidence. A graphical scope is a view of these
recorded signals, and must show units, sample time, decimation and missing/failed
intervals accurately.

Tuning should create a recorded parameter revision with an effective simulation
time and defined state-handling rule. Edits that change structure, dimensions,
rates or incompatible state should require recompilation/restart according to
the contract. Replay should reproduce the original initialization, parameter
changes, events and outputs; an accepted baseline stays immutable. Harnesses
should identify the subsystem under test, fixtures, disturbances, stubs,
assertions and expected behavior, with independent expected results where a
numerical claim is being validated.

Future importers must name their supported construct/version subset and reject
unsupported blocks, timing or solver semantics rather than silently approximating
them. A future state-machine track needs separate transition/priority/reset and
event-semantic evidence. A restricted C-generation track would need a bounded
source-language subset, traceability and software-in-the-loop equivalence to the
reference model/runtime, including floating-point and failure behavior. Target
compiler, scheduling, hardware, timing and integration acceptance would remain
separate obligations. Producing compilable code or passing SIL comparison would
not imply onboard qualification or approval to fly.

## Security and offline operation

Use NIST SSDF as a proposed secure-development practice mapping, with tasks,
owners and evidence chosen for this product. The official final SP 800-218
record identifies SSDF 1.1; its revision 1 / SSDF 1.2 record is a draft at this
review. Recheck status when baselining A02 and do not label draft practices as
mandatory final requirements. [Final SSDF record](https://csrc.nist.gov/pubs/sp/800/218/final),
[revision draft record](https://csrc.nist.gov/pubs/sp/800/218/r1/ipd).

The proposed threat model should enumerate study/model files, archives, imported
flight data, plot assets, project migrations, report rendering, extensions,
dependencies, installers and updates. Treat data correctness, confidentiality,
integrity and availability as separate concerns. Document trust boundaries
between the desktop, worker processes, filesystem, optional extensions and the
operator. UI or importer failures must not corrupt existing evidence or inherit
unnecessary write/network access.

For disconnected or restricted deployments, A07 should implement and verify:

- Complete local operation without login, online licensing, telemetry, remote
  fonts/resources, automatic update checks or remote help dependencies. Any
  optional network operation must be explicit and disableable by deployment
  policy; observe attempted traffic during verification.
- An inventory of every model-data copy: workspace, manifest, cache, temporary
  file, crash dump, undo history, autosave, thumbnail, report and support bundle.
  Handle exact-input manifests as model data, not harmless logs.
- Operator-visible export contents, destinations and data labels. Support a
  redacted derivative with its own identity when sharing restrictions require
  it; never imply that a redacted export is a complete replay package.
- Workspace permissions, bounded temporary storage, recoverable autosave,
  backup/restore and retention controls. Use the organisation's approved
  storage/encryption mechanisms; do not claim secure erasure of arbitrary media.
- Synthetic/public diagnostic bundles by default. Inclusion of model contents
  or process dumps requires an explicit data-owner decision under deployment
  policy. External support transfer is a separate operator action.

These features would support deployment in an organisation-managed environment;
they would not accredit the application or workstation for classified data.
Classification, access, removable-media transfer, cryptography and network
authorisation remain subject to the customer's relevant authority and approved
environment. A customer acceptance record should state exactly which controls
are provided by Galata and which are external prerequisites.

A08 should set documented maximum input bytes, nesting, matrix dimensions,
sample counts, archive expansion, chart vertices, temporary/output storage and
worker CPU/memory time for each supported task profile. Validate before costly
allocation; isolate long-running jobs; support cancellation; give actionable
limit errors. Agree limits from representative legitimate workloads and threat
analysis, then test both sides of each limit. Avoid promising unlimited
availability because the tool runs locally.

Fuzz all imported formats and API boundaries with malformed, truncated,
recursive, nonfinite and scale-extreme inputs. Include archive traversal,
symlink/collision behavior, Unicode filenames, HTML/SVG escaping, corrupted
project migrations and extension messages. Maintain public/synthetic seed
corpora, minimised failures, sanitizer results and a predeclared campaign budget.
Coverage-guided fuzzing and static/dependency analysis supplement functional
tests; absence of observed failures is bounded evidence, not proof of security.

Before any extension API is supported, A11 should define compatibility,
capabilities/permissions, input/output schemas, state ownership, determinism,
versioning and numerical evidence requirements. Prefer isolated workers for
untrusted or externally supplied code, with bounded resources and denied
network/filesystem capabilities unless explicitly granted. A signed extension
establishes publisher identity, not scientific validity. Hardware extensions
would need a separate hazard analysis, command/timeout/failsafe contract and
bench campaign before an optional HIL release.

## Controlled releases, authenticity and replay

A09 should produce one immutable release identity that connects the exact
source bytes, toolchain, dependencies, runtime and acceptance evidence. Resolve
the source commit once; reject mismatches across platforms and dirty release
sources. Developer builds may remain supported but should identify their exact
source content and unaccepted status. Pin action revisions and build inputs;
archive the effective configuration and enough permitted source/dependency
material to rebuild in the declared environment.

Require the complete applicable CI graph for that identity: functional/property/
integration/reference tests, assurance-script tests, generated-document checks,
sanitizers, static analysis, dependency/security review, supported-platform
determinism, installer/relocation tests and package smoke tests. Skipped, missing,
cancelled or wrong-revision evidence must fail the gate. Publish evidence and
documentation only from the accepted source/result set; separately inventory
environment controls and branch/tag protections instead of assuming them.

Each proposed distribution should carry a machine-readable SBOM, complete
runtime inventory, licences/notices, supported-platform contract, known issues,
verification summary and authentic release metadata. Choose and version a
standard SBOM format during A02. Include transitive runtime dependencies and
identify build tools separately. Record vulnerability assessment and disposition
against the actual shipped versions, with a maintained update process.

Support offline verification of signatures and file hashes from pre-provisioned
trust roots. Plan signing-key custody, revocation, rotation and offline freshness
policy; signatures must bind the inventories and evidence rather than a filename
alone. Test tampering, missing libraries, compatible-library substitution,
expired/revoked metadata, wrong architecture and partial downloads. Preserve
system-runtime identity and document which components cannot be bundled.

Updates should verify authenticity before installation, stage changes, preserve
the previous accepted installation and project backups, and recover from
interruption. A rollback should identify the prior signed version and any
known-risk exception, retain an audit record, and restore compatible project
data. Schema migration needs preview, backup and a tested recovery path; do not
overwrite the only evidence copy with a one-way conversion.

Perform an independent clean rebuild of the acceptance candidate using the
archived recipe. Compare bit identity where promised; otherwise explain the
differences and compare the relevant numerical results against declared bounds.
Document compiler, floating-point flags, operating-system/math-runtime changes
and test exclusions. A package checksum proves content identity; it does not
by itself prove source correspondence, authenticity or repeatability.

## Delivery work packages

Owner titles below are roles to assign to named people during planning. Work
packages may run in parallel after their stated dependencies; evidence gates
remain sequential. Dates, staffing and commercial support commitments should
be baselined by the product owner rather than invented in this plan.

| Package | Proposed accountable role | Dependencies | Concrete deliverable and completion test |
|---|---|---|---|
| A01 — Close the audit baseline | Engine lead, with independent verifier | Preserved audit source/fixtures | Reproduce AR-01–AR-08; correct/refuse each affected supported case; retain independent regression evidence; rerun complete affected gates; reconcile claims |
| A02 — Define use, risk and traceability | Product assurance lead and customer domain representative | Audit baseline | Approved intended-use and standards-applicability decision; requirement/risk IDs; verification independence; owners; acceptance criteria; supported environment and evidence retention policy |
| A03 — Make evidence a shared result contract | Engine/API lead with UI and domain leads | A01, A02 | Versioned four-axis artifacts and propagation rules; stale-result invalidation; desktop/CLI/C++/export contract tests; complete diagnostic retention |
| A04 — Establish independent numerical benchmarks | Independent numerical verification lead | A02; A03 for integration | Preapproved protocols, independent references and budgets for every claimed method; negative and conditioning cases; discrepancy dispositions; automated evidence reports |
| A05 — Qualify model data for declared use | Flight-dynamics/model lead and data owner | A02, A04 | Model dossiers and independent validation/holdout data; supported-domain and boundary enforcement; justified uncertainty and missing-physics statements |
| A06 — Establish campaign and uncertainty acceptance | Domain verification lead | A03–A05 | Reproducible condition/parameter campaigns, sensitivity and uncertainty studies, step/event convergence, physical acceptance margins and failed-case retention |
| A07 — Protect offline workspaces and evidence | Security lead with desktop/platform lead | A02, A03 | Reviewed data-flow/threat model; local-only deployment profile; controlled exports/support bundles; recovery and isolation; observed no unintended data movement |
| A08 — Exercise hostile input and resource limits | Security test lead and API owners | A02; each importer/API contract | Limit specifications, fuzz harnesses/corpora, sanitizer/static-analysis results, minimised failures and closure evidence for all supported entry points |
| A09 — Control the build, release and updates | Release/platform lead with independent verifier | A01, A02; A03 for runtime binding | Immutable source/evidence gate, SBOM/runtime inventory, signatures/offline verification, reproducible-build comparison, installed candidate and update/rollback exercises |
| A10 — Verify engineering interaction and review | UX lead with practicing engineers | A03, A07; representative A04/A05 cases | Usability trials for assumptions, units, uncertainty, failed/unresolved cases, plot/export interpretation, recovery and acceptance review; documented corrective changes |
| A11 — Bound external integrations | Integration lead with security/domain leads | A02, A03, A07, A08 | Import/export/API conformance suite and compatibility policy; isolate/disable unassured extensions; optional HIL decision and separate hazard/evidence plan |
| A12 — Establish acceptance and sustainment | Product owner and assurance lead | Relevant completed packages | Named primary/backup maintainers, supported-version/update policy, support/training material, response and recovery drills, archived acceptance dossier and independent release review |
| A13 — Verify visual compilation and hybrid execution | Simulation/compiler lead with independent verifier | A02–A04; A07/A08 for imported models | Reviewed block/IR contracts; type/unit/dimension/rate and algebraic-loop diagnostics; deterministic scheduler/event/tuning replay; graph-to-execution traceability; independent hybrid benchmarks and scope/harness tests |

## Evidence gates from M0 to M5

| Gate | Required exit evidence | Decision that remains unavailable |
|---|---|---|
| M0 — Audit closure | Maintained audit baseline and independent counterexamples; AR-01–AR-08 corrected or safely refused within the supported contract; complete applicable existing gates pass; residual limitations explicitly reviewed | Completion does not validate new aircraft models or add desktop readiness |
| M1 — Contracts and benchmark baseline | Intended use, risk owners, ASR decomposition, result/schema and failure contracts, independent benchmark protocols and justified budgets approved before feature acceptance runs | An implemented feature cannot be called validated merely because its own tests pass |
| M2 — Desktop vertical slice | One representative visual model-to-report workflow through desktop and CLI uses identical IR and evidence contracts; a continuous feedback model and scope execute against a developer regression harness; invalid wiring/algebraic loops are explained; cancellation/recovery/import/export and negative cases work; representative engineers can identify assumptions and unresolved results; offline behavior verified | Sampled/multirate execution is accepted at M3 and the end-user test manager at M4; this slice is not acceptance of the whole fixed-wing feature set |
| M3 — Supported domain and feature depth | The selected M3 numerical/model capabilities and blocks have implemented contracts, independent numerical evidence, model/condition scope and user documentation; hybrid/multirate and reusable subsystem semantics are covered; declared model domains have data dossiers and boundary tests; uncertainty and physical feasibility are visible | Campaign/review and full tuning/harness acceptance remains at M4; unvalidated conditions, absent hardware evidence or an unsupported extension cannot inherit acceptance from a nearby case |
| M4 — Independent campaigns | Frozen release candidate passes the preapproved numerical/model/compiler/hybrid/security/usability/installation campaigns on all supported environments; graph-to-result and tuning replay, changed-runtime and interrupted-update tests pass; all discrepancies have reviewed dispositions | Campaign success does not establish aircraft approval or a tool qualification beyond its agreed use |
| M5 — Product acceptance | Named reviewers accept the exact candidate, claims and residual risks; no known false accepted engineering conclusions remain in supported scope; signed offline-verifiable packages/evidence are archived; support, training, updates and recovery are operational | v1 product acceptance does not authorize flight, approve a customer aircraft or confer blanket DO-330 qualification |

If an exit criterion is not met, keep the gate open or explicitly reduce the
proposed supported scope and reassess dependent requirements. Do not substitute
a calendar date, a demo, a test count or a green aggregate badge for the evidence
required by that gate. Qualification work, if commissioned, should have its own
agreed plan and acceptance decision alongside these product gates.

## Sustainment and human review

A12 should establish a funded support model before enterprise acceptance:
supported operating-system/compiler/runtime combinations, release and security
support windows, older-version maintenance rules, issue severity/response
targets, escalation contacts and named backup maintainers. Targets should be
trialled and then published as commitments that can be met. Retain maintainable
build recipes, regression datasets, signing/recovery procedures and release
records beyond an individual maintainer's availability.

Publish practical operator and integrator guides covering units/frames, model
pedigree, validity domains, solver reliability, uncertainty, criteria selection,
review/export/replay, failure recovery and known limitations. Training should
include misleadingly small residuals, insufficient frequency sweeps, invalid
models, unresolved stability, actuator/sensor limits and incomplete runs. Check
competence through a reviewed example with deliberate faults, not attendance
alone. Examples must distinguish illustrative hardware/controller inputs from
validated customer data.

For every consequential engineering decision, retain the intended use,
requirement set, model/condition identity, error/uncertainty argument, independent
checks, deviations and reviewer rationale. Allow an organisation to record a
risk decision while preserving the original failed or unresolved evidence;
never let an override turn an invalid calculation into a validated one.

After release, assess every numerical, security, dependency, UI and model-data
change for affected requirements and accepted studies. Notify affected users
through their authorised support channel with the impacted versions, outputs
and conditions, a way to find affected studies, and a correction/recheck path.
Reopen acceptance when source, runtime, model evidence, intended use or the
applicable assurance basis changes materially.
