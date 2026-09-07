# M1 discovery contracts and acceptance boundary

**Status: working contract for the authorized M1 feasibility phase; review and
external decisions remain open.** This document records the current scope and
the evidence needed to move beyond a prototype. It does not declare M1 complete,
approve an aircraft model or promise a desktop delivery date. The
[discovery agenda](DISCOVERY.md), [delivery plan](DELIVERY.md),
[assurance plan](ASSURANCE.md) and
[executable-model architecture](../architecture/EXECUTABLE_MODEL.md) provide the
larger product context.

## Current decisions and assumptions

| Item | Current position | Authority and remaining uncertainty |
|---|---|---|
| Platform priority | macOS first, Linux next; Windows desktop/product delivery deferred | User direction. Existing Windows engine portability/CI remains in scope; minimum supported OS releases, hardware and installer policy remain undecided |
| Initial user task | Author an aircraft/controller block diagram, execute it with the shared headless engine and review the results | Working workflow assumption, open to user steering; no customer acceptance authority has approved it |
| Intended reliance | Offline exploratory analysis and engineering analysis that receives independent review before a consequential decision | M1 scope assumption. No onboard execution, aircraft approval, contractual compliance finding or tool qualification is claimed |
| Aircraft reference | Existing NT-33A flight-condition-1 material, with its recorded provenance and limitations | Existing repository reference route; neither new flight evidence nor broader data rights are established by this phase |
| Compiler/runtime experiment | A bounded, headless continuous fixed-step signal-flow profile around existing numerical kernels | M1 feasibility work. The full S03 palette, sampled/multirate behavior, graphical authoring and aircraft blockset are later acceptance increments |
| Numerical references | Independently specified analytic/synthetic cases for compiler and scheduler conformance | Synthetic cases establish mathematical/software behavior within their protocol; they do not establish aircraft fidelity |
| Product ownership | Engineering and review roles are identified in the plans | Named customer, receiving authority, support owner, independent reviewers and funded delivery capacity are not supplied |
| Network/data boundary | Native analysis should run offline; use public/synthetic fixtures for shared development and CI | No authorization to contact customers, upload private models, access classified data or connect a hardware bench is inferred |

The task has two separate acceptance horizons. M1 demonstrates that a small
executable model can be checked, compiled and simulated reproducibly. M2 and
later milestones turn that engine into the user-facing aircraft/controller
workflow. A scalar feedback prototype cannot close the desktop, aircraft-data,
hybrid-control or customer-acceptance portions of that workflow.

## Acceptance scenarios

Scenario IDs identify evidence to retain, not passing results already obtained.
The independent [model conformance protocol](../architecture/MODEL_CONFORMANCE.md)
owns numerical case definitions and budgets. This contract does not widen its
tolerances or replace its independent references with captured Galata output.

| ID | Reviewable scenario | Acceptance criterion | Current scope |
|---|---|---|---|
| M1-S01 | Compile and run a constant-rate state and analytic continuous feedback model | MC01–MC06 meet their applicable predeclared B01–B04 budgets; initialization, state ownership and logged quantities match the written equations | M1 headless feasibility |
| M1-S02 | Reorder block/connection declarations and repeat the same accepted model | MC10–MC12 establish stable ordering, fixed input-port arithmetic and sample/repeatability; graph presentation fields are unsupported and refused in this profile | M1 identity/scheduling proof |
| M1-S03 | Submit invalid dimensions, units, frames, connections, duplicate IDs and pure feedthrough feedback | MC20–MC26 refuse unsupported models before numerical execution, identifying the offending model/block/port or resource limit; no hidden delay or conversion is inserted | M1 negative acceptance |
| M1-S04 | Refine a supported smooth continuous run and compare against its independent analytic solution | The protocol's justified error/order checks pass; an absent or inapplicable estimate remains explicit rather than becoming a numerical guarantee | M1 numerical evidence |
| M1-S05 | Reconstruct an existing NT-33A trim/linearization/control study through a supported model adapter | Record the exact model and source evidence; compare against the existing headless study and applicable published-reference tests; classify shared-engine agreement as integration consistency | Subsequent adapter experiment; not implied by the scalar prototype |
| M1-S06 | Change a gain, initial state, unit/frame declaration, solver setting or consumed input byte | Model edits change semantic identity; run-option edits are separately recorded in the run request/evidence; old evidence is retained; project-layout behavior remains a future ADR-0011 contract | M1 identity contract; implemented coverage must be listed |
| M1-S07 | Exceed the declared model, allocation, step or sample budget, or produce a nonfinite evaluator result | Controlled refusal before unsafe allocation/indexing or a completed result; limit/overflow and failure behavior are tested at the public boundary | M1 bounded execution |
| M1-S08 | Review a complete, refused and numerically unresolved case | Execution, numerical reliability, model validity and requirement acceptance remain separate; source diagnostics and known limitations survive every published artifact | Existing evidence foundation plus explicit prototype integration evidence |
| M1-S09 | Install/run the eventual authoring slice offline on macOS, then Linux | An engineer opens, edits, compiles, runs and reviews the same model without a network service; CLI and desktop share execution/evidence; recovery and accessibility meet the later agreed task script | M2/M5 target; no desktop acceptance during M1 |

For every exercised scenario retain the exact source/build, inputs, protocol
revision, environment, observed values, tolerance rationale, diagnostics and
review disposition. Mark unimplemented scenarios pending, not skipped-success.
The reviewer must be able to identify whether an expected result is analytic,
published, independently implemented or merely a regression lock.

The current [prototype API](../../include/galata/modeling/model.hpp) declares
schema `galata.model.v1` and profile `continuous-scalar.v1`: real scalar
Constant, Gain, ordered signed Sum, Integrator and Output blocks. A dimension
has seven SI exponents plus an additional semantic angle exponent, each bounded
to [-16, 16]; frames are None, Body and NED. These are compile-time connection
contracts, not a frame-transformation library. No implicit scale/offset
conversion, Boolean/vector/bus block, model reference, presentation field,
sampled clock or algebraic solver is admitted by this profile.

The API separates immutable compiled-model semantics from simulation options.
State/output/schedule IDs are inspectable, including stateless models. Fixed
step/count/stride options and returned samples are run-specific; changing step
size does not change the underlying model's semantic digest. Test the complete
recorded model-plus-run request before calling two executions equivalent.

## Data pedigree and permitted evidence route

The initial reference is the existing
[NT-33A model](../../models/nt33a/nt33a-fc1.yaml), its
[pedigree record](../../models/nt33a/PROVENANCE.md), and the
[published-value transcription](../../tests/validation/reference/nt33a_fc1.csv).
They cite Heffley and Jewell, NASA CR-2144, December 1972, NTRS 19730003312.
The repository records a source-PDF digest, visual transcription method,
page/table locations and a second reading of selected values. Those records
are useful existing evidence; this contract does not claim a new independent
transcription or flight-data validation campaign has occurred.

The selected condition is a local first-order expansion at the documented
power-approach configuration. It does not provide a stall model, propulsion
availability, configuration transitions or a validated operating envelope.
The reference trim pitching moment/elevator convention is an explicit model
choice, so a near-zero trimmed elevator is not an independent prediction.
Stability-to-body-axis conversion is part of the model interpretation and must
remain visible. Numerical rounding/conversion error, parameter uncertainty and
omitted physics have different meanings and must not share an invented error
budget.

The existing rights record relies on the report's public/unclassified
distribution and lack of identified restrictions. It also characterizes the
contractor report as a government work. A distribution statement alone should
not be promoted into a new blanket copyright or redistribution conclusion:
NASA's current STI terms distinguish government works from contractor works and
direct users to record-specific copyright metadata. This phase has not
independently retrieved that record's current rights metadata or obtained a
rights-holder determination. Retain the existing provenance, apply
[ADR-0007](../adr/0007-reference-values-from-copyrighted-sources.md), and resolve
the record-specific basis before expanding the dataset or its redistribution
claims. This is a review gap, not a finding that the existing transcription is
unlawful. [NASA STI terms](https://sti.nasa.gov/disclaimers/).

The M1 data register must distinguish these routes:

| Data class | Admitted M1 use | Evidence still required |
|---|---|---|
| Existing NT-33A reference files | Reproduce the currently documented reference/consistency studies with their existing limitations | Confirm record-specific rights basis; retain field provenance and explicit modeling choices; no expanded fidelity claim |
| Independently authored synthetic equations and fixtures | Compiler, scheduler, parser, resource and numerical-conformance experiments | Equation/fixture authoring record, independent expectations, prespecified budgets and a synthetic/infrastructure label |
| New public aircraft or benchmark data | Candidate discovery input only until reviewed | Exact origin/revision, license/rights basis, transformations, reference conditions, uncertainty and permitted redistribution |
| Customer private aircraft data | Future authorized local ingestion path | Data owner, permitted purpose, access/storage/retention rules, review and sharing policy; manifests may contain original input bytes |
| Restricted/classified data | Outside this shared development/CI acceptance profile | A separately authorized environment and handling/accreditation decision; offline operation alone establishes neither |

For a new field record its name, quantity/unit/frame/reference point, source
location and revision, original value, transformation, SI value, uncertainty or
unknown uncertainty, configuration and domain, reviewer and rights route.
Missing engine/actuator/flight evidence narrows the use; it must not be filled by
an undocumented zero or by relabeling a synthetic value as measured.

## Public contracts required for the M1 profile

These requirements are acceptance obligations for the bounded profile. They do
not assert that all are already implemented by a newly compiled prototype.

| ID | Contract | Verification or refusal boundary |
|---|---|---|
| M1-C01 | Distinct study DAG, signal-flow source and compiled model types | Existing study YAML keeps its meaning; cyclic signal equations are compiled within a capability rather than executed as stage cycles |
| M1-C02 | Explicit experimental source/IR/evidence versions and supported profile | Unknown semantic versions, block kinds and executable fields fail closed; no public-format or plugin ABI freeze is implied |
| M1-C03 | Closed typed ports, physical dimensions, units and frame metadata | Validate before execution; canonical SI doubles reach the kernels; conversions are explicit boundary operations under ADR-0003 |
| M1-C04 | Explicit ordered connections, direct feedthrough and state ownership | Unsupported instantaneous cycles are refused; state-breaking feedback is supported only under the declared block contract; no hidden delay |
| M1-C05 | Pure fixed-step derivative evaluation | Temporary RK states do not commit state, emit accepted samples, draw random values or apply UI events; initial/final times and step counts are representable |
| M1-C06 | Content-bound source/compiled/run identity | ADR-0011 separates raw bytes, executable semantics, presentation and run evidence; source IDs, initial conditions, parameters and external data are bound |
| M1-C07 | Four independent result/evidence dimensions | A successful compile/run does not establish numerical reliability, aircraft validity or engineering acceptance; unknown and refused outcomes remain explicit |
| M1-C08 | Public-boundary finite/dimension checks | Direct C++ and file-driven callers cannot bypass supported model/resource contracts; diagnostic data itself must remain finite and structurally meaningful |
| M1-C09 | Declared resource profile and checked arithmetic | Record finite maxima for source bytes, nodes/edges/ports, states, steps, samples and stored scalar values as applicable; check products/count conversions before allocation and reject excess |
| M1-C10 | Deterministic local execution within the stated environment | Same accepted input/configuration/runtime follows ADR-0004; graph conformance explicitly establishes covered values; no automatic extension of existing cross-platform bounds |
| M1-C11 | Contained and explicit I/O | Source/model loading has no automatic remote fetch, shell command, script, plugin or FMU execution; input/output access uses the owned run boundary and records consumed bytes |
| M1-C12 | Evidence-preserving serialization and review | Retain compiler diagnostics, source maps, typed outputs and upstream numerical evidence; generated report text cannot silently replace or suppress required records |
| M1-C13 | Source compatibility and migration policy | Existing stable conventions remain fixed; experimental graph changes are versioned and documented; original documents/runs remain retained and unsupported versions are refused |
| M1-C14 | Rebuildable derived artifacts | Recompile after incompatible semantic/compiler/kernel/data changes; do not load unverified cached IR as executable source; a cache does not own the only copy |
| M1-C15 | Controlled build/release and dependency boundary | Preserve M0 exact-source CI/provenance gates, CMake 3.25+/Python 3.9+/Ninja requirements, repository license/notices and the static downloadable-archive boundary |
| M1-C16 | Honest product status | Registry/docs distinguish feasibility code from the v1 blockset, GUI, aircraft validation, hybrid scheduling, qualification and accepted installation |

Compiler metadata is checked before numerical execution; it does not add a unit
tag branch to every Eigen arithmetic operation. Angle/frame semantics still
need explicit declarations even where SI dimensional exponents alone would be
identical. The profile must refuse shapes or conversions it cannot establish.

The declared prototype limits below are engineering containment limits, not
customer performance SLAs or measured throughput. Their checks and boundary
tests remain acceptance obligations; merely declaring a constant is insufficient.

| Resource | Declared maximum |
|---|---:|
| Model source bytes | 1 MiB |
| Blocks | 1,024 |
| Connections | 8,192 |
| Ordered inputs to one Sum | 64 |
| Integration steps | 1,000,000 |
| Retained scalar values, including time/state/output samples | 1,000,000 |
| Scheduled block evaluations | 100,000,000 |

A node-count limit alone does not bound memory: state, logging width, sample
count and their checked products also matter. The prototype's cancellation is
cooperative at defined work boundaries; a caller callback is not guaranteed to
return promptly. An in-process C++ API is not an operating-system sandbox;
hostile native code and hard wall-time containment require a later worker
isolation decision.

No additional runtime dependency is selected by this document. Retain the
existing open-source engine/dependency notices; use Python through the current
CLI/build/test route unless a native binding is separately reviewed. A model
format, extension interface or optional adapter does not override third-party
license obligations or justify a mandatory proprietary simulator dependency.

## D01–D16 dispositions for this phase

“Proceed” authorizes the bounded investigation described here, not closure of
the discovery package or acceptance by an external party.

| Package | Disposition | Current decision | Unresolved evidence and consequence |
|---|---|---|---|
| D01 — User decision | Narrow | Use the aircraft/controller author-run-review workflow as the initial hypothesis and continuous graph as its technical spike | Named customer/reviewer, actual decision and accepted task script absent; cannot close product acceptance |
| D02 — Intended reliance | Narrow | Offline exploration and independently reviewed engineering; no qualification claim | Customer assurance applicability, independence and retention obligations absent; qualified/contractual use deferred |
| D03 — Data pedigree | Narrow | Existing NT-33A references plus synthetic infrastructure fixtures | Record-specific rights review and independent aircraft evidence remain open; no wider dataset or envelope claim |
| D04 — Numerical evidence | Proceed | Execute the independent continuous-model conformance protocol; retain existing numerical kernels | Broader CARE/norm/conditioning campaigns remain separate; no new algorithm accepted solely by self-consistency |
| D05 — Scheduled aerodynamics | Defer | Preserve existing local model semantics and compile-time units/frame checks | No selected multidimensional aircraft tables/interpolant/uncertainty contract; scheduled aircraft fidelity deferred |
| D06 — Feasible trim | Narrow | Retain current equilibrium/chart checks and their evidence | Hardware thrust/surface/configuration limits absent; numerical equilibrium cannot establish physical feasibility |
| D07 — Graph/clocks/events | Proceed narrowly | Explicit continuous fixed-step profile, state ownership and feedthrough refusal | Hybrid/multirate/event/reset/PRNG semantics need their own accepted fixtures before admission |
| D08 — Controller lifecycle | Narrow | Ideal continuous feedback for the prototype | Sampled controllers, estimator/noise models, gain schedules and state machines deferred; no measurement feasibility claim |
| D09 — Aircraft validation | Defer | Retain published-reference and consistency evidence with its actual scope | Independent flight/holdout data and decision-level uncertainty absent; model-validation gate stays open |
| D10 — Interoperability | Narrow | Native C++/CLI and existing file artifacts; define experimental graph boundaries | No universal SLX, FMI, JSBSim, MAT or code-generation compatibility; select adapters only from a demonstrated user task |
| D11 — Desktop stack | Defer selection | macOS-first authoring workflow defines the future comparison task | Qt/web stack, accessibility, deployment and licensing measurements absent; no toolkit chosen by this prototype |
| D12 — Storage/recovery | Proceed on contracts | Separate editable source, disposable IR/indexes and immutable run evidence under ADR-0011 | Database/package format, migration window and crash-durability evidence remain open; shared editing deferred |
| D13 — Offline trust | Narrow | Public/synthetic inputs, closed supported blocks and existing run I/O controls | Signed/offline updates, hostile-content worker containment and restricted-site policies remain future gates |
| D14 — Platforms/performance | Proceed narrowly | macOS first, Linux second; preserve Windows engine CI | OS/hardware minima, workload scale and measured response/cancellation budgets absent; no product performance promise |
| D15 — SITL/HIL | Defer | Keep virtual-time offline execution independent of hardware | No named bench, interface owner or deadline/failure budget; no real-time or onboard inference |
| D16 — Delivery/sustainment | Narrow | Authorize M1 technical evidence and decision work | Named staffing, funding, independent review, support horizon and procurement facts absent; no invented schedule or funded v1 claim |

## Exit review and next decisions

M1 may close its **technical feasibility subgate** when the supported graph
profile is explicit, the independent conformance cases and negative/resource
tests pass on the claimed environments, source/run identity is demonstrated,
and the implementation/status limitations are reviewable. Preserve M0 release
and numerical evidence while adding the new component; a green prototype test
must not waive a failed existing gate.

The **complete M1 product gate** also needs a named target user/reviewer and
accepted workflow, a defensible data-use route, error/validity budgets for the
intended decisions, selected platform/workload requirements, relevant reviewed
ADRs, and funded ownership/independent review estimates. Those external facts
remain open. Technical progress can proceed without pretending they have been
supplied.

The next review should confirm or change the workflow assumption, record
macOS/Linux OS and hardware targets, assign the receiving reviewer, resolve the
NT-33A rights-record basis, and decide the smallest aircraft adapter and desktop
spikes. Any scope change updates this contract, the conformance profile and the
associated ADRs together. No customer contact, data acquisition agreement or
licensing approval is created by writing this plan.
