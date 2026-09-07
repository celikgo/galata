# Executable model architecture

**Status: broader proposed architecture, with a bounded M1 implementation.**
[ADR-0010](../adr/0010-continuous-scalar-executable-model.md) defines the implemented
continuous scalar compiler/runtime profile. The remaining
[Simulink-style feature plan](../product/SIMULINK_FEATURES.md), desktop toolkit,
project storage, aircraft blocks and sampled/hybrid execution remain proposed.
The experimental source format is not a public compatibility freeze. See the
[M1 implementation record](../product/M1_IMPLEMENTATION.md) and
[delivery plan](../product/DELIVERY.md) for scope and outstanding gates.

The intended product is an offline fixed-wing engineering workbench, with the
same model compiler and numerical runtime available to the desktop, CLI and
test harnesses. Aircraft approval, tool qualification and future generated-code
target acceptance remain separate decisions under the
[assurance plan](../product/ASSURANCE.md).

## Existing foundation and proposed dependencies

| Layer | Existing foundation | Proposed responsibility and dependency rule |
|---|---|---|
| Numerical kernels | [Fixed-step integration](../../include/galata/numerics/integrator.hpp), model, trim, linearization, simulation, analysis and synthesis libraries | Remain independent of the editor, project storage and graph presentation; add kernel functionality only with its numerical contract and independent evidence |
| Study execution | [Pipeline](../../include/galata/pipeline/pipeline.hpp) and [capability registry](../../include/galata/pipeline/registry.hpp), including `model.compile` and `sim.model` for the bounded profile | Extend typed artifact adapters; keep study execution separate from signal scheduling |
| Executable model | Bounded continuous scalar compiler and signal scheduler under ADR-0010; no graphical editor | Own model resolution, semantic checks, typed IR, state layout and deterministic scheduling; call the existing kernels through explicit adapters |
| Worker and harness | CLI invokes the current engine; a desktop worker service is not present | Own bounded job execution, cancellation, progress, result storage and replay; run the same compiler/runtime as headless tests |
| Project services | YAML studies and file-based inputs/outputs exist | Own model/data dictionaries, immutable revisions, migrations, library resolution, semantic diffs and evidence links |
| Desktop | No desktop modeling application is present | Edit source models, submit jobs, inspect diagnostics and plot retained signals; consume public engine artifacts without implementing separate solver mathematics |
| Extensions | Built-in C++ capability registration exists; [ADR-0001](../adr/0001-independent-c-abi.md) proposes an independent C ABI | Design a block SDK and conformance profile separately; the current registry is not a stable plugin ABI or a security sandbox |

Dependency direction is desktop/project services to compiler/worker, then to
runtime and existing kernels. The headless compiler/runtime must not depend on
a display server, desktop event loop or an installed MATLAB environment. The
study pipeline may invoke a compiled-model run and consume its results. The
model runtime must not recursively invoke arbitrary study stages while
evaluating a derivative.

Presentation data and engineering source are separate. An editor is one client
of the model document; closing it must not change an already submitted job.
Desktop framework, worker transport and project database choices remain open
until representative offline packaging, accessibility, recovery and workload
measurements are available.

## Two different graphs

A study DAG expresses dependencies between operations: load an aircraft, trim,
linearize, simulate, analyze and write a report. Its edges carry completed
artifacts. It is acyclic so an operation runs only after its inputs exist.

A signal-flow model expresses simultaneous equations and state evolution. Its
edges carry typed signals, and physical/control feedback is normal. For
example, a sum, gain and integrator can form a valid continuous closed loop.
Deleting a feedback edge or inserting a delay to fit that model into a DAG
changes its equations and is forbidden.

The compiler derives an instantaneous dependency graph using each block's
direct-feedthrough declaration. An integrator's output depends on its state;
its derivative depends on its input. A unit delay's output depends on stored
state; its next state depends on the sampled input. Those distinctions break
instantaneous cycles without deleting dynamical feedback. A state-space block
with a nonzero direct term has an instantaneous input/output dependency even
though it also contains state.

Strongly connected instantaneous components are algebraic loops. M2 rejects
unsupported loops before starting a worker and highlights the contributing
ports, blocks and dependency path. It does not iterate opportunistically or
accept a loop because one evaluation happened to converge. A later algebraic
solver needs a separately accepted equation class, initialization policy,
residual/error checks and bounded failure behavior (S08/S19).

## Versioned source, IR and identity

The proposed broader source model contains stable block/port/connection IDs, subsystem instances,
explicit interface declarations, dictionary references, parameter expressions,
initial conditions and selected variants. Solver configuration belongs to the
separate run request under ADR-0011. Library
references resolve to immutable content identities; an unpinned moving library
version cannot define a reproducible run. Each instance owns distinct state
even when several instances reference the same library definition.

The proposed IR has its own schema and semantics versions. Its contract includes:

| IR field group | Required meaning |
|---|---|
| Source map | Stable source model, instance, block, port and connection IDs for every lowered operation, state slot and diagnostic |
| Signal types | Declared real/Boolean scalar, vector or matrix type; shape; physical dimensions and unit scale; coordinate frame; time domain/clock |
| Parameters | Resolved values, units, range/domain constraints, dictionary identity, override origin and whether a value may be tuned during execution |
| Block definition | Immutable library/version identity; parameter and port contracts; pure output/derivative/update functions; direct-feedthrough relation; validity/evidence references |
| State | Continuous and discrete slots, dimensions, initialization/reset rules, ownership, output dependence and checkpoint representation |
| Schedule | Ordered operations, clock activations, data transfer rules, initialization/event phases and solver-stage evaluation rules |
| Build identity | Compiler/IR/runtime version, kernel and block-library identities, effective numerical options and source semantic digest |
| Evidence | Supported profile, diagnostics, rejected/unsupported constructs, analysis origins and applicable model validity domains |

The proposed broader workbench profile uses finite floating-point real values and Boolean logic with
declared shapes. Named buses have explicit typed fields. Integers require
explicit conversion/overflow semantics before admission; variable-size arrays,
complex signals, arbitrary expressions with side effects, recursion and native
callbacks are outside that proposed profile. Units and coordinate frames are checked
at compilation and converted only by explicit supported operations. Equal
physical dimensions alone do not make two coordinate frames compatible.

Compilation proceeds through parse/version checks, reference and instance
resolution, type/unit/frame/clock propagation, initialization validation,
direct-feedthrough analysis, state allocation, schedule construction and kernel
lowering. Ambiguous inference is a diagnostic requiring an explicit declaration.
Flattening or optimization must preserve the source map. Each optimization
needs semantic equivalence tests; floating-point reassociation is not silently
enabled as a performance optimization.

Model-schema migration is a deterministic, separately versioned transformation
that retains the original document and a reviewable change report. Unsupported
versions or constructs fail closed. Compiled IR is disposable and rebuilt when
its semantics/compiler/library identity changes; it is never the only saved
representation of a user's model.

For that broader profile, canonical model serialization covers resolved
engineering values, equations, library references, variants and initial
conditions. Run-specific numerical settings remain in the run request identity
under ADR-0011; the current scalar profile implements the narrower ADR-0010 contract. It
specifies ordering, text normalization and round-trippable floating-point
encoding before hashing. Nonfinite numeric inputs are rejected. Layout,
selection, plot colors and viewport coordinates have a separate presentation
identity and cannot invalidate numerical evidence. Notes that change a validity
claim or requirement decision belong to versioned evidence, even if displayed
on the canvas.

A run key combines semantic model/IR identity, runtime/build identity, input-data
hashes, initial state, seeds, numerical configuration and any tuning-event
history. Cache reuse additionally requires compatible evidence and complete
outputs. Equal model hashes alone are insufficient when the runtime, data or
solver settings differ. Corrupt, partial or unverifiable caches are discarded.

## First execution profile: fixed-step continuous systems

M2 starts with explicit continuous dynamics and time-dependent inputs:

\[
\dot{x}_c = f(t, x_c, u; p), \qquad y = g(t, x_c, u; p).
\]

After initialization, a pure scheduled graph evaluator supplies the derivative
and outputs to the existing fixed-step integration kernels. The same accepted
model has one simulation-time sequence and one numerical implementation across
desktop and CLI. RK substages evaluate temporary continuous states; they must
not mutate block state, draw new random values, emit accepted samples or apply
tuning. State projection, if required by a supported state representation,
occurs at the documented completed-step boundary.

Time uses an explicit origin, step and integer step count, following
[ADR-0004](../adr/0004-determinism-policy.md). Logging intervals are explicit and
do not control the integration step. Initial-state and every-stage dimensions,
finiteness and resource bounds are checked. Fixed step supplies no automatic
accuracy guarantee: accepted uses require step-size studies and independent
reference comparisons with predeclared budgets. Discontinuous signals need
declared breakpoints aligned to supported boundaries; unlocated zero crossings
must not be presented as accurately timed events.

Cancellation and pause are observed at safe numerical boundaries. Wall-clock
pauses have no effect on simulation time. An interrupted job retains a partial
status and cannot be mistaken for a completed trajectory or accepted result.

## Proposed M3 hybrid and multirate semantics

Hybrid simulation adds discrete state and declared clock activations:

\[
x_d[k+1] = F(x_d[k], x_c(t_k), u(t_k); p),
\qquad y_k = G(x_d[k], x_c(t_k), u(t_k); p).
\]

These equations express a default pre-update output contract. A block needing
different output/update behavior must declare it explicitly and have matching
scheduler tests; source insertion order cannot determine the answer. A clock
has an integer period and phase on an explicit base tick. Reject unsupported
incommensurate or nonrepresentable timing rather than rounding it silently.
Continuous substeps align to scheduled sample and discontinuity boundaries.

The M1 timing ADR must resolve and test the exact phase table before M3
implementation. The candidate profile is:

1. Reach the event time with continuous state integrated under the preceding
   held values; form the pre-event continuous/discrete snapshot.
2. Apply recorded external input/tunable-parameter events in their declared
   phase and stable order. Conflicting simultaneous writes are rejected unless
   the model explicitly defines their resolution.
3. Evaluate active-clock inputs and combinational outputs from that snapshot,
   using the compiled dependency order and explicit rate-transition rules.
4. Compute next states without mutating any state another block still reads;
   commit the scheduled discrete updates simultaneously.
5. Publish the specified post-event/held outputs for the next continuous
   interval and log the declared pre/post-event samples without ambiguity.

Each clocked block declares output availability and whether an output holds its
last value between activations. Cross-rate edges require a transition contract
with initial values, producer/consumer sampling phases and supported hold or
interpolation behavior. The first supported transition profile should favor
explicit zero-order holds and documented latency; interpolation needs evidence
before admission. A sampled controller updates once at its clock event, never
at an RK substage. Random sources have independent recorded streams and draw
on declared clocks, not incidental function-call counts.

Initialization, simultaneous events, resets, zero-duration runs and final-time
events receive conformance fixtures. Event-count, graph-size, memory and run-time
limits bound resource use. State-machine zero-time transition loops, implicit
equation solving, adaptive/stiff integration and arbitrary co-simulation master
algorithms are later profiles, not implied by adding a switch or multirate block.

## Artifacts, replay and engineering authority

Compilation emits a manifest, immutable IR, source map, schedule description and
diagnostics. Execution emits raw signal/state records, units and clock metadata,
event/tuning history, completion state, numerical diagnostics, model-domain
checks and runtime provenance. Scope decimation is a display artifact and never
replaces raw samples or the data used for requirement evaluation.

Every result retains separate execution success, numerical reliability, model
validity and engineering acceptance. Compiler success establishes only that a
model fits the accepted execution profile. A bounded solver error does not
validate an aircraft model. Derived linearizations and control analyses retain
the source operating point, signal sign/frame/rate, retained states and chart or
conditioning evidence, following
[ADR-0008](../adr/0008-numerical-evidence-authority.md). A derived plot or
synthetic report cannot discard a required diagnostic.

Traceability follows requirement/scenario to source block/port, resolved IR and
kernel operation, then run samples/diagnostics and reviewer decision. Pipeline
artifacts wrap these typed records for downstream engineering studies. Any
adapter from the present C++ artifacts must preserve unresolved and invalid
evidence explicitly; a generic display summary is not the evidence contract.

A checkpoint stores continuous/discrete state, held signals, block internal
state, all clock counters, event cursor, PRNG states, controller/actuator state,
tuning revision and accepted solver history. It also binds model/IR, data,
library/runtime identities and floating-point execution policy. Restore refuses
incompatible identities. At M4, uninterrupted versus pause/checkpoint/resume
runs must satisfy the declared determinism policy and exact event-history
comparison; cross-platform bitwise identity is not assumed.

Live tuning is limited to parameters with reviewed boundary-update semantics.
Each accepted change records simulation time, phase, parameter ID, old/new value
and revision. Structural edits require a new compilation and run. A replay must
consume the same recorded events without UI timing or external live state.
Checkpoint loading and model import receive bounded-parser and malformed-data
tests, with no embedded executable code trusted by default.

## Block SDK and interoperability boundary

Start with built-in blocks exposing the same declarative port, state, clock and
evidence contract. Publish a conformance suite before freezing an extension
surface: initialization/reset, shape/domain checks, feedthrough correctness,
state ownership, deterministic output/update, checkpoint round-trip and bounded
failure behavior. Keep pure numerical evaluation distinct from host file/UI
services so a derivative call cannot conceal I/O or external state changes.

A future binary SDK follows ADR-0001's independent ABI decision, including
version/ownership rules and compatibility tests. The descriptor format and
execution ABI are different contracts; a stable descriptor does not establish
binary compatibility. Native extensions execute code and need a separately
reviewed trust/isolation profile. The first product does not acquire a sandbox
merely by placing a library behind a registry. Extension identity and evidence
must appear in model compilation and run provenance.

Keep the native model/compiler/runtime usable under the project's declared
license without a proprietary simulator dependency. Review each block's code,
data, reference and redistribution rights separately and retain required
notices. An extension boundary is not evidence that any arbitrary dependency
is license-compatible. Implement independent block behavior; do not import
third-party source, assets or proprietary reference datasets without permission.

Data exchange, model translation and co-simulation have different adapters.
The first exchange contract covers units, shapes, rates, state ordering and
provenance for supported data. Later native-model translation inventories every
supported, transformed and rejected construct; callbacks, custom blocks,
unresolved library links and unsupported variants cause refusal. Licensed source
tools may support comparison fixtures, but importing a diagram does not prove
behavioral equivalence or make those tools a native runtime prerequisite.

Restricted controller code generation is a later compiler track. It needs a
supported static-state subset, block-to-IR-to-generated-code traceability,
initialization/step/reset semantics and generated-source/toolchain identity.
Independent software-in-the-loop equivalence precedes processor timing,
resources, interfaces and target acceptance. Neither desktop equivalence nor
code generation implies onboard qualification or aircraft approval.

## Evidence gates and migration

| Gate | Architecture increment | Acceptance evidence required before promotion |
|---|---|---|
| M0 | Repair the current numerical authority and release/provenance foundation | Close the [audit counterexamples](../product/AUDIT_BASELINE.md) at affected public interfaces; demonstrate the controlled-release gate; no graphical capability claim |
| M1 | W08/W09 contracts; S02/S06 profile and timing ADR; A13 protocol | Independent analytic continuous/discrete/hybrid fixtures, explicit loop/event/initialization semantics, resource limits and justified error budgets agreed before implementation |
| M2 | Headless compiler plus editor vertical slice; S01/S03/S07/S08 | Analytic feedback and equivalent existing studies agree within predeclared budgets; invalid wiring/loops fail before execution; CLI and desktop consume identical artifacts |
| M3 | Subsystems, aircraft blocks, dictionaries, operating points and the accepted hybrid profile | Independent multirate/hold/delay/saturation/reset/linearization fixtures; state isolation; licensed aircraft-domain validation; unsupported rates/imports rejected |
| M4 | Harnesses, semantic diff, scopes, tuning, checkpoint/replay and campaigns | Every case accounted for; raw-data integrity; pause/resume and worker-count invariance where promised; semantic edits invalidate affected evidence; requirement decisions retain unresolved cases |
| M5 | Installed and supported workbench | Independent engineer completes the accepted model-edit/run/review task offline; migration/recovery/resource failures are exercised on supported platforms; exact accepted artifacts and limits recorded |
| Later | State machines, model translation, code generation, advanced solvers and HIL | Separate semantics, equivalence, security and target acceptance plans; no inherited acceptance from the graph canvas |

Reference campaigns must include more than reproductions of implementation:
analytic first/second-order systems, state-space equivalence with direct terms,
sampled recurrences, simultaneous clocks, explicit transport/hold behavior,
discontinuities, malformed graphs, extreme dimensions and resource exhaustion.
Choose independently derived expected results and justify tolerances before
measuring Galata. Test compiler/source-map preservation and typed artifact
adapters separately from numerical correctness. UI tests establish authoring,
diagnostic navigation and review behavior rather than duplicate solver formulas.

Existing study YAML remains a supported engineering-operation document. Add
compiled-model capability inputs under explicit versioning; do not reinterpret
old stage references as signal edges. Provide an opt-in converter only for
representable studies, retaining original input and listing assumptions. Old
run artifacts remain readable under their original evidence limitations and
are not relabeled as meeting a newer schema. Public source, IR, result and
extension compatibility policies are reviewed before their respective freezes.

This architecture is ready to guide M1 spikes and contract review. Acceptance
of this document alone does not complete those spikes, prove the proposed event
semantics or deliver any of S01–S20.
