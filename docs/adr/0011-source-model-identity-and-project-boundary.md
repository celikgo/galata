# ADR-0011: Separate executable source identity from project presentation

- **Status:** proposed M1 contract; no public project-format freeze or storage implementation claimed
- **Date:** 2026-09-07
- **Deciders:** project maintainer review pending

## Context

The proposed aircraft/controller authoring workflow needs editable models,
repeatable headless execution and reviewable results. The current
[study pipeline](../../include/galata/pipeline/pipeline.hpp) is a DAG of
engineering operations. A compiled cyclic signal-flow model is a different
document and runtime object. Treating canvas layout or a generated execution
schedule as the sole saved model would make engineering identity depend on UI
details or a particular compiler revision.

M0 [source/build provenance](0009-release-evidence-and-source-identity.md)
identifies the engine and the bytes used to build it. It does not by itself
define model semantics, reference resolution, library revisions, cache validity
or a future project's recovery policy. The
[M1 contracts](../product/M1_CONTRACTS.md) need those boundaries before public
file formats and editor behavior become expensive to change.

## Decision

Keep engineering source, presentation, compiled artifacts and run evidence as
separate objects with explicit identities. The first implementation may expose
only an experimental model API and source adapter. That does not imply that a
desktop project store, database, package format or stable extension ABI exists.

The current [M1 API](../../include/galata/modeling/model.hpp) defines the
experimental `continuous-scalar.v1` profile. Its model source has no presentation
fields or external model/library references. Those fields are rejected rather
than ignored. The project/presentation and reference-handling rules below define
a future boundary; they do not claim such support has landed in this profile.

### Authoritative and derived objects

| Object | Authority | Identity and ownership |
|---|---|---|
| Original input document | What the engineer actually supplied | Retain exact bytes and digest, source name and parser/schema version; preserve the original across migration |
| Resolved executable model | Equations, state, parameters, types and block execution semantics | Versioned canonical semantic representation/digest; immutable for a submitted run |
| Presentation document | Canvas positions, routes, viewport, selection and display choices | Separate presentation identity; cannot silently add or alter executable meaning |
| Compiled IR and source map | A particular compiler's accepted lowering of the model | Bind semantic source, compiler/IR version, kernel/library identities and options; derived and rebuildable |
| Run request | One execution of resolved source/IR with data and initial conditions | Immutable request identity; includes solver/count/logging settings and any declared input/event history |
| Run result/evidence | Actual completion state, outputs, diagnostics and review references | Content-bound immutable artifacts linked to request, runtime/build and consumed data; partial/refused states remain distinct |
| Project index/cache | Fast discovery, display and reuse | Reconstructible from authoritative objects; never the only copy of source or accepted evidence |

The model compiler owns executable semantics. The project/editor layer owns
document editing, storage and presentation. The worker executes an immutable
request. The existing pipeline may call a compiled-model capability and consume
its artifacts; the graph evaluator cannot invoke arbitrary pipeline stages or
project services during a derivative evaluation.

### Canonical semantic identity

Define and version a canonical representation before relying on its digest for
replay or cache reuse. It includes the accepted profile and semantic version;
stable model/block/port/connection IDs; block kind and immutable library
identity; declared/resolved types, shapes, units, coordinate frames and clocks;
ordered port inputs; parameters and override origins; state ownership and
initialization; supported variants and block timing semantics; and, when
references are admitted, content identities of resolved model/data dependencies.
Simulation step/count/stride and other run options are a separate run-request
contract. Changing those options must change recorded run identity while leaving
the underlying equations' semantic digest unchanged.

Canonicalization removes differences that the supported semantics explicitly
declare irrelevant, such as map-key whitespace/order and the separate
presentation object. It does not attempt graph isomorphism, algebraic
simplification, common-subexpression equivalence or floating-point
reassociation. Renaming a stable source ID may change identity even if the
equations look equivalent, because traceability changed.

Preserve every sequence whose order affects evaluation, state allocation or
output interpretation. Sum port order is semantic; sorting it may change
floating-point results. The current profile excludes block/connection
declaration order and uses stable IDs while retaining Sum input-port order.
Later product/operator blocks need the same explicit ordering decision.
Canonicalization follows the accepted profile and its conformance tests; it
must not assume that all YAML arrays are unordered collections.

The encoding specifies text handling, map ordering, integer bounds and
round-trippable finite floating-point representation. The M1 API specifies
exact binary64 parameter-bit encoding, including signed zero; ordinary display
rounding is insufficient for numerical identity. Unspecified locale or host
endianness cannot affect the encoded bytes. The digest names the algorithm and
canonicalization version.
Do not claim compatibility with a named canonical-JSON standard unless its
complete rules and conformance tests are implemented.

Retaining both exact bytes and semantic identity is deliberate. A YAML comment
change can produce a new document digest while leaving the numerical model
unchanged. A future separate layout document has the same independence, but
layout is currently refused inside model source. The run retains which original
document was used and why its resolved semantic representation is equivalent. An integrity digest proves
content identity, not authorship, model validity or acceptance.

### Presentation is a closed boundary

Presentation fields have a closed, versioned vocabulary and are excluded from
numerical evaluation by construction. Unknown executable keys, unsupported
block parameters or unresolved references cause rejection; they cannot be
placed into a generic metadata bag and silently ignored.

Free text used only as a canvas annotation is presentation. A validity
restriction, source citation, requirement allocation or reviewer decision is
engineering/evidence content and receives its own retained revision even when
displayed as a note. Plot colors and decimation settings do not replace the raw
records or alter the samples used for numerical acceptance.

### External references and immutable run snapshots

Resolve explicit project-relative names against the owning document, not the
process working directory. Before execution, resolve supported references to
the exact consumed bytes and preserve their content identity and origin.
Absolute machine-local locations may be retained as provenance, but they are
not substitutes for dependency identity. A moved project with unchanged
resolved content should remain interpretable under the same supported profile.

The first profile may forbid external model/library references entirely. When
admitted, missing, ambiguous, cyclic or incompatible references fail closed;
no automatic network fetch or moving latest-version resolution occurs during a
run. Customer data outside a project needs an explicit authorized import/read
route and a recorded snapshot policy. File loading uses the existing contained
run I/O boundary and its input-byte records rather than hidden block-level I/O.

A submitted run binds model/IR, effective parameters, data, initial state,
solver settings, logging/sample configuration, seeds/events where supported,
compiler/runtime identity and accepted numerical profile. Changing any of these
creates a new request or an explicitly recorded replay event. Ordinary editing
cannot mutate a job already executing. Live tuning, checkpoint restore and
cross-version continuation are unsupported until their state/event semantics
and compatibility checks are implemented and independently tested.

### Compatibility, migration and storage

Keep product version from the existing VERSION mechanism. Source-schema, IR,
canonicalization and result-schema versions identify separate compatibility
contracts; they are not independent product-version truth sources. The M1 graph
surface remains explicitly experimental and may require documented migration.
Do not label an experimental schema v1-stable merely because its first parser
uses a numeric version field.

Existing study YAML retains its engineering-stage meaning. Do not reinterpret
old stage references as graph signal edges or overload a prior schema silently.
An adapter or migration is explicit, deterministic and reviewable, retains the
old input and lists any unsupported transformations. Older readers refuse new
semantics they cannot understand. Legacy run evidence remains readable under
its original limitations; migration does not retroactively validate it.

Treat compiled IR, indexes and visualization caches as derived data. Reuse
requires matching source, compiler/semantic profile, data/runtime requirements
and complete verified output. On mismatch or corruption, rebuild or refuse;
never substitute a nearby prior result. A recovered partial result cannot
acquire completed status from its cache entry.

Prefer a portable document/artifact boundary for the initial headless work.
Whether a later project uses plain directories, a package container or a local
indexed database remains a measured storage decision. No shared-drive editing,
transaction durability, encryption, signed model authorship or database
compatibility guarantee is established by this ADR. Model/input bytes can be
sensitive even when the application runs offline.

### Required conformance evidence

Identity tests must show that semantic edits change the declared digest,
raw-byte edits remain observable, ordered arithmetic is retained, and run-option
changes remain distinguishable. Presentation fields are refused in M1 source;
when a project layout contract is added, its edits must not change model
semantics. When references are added, changed dependency bytes must invalidate
the relevant request/cache identity.
Round-trip/migration tests preserve source IDs, units/frames, finite numeric
values and unknown-version refusal. Headless execution and future desktop
submission use the same resolved request and source map.

Malformed/oversized documents, duplicate IDs, reference cycles, conflicting
keys, path escapes and count/allocation overflows receive public-boundary
negative tests appropriate to the admitted profile. Retained evidence follows
[ADR-0008](0008-numerical-evidence-authority.md): successful parsing or matching
hashes cannot establish numerical reliability, aircraft validity or engineering
acceptance.

## Alternatives considered

**Use exact source bytes as the only model identity.** This is straightforward,
excellent for integrity, and sufficient for an initial opaque-input runner. It
cannot distinguish layout/comment edits from changed equations, so it is
retained alongside semantic identity rather than used as its replacement.

**Hash only compiled IR.** This directly identifies the schedule being run and
can support a compiler cache. It loses an independent source identity, ties
editing/replay to one compiler and can hide what changed during lowering. Keep
the IR digest, source model and source map together.

**Place all model/project data in one authoritative database.** Transactions
and indexes could make project operations convenient. The cost is a stronger
dependency on database compatibility/recovery before the smallest numerical
workflow is established. Revisit after representative workloads and recovery
spikes; an optional index need not own the model.

**Treat arbitrary metadata as harmless and omit it from hashing.** This makes
format evolution and third-party annotations convenient. It also lets an
unrecognized executable field disappear from both validation and identity.
Use an explicit presentation/annotation boundary and reject unknown semantics.

## Consequences

More than one digest is necessary, and interfaces must label their scope.
Semantic canonicalization and migrations become maintained compatibility
surfaces with independent fixtures. In return, editor changes cannot silently
alter a submitted calculation, and an investigator can follow source bytes,
resolved equations, executed artifacts and evidence separately.

The architecture stays usable through C++ and CLI without a proprietary
simulator or a selected GUI toolkit. No new dependency, license approval,
plugin sandbox or aircraft assurance claim is introduced. The larger project
store and native-extension ABI remain separate work.

## Revisit when

Freeze a public model format; add library/model references, variants, tunable
parameters or checkpointing; introduce numerical optimizations or parallel
schedules; select project storage; support external model translation; or accept
a customer requirement for signed models, restricted-data storage or retained
evidence across a declared compatibility window.

## Subsequent bounded implementation

[ADR-0012](0012-project-worker-preview.md) implements experimental local project
revisions, a separate presentation document and retained CLI worker requests.
That preview advances the storage boundary described here without freezing the
public project format or supplying the broader model-reference/migration scope.

Two further increments build on it.
[ADR-0013](0013-typed-linear-graph-adapter.md) imports a linear plant and its
controller as an editable typed graph, and exercises the source/presentation
separation argued here against real evidence: the imported origin stays
immutably attached across edits and reports whether the current model still
matches it, rather than certifying an edited model.
[ADR-0014](0014-project-revision-recovery.md) makes the retained revisions
reviewable and restorable, which is what gives immutability a user-visible
purpose. Neither freezes the public project format either.
