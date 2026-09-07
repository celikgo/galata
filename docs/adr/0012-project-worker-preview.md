# ADR-0012: Portable project revisions and an isolated preview worker

- **Status:** experimental M2 preview contract; locally verified on macOS arm64, maintainer review pending
- **Date:** 2026-09-07
- **Deciders:** project maintainer review pending

## Context

The [M1 continuous model](0010-continuous-scalar-executable-model.md) supplies a
shared compiler and simulator. The next bounded increment needs an editable
project, immutable submissions, cancellation and review after a worker exits.
[ADR-0011](0011-source-model-identity-and-project-boundary.md) separates source,
presentation and evidence but deliberately leaves project storage unimplemented.
The [M2 delivery gate](../product/DELIVERY.md) additionally requires a graphical
aircraft/controller workflow and installation evidence. This preview advances
the project/worker boundary using a synthetic continuous feedback model; it does
not close that complete gate or the remaining external M1 decisions.

## Decision

Use a portable local directory with content-bound immutable revisions and
run-owned artifacts. Keep execution in the existing C++ compiler, simulation
and study pipeline. A desktop shell launches the same project CLI as a separate
process and inspects the same saved results as headless callers. The preview
supports macOS first and Linux next; Windows engine portability remains separate
from this POSIX worker increment.

### Editable and authoritative documents

The project head is `project.json`, a closed object with schema
`galata.project.v1` and a revision SHA-256. That digest selects the exact draft
bytes in the revisions directory. A draft has schema `galata.project-draft.v1`
and exactly three content objects: model, presentation and simulation.

The model uses the closed M1 model vocabulary. A draft parser checks supported
versions, keys, identifiers, finite values and declared source/resource limits.
Editing may retain incomplete wiring, incompatible connections or an algebraic
loop. Compilation at submission refuses those execution defects and retains the
failure. A structurally invalid document is refused before it can replace the
head. The existing strict executable parser remains strict for its callers.

Presentation uses schema `galata.presentation.v1`, with a positions map from
stable block IDs to finite canvas x/y coordinates. It cannot contain executable
parameters or arbitrary unknown fields. Simulation contains initial time,
step size, step count and sample stride using the M1 run-option semantics.
Neither object is silently folded into the executable model vocabulary.

Every saved revision binds the complete draft, so changing layout or solver
options creates a different project revision. The engine's semantic model
digest excludes those changes. A semantic edit changes the model identity;
a solver edit changes the submitted run configuration; a presentation-only edit
must preserve both the mathematical model and its exact same-environment
trajectory. Digests establish content identity, not validity or approval.

The save command requires the revision observed by the editor. Under an
exclusive project write lock it checks that expectation, publishes the immutable
new revision, then replaces the head. A stale editor receives a conflict and
does not silently overwrite a newer edit. Previous revisions remain available.
An interrupted publication may leave an unreferenced revision; it cannot make a
half-written revision the accepted head. A process exit releases its OS lock.
The initial contract supports one local filesystem, not shared-drive editing or
power-loss durability guarantees.

### Public CLI and worker ownership

The project CLI has four operations:

- Create a project in a new or empty directory using the synthetic starter model.
- Inspect the current revision, presentation, simulation and retained run states.
- Save a supplied draft with an expected revision.
- Run the current revision in the CLI process, isolated from the desktop process.

Create, inspect and save return a single `galata.project-view.v1` JSON document.
Run returns its run ID, execution status, diagnostic and artifact locations.
Successful commands exit zero; refused, failed and cancelled execution exits
nonzero. Human diagnostics cannot masquerade as a successful JSON response.
Paths reported to the current caller are absolute, while owned artifact paths
stored within the project are relative so the directory can be moved. Original
source locations remain provenance rather than active lookup instructions.

A run first acquires its own OS lock and publishes an immutable request naming
the submitted revision. It snapshots the executable model and generated study
inside its own run directory, then invokes the ordinary pipeline. Subsequent
editing changes the project head and cannot alter those submitted bytes.
The project lock is held only for the snapshot/publication transaction, so a
long-running worker does not prevent saving a new working revision.

The engine receives a cooperative cancellation callback. The pipeline polls
before stages and before committing a completed manifest; the continuous
simulator also polls at its defined step boundaries. A SIGTERM requests this
cooperative cancellation. Native callbacks or existing stages may still take
time to return. Hard termination supplies process isolation and recovery
evidence; it is not a claim of operating-system sandboxing, a fixed wall-time
budget or hostile native-code containment.

### Terminal state and evidence verification

A run directory retains its request, model/study snapshots, output directory
and terminal record when one can be published. These states are distinct:

| Observed state | Meaning |
|---|---|
| Running | The request exists and its worker still holds the run lock |
| Completed | A terminal completion record and the required content-bound pipeline/model evidence verify |
| Failed | Parsing, compilation, execution or publication failed, with a retained diagnostic |
| Cancelled | The worker observed cancellation and recorded that outcome |
| Interrupted | A worker no longer holds its lock and no terminal outcome was committed |
| Invalid | Retained records or artifacts fail integrity/schema checks and cannot support a completed result |

PID presence alone cannot establish that the original worker still owns a run;
process IDs may be reused. Inspect uses the run lock for that ownership check.
A stale index, an output CSV or exit code zero alone cannot establish completion.
Review verifies the manifest's content identity, its consumed source/output
bindings and the model evidence/trajectory digest. A tampered previously
completed artifact becomes invalid in the current view; the original terminal
bytes are retained for investigation. A relocated project resolves artifacts
against its current owned run directory and retains original provenance.

Successful computation keeps numerical accuracy, model validity and engineering
acceptance as separate evidence dimensions. This synthetic preview leaves those
assessments explicit. Cancelled/interrupted partial files cannot acquire
completed status because a later read finds plausible data. The initial worker
does not resume a partially executed simulation or reuse compiled caches.

### Limits, trust and compatibility

The preview bounds individual model/source documents to 1 MiB, project drafts
to 2 MiB, and retained revisions and runs to 1,024 each. M1 block, connection,
step, scalar-storage and scheduled-evaluation limits remain effective. Validate
sizes and checked counts at public boundaries before allocating or executing.
Unknown project, draft, presentation, model and evidence versions fail closed.
There is no implicit migration, network resolution, script execution, plugin
loading or external-model reference in this profile.

Project-relative names are generated or checked against their owning directory.
Malformed digests, path traversal and symlinked owned files are refused. External
draft import is explicit and bounded. No operation may use a supplied artifact
name as a command or trust an old absolute provenance path as the current output
location. Source bytes and diagnostics can contain sensitive model information;
the preview does not upload them automatically.

Project/schema numbers describe experimental compatibility surfaces, while
the existing VERSION mechanism remains the sole product-version source.
Original revisions and runs remain retained; cache rebuilding and migrations
are deferred until their compatibility and recovery protocols are specified.
No public-format freeze, long-term migration window, cryptographic authorship,
encryption-at-rest, notarized installer or clean-machine acceptance is claimed.

## Required acceptance evidence

The public CLI acceptance suite checks stale save refusal; immutable old
revisions and run requests; semantic/layout/solver separation; an analytic
continuous feedback endpoint with a tolerance fixed before measurement;
unwired/type-invalid/loop refusal at execution; malformed, duplicate-key,
oversized and unknown-schema documents; traversal and symlink refusal; retained
failed/cancelled/interrupted states; output tampering; relocation; and saving
while a worker executes. Both normal and hard termination must leave the project
reviewable without manually deleting a stale process lock.

Pipeline cancellation tests exercise the callback before execution, during a
continuous run and before final manifest publication. No cancelled case may
publish a successful manifest. Already written partial output need not be
rolled back, but its status must remain distinguishable.

The native macOS shell is a feasibility candidate using this protocol. Its
graph editing, keyboard accessibility, diagnostic navigation, offline packaging
and recovery must be measured before a desktop stack is selected. Qt and a
local web-shell comparison, Linux GUI support, minimum OS/hardware decisions,
the aircraft adapter and external-user acceptance remain open under the
[discovery agenda](../product/DISCOVERY.md).

## Alternatives considered

**Put the engine inside the desktop process.** This reduces IPC work but makes
an engine crash or long-running native call affect editing and recovery. Keep
the CLI process as the execution boundary.

**Own all source and evidence in a database.** A database could help indexing
and transactions, but would introduce storage/migration obligations before the
smallest reviewable workflow. An index may be added later as derived data.

**Treat existing output files as a successful run.** This loses interruption,
revision and tamper distinctions. Completion requires a verified terminal record
and the evidence to which it refers.

**Select the desktop toolkit now.** The installed toolchain can support a native
macOS feasibility experiment, but availability alone does not establish the
deployment, accessibility or support comparison required by D11.

## Consequences and revisit conditions

The CLI protocol permits independent process/recovery tests and a thin graphical
client while preserving the numerical implementation. Multiple identities and
terminal states become maintained interfaces. The directory is portable and
inspectable, but local locking and atomic publication do not establish every
filesystem's crash-durability behavior.

Revisit before public project-format stability, shared editing, automatic
migration, database ownership, sampled-controller semantics, external model/data
references, concurrent campaigns, worker sandboxing, signed distribution or a
declared long-term evidence compatibility window.
