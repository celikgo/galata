# Experimental project files and worker commands

The project preview stores editable continuous block models, presentation and
simulation settings alongside immutable run evidence. It implements the bounded
[M2 project/worker increment](product/M2_IMPLEMENTATION.md) under
[ADR-0012](adr/0012-project-worker-preview.md), plus the typed linear import in
[ADR-0013](adr/0013-typed-linear-graph-adapter.md) and retained revision recovery in
[ADR-0014](adr/0014-project-revision-recovery.md). These experimental formats do not
freeze a public project API or change the product version from 0.3.0.

The project commands support the macOS/Linux POSIX worker boundary. Current
verification claims are listed in the implementation record; hosted Linux and
macOS verification remain pending. The existing study CLI and numerical library
retain their own platform and evidence scope.

## Commands

After building the CLI, create and run the synthetic feedback starter:

```bash
./build/dev/src/cli/galata project create build/feedback.galata
./build/dev/src/cli/galata project inspect build/feedback.galata
./build/dev/src/cli/galata project run build/feedback.galata
```

Import the local NT-33A plant/controller graph into a new project:

```bash
./build/dev/src/cli/galata project import-linear build/nt33a.galata examples/nt33a-graph-design/study.yaml
./build/dev/src/cli/galata project inspect build/nt33a.galata
./build/dev/src/cli/galata project run build/nt33a.galata
```

Import runs the study into project-owned storage and adopts its exported model
and simulation settings. It requires exactly one `model.linear_graph` stage
and exactly one directly connected `sim.model` stage. Reference `sim.linear`
stages and reports are allowed. The study may contain at most 64 stages; its
retained origin accepts at most 128 input records and 128 artifact records,
including the manifest. A failed or cancelled import does not publish a project
head; incomplete import outputs remain for investigation. Retry uses another
new or empty destination. There is no automatic cleanup or import resume.

The .galata suffix is a directory naming convention, not an archive format.
Create and import require a new or empty directory. Inspect returns a single JSON document
with the current revision, model, presentation, simulation settings and run
history. Run executes synchronously in its own CLI process and returns a run
record with execution status and artifact paths. A desktop caller launches this
same command as a child process.

Save a draft using the exact revision returned by the inspect operation that
supplied the editor's working copy. In this syntax, replace the angle-bracketed
arguments with the draft file and observed revision:

```text
galata project save <directory> <draft.json> --expected-revision <64-hex-revision>
```

For example, the directory may be build/feedback.galata. A stale revision is
refused, leaving the newer head in place. Create, import, inspect and save return schema
`galata.project-view.v1`; the view includes `revision`, `draft_schema` and `runs`
in addition to the three draft content objects. Imported projects also return
`origin_sha256` and a verified `origin` view. Successful commands exit zero. Refused, failed
and cancelled commands exit nonzero with a diagnostic.

### Review and restore saved work

List retained drafts, inspect a selected draft, then restore it using the head
observed by the editor:

```text
galata project revisions <directory>
galata project revision <directory> <64-hex-revision>
galata project restore <directory> <64-hex-revision> --expected-revision <observed-head>
```

The list returns `galata.project-history.v1`, with `current_revision`,
`current_status`, an optional `current_diagnostic`, `ordering` and `revisions`.
The ordering is `revision_filename_ascending`; content hashes and filesystem
timestamps do not establish chronology or ancestry. Each entry reports its
retained `filename`, `revision` stem, `is_current` and `status`. A `valid` entry
also includes `model_profile`, `block_count`, `simulation` and, for an imported
draft, `origin_relation`. An `invalid` entry includes a diagnostic. A damaged,
malformed or symlinked entry remains visible alongside other valid entries.
A missing current file produces an invalid current status without adding a
synthetic retained entry. A malformed project pointer refuses the listing.

Selected inspection returns `galata.project-revision.v1`, containing `revision`,
the complete validated `draft` and optional `origin_relation`. These are saved
bytes interpreted through the draft contract; valid history can still contain
unfinished wiring. Neither inspection publishes a head or starts execution.
Both hold the existing project lock while obtaining their view.

Restore checks the expected head and both drafts under the same lock. It
requires an unchanged draft schema and origin attachment identity, then
atomically points the head at the exact selected revision and returns
`galata.project-view.v1`. It does not create another revision or change any
retained draft bytes. Restoring the current revision is an idempotent success.
A stale editor, missing or invalid target, incompatible schema or different
origin is refused. The current head and draft must also be valid; this command
does not repair a damaged storage pointer or reconstruct a missing origin.

All newer revisions and runs remain available. A worker already executing
continues with its submitted revision. A later restore can select any compatible
retained draft again. The history view validates each entry's own content and
origin; restore additionally checks its compatibility with the current project.

## Draft versus executable model

A save input is a closed JSON object with these required fields:

| Field | Contract |
|---|---|
| `schema` | `galata.project-draft.v1` for ordinary projects; `galata.project-draft.v2` for imported projects |
| `model` | A model object using the [continuous scalar or linear model vocabulary](MODEL_FILES.md) |
| `presentation` | `galata.presentation.v1` for block positions, or `galata.presentation.v2` for positions and manual wire routes; see the contract below |
| `simulation` | `initial_time_s`, positive `step_s`, nonnegative integer `steps`, and positive integer `sample_stride` |
| `origin_sha256` | Required only in draft v2: the immutable SHA-256 of the retained source-origin attachment |

An inspect response is a view rather than a save input: construct the draft
from its `model`, `presentation` and `simulation`, and use the view's
`draft_schema` as the draft `schema`. For v2 retain `origin_sha256` exactly.
Retain the view's revision separately for the expected-revision argument.
Save refuses changing the draft schema, removing the origin or replacing its
digest. The source attachment is not an editable property of a project.

The model keeps the M1 Constant, Gain, signed ordered Sum, Integrator and Output
block kinds; the explicit `continuous-linear.v1` profile adds ordered typed
`linear_combination` rows. Save checks supported versions, closed keys, finite numeric values,
identifiers and resource limits. It can preserve unfinished wiring, incompatible
connected types and algebraic loops so that an engineer can save work in
progress. Run applies the complete compiler checks and records refusal as a
failed job before numerical execution. Existing strict executable-model parsing
and direct C++ compilation keep their validation contracts.

Numeric JSON strings and Boolean values are not numeric parameters. Binary64
values, including signed zero and representable subnormals, retain the model
format's conversion semantics. Integer input-port indexes must survive saving
without conversion through binary64; an exact index outside the block's ports
remains an execution error. Stable IDs such as `true` and `false` are strings.

Presentation fields cannot introduce executable behavior. A layout edit changes
the project revision while preserving the executable semantic digest. Solver
settings also belong to the project/run request rather than the equations:
changing a step size preserves model identity but changes the submitted run.
An ordinary semantic edit changes the model's semantic identity.

The imported origin view has schema `galata.project-origin-view.v1`, the original
`model_semantic_sha256`, parsed `adapter` and `manifest` objects, the retained
manifest path and a `relation` field. `matches_imported_model` means the current
compiled graph has the imported semantic digest. `modified_from_import` means
it has a different digest; `uncompiled_draft` means the current draft cannot
compile. These labels describe identity, not numerical or engineering approval.
Inspected runs also report their revision's relation to the same origin.

### Presentation and manual wire routes

Presentation is a closed object independent of the project-draft version.
Both ordinary draft v1 and imported draft v2 accept either presentation schema:

| Presentation schema | Required fields |
|---|---|
| `galata.presentation.v1` | `schema`, `positions` |
| `galata.presentation.v2` | `schema`, `positions`, `routes` |

`positions` is a map with at most 1,024 entries. Each key names an existing block;
each value is a closed `{ "x": number, "y": number }` object. Coordinates must
be finite and in [-100,000, 100,000]. A block may omit its stored position.
Presentation v1 rejects a `routes` field. Presentation v2 requires a `routes`
array, which may be empty and contains at most 256 entries.

Each route is a closed object with these required fields:

| Field | Contract |
|---|---|
| `source` | Source block ID from an existing model connection |
| `target` | Target block ID from that same connection |
| `input` | That connection's exact input index: an unquoted, nonnegative decimal integer representable by `std::size_t` |
| `points` | An ordered array of 2 through 64 closed `{ "x": number, "y": number }` objects |

At most one route may name each `(source, target, input)` tuple. Every route
coordinate is finite and in [-100,000, 100,000]. Each consecutive pair must
change exactly one coordinate: segments are nonzero and horizontal or vertical.
Unknown fields, nonexistent connections, duplicate route identities, malformed
coordinates and oversized routes are refused. Route coordinates are logical
canvas positions; view zoom, scrolling and display offsets are not stored.

The worker validates this bounded presentation syntax and connection identity.
It does not derive card dimensions, enforce port attachment or assess a route's
visual clearance. The native preview separately checks that custom routes leave
and enter ports rightward, attach to their current positions and avoid card
interiors. When a retained route fails those display checks, the preview shows
an automatic path with a diagnostic while retaining the custom route until the
user reshapes or resets it. Rendering limits still apply.

Existing presentation v1 drafts remain accepted. Opening a project does not
upgrade its presentation. The native preview switches to presentation v2 when
a custom shape is applied to the draft; Save retains it. Resetting the last
custom route returns the draft to presentation v1. Moving a block removes the
custom routes attached to it; Arrange removes all custom routes. These edits
support Undo/Redo. Neither the project-draft schema nor the imported origin
identity changes with presentation edits.

Older workers that support only presentation v1 refuse presentation v2 drafts;
they do not silently discard saved routes. Use the updated worker with custom
routes, or explicitly reset all routes and save before opening with an older one.

Manual route edits change the project revision, but they do not change the
connection graph, executable semantic digest or numerical execution. The
compiler's model validation and the separation of execution evidence from
accuracy, physical validity and engineering acceptance remain unchanged.

## Storage and identity

The following names illustrate files generated inside a project:

```text
project.json
revisions/<revision-sha256>.json
origins/<origin-sha256>.json
imports/<import-id>/model.yaml
imports/<import-id>/adapter.json
imports/<import-id>/run-<manifest-sha256>.json
runs/<run-id>/request.json
runs/<run-id>/model.yaml
runs/<run-id>/origin.json
runs/<run-id>/study.yaml
runs/<run-id>/result.json
runs/<run-id>/output/response.csv
runs/<run-id>/output/evidence.json
runs/<run-id>/output/run-<manifest-sha256>.json
```

The project head has schema `galata.project.v1` and names the SHA-256 of the
stored draft revision. Revisions are immutable; a successful save writes a new
revision and publishes the head under an exclusive project lock. Previous
revisions and runs remain retained. An interrupted save may leave an
unreferenced revision, which does not replace the accepted head.
Explicit restore only moves this pointer back to a compatible retained draft;
history discovery also includes valid unreferenced drafts from an interrupted
save. No save order or reachability graph is inferred from directory contents.

Imported projects retain all study outputs under `imports/<import-id>/` and a
bounded `galata.project-origin.v1` attachment under the origins directory. The attachment
contains the original model semantic digest, exact exported `model_yaml`,
`adapter_json` and `manifest_json` strings, project-relative `manifest_path`,
and `artifacts` records with project-relative paths and SHA-256 values. This
binds original matrix/channel/controller evidence and full source linearization
diagnostics to the imported files and source snapshots. Inspect and run verify
these bindings before presenting an imported project's source context.

A submitted request binds one saved revision. The worker snapshots the model
and generated study in its own run directory before execution. Imported
revisions also snapshot the exact attachment as `origin.json`; their generated
study supplies it through `model.compile.context_path`. The new run manifest
retains that file as a consumed input. Saving later
edits does not mutate that request or its inputs. Project locking covers the
snapshot transaction rather than the full simulation, so working edits can be
saved while an earlier request runs.

Stored artifact paths are project-relative. Paths emitted to the current caller
are absolute and resolve inside the owned run. Moving the project directory
preserves those bindings; original absolute source locations remain provenance.
Malformed revision digests, traversal and symlinked owned files fail closed.
There is no automatic external model lookup or network resolution.

## Run status and review

| Status in an inspected view | Interpretation |
|---|---|
| `running` | The request exists and its worker owns the OS run lock |
| `completed` | The terminal record, manifest, consumed source and required output/evidence bindings verify |
| `failed` | The worker retained a parsing, compilation, execution or publication failure |
| `cancelled` | The worker observed cooperative cancellation and recorded it |
| `interrupted` | The worker no longer owns its lock and no terminal record was committed |
| `invalid` | Retained schema or integrity checks fail; the record cannot support completed status |

SIGTERM requests cooperative cancellation. The pipeline checks between stages
and before completed-manifest publication; the continuous simulator also checks
at step boundaries. A native callback or long stage can still delay a response.
Hard termination releases the worker's OS lock, and a later inspect identifies
the interrupted request. Process ownership does not rely on a potentially
reused PID. No manual stale-lock deletion is required after a process exits.

The terminal record uses schema `galata.project-result.v1` and retains its
revision, ID, status and diagnostic. A completed record binds trajectory,
model-evidence and pipeline-manifest paths and digests. Review verifies those
bindings and the exact consumed source bytes against the submitted snapshot.
Rehashing a manifest that names a different source does not establish that it
belongs to this request. If a retained output is changed, inspect reports an
invalid result and preserves the original terminal record for investigation.

Completion remains execution evidence. Numerical accuracy, model validity and
engineering acceptance stay separate and `not_assessed` for both continuous
profiles. The model evidence retains semantic identity, solver options, ordered
state/output/schedule IDs and a trajectory digest. The ordinary pipeline
manifest retains input snapshots and build/runtime identity. A plot or partial
CSV cannot replace these records or turn an interrupted run into a completed
study.

The imported manifest's original Jacobian and CARE diagnostics remain visible
as source evidence. Project reruns do not inherit those diagnostics as an
assessment of edited model rows. Even an unchanged graph requires separate
accuracy, physical-validity and engineering-acceptance evidence. Retaining
the origin supplies traceability; it does not establish a newly validated
aircraft model or controller.

## Limits and compatibility

Model source is limited to 1 MiB; a project draft to 2 MiB; source-origin
attachments to 8 MiB; and retained runs
and revisions to 1,024 each. The M1 compiler/runtime limits on blocks,
connections, steps, stored scalars and scheduled evaluations continue to apply.
Unknown schemas, duplicate keys, malformed and oversized documents are refused.
Canvas coordinates are bounded to [-100,000, 100,000]. Initial time is bounded
to [-10^12, 10^12] seconds; step size is between the smallest normal binary64
value and 10^12 seconds. Step count is at most 1,000,000 and sample stride is
in [1, 1,000,000]. The runtime additionally checks representable time arithmetic
and execution/storage budgets. Artifact review reads at most 128 MiB per file.

This preview has no automatic migration, autosave, checkpoint resume, compiled
cache, shared-drive editing or long-term compatibility guarantee. OS locks and
atomic publication are a local process-recovery contract, not proof of every
filesystem's power-loss durability. The worker is a separate process without
an operating-system sandbox or accepted hard wall-time/resource budget.
The linear adapter reconstructs a local perturbation plant/controller graph;
it adds no nonlinear aircraft block, sampled/hybrid behavior or validity
assessment.

For native macOS build and interaction instructions, see the
[M2 implementation record](product/M2_IMPLEMENTATION.md) and
[desktop preview guide](../src/desktop/README.md).
