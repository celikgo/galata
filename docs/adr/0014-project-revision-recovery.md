# ADR-0014: Review and restore retained project revisions

- **Status:** experimental M2 contract; implemented and locally verified; maintainer acceptance pending
- **Date:** 2026-09-07
- **Deciders:** maintainer review pending

## Context

[ADR-0012](0012-project-worker-preview.md) retains immutable drafts but exposes
only the current editor view. A saved mistake therefore requires manual file
editing to recover, despite the original draft still being available. The
desktop needs bounded history discovery, inspection of a selected draft and an
explicit operation to restore it without changing any submitted run evidence.

Imported projects also retain immutable original evidence under
[ADR-0013](0013-typed-linear-graph-adapter.md). Recovery must not remove or
replace that origin or silently migrate between draft versions.

## Decision

Add three operations to the existing POSIX project CLI:

- `project revisions <directory>` lists retained revision filenames in ascending
  byte order and identifies the current revision.
- `project revision <directory> <revision>` returns the full validated retained
  draft for review.
- `project restore <directory> <revision> --expected-revision <observed-head>`
  restores that exact retained draft as the current project head.

History uses schema `galata.project-history.v1`. It reports `current_revision`,
`current_status`, an optional `current_diagnostic`, the explicit ordering label
`revision_filename_ascending`, and a `revisions` sequence. Entries contain the
retained `filename`, its `revision` stem, `is_current` and `status`. A valid
entry additionally reports `model_profile`, `block_count`, `simulation` and,
for imported drafts, `origin_relation`. An invalid entry reports a diagnostic.
Malformed filenames, symlinks, nonregular files, corrupt content and unsupported
drafts remain visible as invalid entries; they do not suppress unrelated valid
entries. A missing current file is reported by the top-level current status.
An invalid project pointer remains an overall refusal because its identity is
unknown. Existing limits admit at most 1,024 retained directory entries and
2 MiB per draft. History contains summaries, not duplicated full draft bodies.

Selected inspection uses schema `galata.project-revision.v1` with `revision`,
the complete `draft` and optional `origin_relation`. Both inspection operations
use the existing project lock to obtain a view consistent with cooperating
saves, restores and worker submissions. A valid history entry means its
own stored content and origin verify. It does not prove ancestry, compatibility
with the current project's origin, compilability or engineering acceptance.
Draft validation continues to permit unfinished wiring. No timestamps,
ancestry, author identity or chronological save-order claims are inferred from
filesystem metadata or sorted content digests.

Restore acquires the same exclusive project lock, checks the expected head,
validates both the current and selected revisions, then requires identical
draft schemas and identical origin attachment identities when present. It
atomically replaces only the project head pointer with the selected digest and
returns the ordinary project view while still holding the lock. No new revision
is manufactured; immutable draft bytes remain unchanged. Restoring the already
current revision is an idempotent success. Stale, absent, malformed, corrupt,
symlinked or foreign-origin targets are refused without changing the head.
The command cannot repair an invalid current pointer or current draft, because
the required compatibility boundary would not be established.

All other revisions and runs remain retained, including revisions saved after
the selected one and requests already executing. A running worker continues
with its immutable submitted revision. Another restore can return the editor
to any compatible retained revision. This is explicit local recovery, without
automatic rollback, history deletion, migrations, autosave or run resumption.
It inherits the existing local-filesystem atomic-publication scope rather than
adding a power-loss durability guarantee.

## Required acceptance evidence

Independent public CLI tests must establish exact retained-draft restoration,
bounded deterministic listing, selected-draft review, stale-head refusal,
preservation of newer revisions and submitted runs, and idempotent restore.
Negative cases cover malformed or missing identities, altered stored bytes,
nonregular and symlinked entries, incompatible draft schemas and changed
import-origin identities. Damaged entries must remain visible alongside valid
history. No numerical algorithm, simulation budget or evidence acceptance
meaning changes in this increment.

## Alternatives considered

**Copy a restored draft into a new revision.** Identical bytes already have the
same content digest. Adding an operation marker to create another digest would
mix history metadata into the immutable draft contract. Keep restoration as an
explicit head-pointer move until a separately specified operation log is needed.

**Read filesystem timestamps as history order.** Copying or relocating projects
can change those values and content-addressed saves can reuse a revision.
Filename ordering is deterministic and makes no chronology claim.

**Restore despite a damaged current draft or changed origin.** That would lose
the boundary that keeps imported evidence attached across edits. Such repair
requires a separate recovery protocol with an independently established origin.

## Consequences

The desktop can review and restore saved work through the public CLI without
writing storage internals. Immutable evidence stays bound to its original
revision. Listing and verification are bounded by the existing retention
limits; no index or mutable history database is introduced. Full M2 desktop
acceptance and later delivery milestones remain open.
