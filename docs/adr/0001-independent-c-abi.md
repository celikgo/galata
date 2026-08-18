# ADR-0001: Plugin C ABI is an independent sibling of `souxmar-c`

- **Status:** accepted
- **Date:** 2026-08-18
- **Deciders:** project authors

## Context

galata needs a stable C plugin ABI so that aerodynamic models, actuator and
sensor models, synthesis methods, analyses and hardware bridges can ship as
out-of-tree binaries without recompiling the host.

A sibling project, [`souxmar`](https://github.com/celikgo/souxmar), already
implements exactly this pattern for computer-aided engineering: one exported
registration symbol per plugin, an opaque host-owned registry, capability
identifiers dispatched by prefix, an appended-only status enumeration, and an
additive-only minor version ratchet policed by a CI gate. That ABI is at v1
FINAL with the major frozen for the life of its 1.x series and its minor
ratcheted to v1.9.

The pattern is proven. The question is whether galata should share souxmar's
headers, extract a jointly-owned kernel from them, or implement the same shape
independently.

The shared surface is genuinely small. `abi.h` (version macros, symbol export,
C-linkage helpers), `status.h` (status codes and a status struct), `plugin.h`
(the host-info struct and the entry-point signature), `registry.h`
(registration functions) and `value.h` (a read-only typed input tree) are
domain-neutral. Everything else in `souxmar-c` — meshes, BREP sessions,
sketches, fields, surface streams — is CAE and has no meaning in flight
dynamics. Conversely, galata will need capability headers for aerodynamic
coefficient models, trim problems, state-space systems and time histories that
mean nothing to a CAE tool.

## Decision

galata implements its own `include/galata-c/` with the same shape and the same
governance rules, and shares no headers, no build system and no release cadence
with `souxmar`.

The shape is copied deliberately and the debt is acknowledged: `abi.h`,
`status.h`, `plugin.h`, `registry.h` and `value.h` will be structurally similar
to their souxmar counterparts, with `GALATA_`/`galata_` in place of
`SOUXMAR_`/`souxmar_`. That is duplication of a design, not of a dependency.

## Alternatives considered

**Share `souxmar-c` headers directly.** The honest case for this is strong on
paper: five headers already exist, are frozen, are conformance-tested, and have
survived nine additive minor ratchets in production. galata would inherit a
battle-tested surface on day one and would never have to relitigate the
ownership rules for borrowed pointers or the numbering discipline for status
codes.

The cost is that it inverts the dependency in the wrong direction. galata would
depend on a project whose ABI is *already frozen for the life of 1.x*. Any
surface galata needs that souxmar does not — and there will be many, starting
with a state-space system handle — could only be added by ratcheting souxmar's
minor version, which means a CAE project's ABI version increments for reasons
its own users cannot observe. Plugin authors would build against a header set
containing `souxmar_mesh_face_tag` to write an aerodynamic table lookup. That is
not a technical failure, it is a comprehension failure, and comprehension is the
entire point of a stable ABI.

**Extract a jointly-owned kernel.** This is the theoretically correct answer and
would be the right one if both projects were at commit zero. The five neutral
headers become an `engine-c` package; each project adds its domain headers on
top; the ownership rules are written once.

It is rejected on timing, not on merit. souxmar's ABI is v1 FINAL: its major is
frozen and its header inventory is under a CI lock that fails any change without
a Tier-3 ADR. Extracting those headers into a separate package changes every
include path in every published plugin and every conformance fixture in a
shipped 1.0 product, and it does so to benefit a project that has not yet
written its first capability. The refactor's entire cost lands on the mature
project and its entire benefit lands on the immature one. If galata reaches its
own v1 and the two ABIs have in fact stayed structurally identical, the
extraction can be done then, at a major version boundary on both sides, with
evidence instead of speculation.

## Consequences

- Two ABIs must be maintained. A fix to a shared concept — say, a clarification
  of string lifetime rules — has to be applied twice, by hand, and can drift.
  Accepted: the surface is five small headers and the drift is visible in
  review.
- galata's `abi_version_minor` starts at 0 and ratchets on its own schedule,
  driven only by flight-dynamics needs.
- galata will ship its own conformance suite (`galata-conformance <dir>`) and
  its own `scripts/check-frozen-headers.sh`, landing in the same commit as the
  first ABI header rather than before it. These are structurally the same tools
  as souxmar's and are written independently.
  *Neither exists as of this record; `include/galata-c/` is empty.*
- Nothing in galata may `#include <souxmar-c/...>`. There is no build-system
  path by which it could, and no CI gate is needed to prevent it, because
  souxmar is not a declared dependency in `vcpkg.json`.
- If both ABIs are still structurally identical when galata freezes its own v1,
  this decision should be revisited rather than assumed.

## Revisit when

galata's ABI reaches a v1 freeze candidate, at which point compare the two
neutral header sets. If they have not diverged, evaluate extraction for
galata 2.0 / souxmar 2.0 — the only boundary at which it is cheap for both.
