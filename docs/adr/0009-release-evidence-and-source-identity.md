# ADR-0009: Releases consume complete evidence for one immutable source

- **Status:** implemented; maintainer review and hosted execution pending
- **Date:** 2026-09-07
- **Deciders:** implementation team within the authorized M0 reliability work

## Context

The offline workbench emits engineering results and distributes executable
archives. A release tag alone does not prove that its source passed the required
checks. Previously the release workflow ran its platform CTest suites but did
not depend on governance, generated-evidence checks, sanitizers, static analysis
or cross-platform determinism. The resolver passed a mutable tag name into each
matrix job, so it did not actually bind one commit across the release.

Run provenance identified the executable, Git commit and dirty flag. The dirty
flag could not distinguish two local source edits. Compiler/version labels did
not identify effective target flags. These gaps matter when investigating a
result, even where no qualification claim is made.

## Decision

The [CI workflow](../../.github/workflows/ci.yml) is reusable and accepts an
immutable source commit. Every executing job checks out that commit and verifies
Git HEAD before producing evidence. The source parameter is passed explicitly
to the nested [determinism workflow](../../.github/workflows/determinism.yml),
including its fingerprint and comparison jobs; a caller's default branch or
dispatch SHA must not accidentally replace the requested release source.

The aggregator accepts only the complete required job set, with every job
successful and every output source identity matching. Missing, skipped,
cancelled, extra or wrong-source evidence is rejected by
[the shared evidence contract](../../scripts/ci_evidence.py). After that check,
the workflow exposes the verified source and retains a CI evidence artifact
with the source, job results and originating workflow/run identity. Adding a
required job means updating both the workflow dependency graph and this
contract; mismatch fails closed.

The [release workflow](../../.github/workflows/release.yml) resolves a version
tag to its commit once, checks the source version, calls complete CI for that
commit and packages the same commit. Packaging requires the CI artifact and a
clean matching source/build snapshot in release mode. Before upload, the
[release evidence check](../../scripts/check-release-evidence.py) verifies the
full platform set, archive hashes and matching source/evidence metadata. The
tag is fetched again and refused if it has moved. The release must already
exist; these changes do not create or publish any release by themselves.

Manual release dispatch remains supported for tags that contain the required
assurance infrastructure. Historical tags lacking the gate scripts are not
backfilled without checks. The workflow definition comes from the dispatch
revision, while every source checkout is explicit; both identities are retained
in evidence. The [Pages workflow](../../.github/workflows/pages.yml) calls the
same complete gate before rendering and deploying the corresponding source.

GitHub supports reusable-workflow inputs/outputs and resolves a relative
workflow reference from the caller workflow's commit. Passing the source SHA
explicitly is therefore necessary when a manual release selects another tag.
[GitHub reusable-workflow documentation](https://docs.github.com/en/actions/how-tos/reuse-automations/reuse-workflows).

At build time, [the provenance generator](../../scripts/build-provenance.py)
hashes a canonical inventory of tracked and nonignored untracked source bytes,
including paths and executable modes. Build trees, caches and generated example
outputs at explicitly listed paths are excluded, including legacy tracked
reports. Generated-name patterns additionally filter untracked example output;
they do not omit a tracked reference input with a similar name. Source/data files are
included regardless of extension, including example C++, study scripts and
model CSV tables; an extension allowlist must not hide those inputs. Source
symlinks are refused. Downloaded source snapshots
without their own Git checkout use their declared file inventory and rehash the
actual bytes; they do not inherit an enclosing repository's commit identity.

The generator also retains canonical effective compilation commands, selected
configuration/linker flags, compiler/build-tool/toolchain-file identities and
host information. Its configuration digest hashes the exact serialized JSON
bytes. The full compilation database has its own digest and accompanies the
configuration in packaged build evidence. The embedded configuration records
that digest instead of placing the entire database in one compiler string
literal. The [build-time CMake stamp](../../cmake/GalataProvenance.cmake) passes
the source/configuration identities into run provenance.

[Local packaging](../../scripts/package-release.py) remains available for dirty
developer source without implying CI approval. It rebuilds before snapshotting,
requires the copied source inventory to match the built source digest, and
checks that configuration/compilation evidence matches the compiled metadata.
Its staging directory is outside the checkout so creating an archive cannot
itself turn a clean release source dirty.

For this milestone, downloadable archives require static Galata libraries.
Packaging explicitly refuses an enabled BUILD_SHARED_LIBS configuration. CMake
library consumers may still build/install shared libraries; a future archive
bundling implementation needs its own loader/relocation verification. Existing
Windows dependency DLL staging remains part of the supported archive path.

Run manifests also retain an endpoint inventory of loaded modules. Readable
module files are hashed before and after execution; OS-managed shared-cache or
virtual images carry explicit unavailable-digest status and loader/OS identity.
The inventory protects readable runtime paths from study output replacement.
This is on-disk identity, not process-memory attestation or continuous monitoring.
On macOS, synchronized dyld callbacks retain copied module metadata. Because dyld
cannot unregister those callbacks, their owning shared library or bundle is
pinned for process lifetime after the first snapshot; embedding hosts must
account for that lifetime cost. Concurrent load/unload and callback-owner
lifetime have dedicated regression tests.

## Alternatives considered

**Query a green badge or a check name for the tag.** This is cheap, but a label
alone neither proves the full required graph ran nor binds its source. A robust
remote-query design would need trusted workflow identity, exact commit, required
job/results, freshness and artifact provenance checks. Reusing the gate directly
keeps that logic within one workflow run.

**Duplicate CI steps in the release workflow.** It avoids another workflow
call, but the previous failure was precisely a release graph that omitted
checks. One reusable graph makes additions apply to releases and documentation.

**Only permit releases from already-green main-branch commits.** This can be a
valid repository policy but prevents legitimate manual tag verification and
maintenance releases. The explicit source parameter permits these while
requiring the same evidence.

**Treat a dirty flag and compiler version as sufficient provenance.** That
identifies a broad class of builds, not the local source or effective flags.
Canonical content inventories cost additional build-time hashing and provide
the missing distinction without prohibiting local development.

**Build a complete signed/reproducible supply-chain system immediately.** That
would also require controlled builders, signing-key custody, offline trust and
revocation, immutable hosting policy and a reproducibility campaign. The present
change closes the identified source/evidence gate and preserves inputs for that
future work; it does not claim those additional controls exist.

## Consequences and limits

Release and Pages workflows run the complete verification graph and therefore
cost more runner time. Their source identity is explicit even for manual
dispatch. Workflow definitions, scripts, runner images and repository access
remain part of the trusted build environment. A maintainer able to replace all
those controls can replace their evidence too.

The CI record is a workflow artifact, not an independently signed attestation.
Checksums establish content identity rather than authorship. Source and
configuration digests do not prove that a compiler emitted the claimed binary,
and recorded tool files are not a complete SDK/sysroot inventory. An independent
rebuild, signed distributions and qualified use-specific evidence remain
separate work.

Rechecking a tag immediately before upload detects observed movement; it cannot
atomically lock the remote tag and upload in one transaction. Protected or
immutable release/tag policies remain necessary if that hosting guarantee is
required. Those external settings were not verified or changed by this work.

Python is now a build-time prerequisite. The preset schema's actual minimum
CMake version is declared consistently in the project and presets. Generator
support currently requires an effective compilation database; the shipped
Ninja presets provide it.

Local tests exercise missing/skipped/wrong-source evidence, tag movement,
modified archive bytes, source changes, ignored generated files and effective
flag changes. Local YAML parsing cannot establish that GitHub accepts and runs
the entire graph on every hosted platform. Verify the first remote candidate
before calling this workflow operationally demonstrated.

## Revisit when

Revisit for shared-library archive support, a new required CI job/platform,
historical release backfills, different workflow hosting, supported generators
without compilation databases, independently reproduced builds, signed offline
distribution, or a customer's tool-qualification and evidence-retention needs.
