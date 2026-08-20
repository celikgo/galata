<!-- SPDX-License-Identifier: Apache-2.0 -->

## What this changes, and why

<!-- One paragraph. If it changes a published number, say so in the first line. -->

## Evidence

<!--
Charter rule 1: a capability arrives with its CI gate and its tests in the same
commit. Delete the rows that do not apply, but do not delete the table.
-->

| | |
|---|---|
| Tests added or changed | |
| What they would catch that nothing else does | |
| Published numbers this moves (`docs/VERIFICATION.md`, README, release notes) | none |

## Checklist

- [ ] `cmake --build --preset dev && ctest --preset dev` passes locally.
- [ ] `ctest --preset dev -L validation` passes — the tier that compares against published sources.
- [ ] Formatting is clean (`clang-format`), per CONTRIBUTING.
- [ ] `docs/VERIFICATION.md` is **regenerated, not edited by hand**. It is produced by
      `tools/validation/report_main.cpp` and CI diffs it, so a hand edit fails the build.
- [ ] Any new reference value carries its document, edition, page or table, rights
      position and transcription method, per ADR-0007 — and lives in
      `tests/validation/reference/`, not in a test body.
- [ ] Units at the boundary follow ADR-0003: SI inside, conversion at the edge.
- [ ] No capability is documented that does not work, and no capability that works is undocumented.

## Anything you are unsure about

<!--
Say it here rather than leaving it to be found. A stated uncertainty is cheaper
to review than a confident claim that turns out to be wrong — which is the same
reason docs/VERIFICATION.md publishes its own open discrepancies.
-->
