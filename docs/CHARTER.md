# Engineering charter

Nine rules. They are hard gates: a change that violates one does not merge. Each
names how it is enforced, because a rule with no enforcement is a preference.

### 1. CI exists before the feature

The first commit that adds source also adds `.github/workflows/ci.yml`. There is
never a moment where this repository has code and no CI. A commit that adds a
capability adds the job that gates it in the same commit.

*Enforced by:* review, and by the fact that `ci-ok` is the single required
status check, so a job that is added is automatically required.

### 2. Nothing is documented before it works

No aspirational README. Nothing that does not exist is written in the present
tense. If a capability is a stub, its own output says so and the documentation
says so. A documentation claim that CI does not verify is a bug.

The README's Status table is the contract. Sections describing intent are
labelled as intent, in the same breath.

*Enforced by:* review for the Status table, which is still maintained by hand;
and by generation for the capability table below it, which comes from the
registry the CLI dispatches through — `scripts/gen-status-table.sh --check` fails
CI if the committed copy disagrees, so that table cannot describe a capability
that does not exist.

### 3. One source of version truth

The `VERSION` file. The build system, the packaging metadata, the CLI
`--version` and the desktop About box all derive from it.

*Enforced by:* `scripts/check-version-consistency.sh`, which compares values
where values are comparable and mechanisms where they are not, and which names
unwired surfaces as skipped rather than passing silently. See
[ADR-0005](adr/0005-single-source-of-version.md).

### 4. Every reference in every document resolves

Every URL, including the clone URL in the README, which must be the real clone
URL. And every file path and test name a document names in backticks.

The second half was added after a correction pass found nine untrue claims
across seven ADRs with every gate green. Three of the nine were a rotted *name*:
a header path that had never existed, a test that did not, a directory list from
the gate's first day. A name is the one part of a documentation claim a machine
can check, so it is now checked. The other six were claims about behaviour, and
those still belong to review — the gate's own header says so, so its green tick
is not read as more than it is.

*Enforced by:* `scripts/check-doc-links.sh`, which fails on NXDOMAIN and on
HTTP 404/410 for absolute URLs, and on any relative link whose target file does
not exist; and `scripts/check-doc-references.sh`, which resolves backticked paths
and gtest names against the tree. A reference that is deliberately
forward-looking goes in `scripts/doc-references-allow.txt` with a justification,
which makes rule 2's "nothing is documented before it works" a reviewable list
rather than an assumption.

### 5. Strict SI internally

Metres, seconds, kilograms, newtons, newton-metres, radians, radians per second,
kelvin, pascals. Degrees, feet, knots and degrees Celsius exist only in the UI
layer and in file-format adapters, converted at the boundary by one documented
set of functions. No unit conversion anywhere in the numerical core. Every
public struct field carries its unit in a comment.

*Enforced by:* `scripts/check-si-boundary.sh`. See
[ADR-0003](adr/0003-strict-si-and-boundary-conversion.md).

### 6. Determinism is a tested property

Same input, same platform, same bits out. Fixed-step integrators. Fixed
iteration counts in root-finders, never tolerance-based early exit. Seeded
PRNGs, with the seed recorded in the output.

The guarantee holds under this project's own compiler flags and does not survive
`-ffast-math` or FMA contraction. Across platforms the guarantee is agreement to
a published bound rather than bit-identity, because platform math libraries do
not agree on transcendental functions in the last bits. Claiming otherwise would
be false, and the reasoning is written out in
[ADR-0004](adr/0004-determinism-policy.md).

*Enforced by:* `cmake/GalataDeterminism.cmake` for the flags; by the determinism
test tier and `determinism.yml` for the property, from the commit that adds the
first integrator.

### 7. Every physics and numerics source file carries a citation

Author, title, publication, year, for the model or algorithm it implements. Plus
a "what this is NOT" block naming the model's validity envelope and the
direction and magnitude of its known error.

A model without a stated error direction is a model whose user cannot tell
whether it is conservative.

*Enforced by:* review, and by the validation tier, which cannot be written for a
model whose source is unknown.

### 8. Reference values in tests come from published sources

Never from the implementation. A test that captures current output as its
expected value is a regression-lock, is labelled as one in its own name, and is
cross-referenced to a validated case.

Where no published value can be found for a case, `docs/VERIFICATION.md` marks
that case **unvalidated**. An honest "unvalidated" is worth more than a
fabricated match.

*Enforced by:* `load_reference`, which refuses a reference file with no citation
header, and by the registry reconciliation tests in
`tests/validation/test_case_registry.cpp` —
`EveryCaseClaimingAComparisonNamesAReferenceAndItsEvidence`,
`CapabilitiesClaimingValidationAreBackedByACase`,
`NotImplementedCasesDoNotNameACapabilityThatExists` and the evidence-registration
check, which together make the claim unfalsifiable by hand.

### 9. No number reaches the user without provenance

Every result carries which capability produced it, with what inputs, at what
version, from which build.

*Enforced by:* `galata::build_identification()` in every result header, and by
the result-type contract each capability is written against.
