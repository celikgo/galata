# ADR-0007: Reference values quoted from copyrighted sources

- **Status:** accepted
- **Date:** 2026-08-18
- **Deciders:** project owner

## Context

galata's data policy is strict and deliberately so: coefficient data ships
in-tree only when it is transcribed from a US Government work and therefore in
the public domain. Anything traceable only to a copyrighted source ships as a
loader plus fetch instructions, never as data, and a dataset whose licensing
cannot be established does not ship at all. That rule is what lets a stranger
clone this repository without inheriting somebody else's licence problem.

Charter rule 8 pulls in a different direction. Test reference values must come
from published sources, never from the implementation. The whole point is that
a validation number has an author who is not us.

These meet head-on the first time a numerical method's authoritative reference
is a copyrighted paper. The disk margin is exactly that case: its definition and
its only published worked example are in Seiler, Packard and Gahinet, *An
Introduction to Disk Margins*, IEEE Control Systems Magazine 2020 — an IEEE
publication, not a government work. Under a literal reading of the data policy
the eleven numbers in that paper's worked example could not be committed, and
`analyze.diskmargin` could then never be validated against anything but itself.

## Decision

Distinguish shipping a DATASET from quoting a RESULT.

A dataset — a table whose extent and organisation is the value of the source
work, such as an aircraft's aerodynamic coefficients across a flight envelope —
remains subject to the existing rule: US Government primary sources only.

A small number of scalar results from a worked example, quoted to verify an
independent implementation, may be committed to `tests/validation/reference/`
regardless of the source's copyright, subject to all of the following:

1. The source is cited in full in the file header, including DOI and a
   resolving URL, and the exact location of each value inside the document is
   recorded per value.
2. No part of the source document is reproduced — not its text, not its
   figures, not its derivations. Only the numeric results, and only those
   actually used as gates.
3. The header states plainly that the source is not a US Government work and
   is not public domain.
4. Every value is independently reproduced from the method's stated definition
   before being committed, and the file records that it was. The table is then
   evidence of agreement between two parties, not a copy of one of them.
5. The quantity of material is bounded by the verification purpose. If a test
   would need enough of a source that the transcription starts to substitute
   for the source, the rule has been exceeded and the data does not ship.

## Alternatives considered

**Refuse all copyrighted reference values.** The honest case for this is that it
needs no judgement call and no line to defend: one rule, uniformly applied, and
nobody has to assess whether a given transcription is de minimis. The cost is
that it makes several capabilities permanently unvalidatable, because the
authoritative treatments of disk margins, Riccati benchmarks and
structured singular values are journal publications and there is no government
substitute. Shipping an unvalidated robustness margin is a worse outcome for a
user than shipping a validated one with a citation.

**Ship the loader, fetch the paper at test time.** This is the pattern already
used for restricted datasets and it is consistent, which is its real merit. It
fails here for a practical reason: the values are prose in a PDF, not a
downloadable table. A test would have to parse a paywalled PDF, and CI would
depend on a publisher's availability and bot policy. `check-doc-links.sh`
already treats IEEE Xplore as unreachable to automated clients.

**Recompute the values ourselves and cite only the method.** Attractive, and
partly done — every value here was reproduced independently. But if the
committed number is ours, charter rule 8 is broken: the test then compares the
implementation against itself, and the agreement it reports is vacuous.

## Consequences

Validation can proceed for methods whose literature is commercial, which is
most of modern robust control. Each such file now carries a heavier header, and
a reviewer has a specific checklist to apply rather than a judgement to make.

The line requires judgement at the margin, which the previous rule did not.
Condition 5 is the one that will be argued about; it is stated as a purpose test
rather than a count because a count would be arbitrary.

Reversing this means deleting the affected reference files and demoting the
capabilities they validate to `ImplementedUnvalidated` in the pipeline registry
and to `Unvalidated` in the case registry — a mechanical change, since the
registry gates already refuse a capability that claims validation without a
backing case.

## Revisit when

A rights holder objects; or a US Government or openly licensed source publishes
equivalent worked examples, in which case prefer it and delete the quoted values;
or a reference file starts to look like a transcription of a document rather
than a handful of gates, which is condition 5 being breached.
