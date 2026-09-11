<!-- SPDX-License-Identifier: Apache-2.0 -->

# ADR-0018: A fitted model is a derived plant with its own identity, never an edit to its source

- **Status:** accepted
- **Date:** 2026-09-10
- **Deciders:** project maintainer, on the RFC-0002 WP4 delivery

## Context

`identify.greybox` estimates a declared subset of a multirotor's parameters from a
measured record. Until this decision it produced *numbers*: a vector of values whose
meaning was a parallel vector of path strings. Nothing downstream could consume that.
`trim.hover`, `linearize.extended` and `sim.plant` all take a plant, so the result of an
identification could not be trimmed, linearised or flown — which is the entire point of
identifying it. RFC-0002's WP4 asks for a fit; the workflow it asks the fit *for* is
model → measure → identify → design → simulate, and that workflow was cut at the third step.

Three things make this a decision rather than an obvious step.

**A fitted model file is indistinguishable from a measured one.** `model::serialize_quadrotor`
writes the six-root-key YAML that `model::parse_quadrotor` reads, and that format has no place
to say where its numbers came from. A file whose `thrust_coefficient_n_s2` was measured on a
thrust stand and a file whose value an optimiser reached are the same document. This
repository's existing answer for hand-written models is a `PROVENANCE.md` beside the YAML in
every directory under `models/`; the question is what the equivalent is for a file a tool
writes, and whether it is optional.

**The source model must survive.** A fit reads a base model and produces a different one. If
the base file were rewritten in place, the run would have destroyed its own input: the record
of what was fitted *from* would be the thing that was fitted *to*, the estimation could not be
repeated, and a second fit would start from the first fit's answer without saying so.

**A fitted plant is still a plant.** Every consumer wants a `model::Quadrotor` and does not
care how it came to exist — until one does. `identify.validate` cares intensely: it must know
which record trained the model, and a caller retyping that digest into the study beside the
model can retype it wrong. So the provenance has to travel *with* the model rather than beside
it in a file nothing reads back.

ADR-0011 already separates authoritative source from derived objects and requires each to
carry an explicit identity. ADR-0008 requires evidence to keep "completed", "numerically
accurate" and "engineering accepted" separable. ADR-0003 makes the key carry the unit. This
decision is those three rules applied to a new kind of derived object, not a new principle.

## Decision

**A grey-box fit produces a `quadrotor` artefact — the same kind `model.quadrotor` produces —
whose payload is `pipeline::QuadrotorArtifact{model, identity}`, and whose `identity.origin`
is `"fit"` rather than `"file"`. The base model is neither modified nor replaced. The fitted
plant is exported durably by `model.quadrotor.export`, which writes the model YAML and a
required evidence file recording where every number in it came from.**

Six parts, each load-bearing.

**1. One artefact kind, not two.** The fit's product is `kind = "quadrotor"`. A fitted plant
*is* a plant: `trim.hover`, `sim.plant` and `sim.sampled` read it through the accessor they
already had, and `linearize.extended` reaches it through `HoverTrimArtifact` as before. A
study replaces `{from: plant}` with `{from: fit}` and nothing else. Whether a model was fitted
is therefore a **field** — `identity.is_fitted()` — and not a **type**, which is what a
downstream stage actually wants to interrogate: `identify.validate` asks, and every other
stage never has to.

**2. Every model artefact carries its identity, fitted or not.** `ModelIdentity` holds a
closed two-value `origin`, the declared path and the SHA-256 of the model file's **bytes** for
a loaded model, and a `shared_ptr<const FittedModelProvenance>` engaged exactly when the
origin is `"fit"`. The digest is of the bytes and never of the path, for the reason
`data::Record` gives about its own source. `trim.hover` forwards the identity into
`HoverTrimArtifact::model_identity`, so a chain that trims and linearises has not lost track
of what it did that to.

**3. Unfitted parameters are preserved structurally and enumerated explicitly.**
`identify::fit_greybox` builds its returned plant as a whole copy of the base and writes only
the declared paths into it, through the same `resolve` the objective used — so there is no
second path-resolution implementation for a parameter to land in the wrong place through. On
top of that, `FittedModelProvenance::preserved_parameter_paths` lists every parameter the fit
did **not** touch, in full, rather than leaving it as "everything else". A reader asking which
numbers in an exported file are estimates and which are inherited must be able to answer that
from the record alone, without diffing two YAML files.

**4. The identity chain is what the provenance carries.** Base-model path, digest and
description; estimation-record path, digest, sample count, covered interval and whether it is
a window; every fitted parameter with its unit, bounds, starting value, estimate, standard
error and whether it rests on a bound; the objective in words; the output matches and their
scales; and the optimiser diagnostics. The unit is read off the parameter path's own suffix
rather than declared a second time, because ADR-0003 already put it there and a study
restating it could disagree with the model. A path whose unit is not known is **refused**: a
coefficient reaching a provenance record without a unit is exactly the failure ADR-0003
exists to stop.

**5. The evidence file is required, for a fitted model and a loaded one alike.**
`model.quadrotor.export` declares `path` and `evidence_path` as output file roles and neither
is optional. A uniform rule has no branch for a caller to take, and "no parameter was fitted;
these are the bytes it came from" is itself worth recording. The two paths may not name the
same file. The model YAML additionally carries its own account in the two free-text fields the
format does have: `identify::fit_greybox` sets `description` and `citation` on the fitted plant
at the moment it comes into existence, naming the record, its digest, the parameters fitted,
and that a fitted model is not measured aircraft data. A library caller who wants different
words can overwrite them; one who forgets does not thereby publish a fitted model wearing its
base model's description.

**6. Not overwriting the source is enforced by the executor, not promised by the exporter.**
`model.quadrotor` declares `path` as an input file role, so the base model's bytes are
recorded in `RunFiles::inputs_` during preflight, before any output is reserved.
`RunFiles::check_input_collision` then refuses any output that resolves to a recorded run
input — `an output may not overwrite a run input or executable` — for every capability, not
just this one. The guarantee is a property of the run rather than of the capability's good
behaviour, and it is held by a test.

## Alternatives considered

**A separate `fitted_quadrotor` artefact kind.** Its honest best case is type safety: a stage
that must not silently accept a fitted plant cannot, because the kinds differ, and the error
names both. It also leaves `model.quadrotor`'s payload alone. Rejected because every consumer
then grows either a `payload_as` branch or a shared `quadrotor_of(const Artifact&)` accessor,
and each such site is one more place a study can wire the wrong plant and get an answer that
looks legible. The safety it buys is against a mistake nobody makes — nothing in the tree
wants to reject a fitted plant — and the cost is paid at four consumers to protect against it.
Making the fit's presence a field puts the check where the one interested stage can make it.

**Provenance in the model file itself, under a `provenance:` key.** Best case: one file, and
the account can never be separated from the numbers, which is the failure mode the sidecar
does not prevent. Rejected because the format's root map is closed to six keys and
deliberately so — `io::yaml_keys` refuses everything else at every level, and that closure is
what makes a typo in a model file an error rather than a silently wrong plant. Opening the root
to a metadata block would mean either loosening that check or teaching the loader to parse and
ignore a section, and a loader that ignores a section is a loader that will one day ignore a
misspelt one. The closure is worth more than the co-location.

**A generated `PROVENANCE.md`, matching the `models/` directory convention.** Best case: it is
literally the existing convention, and a human reading `models/souxmar-quad/` would find the
same shape beside a fitted model. Rejected because a `PROVENANCE.md` in this repository is a
*hand-authored* document about rights position, transcription method and the choices a human
made that the source did not — and a generated file in that shape would be read as one. The
evidence file is machine-written and machine-readable, says so in its own header, and is
explicitly not a study input. The two documents answer different questions and confusing them
would devalue the hand-written one.

**Making the fit write the model file itself.** Best case: one stage, no chance of a fit whose
result was never durably recorded. Rejected because it welds a numerical operation to a file
write, which no other capability in the registry does — `linearize.extended` computes and
`model.linear.export` writes, and a study that wants only the in-memory result should not be
made to name an output path for it. It would also make a fit-then-refit chain write an
intermediate file nobody wanted.

**Carrying a digest of the fitted parameter set, canonicalised through
`serialize_quadrotor`.** Best case: two studies could prove they hold the same fitted plant.
Rejected as premature: it makes every future change to the serialiser — reordering a key,
emitting a defaulted value it currently omits — a silent invalidation of every digest ever
recorded, and nothing in the tree yet needs to compare two fitted models. The base-model and
estimation-record digests together already identify the fit's inputs, which is what the
identity chain is for.

## Consequences

**Easier.** A fit is consumable: `identify.greybox` → `trim.hover` → `linearize.extended` →
`analyze.*` and `sim.plant` is a chain with no adapter in it, exercised end to end by
`examples/quadrotor-identification/`. `identify.validate` reads the estimation record's digest
from the model's own provenance rather than from a string the study typed, and refuses a study
whose typed digest disagrees with what the model records — the class of error where a
validation is run against the wrong training-data identity is now unreachable rather than
merely unlikely. Every `quadrotor` artefact now knows its own file digest, which the run
manifest knew and no capability could reach.

**Harder.** Exporting a model costs two output paths instead of one, and a study that wants
only the YAML must still name the evidence file. Adding a fittable parameter to
`identify::Parameter`'s path grammar now requires adding its unit to `unit_of_parameter` and
its path to `every_parameter_path`, or the first study that fits it fails — deliberately, but
it is a second and third place to edit. `QuadrotorArtifact` is a wrapper where a bare
`model::Quadrotor` used to be, so a future capability consuming a plant has one more
indirection to write.

**Expensive to reverse.** The `quadrotor` artefact payload type is now `QuadrotorArtifact`, and
every producer and consumer of that kind depends on it; undoing this means touching
`model.quadrotor`, `model.quadrotor.export`, `identify.greybox`, `trim.hover`, `sim.plant` and
`HoverTrimArtifact` together. The exported model file format is unchanged and is *not*
expensive to reverse — it is the same format it always was. The evidence file's shape is
cheap to change because nothing reads it back, and its header says so.

**What this decision does not buy.** A fitted model file separated from its evidence file is a
plant with no mark on it that its numbers were estimated, beyond two free-text fields a
determined copy can strip. The generated `description` and `citation` are mitigations and not
guarantees, and this ADR says so rather than implying the evidence file is load-bearing when it
is only adjacent. Nor does any of this make a fit trustworthy: a completed fit is not a
validation, an identifiable parameter set is not an accurate one, and whether a residual is
small enough for a use is an engineering judgement nothing here makes. ADR-0008's three
questions stay three questions.

## Revisit when

A second model class becomes fittable. `ModelIdentity` and `QuadrotorArtifact` are named for
the multirotor because it is the only plant `identify.greybox` fits; a fixed-wing fit would
want the same identity on a `model::Aircraft` artefact, and at that point the wrapper and the
provenance should be generic over the model type rather than duplicated. Also revisit if a
capability ever needs to *read* the evidence file, since that would make its shape a contract
rather than a report.
