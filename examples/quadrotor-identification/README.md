# Identifying two parameters, and then asking whether the result predicts anything

A fit's own residual is not evidence that a model predicts anything. It is
evidence that an optimiser found the best it could on the data it was given, and
enough free parameters drive it to zero on any record at all. This example runs
the whole path — import, split, fit, export, trim, linearise, score — and spends
its last two stages on the question the fit cannot answer about itself.

## Run it

```bash
galata run examples/quadrotor-identification/study.yaml
```

Twelve stages. `make-record.yaml` regenerates the record, and does not need to be
run first: `flight.csv` is committed.

## What it demonstrates

**A self-test with the answer in the tree.** `quad-truth.yaml` generated the
record. `quad-base.yaml` is that same model with two parameters deliberately
wrong — mass 9.4 percent light, pitch angular drag twice too high — and the fit
must recover the truth from the record alone. Both models are committed, so
"the fit converged" is not the claim being checked; "the fit recovered the right
numbers" is.

**Two parameters chosen because this record can separate them.** Mass sets the
scale of the vertical response; pitch angular drag sets the decay of the body
pitch rate. A record that pitches *and* climbs sees both, independently. Mass
together with a single rotor's thrust coefficient would **not** be separable
from a vertical manoeuvre — both scale the same acceleration — and
`identify.greybox` refuses such a pair rather than reporting a number for a
direction the data cannot see. Try it: the refusal names the condition number
and tells you to excite the parameter or hold it.

**The fit produces a model, not a table.** `identify.greybox` emits a
`quadrotor` artefact, so `trim.hover`, `linearize.extended` and `analyze.modes`
consume it with exactly the wiring a loaded model gets. `model.quadrotor.export`
then writes it as a model file the loader reads back, beside a required record of
where each of its numbers came from — including the full list of the twenty
parameters the fit did **not** touch, by name, so preservation is checkable
rather than promised. ADR-0018 records why the evidence file is not optional and
why the base model is never rewritten in place.

**The split is proven, not asserted.** `data.window` cuts one import into two
half-open intervals that keep the file's identity, so `identify.validate` can
compare their lineage and their observations and report **verified disjoint**.
This matters because the obvious alternative does not work: two files with
different SHA-256 digests may hold the same observations — a file reformatted,
exported twice, or a segment copied between files — and an inequality of hashes
is an inequality of *bytes*, not a statement about data. The four labels are

| Label | What it means |
|---|---|
| `verified disjoint` | Two windows of one file over intervals that do not meet, confirmed by scanning the shared channels. Proven. |
| `caller-declared` | The study said these are different data; every check that could be made was made and did not contradict it. The caller's claim, in the caller's name. |
| `unknown` | Nothing establishes it. No claim, or no shared channel to check one against. |
| `not held out` | The records demonstrably share observations. Still computed, still labelled. |

**The same call on the training data is kept in the study on purpose.**
`check_estimation` runs the identical model with identical settings against the
window it was fitted to. Its errors are excellent — which is exactly why the
label has to be there. It reads `not held out`, and the difference between that
stage and `check_heldout` is the difference between a diagnostic and a claim.

**The estimation digest is read, not typed.** `check_heldout` never states which
record trained the model: the fitted model carries that in its own provenance,
and `identify.validate` reads it from there. If a study types a digest that
disagrees with what the model records, the stage refuses and names both — the
model knows what it was fitted to, and a caller who disagrees with it is
describing a different run.

**The numbers are in the run, not in this file.** `identification.md` carries the
estimates with their standard errors and bounds, the sensitivity condition
number, both independence verdicts with the sentence that establishes each, and
the per-output errors, fit fractions and residual autocorrelations.
`quad-identified.provenance.yaml` carries the identity chain.

## What this is not

**Not evidence about any aircraft.** `flight.csv` is the truth model's own
output. Nothing was flown, nothing was measured, and no bench produced any
coefficient here.

**Not evidence about behaviour under noise.** The record is noiseless to
round-off, so the residuals are at the arithmetic floor and the reported standard
errors are correspondingly meaningless in magnitude — they say the record
demonstrates no scatter, which is true and useless. Real data carries noise,
unmodelled dynamics and a timebase that drifts. None of those is exercised here,
and a fit that behaves on this record has been shown to be *wired* correctly and
nothing more.

**Not a validation of the plant.** `analyze.modes` correctly declines
participation factors at hover, because the double-integrator chains are
defective; that is the right behaviour, not a defect. And a completed fit
followed by a passing held-out score is still not a statement that the model is
accurate enough for any particular use. That judgement is an engineering one and
nothing in this repository makes it.

## What checks this

`ExampleQuadrotorIdentification.RunsEndToEnd`,
`ExampleQuadrotorIdentification.RecoversTheParametersItsTruthModelDeclares`,
`ExampleQuadrotorIdentification.PreservesEveryParameterItWasNotAskedToFit` and
`ExampleQuadrotorIdentification.LabelsTheHeldOutWindowAndTheTrainingWindowDifferently`
in the `integration` tier run this exact study file. The first three read the
truth model back and compare; the last requires the two verdicts to differ, so a
regression that collapsed the classification to a digest comparison fails there.
