<!-- SPDX-License-Identifier: Apache-2.0 -->

# Quadrotor programme: requirements to evidence

The acceptance record for the thirteen work items RFC-0002's audit identified, and for the
three workflows that had to run through public pipeline capabilities before any of it counted
as delivered.

**This document is the delivery record.** It is committed here rather than kept in a run log or
an external page, because an acceptance record whose only copy lives outside the repository is
an acceptance record nobody can check against the tree it describes.

**No figure measured by a run appears below.** Charter rule 2: a number without a traceable
routine and a stated budget does not go in a document, and a number copied out of a run is a
number no later run can contradict. What appears instead is the identity of the revision, the
commands, the gtest names that gate each claim, and the SHA-256 of every retained output — a
digest being an identity rather than a measurement. The figures themselves are properties of
the retained evidence and of the tests, and are read from there.

---

## Status vocabulary

Four claims, deliberately kept apart. Conflating them is how a green badge comes to stand for
something nothing checked.

| Status | What it asserts |
|---|---|
| **Implemented** | The capability is registered with a closed input vocabulary and is reachable from a study file. Asserts nothing about accuracy. |
| **Locally verified** | The named tests pass in the tier stated, on the platform stated, at the revision stated. |
| **CI-verified** | Hosted CI ran the complete required graph — Charter gates, Format, Engine on linux-gcc / linux-clang / macos, Determinism fingerprints and tier 2, Clang static analyzer, ASan/UBSan — against a named head commit. An **empty check rollup is unverified**, not green; see ADR-0019. |
| **Merged** | Landed on `main`. |

Every capability introduced by this programme registers as **implemented, unvalidated** in the
capability registry. None is anchored to a published reference, for the reason RFC-0002's
acceptance section gives: no published source exists for a 650 mm quadrotor's coefficients, and
an honest "unvalidated" is worth more than a fabricated match.

---

## The stack under test

The combined stack is the merge of every constituent branch plus the integration work. It is a
different tree from any constituent branch: it carries two conflict resolutions that exist in
none of them, which is why ADR-0019 makes the integration head the merge gate rather than
treating a green stack as a green integration.

| | |
|---|---|
| Integration branch | `integration/programme-acceptance` |
| Base | `main` |
| Constituent pull requests | 15, listed per item below |
| Conflict resolutions | 2 — `include/galata/pipeline/artifacts.hpp` and `docs/adr/README.md` at the sampled-control merge; `tests/integration/test_quadrotor_workflow.cpp` at the held-out-validation merge. The last had been committed with conflict markers still in the file, compiled nowhere, and no constituent branch's CI could have caught it. |
| Platform of record | Darwin arm64, AppleClang, `CMAKE_BUILD_TYPE=Debug` |

The exact integration head SHA, tree hash, binary digest and build identification for each
acceptance run are recorded in the run's own `00-identity.txt` under the retained evidence
directory, and in the run manifests. They are not restated here: this document describes the
programme, and a revision typed into it goes stale on the next commit.

---

## Reproducing the acceptance runs

Everything below runs from a clean checkout of the integration head. `VCPKG_ROOT` must point at
a vcpkg checkout.

```bash
export VCPKG_ROOT=/path/to/vcpkg

# 1. Build and run the whole suite.
cmake --preset dev
cmake --build --preset dev
ctest --preset dev

# 2. The two cross-implementation checks, which SKIP without the fixture and are
#    therefore established by a configured local run and by nothing in CI.
#    The fixture is not committed: ADR-0007 routes it to a path plus regeneration
#    instructions, and RFC-0002 publishes the regeneration command.
cmake -S . -B build/fixture -DGALATA_SOUXMAR_FIXTURE_DIR=<fixture dir>
cmake --build build/fixture --target galata_validation_tests
./build/fixture/tests/validation/galata_validation_tests \
    --gtest_filter='*Souxmar*' --gtest_output=xml:<out>/souxmar-cross-check.xml

# 3. Workflow A — model, heterogeneous trim, linearisation, controller, sampled run.
galata run examples/quadrotor-sampled-control/study.yaml --output-dir <out>/A --overwrite

# 4. Workflow B — imported data, fit, model artifact, trim and analysis, held-out validation.
galata run examples/quadrotor-identification/study.yaml --output-dir <out>/B --overwrite

# 5. Workflow C — the requesting programme's own study file, byte-unchanged.
galata run <fixture dir>/study.yaml --output-dir <out>/C --overwrite

# 6. The gates, and the six generated artefacts CI diffs.
scripts/check-si-boundary.sh
scripts/check-doc-references.sh
scripts/check-version-consistency.sh
scripts/check-doc-links.sh
shellcheck --severity=warning scripts/*.sh
scripts/gen-verification.sh  <validation-report-binary> --check
scripts/gen-status-table.sh  <cli-binary> --check
scripts/gen-modal-map.sh     <modal-map-binary> --check
scripts/gen-report.sh        <report-data-binary> --check
python3 scripts/gen-report-page.py --check
python3 scripts/gen-social-preview.py --check
```

Formatting is checked at the version CI pins, which is not the version a package manager will
install today:

```bash
python -m pip install 'clang-format==20.1.8'
git ls-files '*.cpp' '*.hpp' '*.h' '*.cc' '*.c' '*.mm' | xargs clang-format --dry-run --Werror
```

`examples/quadrotor-identification/flight.csv` is committed so workflow B runs with no prior
step, and `examples/quadrotor-identification/make-record.yaml` is committed so the record is
regenerable rather than a table of numbers whose origin is a claim:

```bash
galata run examples/quadrotor-identification/make-record.yaml --overwrite
```

### Retained evidence

Each acceptance run writes an immutable `run-<SHA256>.json` manifest carrying input snapshots,
output digests and build provenance. Those manifests, the run outputs, the gtest XML from the
cross-checks and the full `ctest` log are retained **outside both repositories**, under a
dated evidence directory whose every file is digested in an `evidence-digests.txt` beside it.
They are build artefacts and are deliberately not committed: ADR-0007's reasoning about the
fixture applies to what a run of it produces, and a decimated copy in `tests/` would be the
same dataset in a directory whose loader happens not to ask for a citation header.

The digest is what makes each file citable. An uncommitted artefact at a stable path is not an
identification, and two runs citing that path can have read different files.

---

## The thirteen items

Each row's local evidence names the gtest suites that gate it, with the tier in brackets. Where
an item has two pull requests, the second is the ADR the first depends on.

| # | Item | Capabilities | PRs | Implemented | Locally verified | CI-verified | Merged |
|---|---|---|---|---|---|---|---|
| 1 | Trim a quadrotor whose rotors have different thrust coefficients | `trim.hover` | #18 | yes | yes | yes | no |
| 2 | Reconcile the reported mode count with the state count | `analyze.modes` | #19 | yes | yes | yes | no |
| 3 | Make the Hurwitz refusal name the cause a caller can act on | `analyze.margins`, `analyze.diskmargin` | #20 | yes | yes | yes | no |
| 4 | Integrate the nonlinear plant through a public capability | `sim.plant` | #22 | yes | yes | yes | no |
| 5 | Declared input histories, and wind that changes over time | `sim.plant`, `sim.linear` | #23 | yes | yes | **no** | no |
| 6 | A public two-way attitude-error chart map | ADR-0017, chart map | #25, #24 | yes | yes | **ADR only** | no |
| 7 | Execute a controller at a declared rate against the plant | `sim.sampled` | #26 | yes | yes | **no** | no |
| 8 | A measured record, and a CSV reader that refuses to guess | `data.import.csv` | #27 | yes | yes | yes | no |
| 9 | A pack whose voltage falls under the load it is carrying | `model.quadrotor`, `trim.hover` | #28 | yes | yes | yes | no |
| 10 | Read a PX4 ULog, and refuse what it does not contain | `data.import.ulog`, ADR-0016 | #29, #21 | yes | yes | **ADR only** | no |
| 11 | Bench maps by least squares, with honest uncertainty | `identify.static_fit` | #30 | yes | yes | **no** | no |
| 12 | Grey-box identification against the nonlinear plant | `identify.greybox`, `model.quadrotor.export`, ADR-0018 | #31 + integration | yes | yes | **no** | no |
| 13 | Held-out validation, and the identity that makes it one | `identify.validate`, `data.window` | #32 + integration | yes | yes | **no** | no |

The **CI-verified** column above is a per-pull-request reading, and for seven items it says
`no` or `ADR only` for one structural reason: their pull requests are stacked on other feature
branches, so `.github/workflows/ci.yml`'s base-branch filter meant the workflow never fired and
GitHub reported zero checks rather than a failure. ADR-0019 removed that filter.

### The combined stack is CI-verified; the individual stacked pull requests are not

These are two different claims and the table above answers only the second.

The integration pull request runs the complete required graph against the combined head, and
that head is a tree no constituent branch has: it carries the merge resolutions. Its run is
therefore the only hosted evidence about the combination, and after ADR-0019 it is also the
first hosted evidence of any kind covering `sim.sampled`, the public chart map, the ULog parser
and all three `identify.*` capabilities.

**The trigger fix does not reach the already-open stacks retroactively.** GitHub resolves a
`pull_request` trigger against the workflow file in the merge of head into base, so a stacked
pull request keeps firing — or not firing — according to whichever `ci.yml` its own base carries.
Measured after the fix landed on the integration branch, `#26`, `#29`, `#31` and `#32` still
reported zero checks. ADR-0019 records the three ways to close that and their costs; until one
of them happens, the seven remain individually unchecked.

**Where validation happens, decided and recorded.** On *each stack branch, at its own head*, so
no pull request can be reviewed against an empty rollup — and *additionally at one integration
head*, which is the merge gate, because a green stack is not a green integration. The
integration head merges once; the constituents close as merged-by-integration rather than being
merged individually, so the same commits are not applied twice and `main` never carries an
untested combination between the first merge and the last.

**And the filter cannot come back by accident.** `scripts/check-ci-coverage.sh` runs in the
Charter gates job and fails if a base-branch filter reappears under `pull_request:`, or if the
trigger is deleted. It is a gate that inspects the workflow file it runs from, which earns its
place because the regression it catches has no other symptom: every other gate here fails by
going red, and this one's failure mode is a tick column that is simply empty.

### Per-item local evidence

| # | Gtest suites (tier) | Outstanding limitation |
|---|---|---|
| 1 | `QuadrotorHoverTrim` (validation), `HoverTrim` (unit) | The four thrust coefficients in `examples/quadrotor-heterogeneous-rotors/quad-heterogeneous.yaml` are synthetic and its own header says so. No motor was run up. |
| 2 | `QuadrotorWorkflow` (integration) | Legibility only; no numerical behaviour changed. |
| 3 | `Margins` (unit) | The refusal is now legible. Frequency-domain margins on a hover-linearised multirotor remain unavailable — see the outstanding-requirements section. |
| 4 | `QuadrotorWorkflow` (integration), `Determinism` (determinism) | Closes the audit's most basic gap: the plant was previously reachable only from C++ inside this repository's own tests. |
| 5 | `InputSchedule` (unit), `QuadrotorWorkflow` (integration) | A discontinuity strictly inside an integration step is refused rather than rounded, which moves the event and says nothing about having done so. |
| 6 | `ChartMapping` (unit) | A green check on ADR-0017 is not a green check on the implementation; they were separate pull requests and only one ran. |
| 7 | `QuadrotorWorkflow` (integration), `ExampleQuadrotorSampledControl` (integration) | Executes a continuously-designed law at a discrete rate. That is not designing in discrete time, and it establishes nothing about the sampled loop's own robustness. |
| 8 | `CsvImport` (unit) | No measured file exists to import. Every record exercised is galata's own output. |
| 9 | `BatterySag` (validation), `QuadrotorBattery` (unit) | The resistive sag model is opt-in and its parameters are unmeasured. The shipped model file carries no battery block. |
| 10 | `UlogImport` (unit) | The fixture is generated by `tests/data/make_ulog_fixture.py` rather than committed. No PX4 log from the intended aircraft exists, and the independent check against `pyulog` has not been run. |
| 11 | `StaticFit` (unit) | No bench run exists. The reported valid input range is the range of a synthetic record. |
| 12 | `Greybox` (unit), `IdentifyWorkflow` (integration), `ExampleQuadrotorIdentification` (integration), `QuadrotorSerialisation` (unit), `Determinism` (determinism), `IdentificationRecovery` (validation) | See the identification-acceptance section for what recovery under declared noise does and does not establish, and the optimizer-status section for the five questions a result keeps apart. Nothing here is evidence about an aircraft. |
| 13 | `RecordSeparation` (unit), `Validate` (unit), `RecordWindow` (unit), `IdentifyWorkflow` (integration) | `VerifiedDisjoint` means demonstrated sample separation and **not** statistical independence. See below. |

---

## The three acceptance workflows

All three run through public pipeline capabilities only — no C++ test harness, and no library
call a study file cannot make.

### A — Model, heterogeneous trim, linearisation, controller, sampled nonlinear simulation

`model.quadrotor` → `trim.hover` → `linearize.extended` → `model.channels` → `synth.lqr` →
`sim.sampled` → `report.csv` + `report.markdown`. Eight stages, on a vehicle whose four rotors
differ, so the heterogeneous trim path is exercised end to end. The law is designed on the
16-state hover linearisation with the wind columns dropped — a law fed back on a disturbance it
cannot measure is not a law anybody can fly — and then flown against the nonlinear plant with
zero-order hold, a whole-period delay and per-rotor saturation.

Gated by `ExampleQuadrotorSampledControl.RunsEndToEnd` and
`ExampleQuadrotorSampledControl.TheLawDrivesTheDisplacementOut`, the second requiring the closed
loop to remove at least nine tenths of the displacement it started from. The sampled logic
itself — the law, the saturation and the delay line — is re-derived independently from the
recorded states by `QuadrotorWorkflow`, so a loop that sampled at the wrong instant or shifted
its delay by one tick fails there rather than passing on a plausible trajectory.

### B — Imported data, fitting, model artifact, trim and analysis, held-out validation

`model.quadrotor` → `data.import.csv` → `data.window` ×2 → `identify.greybox` →
`model.quadrotor.export` → `trim.hover` → `linearize.extended` → `analyze.modes` →
`identify.validate` ×2 → `report.markdown`. Twelve stages.

It is a self-test with the answer committed: `examples/quadrotor-identification/quad-truth.yaml`
generated the record, `examples/quadrotor-identification/quad-base.yaml` is the same model with
two parameters deliberately wrong, and the fit must recover the truth from the record alone.
The two validation stages run the same model with the same settings against two windows of one
file — so their digests are equal, and a classification that read the digests could not have
separated them.

Gated by `ExampleQuadrotorIdentification.RunsEndToEnd`,
`ExampleQuadrotorIdentification.RecoversTheParametersItsTruthModelDeclares`,
`ExampleQuadrotorIdentification.PreservesEveryParameterItWasNotAskedToFit` and
`ExampleQuadrotorIdentification.LabelsTheHeldOutWindowAndTheTrainingWindowDifferently`.

### C — Unchanged bridge, galata analysis and simulation, independent comparison

`model.linear.statespace` → `analyze.modes` → `sim.linear` ×3 → `report.csv` ×3 →
`report.markdown`. Nine stages, on the requesting programme's own study file, byte-unchanged.
The fixture's digests are recorded before and after every run and are unchanged: nothing was
mutated.

The independent comparison is
`QuadrotorHoverLinearisation.MatricesAgreeWithTheIndependentSouxmarExport` and
`QuadrotorCrossImplementation.ReproducesTheSouxmarOpenLoopTrajectory`, both in the `validation`
tier, both recording the fixture's SHA-256 as a test property so the XML identifies the bytes it
read rather than only the path. Both **skip** without `GALATA_SOUXMAR_FIXTURE_DIR`, which no
workflow sets, so they are established by a configured local run and by nothing in CI. A skip is
not a pass.

Agreement between two implementations is not validation, and the case registry records this
comparison as self-consistent. It is nevertheless the only check here that could catch a shared
mistake in galata's own reasoning about the attitude-error chart, because the other
implementation trims and linearises in its own frames with its own code and reaches these
conventions by an explicit similarity. One entry disagrees by design, and neither side changes:
the reference's tenth output is a down-position channel and galata's is an altitude channel,
which are different quantities under similar names. That row is excluded from the bulk
comparison and held by a two-sided check instead, per charter rule 3.

The comparison from the other side — the requesting programme's own gated integration test
against this binary — is run as part of the acceptance and its result retained with the rest.

---

## Held-out standing is separation, not independence

`identify.validate` classifies the relationship between the validation record and the record
the model was fitted to. The four values and what each does **and does not** assert:

| Value | Established by | Does not assert |
|---|---|---|
| `VerifiedDisjoint` | Two windows of one imported file over intervals that do not meet, confirmed by scanning the channels both records carry for a sample instant occurring in both. | **Statistical independence.** Two windows of one flight share the aircraft, the trim, the air mass, the sensor calibration and every unmodelled effect that persists across the cut. Disjoint samples bound what the fit *saw*; they say nothing about whether the two stretches are independent draws. |
| `CallerDeclared` | The study asserted it, and every check that could be made was made and did not contradict it. | Anything galata checked. It is the caller's claim, in the caller's name. |
| `Unknown` | Nothing. No claim was made, or the records share no channel by which one could be checked. | — |
| `NotHeldOut` | The records demonstrably share observations: the same bytes, the same file over overlapping intervals, or a sample instant found in both. Evaluated before every positive branch, so a caller cannot declare away a proof of overlap. | That the numbers are worthless — re-running a fit on its own record is a legitimate diagnostic. The label is what separates a diagnostic from a claim. |

A digest inequality establishes none of these. A digest identifies **bytes**; held-out is a
claim about **data**. A file reformatted, exported twice or written to a different number of
decimals has a different digest and the same observations in it; a segment copied between files
shares every sample it copied; two exports of one flight at two sample rates describe the same
seconds of the same aircraft. `RecordSeparation` in the `unit` tier is a set of record pairs
each of which a digest comparison classifies wrongly.

Exact sample comparison has its own blind spot, stated in the basis sentence every result
carries: it cannot see the same run rescaled or resampled into a second file.

---

## Identification acceptance

RFC-0002's WP4 verification asks for a synthetic self-test that recovers known parameters from a
simulated record **with declared noise**, to within the reported confidence intervals, and for a
deliberately unidentifiable parameter set refused with the reason stated. Both are held in the
`validation` tier by `IdentificationRecovery`, against records generated with a declared,
platform-independent pseudo-random sequence so the cases are reproducible under ADR-0004 rather
than merely repeatable on one machine.

The noiseless self-test in `examples/quadrotor-identification` is a separate and weaker claim.
It demonstrates that the machinery recovers a parameter it can *see* and that the plumbing from
import to export to trim is correct. It says nothing about behaviour against a sensor, and the
standard errors it reports are correct and useless in magnitude — they say the record
demonstrates no scatter, which is true and carries no information.

Real hardware data is required for none of these tests and remains absent for all
physical-validation purposes. There is no aircraft, no bench run and no PX4 log from the
intended vehicle.

---

## What an identification result says about itself

`GreyboxResult` used to carry a single `optimiser_finished` flag, assigned `true`
unconditionally. With a declared iteration count that is a field which can never be false, so
it read as a check that had passed while asserting only that the routine ran. It is removed
rather than documented, and the five questions it blurred are answered separately.

| Question | What answers it | What it does not mean |
|---|---|---|
| **Did it run?** | `iterations_declared` and `iterations_run` | Equal by construction today. Both are reported so the invariant is visible, and so a future stopping rule cannot quietly make them differ unseen. |
| **Why did it stop?** | `stop_reason`, one value | `DeclaredIterationsCompleted` is the only way out of the loop. A tolerance-based reason is not missing; ADR-0004 forbids it, because an optimiser that stops when it is close enough stops after a different number of steps on a different machine. |
| **Did the objective improve?** | `initial_objective` against `objective`, plus `objective_improved` and `accepted_steps` | A run whose objective never improved returned its own starting point. That is a finding, not a refusal, and it is in the capability's headline summary rather than in a field. |
| **Did it converge?** | `ConvergenceEvidence`: `‖Jᵀr‖∞` at the final point, the same measure scaled by each parameter's own declared bound span, the last accepted step's norm, and which iteration last improved anything | **There is deliberately no `converged` boolean.** First-order optimality is a matter of degree against a scale only the caller knows, so a flag would be this code making an engineering judgement it is not entitled to — and a caller would read it as one. |
| **Are the parameters identifiable?** | `jacobian_condition_number` against `identifiability_ratio` | A refusal, not a field: a fit failing this test does not return at all, so these figures describe a fit that passed it. |

None of the five is the sixth question — whether the fit is **acceptable** — and nothing in the
result answers it. A returned result means the routine ran.

The Jacobian behind the last two rows is **recomputed at the point being reported**. The one
the loop leaves behind is taken at the start of the last iteration, before its step is accepted
or rejected, so a condition number, a covariance or a gradient read off it describes a point
the caller is not being handed. `Greybox.*` and `IdentificationRecovery.*` hold both the
progress fields and the no-improvement case; the evidence file
`model.quadrotor.export` writes splits its former single `diagnostics:` block into
`execution:`, `objective_progress:`, `convergence_evidence:` and `identifiability:`, because
one block with them interleaved is how they get read as one verdict.

---

## Outstanding requirements

Separated by whether they are needed to substantiate what this programme delivered, or are
broader roadmap work outside it.

### In scope: needed to substantiate the delivered workflows

| Requirement | State |
|---|---|
| Controllability and observability analysis for a declared input and output set, so an exported model's defective integrator chains and any unobservable direction are reported rather than discovered from a failed synthesis | `analyze.gramians`, in the registry. Reports the reachable and observable subspace ranks with a declared relative tolerance, the directions that fall outside them by state name, and finite-horizon Gramians over a declared horizon. It **refuses** to report an infinite-horizon Gramian for a plant whose spectrum is not strictly stable, which the hover linearisation's six integrator eigenvalues make the ordinary case rather than the exception. |
| Usable frequency-domain margin analysis on the hover-linearised multirotor | `model.control_system` gains `use: single_loop` with a required `channel`, building the loop seen at one plant input with the other loops still **closed**. `analyze.margins`, `analyze.diskmargin` and `analyze.sensitivity` all work on it. The previously-recorded refusal was correct and nothing in the stability check was relaxed: breaking one channel of the MIMO return ratio leaves the other three *open*, that closure genuinely is not internally stable, and its Nyquist encirclement count means nothing. What was missing was the other reading, not a looser gate. |
| A statement of what the sampled controller workflow's margins are **not** | `sim.sampled`'s header, its capability summary and its report section all record that gain, phase and disk margins computed from the continuous linearisation describe the continuous loop, and that the sampled loop's own robustness is a separate question no capability here answers. No claim of sampled-loop robustness is made anywhere from a continuous-time margin. |

### Out of scope: F14 / F17 roadmap work

These are named so that their absence is not read as an omission from this programme.

| Requirement | Why it is outside |
|---|---|
| MIMO robustness from loop-at-a-time margins | Not a scope question but a mathematical one: a set of single-loop margins does not bound simultaneous variation, and each can be generous while a perturbation applied to two channels at once destabilises the loop. `analyze.diskmargin` answers the simultaneous question and `analyze.sensitivity`/`analyze.sigma` the MIMO peaks. Listed here so the absence of a single "MIMO margin" number is not read as an omission. |
| `c2d`-style discretisation of a continuous design, and a sampled DARE | RFC-0002's acceptance section scoped WP5 to *execution* at a declared rate under vehicle-neutral names, with the quadrotor as the acceptance case. Designing in discrete time is a different capability with its own contracts, and `docs/product/FEATURES.md` states an epic cannot pass acceptance while a required dependency is unresolved. F14 and F17 both carry unresolved dependencies no item here scopes. |
| Margins of the sampled loop itself | Requires the discrete-time machinery above, plus a decision about which of several inequivalent definitions a sampled margin means. Raised rather than absorbed. |
| Sensors, estimators and explicit delay models beyond the whole-period transport delay `sim.sampled` applies | `docs/ROADMAP.md`'s control-design row. |
| SITL/HIL interfaces, and independently measured hardware responses | `docs/ROADMAP.md`'s simulation-and-identification row. Blocked on hardware that does not exist. |

---

## What no part of this record claims

- **No physical validation.** No capability in this programme has been checked against a real
  aircraft, because there is no aircraft. Every record exercised is galata's own output, and a
  completed run is not a validation.
- **No merge.** The constituent pull requests and the integration pull request are open. ADR-0019
  records that a stack merges by merging the integration head once it is green, not by merging
  each constituent in turn, so that the same commits are not applied twice and `main` never
  carries an untested combination between the first merge and the last.
- **No claim that a fit is trustworthy.** A completed fit is not a validation, an identifiable
  parameter set is not an accurate one, and whether a residual is small enough for a use is an
  engineering judgement nothing in this repository makes. ADR-0008's three questions stay three
  questions.
