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
#    The toolchain file is required for a from-scratch configure: without it,
#    cmake/GalataInstall.cmake fails on the dependency notice file. The presets
#    supply it, so a command that omits it appears to work in a directory the
#    preset already configured and fails in a fresh one.
cmake -S . -B build/fixture \
    -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DGALATA_SOUXMAR_FIXTURE_DIR=<fixture dir>
cmake --build build/fixture --target galata_validation_tests
./build/fixture/tests/validation/galata_validation_tests \
    --gtest_filter='*Souxmar*' --gtest_output=xml:<out>/souxmar-cross-check.xml

# 3. Workflow A — model, heterogeneous trim, linearisation, controller, sampled run.
galata run examples/quadrotor-sampled-control/study.yaml --output-dir <out>/A --overwrite

# 4. Workflow B — imported data, fit, model artifact, trim and analysis, held-out validation.
galata run examples/quadrotor-identification/study.yaml --output-dir <out>/B --overwrite

# 5. Workflow C — the requesting programme's own study file, byte-unchanged.
galata run <fixture dir>/study.yaml --output-dir <out>/C --overwrite

# 5b. F14 — a discrete design on the native Souxmar plant, flown against the nonlinear model.
galata run examples/souxmar-sampled-lqr/study.yaml --output-dir <out>/D --overwrite

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

The independent ULog comparison needs pyulog, which is a test-time tool and not a dependency of
anything this repository builds or ships:

```bash
python3 -m venv /tmp/pyulog && /tmp/pyulog/bin/pip install pyulog
scripts/compare-ulog-against-pyulog.py --galata <cli-binary> --python /tmp/pyulog/bin/python
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
Measured again after the fix landed on the integration branch, and reported in full rather than
as a sample: of the fifteen constituent pull requests, **eight carry all eleven checks** — `#18`,
`#19`, `#20`, `#21`, `#22`, `#24`, `#27`, `#28` — and **seven report zero**: `#23`, `#25`, `#26`,
`#29`, `#30`, `#31`, `#32`. An empty rollup is unverified, not green. ADR-0019 records the three
ways to close that and their costs; until one of them happens, those seven are covered by the
integration head's run and by nothing of their own.

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

### Per-item CI runs

Durable references. A run URL outlives the branch it tested, which a commit SHA on a
squash-merged branch does not — see the note on citing commits in `CONTRIBUTING.md`. The seven
items absent from this table have no hosted run of their own to cite; the integration pull
request's run is the only hosted evidence covering their code.

| # | PR | Head tested | Result | Run |
|---|---|---|---|---|
| 1 | #18 | `0eb3755` | 11 of 11 | [34452200072](https://github.com/celikgo/galata/actions/runs/34452200072) |
| 2 | #19 | `8dc8aaf` | 11 of 11 | [34453772771](https://github.com/celikgo/galata/actions/runs/34453772771) |
| 3 | #20 | `72ab99a` | 11 of 11 | [34459197530](https://github.com/celikgo/galata/actions/runs/34459197530) |
| 4 | #22 | `275dcc8` | 11 of 11 | [34462187161](https://github.com/celikgo/galata/actions/runs/34462187161) |
| 6 | #24 (ADR only) | `bda5fa7` | 11 of 11 | [34465297923](https://github.com/celikgo/galata/actions/runs/34465297923) |
| 8 | #27 | `5e50f22` | 11 of 11 | [34469244732](https://github.com/celikgo/galata/actions/runs/34469244732) |
| 9 | #28 | `48f131b` | 11 of 11 | [34471404322](https://github.com/celikgo/galata/actions/runs/34471404322) |
| 10 | #21 (ADR only) | `ea0299f` | 11 of 11 | [34459437564](https://github.com/celikgo/galata/actions/runs/34459437564) |

The integration pull request's own runs are on [#33](https://github.com/celikgo/galata/pull/33).
Note that its `concurrency` group cancels a run when a newer commit arrives, so only the run
against the final head is meaningful; an earlier head's partial result is not evidence about
the head that superseded it.

### The integration head's own result, and the sanitizer

The combined stack is **CI-verified** at head `ff8c0e37400e344f2703afb20efa4b49ae63ab76`:
run [34521117651](https://github.com/celikgo/galata/actions/runs/34521117651), **11 of 11 jobs
green**, `637/637` tests under the sanitizer with `Total Test time (real) = 6400.24 sec`. That
is the whole required graph — Charter gates, Format, Engine on linux-gcc / linux-clang / macos,
both Determinism fingerprints, Determinism tier 2, the Clang static analyzer, ASan/UBSan, and
the `CI` aggregate.

**The sanitizer failed first, and why is worth recording.** At the previous head, run
[34506763806](https://github.com/celikgo/galata/actions/runs/34506763806) failed — not on a
sanitizer diagnostic. ASan configure and build both succeeded and there was no leak, undefined
behaviour report or crash. **Fifteen** `galata project ...` invocations lost a 45-second
Python-level deadline at once across all four project suites, with `ProjectLinearImport` losing
its very first call in `setUpClass` and running zero tests.

That deadline is a hang guard, and 45 s was never a sound figure for the instrumented build; it
survived only while the CLI stayed small enough. `tests/CMakeLists.txt` already documented the
mechanism one level up — the worker SHA-256s its own executable once per run, so cost scales
with the **size of the CLI** and not with what the test exercises — and this branch adds
capabilities to that binary. The deadline was scaled by the measured instrumentation factor
rather than raised until the suite went green, and only for the sanitizer build; the
uninstrumented default is untouched at 45 s. The full derivation is committed beside the code.

**The hosted run then confirmed the mechanism rather than merely clearing the gate.** Against
the same four suites at `48f131b`:

| Suite | `48f131b` | `ff8c0e3` | Growth |
|---|---|---|---|
| `ProjectWorkflow` | 653.43 s | 762.54 s | 16.7% |
| `ProjectLinearImport` | 157.76 s | 183.34 s | 16.2% |
| `ProjectRecovery` | 158.60 s | 184.76 s | 16.5% |
| `ProjectRoutes` | 125.08 s | 145.49 s | 16.3% |

against an instrumented binary measured 16.8% larger. Four suites of quite different content,
all growing within half a point of the size of the executable. **This is a wall-clock guard and
not an error budget on a computed quantity** — no numerical tolerance moved, no comparison
loosened, and no test changed what it asserts.

### Per-item local evidence

| # | Gtest suites (tier) | Outstanding limitation |
|---|---|---|
| 1 | `QuadrotorHoverTrim` (validation), `HoverTrim` (unit) | The four thrust coefficients in `examples/quadrotor-heterogeneous-rotors/quad-heterogeneous.yaml` are synthetic and its own header says so. No motor was run up. |
| 2 | `QuadrotorWorkflow` (integration) | Legibility only; no numerical behaviour changed. |
| 3 | `Margins` (unit) | The refusal is now legible. Frequency-domain margins on a hover-linearised multirotor remain unavailable — see the outstanding-requirements section. |
| 4 | `QuadrotorWorkflow` (integration), `Determinism` (determinism) | Closes the audit's most basic gap: the plant was previously reachable only from C++ inside this repository's own tests. |
| 5 | `InputSchedule` (unit), `QuadrotorWorkflow` (integration) | A discontinuity strictly inside an integration step is refused rather than rounded, which moves the event and says nothing about having done so. |
| 6 | `ChartMapping` (unit) | A green check on ADR-0017 is not a green check on the implementation; they were separate pull requests and only one ran. |
| 7 | `QuadrotorWorkflow` (integration), `ExampleQuadrotorSampledControl` (integration) | Executes a continuously-designed law at a discrete rate. That is not designing in discrete time, and it establishes nothing about the sampled loop's own robustness. Designing in discrete time is delivered separately — see the F14 section — and it establishes no sampled-loop margin either. |
| 8 | `CsvImport` (unit) | No measured file exists to import. Every record exercised is galata's own output. |
| 9 | `BatterySag` (validation), `QuadrotorBattery` (unit) | The resistive sag model is opt-in and its parameters are unmeasured. The shipped model file carries no battery block. |
| 10 | `UlogImport` (unit), plus the independent `pyulog` comparison below | The fixture is generated by `tests/data/make_ulog_fixture.py` rather than committed. **No PX4 log from any aircraft has been read by either implementation** — a limitation separate from the parser comparison, and it stays open. |
| 11 | `StaticFit` (unit), `IdentifyWorkflow.ABenchMap*` (integration) | No bench run exists. The reported valid range is the range of a synthetic record — and it is the fitted CHANNEL's span in the channel's own unit, not the span of the design column the term becomes; the fields were named `regressor_*` until 2026-09-10, which invited a reader to be wrong by a square. |
| 12 | `Greybox` (unit), `IdentifyWorkflow` (integration), `ExampleQuadrotorIdentification` (integration), `QuadrotorSerialisation` (unit), `Determinism` (determinism), `IdentificationRecovery` (validation) | See the identification-acceptance section for what recovery under declared noise does and does not establish, and the optimizer-status section for the five questions a result keeps apart. Nothing here is evidence about an aircraft. |
| 13 | `RecordSeparation` (unit), `Validate` (unit), `RecordWindow` (unit), `IdentifyWorkflow` (integration) | `VerifiedDisjoint` means demonstrated sample separation and **not** statistical independence. See below. |

---

## F14 — discrete models and sampled LQR, through the public interface

The objection this closes: F14's library half — exact discretisation, a bounded DARE and sampled
LQR — was reachable from C++ and from no study file. It is now registered and consumed by
`sim.sampled`. A shipped example on the native Souxmar plant runs it end to end.

### Where it lives, and what hosted evidence covers it

F14 has **no constituent pull request**. Its commits exist only on the integration branch:

- the library half, [f7fb895](https://github.com/celikgo/galata/commit/f7fb89559a8019e574741f939249af43e22fb9ec);
- the pipeline wiring and example that follow it.

So there is no per-pull-request reading of F14 at all. That is not an empty rollup; there is
simply no pull request. The integration pull request's run at the final head is the only
hosted evidence that can cover F14.

That run does not change the standing of the constituent pull requests, and the distinction
drawn above holds exactly as stated:

- The **combined stack** is CI-verified at a named head.
- The seven stacked constituents — `#23`, `#25`, `#26`, `#29`, `#30`, `#31` and `#32` — remain
  **individually unchecked**, each with an empty rollup.

A green integration head covers their code as merged into the combination. It is not a run of
any of them.

### Hosted results by revision

Each result covers the tree at the revision named, and nothing after it. The `concurrency` group
cancels a superseded run, so a result is recorded only for a head whose run completed.

| Revision | What that tree adds | Run | Result | Covers F14 |
|---|---|---|---|---|
| [ff8c0e3](https://github.com/celikgo/galata/commit/ff8c0e37400e344f2703afb20efa4b49ae63ab76) | The thirteen items, and the sanitizer deadline scaled for the instrumented build | [34521117651](https://github.com/celikgo/galata/actions/runs/34521117651) | 11 of 11 | **no** — it predates both F14 commits |
| [f7fb895](https://github.com/celikgo/galata/commit/f7fb89559a8019e574741f939249af43e22fb9ec) | F14's library half | none of its own | — | only through the row below |
| The head carrying the pipeline wiring | F14's public interface, the example, and this section | pending when this was written; recorded in the commit that follows it | — | yes |

### Requirements

| Requirement | Delivered by | Held by | State |
|---|---|---|---|
| **1. Register `model.discretize`, `synth.dare` and `synth.sampled_lqr` with strict schemas, explicit time-domain types and complete evidence propagation** | Three capabilities, each with a closed input vocabulary. Continuous and discrete models are different artefact kinds — `linear_system` and `discrete_linear_system` — and `dare_solution` and `sampled_control_law` sit beside them, so every existing continuous capability refuses a discrete artefact by kind. `model.discretize` is the only adapter, and it requires `sample_time_s` and `hold`. `synth.sampled_lqr` requires an `evidence_path`. The executor carries an upstream linearisation's record through discretisation and design as the same object. | `DiscreteWorkflow.EachTimeDomainRefusesTheOtherByName`, `DiscreteWorkflow.TheSampleTimeAndTheHoldAreDeclaredAndNeverDefaulted`, `DiscreteWorkflow.UnknownKeysAreRefusedBeforeAnythingRuns`, `DiscreteWorkflow.LinearisationEvidenceSurvivesDiscretisationAndDesign` | implemented, unvalidated; locally verified |
| **2. The sampled controller can be consumed by `sim.sampled`, with its design period and execution period consistent** | `sim.sampled` accepts a `sampled_control_law` and executes it only at the period it was designed for. **No transformation between rates is supported**, so every mismatch, faster or slower, is refused. The refusal names the remedy: a redesign at the execution period. A discrete design must declare its hold and its delay. For either kind of law, a gain is refused if its inputs are not the rotor commands in the model's order, or if its chart is not the vehicle's. | `QuadrotorWorkflow.ADiscreteLawIsRefusedAtAPeriodItWasNotDesignedFor`, `QuadrotorWorkflow.ADiscreteLawMustDeclareItsHoldAndItsDelay`, `QuadrotorWorkflow.TheDiscreteLawIsExecutedAsDesigned`, `QuadrotorWorkflow.ALawWhoseInputsAreNotTheRotorCommandsInOrderIsRefused` | implemented; locally verified |
| **3. The discrete cost's state-input cross term is retained through synthesis and the exported evidence, and the conventions are documented** | The hold's cross term goes to the solver, the artefact, the report section and the required evidence file. The following are stated in the report section, the evidence file, `include/galata/synth/discrete_control.hpp` and RFC-0002's WP5 addendum: the cost convention; the gain convention `u[k] = -K x[k]` with `K = (R + B'XB)^-1 (B'XA + N')`; the residual of the equation as posed; and the unit-circle and symplectic-separation checks. | `DiscreteControl.TheDiscretisedCostMatchesTheHandIntegratedInterval`; `DiscreteWorkflow.ASampledDesignThroughThePipelineAgreesWithValueIteration`, which reads the cross term back out of the evidence file bit for bit | implemented; locally verified |
| **4. A runnable Souxmar example — native plant, trim, linearisation, discretisation and sampled synthesis, nonlinear sampled simulation — with rotor lag, a declared hold and delay, and actuator limits, comparing the discrete prediction with the nonlinear response under a stated budget** | `examples/souxmar-sampled-lqr`, reading `models/souxmar-quad/souxmar-quad.yaml` in place. The budget was fixed in the study file before its first run, and **the run lands just outside it**. The budget was not moved. The discrepancy was localised instead: it is second order in the perturbation; it is not the quadratic drag; and it is carried by the collective channel, where the thrust's ω² curvature turns differential rotor commands into collective thrust — a mechanism the budget's derivation did not count. Separately, the order test that gives the budget its meaning **cannot resolve a one-tick delay error** at this perturbation. Its own test records that, and the delay line is certified by exact re-derivation instead. | `ExampleSouxmarSampledLqr.RunsEndToEndOnTheNativePlant`, `ExampleSouxmarSampledLqr.TheDisagreementIsSecondOrderInThePerturbation`, `ExampleSouxmarSampledLqr.APredictionAtTheWrongPeriodFailsTheSameOrderTest`, `ExampleSouxmarSampledLqr.AOneTickDelayErrorIsBelowThisComparisonsResolution`, `ExampleSouxmarSampledLqr.TheOverBudgetDiscrepancyIsHeldByATwoSidedLock` | runs end to end; **outside its declared budget**, held by a two-sided labelled lock |
| **5. The independent reference cases and refusal tests are retained, and the hosted checks run against the final head** | Every library reference is unchanged: the scalar closed form, value iteration with and without a cross term, the hand-integrated interval cost, held-input trajectories, and the stabilisable, unstabilisable, undetectable and invalid-cost cases. The same references are joined at the pipeline level, and a discrete-prediction unit test checks the delay against the augmented-state matrix. Determinism is held by a bit-identity test and a fingerprint-battery section compared across platforms. The case registry records `sampled.discrete_references` and `sampled.small_perturbation`; both are self-consistent and neither is validated. | `DiscreteControl`, `DiscreteSystem` and `DiscretePrediction` (unit); `DiscreteWorkflow` (integration); `Determinism.ASampledDesignAndItsPredictionAreBitIdenticalAcrossRuns` (determinism) | locally verified; hosted — see the results by revision |
| **Sampled-loop margins** | **Not delivered.** No capability computes a gain, phase, delay or disk margin of the sampled loop. A DARE solution whose closed loop lies strictly inside the unit circle says only that the *nominal* sampled loop converges. A stable sampled simulation is one trajectory from one initial state. **Neither establishes a margin**, and nothing in this repository claims one from either. The continuous margin path does not stand in for it, because `model.control_system` refuses a `sampled_control_law` by kind. | `QuadrotorWorkflow.TheContinuousMarginPathRefusesADiscreteDesign` | **not delivered** |

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

## The independent ULog comparison

ADR-0016 chose an in-repo ULog parser over a dependency and accepted an obligation in exchange:
the reader must be checked against an implementation that is not this repository's.
`tests/data/make_ulog_fixture.py --verify` discharged half of it — pyulog can read the fixture,
so the fixture is valid ULog. That says nothing about galata's **reader**.

`scripts/compare-ulog-against-pyulog.py` discharges the other half. It generates the fixture,
decodes it with galata through `data.import.ulog` and `report.record`, decodes it with pyulog in
pyulog's own interpreter, and compares:

| Compared | How | Why it is checked rather than assumed |
|---|---|---|
| **Timestamps** | as integer microseconds — galata's seconds scaled back rather than pyulog's integers converted forward | a reader that divided by the wrong power of ten, or lost low bits through a float, produces a plausible timebase |
| **Topic instances** | the fixture logs `sensor_combined` twice, at `multi_id` 0 and 1, offset by exactly 100 in every component | a reader that ignored `multi_id` would **merge** the two into a channel that is neither, and would pass every single-instance test |
| **Field types** | the two `uint32` fields between float arrays in `sensor_combined`, compared exactly, with pyulog's decoded dtype checked | the widths match a float, so a type confusion shifts nothing and returns garbage for the integer while every float still looks right |
| **Arrays** | every compared element by name — `q[0..3]`, `xyz[0..2]`, `control[0]` and `control[11]` | element 1 of a quaternion is a small number whichever index you actually read, so an off-by-one is invisible without naming |
| **Values** | **exact equality**, no tolerance anywhere | a float32 promoted to double is exact and an integer is an integer, so any difference is a decoding defect, and a tolerance would hide exactly what the comparison is for |

**Which pyulog agreed is part of the evidence**, so the comparison reads the version out of the
interpreter it was handed and prints it in its agreement line rather than having it typed here.
A future run against a different pyulog is a different check and says so on its own face.

`report.record` was added to make this possible: until it existed there was no way to see what
galata had decoded, which left the reader's agreement unverifiable from outside the test suite.
It is the counterpart to `data.import.*` and writes the samples as CSV with each channel's unit,
frame and applied conversion in a required evidence file beside them.

**What the comparison catches, measured rather than asserted.** Four defects were injected and
the comparison re-run. A wrong resample rate and a galata-side scale of 1 + 1e-7 on one channel
were both **caught**. A wrong array index and a wrong topic instance in the comparison's own
channel table were **not** — and that is a limitation of the method rather than of the reader:
the table drives *both* sides, so perturbing it moves both, and the script therefore compares
two decodings of one specification. The specification itself is held by the in-tree
`UlogImport.*` tests, which assert values against what the generator visibly wrote, including
two cases added here covering the second instance and the refusal of an instance the log does
not carry. The script's header records all four results.

**What it does not establish.** No real PX4 flight log has been read by either implementation.
A real log carries topics, formats, appended data and corruption this fixture does not, and
nothing here says how the reader behaves on one. That limitation is separate and is not closed
by this comparison.

---

## One more library-only gap, found and closed

The original acceptance objection was that two interfaces were library-only: registered in
nothing, reachable from no study. Auditing the rest of the vertical for the same shape turned
up a third. `identify.static_fit` was registered as a capability and exercised by
`tests/unit/test_static_fit.cpp` — which tests the **library**. Its capability wrapper — the
study-facing schema, the record wiring, the closed input vocabulary, the refusals — was
reachable from a study file and covered by nothing.

`IdentifyWorkflow.ABenchMapIsFittedThroughTheCapabilityAndRecoversItsCoefficient` and
`IdentifyWorkflow.ABenchMapRefusesATermTheRecordCannotSupport` close it: a synthetic thrust
stand imported through `data.import.csv`, fitted through `identify.static_fit`, with the
coefficient compared against the one the record was generated from, the declared unit carried
through to the coefficient, the absence of an estimable uncertainty required to be in the
headline rather than reported as a zero, and three refusals — a channel the record lacks,
`terms` omitted, and an unknown input key.

It also found a naming defect worth recording. `StaticFit::regressor_minimum` and
`regressor_maximum` hold the fitted **channel's** span in the channel's own unit — 0 to 2000
rad/s for a bench reaching 2000 — and not the span of the design-matrix column the term becomes,
which for an `omega^2` term would be 0 to 4e6. A *regressor* is the design column, so the names
invited a reader to be wrong by a square on a figure whose entire purpose is to bound where a
coefficient is evidence. They are now `term_channel_minimum` and `term_channel_maximum`, and the
header says which quantity it is and which it is not.

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
| A comparison between the sampled implementation's lag and the continuous delay margin | Held by a pair of tests, because one alone would measure nothing. `ExampleQuadrotorSampledControl.TheTransportDelayIsWellInsideTheContinuousDelayMargin` requires the shipped study's four channels to clear the equivalent lag — two periods of transport delay, plus about half a period **as an approximation of** the hold — by a factor of five. `QuadrotorWorkflow.AFasterDesignRunsOutOfDelayMarginAtTheSameSampleRate` requires a unit-weighted design on the same plant at the same rate to fall the **wrong** side of it. **This is not a stability condition in either direction**, and an earlier draft of this record wrongly called it necessary: the hold's half-period term approximates its low-frequency lag rather than modelling it, a continuous delay margin bounds a continuous perturbation, and a sampled loop can be stable past the margin or unstable inside it. What it gives is a warning sign in one direction. Every figure is a property of that LQR design and that loop construction — not of the plant, not of the sample rate, and not of any other controller running at the same rate. |

### Sampled-loop margins: still outside, and a discrete design does not bring them in

When this section was first written, discrete-time synthesis was outside the accepted bounded
workflow too. It is now delivered through the public interface; see the F14 section. That
changes nothing below about **margins**, which remain outside for the same reason as before.
This is a conditional statement rather than a free one. It holds only because of exactly what
the delivered workflows claim, so those claims are written out here with the condition
attached.

**What the sampled workflow claims.** That a law designed on the hover linearisation with the
weights `study.yaml` declares, executed at 250 Hz with two controller periods of transport
delay and per-rotor saturation, removes a declared initial displacement on the nonlinear plant
without saturating — and that the law, the saturation and the delay line the run applied are
the ones the study asked for, re-derived independently from the recorded states.

That is an **empirical demonstration on one plant, from one initial condition, with one set of
declared weights**. It is not a stability claim, not a robustness claim, and not a claim about
any family of perturbations, initial conditions or parameter values. Nothing in it requires a
sampled margin, which is why that margin's absence does not leave a delivered result
unsupported.

**What the F14 example claims, and what it does not.** It claims three things about a law
designed in discrete time for its execution period and flown at that period, with a declared
hold, a declared one-period delay and per-rotor limits:

- The run agrees with the design's own prediction to second order in the perturbation.
- At the declared perturbation, that agreement falls just outside the budget the study set in
  advance, for a reason that has been localised.
- The period, the hold, the delay and the basis the run applied are the ones the design and
  the study declared.

It is not a stability claim beyond the nominal loop, and it is not a robustness claim at all.

**What would exceed the evidence.** Any of the following, and each needs machinery galata does
not have rather than more of the machinery it has:

- Claiming the sampled loop is **stable** beyond the nominal statement a DARE solution makes,
  or claiming more than that a run converged.
- Claiming a **margin** for the sampled loop — gain, phase, delay or disk — from any
  continuous-domain figure, from a DARE solution's spectral radius, or from a converging
  sampled run. None of these is a margin. The delay comparison above is explicitly not one
  either.
- Claiming stability or performance **across** initial conditions, weights, sample rates or
  delays, rather than at the one point each study declares.

**Sampled-loop margins are marked delivered nowhere.** The situation is the same in each place:

- No capability computes one.
- `model.control_system` refuses a `sampled_control_law`, so none can be read off the
  continuous margin path.
- The README's generated table lists no such capability.
- RFC-0002's WP5 addendum, `docs/ROADMAP.md` and the F14 requirements table above all name the
  gap.

Per `docs/product/FEATURES.md`, this programme implements the proposed work of F14 and part of
F17 and delivers **neither row**. Both stay open, because an epic cannot pass acceptance while
a required dependency is unresolved.

### Out of scope: F14 / F17 roadmap work

These are named so that their absence is not read as an omission from this programme.

| Requirement | Why it is outside |
|---|---|
| **Simultaneous variation across channels** (multi-loop / MIMO structured robustness) | **Not available, and not obtainable from what is.** Three distinct questions are easy to conflate. `analyze.margins` bounds a pure gain or pure phase change in one channel; `analyze.diskmargin` bounds simultaneous gain *and phase* variation **still in one channel** — `include/galata/analyze/disk_margin.hpp` states it is the SISO condition and that the multi-loop case needs a structured singular value galata does not have; and simultaneous variation **across** channels is bounded by neither, at any number of channels. Applying the disk margin to each single loop in turn does **not** add up to a MIMO result. `analyze.sensitivity` and `analyze.sigma` give MIMO *peaks*, which qualify a design and are not a structured margin. An earlier draft of this record said the disk margin "answers the simultaneous question", which conflated the second row with the third; it does not. |
| Benchmark validation of the discrete Riccati solver, holds other than zero-order, and a pencil-based solver for an ill-conditioned transition | `c2d`-style discretisation and a sampled DARE are now implemented and unvalidated; see the F14 section. Their references are closed forms and value iteration, not the DAREX collection. Only the zero-order hold is implemented. A transition too ill-conditioned to invert is refused by name rather than solved by QZ. F14 and F17 both still carry unresolved dependencies that no item here scopes. |
| Margins of the sampled loop itself | Needs its own machinery, and a decision about which of several inequivalent definitions a sampled margin means. The discrete-time machinery it was waiting on now exists and does not supply one: a stabilising DARE solution and a converging sampled run are both statements about the nominal loop. Raised rather than absorbed. |
| Sensors, estimators and explicit delay models beyond the whole-period transport delay `sim.sampled` applies | `docs/ROADMAP.md`'s control-design row. |
| SITL/HIL interfaces, and independently measured hardware responses | `docs/ROADMAP.md`'s simulation-and-identification row. Blocked on hardware that does not exist. |

---

## What no part of this record claims

- **No physical validation.** No capability in this programme has been checked against a real
  aircraft, because there is no aircraft. Every record exercised is galata's own output, and a
  completed run is not a validation.
- **No sampled-loop margin.** No capability here computes a gain, phase, delay or disk margin
  of the sampled loop. A stabilising discrete Riccati solution is not one, and neither is a
  converging sampled simulation.
- **No merge.** The constituent pull requests and the integration pull request are open. ADR-0019
  records that a stack merges by merging the integration head once it is green, not by merging
  each constituent in turn, so that the same commits are not applied twice and `main` never
  carries an untested combination between the first merge and the last.
- **No claim that a fit is trustworthy.** A completed fit is not a validation, an identifiable
  parameter set is not an accurate one, and whether a residual is small enough for a use is an
  engineering judgement nothing in this repository makes. ADR-0008's three questions stay three
  questions.
