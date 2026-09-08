# M1 executable model feasibility increment

**Status: implemented and locally verified on macOS arm64.** macOS is the
first product target; Linux follows. Hosted platform verification, maintainer
review and the complete external M1 acceptance gate remain open.

This increment implements the bounded graph/compiler/runtime experiment under
[M1 contracts](M1_CONTRACTS.md). It supplies a headless foundation for the
aircraft/controller diagram workflow, with a runnable synthetic example and
explicit architecture. It does not deliver the desktop application or declare
aviation/defence product readiness.

## Implemented scope

- A separate `galata::modeling` library with a strict versioned YAML source,
  stable IDs, physical dimensions, frame identities and five scalar blocks.
- An immutable compiler result with deterministic state/output allocation and
  instantaneous scheduling; invalid connections and algebraic loops are refused.
- A pure evaluator and fixed-step RK4 simulation, including stateless graphs,
  initial/final recording, cooperative cancellation and explicit resource caps.
- Canonical model identity preserving binary64 parameters and port semantics;
  shared content hashing below the pipeline dependency boundary.
- `model.compile` and `sim.model` study capabilities, CSV output, scoped model/run
  evidence and existing exact-byte/build/runtime manifests.
- Bounded input snapshots, installed CMake exports and a shipped
  [continuous-feedback study](../../examples/continuous-feedback/README.md).
- [ADR-0010](../adr/0010-continuous-scalar-executable-model.md), proposed
  [ADR-0011](../adr/0011-source-model-identity-and-project-boundary.md),
  [source format](../MODEL_FILES.md) and an independently authored
  [conformance protocol](../architecture/MODEL_CONFORMANCE.md).

Independent review also exercised a compact-source/canonical-evidence size
counterexample. Compilation now refuses a model whose canonical source cannot
fit the evidence contract, before simulation begins. The regression was observed
to fail before that correction and pass afterward. File-size limits apply during
input snapshots as well as parsing, including cached and shared file roles.

## Verification record

The complete development and AddressSanitizer/UBSan suites passed on macOS
arm64, including the compiler/evidence review regressions. After the final
parser portability correction, all affected model, parser, input-limit and
source-to-run tests passed again in both builds. Python assurance/packaging
checks pass. Selected new/affected source and test translation units compile
under GCC with the project warning set and warnings as errors; the exact
commands, an initially failing test-macro warning and its successful correction
are retained. Core SHA known vectors also pass under a changed numeric locale.

The final decimal parser compiles with a macOS 14 deployment target and passes
independent libc++/libstdc++ conversion probes, including exact halfway values,
subnormals and caller floating-environment preservation. This removes a new
macOS 26-only library dependency; it does not establish runtime acceptance on
macOS 14 or choose the product's minimum supported OS.

Installed CLI studies and a relocated standalone C++ consumer linked only to
`galata::modeling` pass. Generated verification/capability/modal/run/HTML records,
deterministic repeat/locale checks, pinned formatting, SI/version consistency,
document references/links, shell lint and whitespace checks pass. Existing
published-reference discrepancy locks remain intact.

Local JUnit/log evidence is in `build/dev/m1-results.xml`,
`build/dev/m1-ctest.log`, `build/asan/m1-results.xml` and
`build/asan/m1-ctest.log`. The final affected tests are recorded in each build's
`m1-portability-results.xml` and `m1-portability-ctest.log`; the deployment
compile counterexample/correction is in `build/m1-portability`. Supplemental
check logs use the `m1-` prefix in `build/dev`. GCC commands/results are in `build/gcc-m1`, as compile-checks.json.
These generated local artifacts are outside distributed source evidence.
The local macOS archive is a development snapshot, with no hosted CI or release
publication claim. Packaging reruns the shipped continuous-model and aircraft
studies against extracted artifacts and records its own source/build identities.

The protocol fixes references and tolerances before product measurements.
The two new capabilities retain `implemented, unvalidated` registry status;
synthetic conformance and completed execution do not assess an aircraft model
or approve a consequential engineering decision.

## Next delivery work

The [M2 project/worker increment](M2_IMPLEMENTATION.md) now builds on this
foundation with an experimental directory project and native macOS feasibility
preview. Its [project-file contract](../PROJECT_FILES.md) and
[ADR-0012](../adr/0012-project-worker-preview.md) record draft editing, immutable
requests and recovery. The complete external M1 and M2 gates remain open.

1. Review this profile and run the same source on the hosted engine matrix,
   which covers Linux under GCC and Clang and macOS under AppleClang since
   Windows support was withdrawn. Retain failures and source-bound evidence.
2. Complete the macOS desktop feasibility comparison: offline packaging,
   accessibility, graph editing, recovery and a worker cancellation experiment.
   Use the same C++ compiler/runtime and the synthetic acceptance model. The
   native candidate now covers graph editing, recovery, cancellation and offline
   packaging on the synthetic model. Signing, notarization, clean-machine
   installation, full accessibility acceptance and the Qt/web-shell comparison
   remain undone, so the *comparison* this step names is still open.
3. Define the smallest aircraft/linear-system block adapter with units, state
   mapping and source evidence, then test against existing aircraft studies.
   Delivered as the typed `continuous-linear.v1` profile and the
   `model.linear_graph` capability under
   [ADR-0013](../adr/0013-typed-linear-graph-adapter.md), which reconstructs the
   local NT-33A study and retains its source matrices, channel mappings and
   linearization diagnostics. That is agreement with the existing engine, not
   new aircraft validation.
4. Verify the M2 directory-project and worker contract on the claimed platforms;
   complete recovery and installation evidence before promising project
   compatibility or migration support.
5. Add sampled/hybrid behavior only after its clock/event protocols are reviewed.

Named customer/reviewer, decision-specific model validity and data rights,
minimum OS/hardware/workloads, toolkit choice and support ownership still need
recorded decisions. These are the remaining M1 product gates, not fabricated
approvals or reasons to suppress useful technical progress.
