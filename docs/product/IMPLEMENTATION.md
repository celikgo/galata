# M0 implementation record

Work started on 2026-09-07 under the [product plan](../PRODUCT_PLAN.md).
This is the first reliability increment toward the end product. It does not
complete M1–M5, introduce a graphical editor, qualify the tool or establish
aviation/defence application acceptance.

## Implemented reliability changes

| Audit item | Change | Verification contract |
|---|---|---|
| R01: sampled estimates presented as guarantees | Explicit upper-norm evidence API; remove sampled guarantee tables and rename sampled disk ranges; missing crossovers mean not found in the searched band | Analytic out-of-band loop, direct API refusal and report-level regression; published formula inputs remain separate |
| R02: false stability on a nonnormal realization | Shared finite/residual/conditioning/axis-separation assessment for margins, disk, sensitivity and H-infinity paths | Exact unstable realization with misleading numerical poles; ordinary stable and boundary cases; conservative refusal documented |
| R03: unsupported Euler chart | Enforce named chart envelope at equilibrium and perturbations; preserve immutable source diagnostics transitively in pipeline artifacts, reports and manifest | Converged near-vertical trim refused; branch/join report retains source evidence |
| R04: stale trim residual | Recompute all-six dynamic equilibrium residual and actual operating point from supplied model/state/controls; independent acceptance budget | Mutated aircraft/controls and stale metadata; supported state and residual-budget tests |
| R05: invalid numerical boundaries | Finite model/solver/callback checks, dimensions before indexing, representable step/arithmetic checks | NaN/Inf, wrong dimensions, overflow/underflow and zero-Jacobian stagnation cases |
| R06: release gate not bound to full CI | Resolve immutable SHA once; reusable complete CI; exact source and required-job evidence for release/Pages; verify packages and observed tag movement | Negative Python gate tests and workflow structural review; hosted execution remains pending |
| R07: incomplete run/build identity | Source inventory and effective configuration digests; packaged build evidence; endpoint loader inventory and readable module byte hashes | Source/configuration mutation tests, manifest byte identity, extracted archive smoke and loader lifecycle tests |
| R08: prerequisite/documentation drift | CMake 3.25 and Python 3.9 prerequisites; reconcile determinism and qualification wording | Configuration, generated-doc/reference/version/SI checks |

The change is intentionally source-breaking for the pre-1.0
`guaranteed_margins` API. Migrate sampled callers to estimates; provide explicit
upper-norm evidence and establish nominal stability before requesting bounds.
See [the workbench migration notes](../WORKBENCH.md).

## Architectural records

- [ADR-0008](../adr/0008-numerical-evidence-authority.md): evidence authority,
  conservative numerical refusal, source diagnostics and dependency direction.
- [ADR-0009](../adr/0009-release-evidence-and-source-identity.md): complete CI
  tied to immutable source, configuration/source inventories and archive limits.
- [Executable-model architecture](../architecture/EXECUTABLE_MODEL.md): proposed
  Simulink-style compiler, typed IR, instantaneous dependency analysis,
  continuous and multirate schedules, model identity, checkpoint/replay and SDK
  boundaries. This remains proposed until the M1 design decisions are accepted.

## Verification status

Observed local verification on 2026-09-07, macOS arm64:

- Development suite: 379/379 passed.
- AddressSanitizer/UBSan suite: 379/379 passed without sanitizer diagnostics.
- Final report wording changes: 6/6 focused regressions passed in both
  development and sanitizer builds after those changes.
- Python assurance/provenance/packaging tests: 28/28 passed.
- Selected numerical, analysis and pipeline translation units and tests compiled
  with GCC 15 under the project warning set and warnings as errors.
- Relocated installed C++ consumer and installed CLI studies passed. Local macOS
  archive extraction, source/configuration identity, dependency notices and both
  shipped study smoke checks passed.
- Generated verification, capability table, modal/run data and HTML checks;
  deterministic repeat/locale checks; pinned formatting; SI, version, document
  references/links, shell lint and whitespace checks passed.
- Original audit studies were replayed: sampled sensitivity carries no guarantee,
  unresolved internal stability is refused, and an unsupported chart is refused.

The complete suite ran before the final explanatory report-text change; its
six affected example tests were rerun afterward in both builds. Local logs and
archive artifacts are retained under the build directory, outside distributed
source evidence. The existing discrepancy locks remain intact.

The M0 changes are implemented and locally verified. Milestone exit still
requires maintainer review and actual hosted Linux/macOS/Windows CI execution;
local YAML/gate tests cannot demonstrate that remote workflow graph. No release,
remote workflow or external repository setting was changed. Existing user work
was preserved.

The subsequent authorized increment is tracked in [M1 implementation](M1_IMPLEMENTATION.md).

## Next engineering sequence

1. Review the M0 changes and run the exact source through hosted Linux, macOS
   and Windows CI. Investigate any platform failure before a release candidate.
2. Execute M1 discovery packets and record supported operating envelope,
   numerical budgets, model/data rights, project schema, UI/worker transport and
   model/clock semantics decisions. Owners and exit criteria are in the
   [delivery plan](DELIVERY.md); evidence questions are in [discovery](DISCOVERY.md).
3. Build the M2 vertical slice: project persistence → typed model compiler →
   continuous graph execution → desktop/CLI run → retained results/evidence.
   Gate it with analytic continuous feedback models and identical headless/UI
   engine results before broadening the block library.
4. Add sampled/hybrid execution in M3, then harnesses/checkpoints/tuning in M4,
   and distribution/maintainability/assurance completion in M5. State machines,
   general algebraic solvers, code generation and broad Simulink compatibility
   keep their separate acceptance gates; they are not implied by a block editor.
