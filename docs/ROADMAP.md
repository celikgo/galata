# Roadmap

The current release is **v0.3.0, a bounded offline engineering workbench**.
The [README](../README.md) and the capability registry describe what exists;
the [operating guide](WORKBENCH.md) describes its limits. Future work below is
not part of this release and has no promised delivery date.

The rule for each milestone remains that a reader can build that version and
run a complete, reviewable study. Completing a workflow does not qualify a tool
or validate a new aircraft model.

## v0.1 — trim and linearisation

Implemented: frames and units, ISA atmosphere, quaternion rigid-body dynamics,
fixed-step RK4, the CLI and YAML runner, the local derivative aircraft model,
straight-line trim, finite-difference linearisation and classified aircraft
modes. Published-reference comparisons for the NT-33A condition and the
underlying numerical methods are in [VERIFICATION.md](VERIFICATION.md).

The known phugoid discrepancy and its labelled regression locks remain
documented in the [investigation note](notes/phugoid-damping.md). They are not
reclassified as validation successes by later releases.

## v0.2 — frequency-domain analysis

Implemented: frequency response, gain/phase/delay crossovers, SISO disk margin,
MIMO principal gains, sensitivity and complementary-sensitivity peak estimates.
The published NT-33A HTML report adds plots generated from its computed run.

These original frequency-peak searches use a finite refined grid. A sampled
peak can underestimate the true norm, making a derived margin optimistic.
Their reference tests do not establish a bound for every possible transfer
function. The original capabilities retain this limitation and their identity.

## v0.3 — bounded offline workbench

Implemented scope:

- Continuous CARE and LQR synthesis with residual, conditioning and stability
  evidence; filtered PID from explicit gains; channel selection, series and
  negative-feedback interconnections.
- Hamiltonian H-infinity norm brackets, S/T norm brackets and SISO disk-size
  bounds. These supplement the sampled analyses and reject unresolved numerical
  cases. They are floating-point numerical bounds, not interval proofs.
- Fixed-step linear response and local nonlinear aircraft simulation with
  explicit position bounds, rate limits and positive first-order lags for all
  four controls. Continuous state feedback and constant command increments are
  supported about a level translating reference.
- A complete NT-33A local design example, trajectory CSV, Markdown and
  self-contained HTML tables, strict input schemas, contained output writes,
  input snapshots and immutable content-addressed run manifests.
- Installable C++ libraries and CLI, dependency notices, installed-consumer and
  release-archive checks, and expanded numerical and integration tests.

Evidence for the CARE solver is distinguished from aircraft/controller
validation in the [synthesis design record](rfc/0001-control-synthesis.md).
The example's costs and actuator specifications are illustrative. The model
still describes a single local aerodynamic reference condition.

Earlier roadmap versions assigned v0.3 to a desktop application. That scope is
deferred from the release baseline. The subsequent experimental
[M2 increment](product/M2_IMPLEMENTATION.md) adds a native macOS editor for
scalar projects and typed linear aircraft/controller study imports, with a Dim
theme, native block/sample tables and saved-revision review/restore. Source
matrices and diagnostics remain attached after edits. It has no 3-D viewport or
signed desktop installer. It does not complete the former v1.0 feature list.

## Future work — not implemented

The following are separate extensions, each requiring its own model assumptions,
reference data and acceptance criteria before it becomes a release commitment.

| Area | Remaining work |
|---|---|
| Aircraft fidelity | Additional independently validated conditions and aircraft, scheduled aerodynamic data, propulsion and configuration models, quantified model uncertainty |
| Control design | Discrete-time control, sensors and estimators, explicit delay models, constrained synthesis, gain scheduling and robust-design methods |
| Assessment | Handling-quality criteria, root locus, uncertainty campaigns and application-specific acceptance reports |
| Simulation and identification | Independently measured hardware responses, SITL/HIL interfaces, controllability and observability Gramians, discrete-time synthesis (`c2d`, sampled DARE) and the sampled loop's own robustness margins. Measured-data import, grey-box identification, held-out validation, declared input histories and sampled control execution are implemented and UNVALIDATED — RFC-0002's WP3 to WP5 records say what each does not establish, and no capability in this row has been checked against a real aircraft |
| User interface | Desktop shell, interactive plots and pipeline editor, engineering 3-D visualisation, platform installers |
| Extensions and automation | Stable C plugin ABI, aerodynamic/sensor extension contracts, AI and MCP interfaces with separate execution controls and evaluations |

A future stable release needs an explicit scope and compatibility contract; it
cannot be inferred from the number of completed capabilities. There is no
onboard deployment or tool-qualification claim in this roadmap.

## Reference data and evidence

Shipped numerical data requires documented source and rights provenance under
[ADR-0007](adr/0007-reference-values-from-copyrighted-sources.md). A prospective
dataset whose redistribution rights are unresolved does not become acceptable
merely because it would improve an example. Where no published comparison is
available, the capability remains labelled unvalidated in the verification
report; analytic tests and regression locks retain their separate purposes.

## Proposed end-product completion plan

The [product plan](PRODUCT_PLAN.md) develops the future work into a proposed
offline desktop engineering product with Simulink-style executable block
modeling. It links a prioritized feature catalog, discovery investigations,
milestone action plans and assurance gates. These are planning documents, not
newly implemented capabilities or a qualification claim; the current release
scope above remains unchanged.
