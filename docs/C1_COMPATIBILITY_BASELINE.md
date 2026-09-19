# C1 shared-path compatibility baseline

This record freezes the representative studies captured before the shared
vehicle adapter refactor. It is compatibility evidence for operation ordering
and artifact identity; it is not a claim that the historical family-specific
implementations were architecturally shared.

## Capture conditions

- Pre-refactor source commit: `83b29c3` (`feat: deliver executable Python engineering workflow`).
- Working tree: clean before capture.
- Host: macOS arm64, CMake `build/dev`, AppleClang toolchain, Eigen 3.4.1 and
  yaml-cpp 0.9.0 as reported by the build.
- Command form: `build/dev/src/cli/galata run <study> --output-dir <fresh-dir> --json`.
- The external `/Users/celikgo/souxmar-helicopter` tree was not read or
  modified by the refactor.

## Historical output digests

These are the output files from the clean pre-refactor run. The current
implementation reruns the same family-specific studies with the same build
configuration before any shared example is considered compatible.

| Study | Output | SHA-256 |
|---|---|---|
| `nt33a-trim-and-linearise/study.yaml` | `trim-and-modes.md` | `3ca5a498a60c6c0b550fea58828377051b619fb45a8c7fbc84b9282ef6e2c3cd` |
| `quadrotor-sampled-control/study.yaml` | `operating-point.yaml` | `97a43f4632e5998e67e938fdf3d378b789de09764624288bbcb72783df4513f2` |
| `quadrotor-sampled-control/study.yaml` | `sampled-control.md` | `7a1399189ffb4206929b47a1e580b46e8a0bdec8c5903dfdb162a4f7e29148e9` |
| `quadrotor-sampled-control/study.yaml` | `sampled-run.csv` | `feee6fabc0e5493344b711efa68f8f88cf007fff42420cc9eaca9e6510a4800a` |
| `heli-performance-acceptance/study.yaml` | `performance.md` | `01bd9baa54cdf440904d79f27eba36cb45e5e56b3a3b818de064e24e31c3c5d9` |
| `heli-performance-acceptance/study.yaml` | `open.csv` | `f9e32dd6d9e09d58f83b21714e217fcc59e6ab0d23ffd156f8910390cd14f232` |
| `heli-performance-acceptance/study.yaml` | `closed.csv` | `6afe0117b427de231d9dd6e7ceb059a20141b37aa075bea25513699ba7bf1a83` |
| `heli-performance-acceptance/study.yaml` | `closed.csv.controller.csv` | `ddc474083c89fc3fbbb59c2e9e8ff4b917a6978df9d0a86b082a93ed25748aa8` |

The current rerun reproduced every listed digest on this macOS arm64 build.
The comparison is intentionally made on the family-specific compatibility
studies, not by comparing a fixed-wing artifact to a helicopter artifact.

## Shared-path evidence

The new studies `examples/shared-fixed-wing-workflow/study.yaml`,
`examples/shared-multirotor-workflow/study.yaml`, and
`examples/shared-helicopter-workflow/study.yaml` all produce the same typed
artifact sequence: `vehicle_model` → `vehicle_trim` → `linear_system` →
`vehicle_trajectory` → named reports. Their model-specific trim equations are
dispatched behind the common artifact boundary; execution, environment,
linearisation, metadata and reporting are shared.

The integration test
`SharedVehicleExecution.AllBuiltInFamiliesUseTheCommonArtifacts` checks model
identity, metadata, trim residuals, matrix dimensions, sample alignment, and
artifact model identity for all three families. Python adds the composed
acceptance `PythonWorkflowAcceptance.test_shared_vehicle_operations_execute_for_all_families`.

Bitwise equality remains a policy for like-for-like selected outputs. A shared
adapter output is not declared bitwise equal to a historical output merely
because both studies complete; dimensions, units, frames, event semantics and
provenance must match first. Any intentional behavioral correction is recorded
separately from this pure compatibility capture.
