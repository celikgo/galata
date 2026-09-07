# Offline workbench operating guide

galata v0.3.0 supports a complete local aircraft-control study through its CLI
and C++ library: trim, linearise, design, analyse, simulate and report. It can
support supervised offline aviation and defence engineering work, including
method exploration, model review and controller prototyping. It supplies no
onboard controller, tool qualification package or aircraft-specific approval.

The [verification report](VERIFICATION.md) states what has been compared with
published references. A solver residual, a completed trajectory or a passing
regression test does not validate a proposed aircraft or controller.

## Run the example

From a checkout, with CMake, Ninja, a C++20 compiler and vcpkg available:

```sh
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
build/dev/src/cli/galata --version
build/dev/src/cli/galata capabilities
build/dev/src/cli/galata run examples/nt33a-control-design/study.yaml --output-dir build/control-study
```

The [complete study](../examples/nt33a-control-design/study.yaml) loads the
reference aircraft, solves trim, selects the elevator channel, computes LQR
feedback, examines the plant-input return ratio and closed-loop modes, then
integrates linear and nonlinear responses. Its controller costs, perturbation
and actuator specifications are **illustrative inputs**, not verified NT-33A
controller or hardware data.

The output directory receives Markdown and HTML reports, both trajectory CSV
files and a run manifest. The generic HTML report contains self-contained tables
and time-history plots; the separate [reference-case page](reports/nt33a-fc1.html)
has modal and frequency-domain plots. Existing reports require `--overwrite`. Prefer a new output directory
when comparing studies so that both sets of outputs remain available.

The CLI prints each completed stage and exits unsuccessfully if any stage
fails. Earlier report files can remain after a later failure: check the exit
status and completed run manifest before treating a directory as a completed
study. Publication is atomic per file, not a transaction over the whole study.

## Inputs, connections and units

A study uses `version: 1` and a sequence of uniquely named stages. A reference
such as `{from: trim}` supplies an earlier artifact to a later capability.
Relative model paths resolve against the study's directory; report paths resolve
under the selected output directory. Unknown or duplicate keys, nonfinite
numbers and unsupported YAML constructs are rejected. The full contract is in
[STUDY_FILES.md](STUDY_FILES.md).

Use SI values and the state/input names carried by each model. Aircraft
coordinates are NED position, FRD body velocity and a scalar-first Hamilton
body-to-NED quaternion. Linearisation uses named Euler coordinates instead of
the quaternion integration coordinates; do not transfer an unnamed vector
between these representations. See the
[frame convention](adr/0002-state-and-frame-conventions.md).

| Stage group | Input decisions the operator must make |
|---|---|
| Aircraft and trim | Cited derivative data, geometry, mass/inertia, aerodynamic frame and reference point; altitude, speed and temperature offset |
| Linearisation and channel selection | Longitudinal/lateral/full states and the intended input/output channels; inspect names, units and finite-difference error estimates |
| LQR | Compatible Q/R and optional cross-cost N, selected channels and the explicit plant-input loop break; weights depend on the physical scaling |
| Filtered PID | Supplied proportional, integral and derivative gains plus a positive derivative-filter time constant; no automatic tuning |
| Linear interconnections | Channel order, compatible units and negative-feedback sign; the library does not infer physical wiring from channel counts |
| Simulation | Initial disturbance, constant command increment if needed, duration, step and sample spacing; all actuator bounds, rates and lags for nonlinear runs |

LQR uses continuous full-state feedback about trim. It has no estimator,
sampling, constraint handling or automatic gain scheduling. Its report retains
the weights, gain, Riccati residual and conditioning/stability evidence.
The [synthesis API](../include/galata/synth/control.hpp) defines the supported
cost and interconnection contracts.

## Read the analysis correctly

Classical margins describe the specified return ratio and loop break, not an
arbitrary channel of a plant. The state-space margin path checks nominal
closed-loop stability; an unstable or marginal baseline does not have a
positive delay tolerance. Inspect all reported crossovers and the diagnostic,
including cases where no crossing was found.

Two frequency-analysis paths coexist:

- The original frequency response, singular-value, sensitivity and disk-margin
  capabilities use finite frequency searches. Missed peaks make norm estimates
  low and derived robustness margins optimistic. Refining a plot does not prove
  that no peak was missed.
- `analyze.hinfnorm` uses Hamiltonian level tests to bracket the norm of an
  internally stable continuous state-space model. `analyze.robust_bounds`
  constructs S and T for square negative-feedback loops and also returns SISO
  disk-size bounds. The lower disk-size bound uses the upper norm bound.
  Both endpoints, the bracket width and numerical diagnostics are part of the
  result. Unresolved tolerance or conditioning checks fail the pipeline stage.

The Hamiltonian routines include DC and feedthrough limits and refuse unstable
internal modes even if a transfer-function cancellation could hide them.
They use ordinary floating-point arithmetic, not certified interval arithmetic.
There is no descriptor/discrete-time norm, simultaneous MIMO disk margin or
uncertain-plant proof. See the [norm API](../include/galata/analyze/hinfinity.hpp)
for size limits and reliability checks. Channel scaling still matters to every
singular value and MIMO norm.

## Nonlinear simulation and step convergence

The nonlinear driver integrates the full quaternion rigid-body state and four
actuator positions using fixed-step RK4. Each surface and thrust channel needs
finite minimum/maximum position, a positive rate limit and a positive
first-order lag. Commands are clipped, actuator derivatives are rate-limited,
and accepted actuator positions are clamped to their bounds. These are generic
actuator dynamics; a thrust lag is not an engine model.

The reference must be a level trim. Its NED position advances at the trim
velocity while attitude, velocity and baseline controls remain constant.
Feedback is evaluated from exact states at every RK stage. Initial disturbances
are named Euler physical coordinates; the driver integrates attitude as a
quaternion. The trim's ISA temperature offset is retained.

The step must be no larger than any actuator time constant. The linear driver
also rejects steps outside RK4's stability region for stable poles. These checks
do **not** select an accurate step for the user. Establish convergence for each
study:

1. Run with a selected step, then halve it and double the step count to preserve
   duration. Repeat once more. Adjust the sample stride so output times match.
2. Compare corresponding physical quantities at the same times: velocity,
   attitude, rates and actuator positions. State a tolerance per quantity or
   nondimensionalise before using a combined norm.
3. Compare peak responses and limit events as well as endpoints. For a smooth
   unsaturated trajectory, fourth-order RK4 should show fourth-order step
   convergence once sufficiently resolved. Saturation events can reduce the
   observed order; judge the actual engineering quantity instead of imposing
   a smooth-system expectation on a limit transition.
4. Check that the remaining step dependence is smaller than the study's chosen
   numerical tolerance. This controls integration error, not aerodynamic or
   hardware-model error.

The shipped linear response omits actuators; its difference from the nonlinear
response includes actuator lag as well as nonlinear effects. A small-disturbance
consistency comparison must include the same actuator dynamics and state
couplings on both sides. The
[simulation convergence tests](../tests/integration/test_simulation_convergence.cpp)
exercise this distinction; the
[actuator tests](../tests/unit/test_nonlinear.cpp) also use an analytic step
response independent of the aircraft model.

The CLI stops the study on an invalid domain or an advisory envelope excursion,
including one detected at an intermediate RK stage. The C++ result preserves
the reason, failing evaluation time, last accepted samples, envelope extrema
and per-channel limited-step counts. Its explicit envelope override records
the excursion and does not establish validity. The CLI exposes no override.

## Physical and evidential limits

The aircraft uses a first-order coefficient expansion about one aerodynamic
reference condition. It has no stall, Mach-dependent derivative schedule,
ground effect, variable configuration, fuel burn, propulsion map or flexible
structure. The atmosphere and rigid-body equations do not supply those missing
physics. Thrust is force in newtons; trim alone does not establish whether
available propulsion can provide it.

The current envelope guard checks departure in angle of attack and Mach only.
Its thresholds are advisory, not an experimentally established flight envelope.
It does not bound sideslip, angular rates or aerodynamic control-deflection
validity. Staying within the guard and actuator limits is therefore necessary
for a successful default run, but insufficient evidence for a real aircraft.
Straight-line trim rejects unsupported lateral asymmetry and checks all six
dynamic acceleration residuals; it is not a turning or asymmetric trim solver.

Published NT-33A comparisons concern the documented reference condition. The
known dimensional-matrix phugoid discrepancy and associated regression locks
remain in the [investigation note](notes/phugoid-damping.md). A regression lock
records behaviour; it does not resolve a disagreement with a published source.

For an aircraft-specific study, establish the aerodynamic data's applicable
condition/configuration, hardware limits and controller assumptions, then
compare the relevant predictions with independent evidence. This repository's
tests support the documented numerical methods and reference cases; they do
not demonstrate safety, handling-quality compliance or flight-test agreement
for a new application.

## Keep the result reproducible

Retain the reports, trajectory files and manifest together with the binary
identified by the manifest. The manifest stores the exact study/model input
bytes, actual stage inputs, output hashes and build/dependency identity. It
therefore includes the **model contents**, not only filenames; handle it as part
of the study data when sharing it. It identifies the binary but does not embed
that binary.

Hashes establish content identity, not authorship or tamper-resistant storage.
Same-platform repeatability and cross-platform numerical bounds have different
contracts in the [determinism policy](adr/0004-determinism-policy.md). Preserve
compiler settings and dependency versions when repeating a result.

## Install and embed the C++ package

After a successful build, install into a chosen prefix:

```sh
cmake --install build/dev --prefix "$PWD/build/install"
```

A consuming CMake project can use the installed package:

```cmake
cmake_minimum_required(VERSION 3.24)
project(MyStudy LANGUAGES CXX)
find_package(Galata 0.3.0 EXACT CONFIG REQUIRED)
add_executable(my_study main.cpp)
target_link_libraries(my_study PRIVATE galata::pipeline)
```

Configure that project with the install prefix in CMake's prefix path and make
Eigen and yaml-cpp discoverable through the same package manager. For example,
from the galata checkout, using a consumer project with its own vcpkg manifest:

```sh
cmake -S /path/to/consumer -B /path/to/consumer-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_PREFIX_PATH="$PWD/build/install" \
  -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build /path/to/consumer-build
```

Use compatible compiler, architecture, dependency versions and build
configuration. For direct numerical use, link the required exported targets,
such as `galata::synth`, `galata::analyze` or `galata::simulation`; the latter
contains both simulation drivers. The pre-1.0 C++ interface has no frozen ABI.
The installed CLI, example studies, model provenance, reference data and
dependency notices accompany the libraries. The
[installed-consumer check](../scripts/check-install.py) tests package relocation
and an example run without source-tree include paths.

## Reliability boundary and migration

Linearization now verifies the supplied equilibrium independently, including all
six dynamic accelerations. The optional pipeline `equilibrium_tolerance` input
sets its finite positive acceptance budget; it does not inherit a mutable trim
result's claimed residual. Unsupported Euler charts and perturbations fail
before a linear model is returned. Reports carry source chart, equilibrium and
truncation diagnostics through subsequent analysis; manifests retain the full
step vectors and truncation matrices.

The C++ sensitivity-margin API now requires `SensitivityNormUpperBounds` instead
of `SensitivityPeaks`. Supply defensible full-norm upper evidence and establish
nominal stability before using that API. A finite frequency search supplies
lower norm estimates and cannot meet this contract. The separate numerical
bound capabilities retain their own floating-point reliability limits.
Ill-conditioned nominal eigensystems are conservatively refused; a mathematically
stable defective realization may need rescaling or another analysis method.

Run manifests additionally identify current source bytes, effective build
configuration and the loader-reported modules present at both run endpoints.
Readable module files have byte digests; OS-managed shared-cache/virtual images
are explicitly identified without a fabricated file digest. These records do
not attest process memory or detect a load/unload entirely between snapshots.
Keep the packaged build configuration and compilation database with a run's
binary when an investigation must reconstruct its toolchain choices.

See [ADR-0008](adr/0008-numerical-evidence-authority.md) for numerical contracts
and [ADR-0009](adr/0009-release-evidence-and-source-identity.md) for source and
release evidence. These changes do not establish application qualification.

On macOS, the first runtime snapshot registers dyld callbacks. A shared library
or bundle containing those callbacks is retained for the process lifetime,
including after an embedding host closes its own handle. The loader offers no
callback-unregister operation; releasing the callback code would be unsafe.
Ordinary model execution still takes only endpoint snapshots, and does not
provide continuous module monitoring.

Rebuild C++ consumers after this update: public artifact/diagnostic structures
and the sensitivity-margin function signature changed. The future plugin ABI
version does not promise binary compatibility for these C++ value types.

## Experimental executable models

The [continuous scalar model format](MODEL_FILES.md) and
[feedback example](../examples/continuous-feedback/README.md) introduce the shared
headless block compiler/runtime. This profile has five scalar continuous block
types with explicit units and frames. It produces CSV and scoped evidence,
while the existing study manifest retains exact input/build/runtime identity.
The desktop editor, aircraft block adapter and sampled controllers remain later
increments; product delivery targets macOS first and Linux next.

The new `galata::modeling` CMake target is installed alongside the existing
libraries. Model/source/evidence formats are experimental and the resolved IR
is private. Rebuild C++ consumers when updating this pre-1.0 snapshot. The
pipeline SHA-256 API remains available and delegates to a shared core utility.
