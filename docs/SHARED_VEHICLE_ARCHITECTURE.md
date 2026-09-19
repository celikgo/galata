# Shared vehicle execution architecture

## Scope and current boundary

ADR-0020 defines `galata::model::VehicleModel` as the shared numerical seam.
It owns the rigid-body state ordering, quaternion projection, gravity resolution,
wind contribution to ground velocity, model vocabulary, wrench evaluation,
auxiliary-state derivatives, envelope reporting, and named outputs. The shared
linearisation service consumes that interface, and the helicopter path already
uses the shared sampled-loop service for controller timing, saturation, delay,
and zero-order hold.

The fixed-wing `model::Aircraft` and multirotor `model::Quadrotor` types now
enter this seam through explicit owning adapters. The adapters preserve their
authoritative family equations and trim problems while the shared pipeline
owns type erasure, metadata, environment, integration, linearisation and
trajectory artifacts. The established family-specific capability names remain
compatibility adapters; they are not silently renamed.

## Execution matrix

| Concern | Fixed-wing adapter | Multirotor adapter | Helicopter adapter | Shared authority |
|---|---|---|---|---|
| State/control/output vocabulary | `FixedWingVehicleModel` | `MultirotorVehicleModel` | `HelicopterVehicleAdapter` | `VehicleModel` metadata and versioned schemas |
| Model evaluation | adapter → `Aircraft` wrench | adapter → `Quadrotor` wrench | adapter → helicopter wrench | `VehicleModel::derivative` and `sim::rigid_body_derivative` |
| Fixed-step execution | `sim::execute_vehicle` | `sim::execute_vehicle` | `sim::execute_vehicle` | `numerics::integrate` with common projection/envelope hooks |
| Sampled execution | legacy `sim.nonlinear` compatibility path | legacy `sim.sampled` compatibility path | legacy `sim.helicopter.closed_loop` path | Shared path is available for common fixed-step execution; sampled semantics remain specialized |
| Environment and events | shared `model::Environment` | shared `model::Environment` | shared `model::Environment` | Family event schedules remain explicit compatibility capabilities |
| Linearisation | `linearize.shared` | `linearize.shared` | `linearize.shared` | `linearize::linearize_vehicle` and named matrices |
| Trim | `trim.vehicle` dispatches level trim | `trim.vehicle` dispatches hover trim | `trim.vehicle` dispatches established helicopter trim | Trim equations remain model-specific behind one artifact contract |
| Reporting | `report.vehicle_*` | `report.vehicle_*` | `report.vehicle_*` | named channels, metadata, manifests, provenance |

## Evidence and remaining migration

The source paths above are the compatibility baseline. Valid studies must be
captured before replacing an adapter, then rerun with the same compiler/build
configuration and compared by named channel, units, event boundary, and
provenance. Numerical operation ordering is preserved for an adapter migration;
intentional corrections are reported as behavior changes.

Adding a vehicle implementation now requires a model vocabulary and a
`VehicleModel` adapter, while numerical integration, rigid-body composition,
common linearisation, shared fixed-step execution, and reporting schemas are
reusable services. This closes the C1 shared execution scope for the three
built-in families. It does not claim that family-specific sampled controllers,
failure schedules, or trim equations are identical: those remain explicit
compatibility adapters with their own evidence.

## Reproduction

```sh
cmake --build --preset dev -j 4
ctest --preset dev -R 'ExampleHeli|Quadrotor|Nonlinear' --output-on-failure
```

The cross-family artifact and execution evidence is in
`SharedVehicleExecution.AllBuiltInFamiliesUseTheCommonArtifacts`; the Python
composition evidence is in
`PythonWorkflowAcceptance.test_shared_vehicle_operations_execute_for_all_families`.
The pre-migration output digests and comparison policy are recorded in
`docs/C1_COMPATIBILITY_BASELINE.md`. Family-specific regression studies remain
in place and are run separately from the new shared examples.
