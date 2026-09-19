# Shared vehicle execution architecture

## Scope and current boundary

ADR-0020 defines `galata::model::VehicleModel` as the shared numerical seam.
It owns the rigid-body state ordering, quaternion projection, gravity resolution,
wind contribution to ground velocity, model vocabulary, wrench evaluation,
auxiliary-state derivatives, envelope reporting, and named outputs. The shared
linearisation service consumes that interface, and the helicopter path already
uses the shared sampled-loop service for controller timing, saturation, delay,
and zero-order hold.

This milestone captures the actual architecture before claiming a migration. The
fixed-wing `model::Aircraft` and multirotor `model::Quadrotor` types still have
compatibility adapters in their pipeline families. They are not silently called
`VehicleModel` implementations, because that would hide different state and
actuator semantics.

## Execution matrix

| Concern | Fixed-wing adapter | Multirotor adapter | Helicopter adapter | Shared authority |
|---|---|---|---|---|
| State/control/output vocabulary | `model.aircraft.derivatives` and linear study artifacts | `extended_state_names()` and `input_names()` | `VehicleModel::{state,control,output}_names()` | Named artifact/report contracts; `VehicleModel` is the target seam |
| Model evaluation | `sim::Aircraft` coefficient model | `Quadrotor::{wrench,derivative}` | `VehicleModel::{wrench,derivative}` | `sim::rigid_body_derivative` and model-specific wrench |
| Fixed-step execution | `sim::simulate_nonlinear` | `sim.plant` segment runner | `sim.helicopter` segment runner | `numerics::integrate` / `integrate_fixed_step` |
| Sampled execution | legacy linear/nonlinear design path | multirotor sampled adapter | `sim::run_sampled_loop` | Tick/saturation/delay/hold ordering is shared where the adapter uses it |
| Environment and events | aircraft schedule contract | wind history with explicit rebasing | `model::Environment`, wind/failure schedules | `InputSchedule` and declared boundary semantics |
| Linearisation | `linearize_finite_difference` | extended/chart linearisation | `linearize_vehicle` | `linearize` algorithms and named matrices |
| Trim | level-flight aircraft trim | hover/cruise trim | declared helicopter trim | trim problems remain model-specific; no false compatibility |
| Reporting | generic linear/nonlinear CSV/report | named multirotor CSV/report | named helicopter trajectory/controller report | named channels, manifests, provenance |

## Evidence and remaining migration

The source paths above are the compatibility baseline. Valid studies must be
captured before replacing an adapter, then rerun with the same compiler/build
configuration and compared by named channel, units, event boundary, and
provenance. Numerical operation ordering is preserved for an adapter migration;
intentional corrections are reported as behavior changes.

Adding a vehicle implementation now requires a model vocabulary and a pipeline
adapter, while numerical integration, rigid-body composition, sampled timing,
linearisation algorithms, and reporting schemas are reusable services. The
remaining C1 work is to route the fixed-wing and multirotor model objects
through `VehicleModel` (or a formally equivalent type-erased seam) and to
replace their private sampled/event/report runners with those services. This
document therefore records C1 as partial rather than claiming that three
independent paths have already become one.

## Reproduction

```sh
cmake --build --preset dev -j 4
ctest --preset dev -R 'ExampleHeli|Quadrotor|Nonlinear' --output-on-failure
```

The helicopter shared-path evidence is in the sampled-loop unit test and the
helicopter acceptance studies. The fixed-wing and multirotor compatibility
paths remain regression-covered and are not altered by this milestone.
