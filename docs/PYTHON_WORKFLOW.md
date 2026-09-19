# Python engineering workflow

`galata-engineering` is an installable, machine-readable client of the
authoritative Galata C++ CLI. It is an execution API, not a report parser or a
second numerical implementation.

## Installation and executable selection

Build the CLI, then install the package into the environment that will run the
workflow:

```sh
python3 -m pip install --no-deps .
python3 examples/shared-vehicle-python-workflow.py \
  --executable /absolute/path/to/galata \
  --output-dir /absolute/path/to/results
```

The executable is required explicitly by the example. `GalataWorkflow` also
accepts a bare executable name resolved through `PATH`. A missing executable,
model, or incompatible artifact raises a structured `WorkflowError` subclass.

## Composed operations

The public operation order is:

```python
workflow = GalataWorkflow("/absolute/path/to/galata")
model = workflow.load_model("models/nt33a/nt33a-fc1.yaml")
study = workflow.configure(
    model, "results",
    parameter_overrides={"mass.mass_kg": 5405.0},
    flight_condition={"airspeed_m_s": 69.4944, "altitude_m": 0.0},
)
trim = workflow.trim(study)
linear = workflow.linearize(trim)
controller = workflow.design(linear)
simulation = workflow.simulate(controller, steps=1000, sample_stride=10)
evaluation = workflow.evaluate(simulation)
bundle = workflow.export(simulation, "results/bundle",
                         outputs=["response.json", "open.csv", "closed.csv"])
```

Every stage invokes the C++ capability with a generated, immutable run
directory. `trim`, `linearize`, `design`, and `simulate` return artifacts with
real numerical values, not descriptive dictionaries. Each result retains its
run manifest and upstream dependency. The versioned schemas are:

| Artifact | Schema |
| --- | --- |
| model inspection | `galata.vehicle.schema.v1` (shared) or legacy `galata.helicopter.schema.v1` |
| trim | `galata.vehicle.trim.v1` (shared) or legacy `galata.helicopter.trim.v1` |
| linear system | `galata.linear_system.v1` |
| controller | `galata.control_law.v1` |
| response | `galata.vehicle.response.v1` (shared) or legacy `galata.helicopter.response.v1` |
| portable bundle | `galata.bundle.v1` |

Model schemas expose named parameters, states, controls, outputs, units and
frames. Fixed-wing, multirotor, and helicopter models can all use the shared
`model.vehicle` → `trim.vehicle` → `linearize.shared` → `sim.vehicle` path;
the helicopter-specific capability names remain compatibility adapters.
Parameter overrides are validated in Python and applied again by the C++ model
loader; schema identity and trim artifacts record the values that reached
execution. Invalid names, numerical refusals, and incompatible result types
fail explicitly.

## Comparisons and result bundles

`GalataWorkflow.compare()` is full-output bitwise comparison. It passes only
when both runs have the same explicitly selected, non-empty output set and the
manifest digest for every selected output matches. Empty, disjoint, missing,
extra, or partially matching sets do not pass. Use `compare_subset()` only for
a declared subset, and `compare_numeric()` for CSV values with explicit
tolerances; those policies are not interchangeable.

`export()` copies the selected files, optional plots, decoded pipeline
configuration snapshots, a provenance summary, and a bundle manifest
containing relative paths and rechecked SHA-256 digests. It refuses to
overwrite a result directory unless `overwrite=True` is supplied and the
destination is empty. The resulting directory can be copied outside the source
run directory.

## Plotting and playback

The offline plotter reads named CSV channels and validates time ordering,
lengths, units and frame conventions. Open/closed plots require an exact time
grid unless `alignment="linear"` is selected explicitly. Missing observations
remain gaps in SVG traces. Sweep plots use the declared parameter name and
unit. Ensemble plots identify member values (not a population distribution),
failed/excluded members, and the included denominator.

`plot_trajectory_projection()` is the static NED-to-display projection. The
separate `plot_trajectory_playback()` writes a self-contained HTML scrubber
using recorded scalar-first Hamilton quaternions. It displays North/East/up,
with `up = -down`, and rotates body-forward into NED for each recorded frame.
PNG export is optional and requires `pip install 'galata-engineering[png]'`.

The delivered interface remains CLI-backed and uses the authoritative C++
implementations. The shared cross-vehicle path reports basic tracking metrics;
family-specific failure, sampled-sensor, and actuator analyses remain on their
compatibility capabilities. Native Python numerical bindings, estimators,
higher-fidelity rotor physics, and full-aircraft/measured-data validation remain
open. The UH-60 geometry calculation remains source-transcription verification,
not a test of Galata's rotor model.
