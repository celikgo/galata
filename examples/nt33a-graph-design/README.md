# NT-33A local linear graph study

This study turns the existing NT-33A derivative model and illustrative LQR
design into an editable, typed continuous graph. It trims the reference
airframe, linearises the longitudinal motion, selects the elevator, and lowers
the resulting plant and feedback law into explicit state, controller and output
rows. The graph runs through the ordinary model compiler and RK4 executor.

Run from the repository root:

```sh
build/dev/src/cli/galata run examples/nt33a-graph-design/study.yaml --output-dir build/nt33a-graph-study
```

Existing outputs require `--overwrite`. To create a desktop project with a
retained source origin and the study's run settings:

```sh
build/dev/src/cli/galata project import-linear build/nt33a-graph-project examples/nt33a-graph-design/study.yaml
```

The study writes `model.yaml`, `adapter.json`, graph and linear-reference CSV
histories, graph execution evidence, and Markdown/HTML reports. The separate
run manifest binds the consumed inputs and outputs to their hashes and retains
the full source linearization diagnostics. `adapter.json` records the effective
A/B/C/D matrices, source names and citation, explicit channel types, initial
state, command, feedback gain, source-order graph mappings, and original LQR
weights and CARE diagnostics. The source model is documented in
[the NT-33A model provenance](../../models/nt33a/PROVENANCE.md); the design inputs match
[the existing control-design example](../nt33a-control-design/README.md).

Channel declarations are explicit arrays in matrix order: `u,w,q,theta`, then
`elevator`, with identity state outputs. Velocity and pitch rate use the body
frame; pitch and elevator angles are scalar coordinates without a vector
frame. The eight dimension exponents are length, mass, time, current,
temperature, amount, luminous intensity and angle, all in canonical SI.
Matrix row coefficients declare coordinate couplings; no frame rotation or
unit conversion is inferred from names or free-text units.

`model.linear_graph` accepts exactly one `system` or `law` reference. A system
uses `u=command`; an LQR law uses its plant and `u=command-K*x`. Every state,
input and output needs a declared type, and `initial_state` and `command` need
one finite value per corresponding channel. Each axis supports 1–16 channels.
Generated model YAML is limited to 1 MiB and adapter JSON to 2 MiB. Model and
adapter paths are required and distinct, and ordinary graph resource limits
apply.

Use `mapping.state_ids` and `mapping.output_ids` in the adapter to compare graph
CSV columns with the source-order reference history. `mapping.control_output_ids`
identifies the actual elevator perturbation, which is an additional graph
output. Both runs use a 0.005 s RK4 step for 20 s, recording 401 samples. Matrix
and graph arithmetic can differ by floating-point rounding; agreement is a
numerical comparison, not a bitwise contract.

The pitch perturbation and LQR weights are illustrative. This local linear
model omits nonlinear envelope enforcement, actuator dynamics, saturation and
state estimation. Its source linearization and CARE diagnostics qualify their
source operations only. Graph execution does not assess numerical accuracy,
physical validity or engineering acceptance. Editing graph rows retains the
origin record but does not preserve the original plant or controller relation.

For project reruns, optional `model.compile.context_path` attaches a
`galata.project-origin.v1` document, limited to 8 MiB. Its exact bytes are
retained as a manifest input; the attachment supplies source context without
assigning the original Jacobian diagnostics to an edited graph. Unsupported
origin schema versions are refused.
