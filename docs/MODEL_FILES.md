# Executable model files

The experimental `galata.model.v1` format describes the
`continuous-scalar.v1` and `continuous-linear.v1` block profiles. It is a signal-flow model, distinct from
[study YAML](STUDY_FILES.md). The runnable
[continuous feedback example](../examples/continuous-feedback/README.md)
contains all five block types and a complete study. The compiler architecture
and limits are specified in
[ADR-0010](adr/0010-continuous-scalar-executable-model.md).
The typed linear extension and state-space lowering are specified in
[ADR-0013](adr/0013-typed-linear-graph-adapter.md) and exercised by the
[NT-33A graph example](../examples/nt33a-graph-design/README.md).

Every root key is required: `schema`, `profile`, `blocks`, `connections`.
Every block requires `id`, `kind` and `output`. Unknown keys, missing values,
duplicate keys/IDs, unsupported versions, aliases, anchors, nonfinite numbers
and ambiguous port bindings are errors. There are no implicit clocks, units,
block parameters, layout fields or external references.

`output` is a closed map with `dimension` and `frame`. Dimensions are eight
integer exponents in the order length, mass, time, current, temperature, amount,
luminous intensity, angle. Each exponent lies in [-16,16]. Values use canonical
SI units; angle uses radians and is distinguished from dimensionless signals.
Frames are `none`, `body` or `ned`, compared exactly without a transformation.

| Kind | Additional required fields | Inputs |
|---|---|---|
| `constant` | `value`: finite binary64 number in the output's units | None |
| `gain` | `coefficient: {value: number, dimension: [eight exponents]}` | One; input dimensions plus coefficient dimensions equal output dimensions |
| `sum` | `signs: [1, -1, ...]` | One per sign, in port order; all match the output type |
| `integrator` | `initial_value`: finite number in the output's units | One derivative input, output units divided by seconds |
| `output` | None | One input matching the declared output type |
| `linear_combination` | `terms: [{input: {dimension, frame}, coefficient: {value, dimension}}, ...]` | One per ordered term; each incoming signal matches its declared input type; available only in `continuous-linear.v1` |

The linear profile admits all five scalar kinds with their existing semantics.
It additionally permits explicit matrix row couplings between declared input
and output frames. Coefficient dimensions plus each term's input dimensions
must equal the output dimensions. These declarations do not compute a frame
rotation or infer coordinate conventions from signal names. Ordinary Gain,
Sum, Integrator and Output frame checks are unchanged in both profiles.

For example, this block fragment declares an acceleration row from a body
velocity and a scalar angle. It needs two connections to ports 0 and 1:

```yaml
id: acceleration
kind: linear_combination
output: {dimension: [1,0,-2,0,0,0,0,0], frame: body}
terms:
  - input: {dimension: [1,0,-1,0,0,0,0,0], frame: body}
    coefficient: {value: -2, dimension: [0,0,-1,0,0,0,0,0]}
  - input: {dimension: [0,0,0,0,0,0,0,1], frame: none}
    coefficient: {value: 3, dimension: [1,0,-2,0,0,0,0,-1]}
```

Every nested row map is closed. A row contains 1–64 terms, evaluates each
product and then accumulates from positive zero in port order. Products and
every intermediate sum must remain finite. A zero coefficient still requires
its input and remains a direct dependency in an authored model. Ordered term
types, coefficient bits and connections are part of semantic identity.
The old scalar profile refuses this block kind and retains its existing
canonical-byte contract.

A connection is `{source: block_id, target: block_id, input: index}`. The source
has one output; `input` is a zero-based integer target port. IDs match
`[A-Za-z_][A-Za-z0-9_-]{0,63}`. Every input is connected exactly once. A producer
may feed several ports. Sum signs and port assignments determine a fixed
left-to-right reduction; changing declaration order does not change it.

Numerical fields use finite decimal syntax and locale-independent conversion
with nearest-even rounding. Conversion preserves the caller's floating-point
rounding mode and exception flags; representable subnormals are supported, while
nonzero decimal values that round to zero are refused.
The C++ serializer emits enough digits for exact binary64 round-trip, including
signed zero and subnormal values. Canonical semantic bytes use exact parameter
bits rather than printed decimal text. Comments, whitespace and declaration
ordering can change raw-source identity while leaving model identity unchanged.
Unknown executable fields are refused rather than omitted from the digest.
Compilation requires the canonical YAML representation to fit the same source
byte cap. A very compact input can therefore parse successfully and still be
refused at compilation; required evidence must be serializable before execution.

## Headless study interface

```yaml
version: 1
stages:
  - id: model
    capability: model.compile
    input: {path: model.yaml}
  - id: response
    capability: sim.model
    input:
      model: {from: model}
      step_s: 0.125
      steps: 8
      sample_stride: 3
      csv_path: response.csv
      evidence_path: evidence.json
```

`initial_time_s` defaults to zero and `sample_stride` defaults to one. Step size,
integer step count and both distinct output paths are required. The paths obey
the normal study input/output containment, snapshot and overwrite rules. A
successful run retains the initial sample, every stride boundary and the final
sample. The step defines numerical execution; it is not an error tolerance.

The CSV labels state and output columns by stable block ID. Their units/frames
are in the canonical source embedded in `galata.model-run.v1` evidence. That
record also contains the semantic digest, ordered state/output/schedule IDs,
solver settings, sample count, trajectory path/hash and four evidence fields:
execution is `completed`; numerical accuracy, model validity and engineering
acceptance are `not_assessed`. The existing run manifest binds this record and
CSV to original input bytes and build/runtime identity.

An optional `model.compile` input `context_path` names a source-origin
attachment, limited to 8 MiB. The file must have outer schema
`galata.project-origin.v1` and pass strict YAML structure checks. Its exact
bytes are retained as a manifest input. The project workflow verifies the
attachment's complete origin bindings; attaching it to a compiled model
does not assign the original Jacobian diagnostics or controller validity to
edited rows. Unsupported origin schema versions are refused.

## Linear-system and controller adapter

`model.linear_graph` accepts exactly one upstream `system: {from: ...}` with
kind `linear_system` or `law: {from: ...}` with kind `control_law`. The latter
uses the LQR design's original plant and feedback matrix. Required inputs are:

| Input | Contract |
|---|---|
| `channel_types` | Closed map of `states`, `inputs`, `outputs`, each an explicit array of `{dimension, frame}` in source matrix order |
| `initial_state` | Finite numeric vector with one value per source state |
| `command` | Finite constant-command vector with one value per source input |
| `model_path` | Generated executable model YAML output, at most 1 MiB |
| `adapter_path` | Distinct adapter JSON output, at most 2 MiB |

All three channel counts must be between 1 and 16. No types are inferred from
names or free-text units. The adapter materializes default C = I and D = 0,
then lowers `x_dot=A*x+B*u` and `y=C*x+D*u`. A system uses `u=command`; an LQR
law uses `u=command-K*x`. Command constants, state integrators, typed rows,
plant outputs and actual-control outputs are editable graph blocks.

The deterministic `linear-rows.v1` lowering omits only exact-zero matrix
entries, including signed zero, before constructing graph ports. A row with
no retained entries becomes a typed zero constant. This translation rule does
not remove zero terms from an already authored graph. Coefficient dimensions
are derived from the declared input and output types and must fit the existing
exponent bounds. The resulting artifact is an `executable_model`, directly
consumed by `sim.model` through the ordinary graph executor. It does not add
`sim.linear`'s eigenvalue step screening to the graph runtime.

The `galata.linear-adapter.v1` record retains the lowering version, generated
model path and semantic SHA-256, effective source A/B/C/D matrices and ordered
names, description/citation/units, channel types, initial state, command and
feedback gain. Its `mapping` arrays are `state_ids`, `command_ids`,
`control_ids`, `output_ids` and `control_output_ids`, each in source channel
order. `feedback_gain` is an empty array for an open-loop system. LQR imports
also retain `lqr_origin` Q/R/N/K matrices and CARE solution, residual budget,
residual, symmetry, conditioning, separation and closed-loop eigenvalues.

Use the mapping to locate state, plant-output and actual-control CSV columns;
compiled output order follows block IDs. The enclosing pipeline manifest
retains original source bytes and complete upstream linearization diagnostics.
Those diagnostics and the LQR evidence describe source operations only.
The adapter does not validate the aircraft, assess integration accuracy or
certify that later graph edits preserve the original plant/controller.

## Resource bounds and C++ API

Both profiles retain the shared limits: 1 MiB source, 1,024 blocks, 8,192
connections, 64 sum inputs, 64 linear-row terms, 1,000,000 integration steps,
1,000,000 recorded scalars including time, and 100,000,000 scheduled block
evaluations. The compiler checks serializability before execution. Runtime
budgets apply to the submitted graph and options, independent of its origin.

The C++ API provides parsing, serialization, canonical semantics, compilation,
pure evaluation, fixed-step simulation and cooperative cancellation in
[model.hpp](../include/galata/modeling/model.hpp). Link against the installed
`galata::modeling` CMake target. Experimental versioning does not promise a
frozen C++ ABI or compatibility with third-party Simulink/SLX files.
The pure state-space lowering API is in
[linear_adapter.hpp](../include/galata/modeling/linear_adapter.hpp); it performs
no file I/O and introduces no separate integration solver.
