# Executable model files

The experimental `galata.model.v1` format describes the
`continuous-scalar.v1` block profile. It is a signal-flow model, distinct from
[study YAML](STUDY_FILES.md). The runnable
[continuous feedback example](../examples/continuous-feedback/README.md)
contains all five block types and a complete study. The compiler architecture
and limits are specified in
[ADR-0010](adr/0010-continuous-scalar-executable-model.md).

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

The C++ API provides parsing, serialization, canonical semantics, compilation,
pure evaluation, fixed-step simulation and cooperative cancellation in
[model.hpp](../include/galata/modeling/model.hpp). Link against the installed
`galata::modeling` CMake target. Experimental versioning does not promise a
frozen C++ ABI or compatibility with third-party Simulink/SLX files.
