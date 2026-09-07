# ADR-0013: Typed linear rows and an explicit state-space graph adapter

- **Status:** experimental M2 contract; implemented with independent local checks, external acceptance pending
- **Date:** 2026-09-07
- **Deciders:** maintainer review pending

## Context

The M2 preview can author scalar feedback but cannot reconstruct the existing
NT-33A trim, linearization and controller study. A state-space matrix contains
couplings between quantities with different dimensions and coordinate
references. Lowering such entries through the existing Gain and Sum blocks
would require erasing frame information or changing the meaning of the
`continuous-scalar.v1` profile. Neither is an acceptable implicit conversion.

The existing `LinearSystem` describes `xdot = A x + B u`, `y = C x + D u`.
It preserves ordered channel names and free-text units, but does not provide
machine-readable signal types. A graph adapter must require those types and
preserve the original matrix/channel ordering independently of compiler ID
ordering. A successful reconstruction establishes integration consistency with
the existing engine, not independent aircraft validation.

## Decision

Keep `galata.model.v1` and introduce the explicit `continuous-linear.v1`
execution profile. This profile admits the five existing scalar block kinds
and an ordered `linear_combination` block. The existing profile continues to
reject this new kind; its canonical byte contract and frame checks remain
unchanged. Existing readers refuse the new profile.

Each linear row has one scalar output and one input per ordered term. A term
contains `input: {dimension, frame}` and
`coefficient: {value, dimension}`. Its incoming signal must exactly match the
declared input type. Coefficient dimensions plus input dimensions must equal
the output dimensions. Every declared input and output frame must be supported.
Distinct input/output frames declare a coordinate coupling; the row does not
compute a rotation or infer axes, scale conversions or physical correctness.
Gain, Sum, Integrator and Output retain their existing type rules in both
profiles. Scalar frame tags alone cannot identify a vector component or an
Euler-angle convention.

Rows have 1 through 64 terms. Evaluation multiplies each input by its
coefficient, then adds the products from left to right starting at positive
zero. Products and every intermediate accumulation must be finite. Every
input is a direct dependency, including a term with an exact zero coefficient.
Canonical semantics include ordered input types and coefficient bits.
Declaration permutation cannot alter results; term/port permutation can.
Existing source, graph, simulation, cancellation and resource bounds apply.

### Pure adapter boundary

The public adapter accepts a validated `LinearSystem`, explicit ordered state,
input and output `SignalType` arrays, and options containing initial state,
command and optional feedback matrix K. Each channel count is 1 through 16;
all arrays and matrices must have exactly the declared shape and finite
values. An empty K means no state feedback; otherwise K is m by n. There are
no default channel types inferred from names or free-text units.

The adapter constructs the equations

```
u = command - K x
xdot = A x + B u
y = C x + D u
```

using typed rows, integrators, editable command constants and output blocks.
It materializes the existing default C = I and D = 0 semantics. For emitted
nonzero terms, coefficient dimensions are derived from the explicit
source/target types and refused if they exceed the supported exponent range.
Source labels remain descriptive;
generated IDs are deterministic index names such as `state_000`,
`command_000`, `control_000`, `derivative_000`, `output_000` and
`control_output_000`. Returned ID arrays map each source channel to its graph
object. Controls are observable separately from plant outputs.

The versioned `linear-rows.v1` lowering omits only exact-zero matrix entries,
including their ports. It never uses an epsilon or removes an edge from an
already authored graph. A row with no retained terms becomes a typed zero
constant. This explicit translation avoids manufacturing a direct-feedthrough
cycle from an absent D coupling. Original matrices, including omitted entries,
remain in adapter evidence. Lowered arithmetic follows graph port order and
does not promise bit identity with Eigen matrix multiplication or with
precomputing A - B K.

The adapter has no file-system, project or GUI ownership. Graph compilation
and execution continue to use the existing pure evaluator and RK4 kernel.
The model runtime does not acquire `sim.linear`'s eigenvalue-based step
screening merely because the graph originated from a linear system.

### Pipeline, provenance and projects

`model.linear_graph` consumes exactly one upstream `system` or `law`. The law
route preserves its plant and materializes its state-feedback K. Required
inputs include explicit channel types, initial state, command, model output
path and adapter-evidence output path. It produces the compiled executable
graph and writes canonical model source plus `galata.linear-adapter.v1`
evidence. Both files use ordinary pipeline output containment and preflight.

Evidence retains ordered A/B/C/D matrices and names, declared types,
initialization and controller settings, lowering version, channel-to-block
mapping, model semantic digest and source controller information. Existing
pipeline propagation must preserve complete source linearization diagnostics
through the adapter and simulation artifacts and into the run manifest.
Source byte snapshots and build/runtime identity remain authoritative in that
manifest; a citation or human summary cannot replace them.

Project import retains the adapter source and its originating run evidence as
immutable provenance. Editable model revisions and their runs retain their own
identities. A changed controller, connection or state initialization changes
the executable model identity; imported evidence continues to describe the
origin and must not certify equivalence of a later edited model. Presentation
changes remain separate from executable semantics. Project schema/version,
retention and verification rules are specified by the project protocol rather
than by this numerical adapter.

## Independent acceptance protocol

The tests are written from this contract and the public interfaces without
reading the new implementation, following [TESTING.md](../TESTING.md).
References and budgets build on
[MODEL_CONFORMANCE.md](../architecture/MODEL_CONFORMANCE.md), especially
B01 and B03. Results belong in the implementation record, not in this protocol.

1. Exactly representable dyadic fixtures combine multiple inputs, unequal
   dimensions and explicit coordinate couplings. A two-state fixture uses
   nonidentity C, nonzero D and nonzero feedback K. Direct evaluation at a
   supplied noninitial state must give the hand-derived derivatives, plant
   outputs and actual controls. Mapping uses IDs rather than vector insertion
   order. A scalar affine fixture additionally follows the independently
   derived RK4 polynomial recurrence at every retained sample.
2. Row input mismatch, coefficient dimension mismatch, unsupported profile,
   invalid frames, missing/duplicate ports, zero-term instantaneous cycles,
   nonfinite declarations, intermediate product/sum overflow and excessive
   terms/channels must refuse. Old-profile Gain/Sum frame equality remains
   effective. A 64-term row and practical channel boundary cases are admitted.
3. YAML round-trip and source-declaration permutations preserve canonical
   identity and results. Changing ordered term types, coefficients or port
   assignments changes executable identity. The existing hand-specified
   scalar canonical-byte fixture remains required.
4. The NT-33A example reconstructs the existing control-design trim, selected
   plant, LQR weights and initial state. Compare all 401 samples at h = 0.005 s,
   4,000 steps and stride 10 with the existing `sim.linear` response. Compare
   all source states and plant/control outputs through their explicit mapping;
   matching only the final endpoint is insufficient.
5. Inspect adapter files and manifest bindings, complete `full_plant`
   linearization records, source bytes, compiler/build identity and unchanged
   evidence limitations. The execution field may report completion while
   numerical accuracy, model validity and engineering acceptance remain
   `not_assessed`.

For the scalar affine fixture, each rounded operation allowance is counted as
in B01, with the independent polynomial recurrence included. The NT-33A
comparison uses a budget calculated before comparing observed trajectories.
Let P = I + h Ac + (h Ac)^2/2 + (h Ac)^3/6 + (h Ac)^4/24 for Ac = A - B K,
computed independently in extended precision. Let G be the maximum infinity
norm of P^j for j = 0 through N. Normalize each component by one unit of its
declared SI quantity before using these numeric norms. Let F be the infinity
norm of |A| + |B||K| and S = max(1, ||x0||inf)(1 + h F)^4.

The operation allowance q = 64 (b + t + n(n + m) + n + 1) per step counts
graph blocks b, ordered row terms t, dense reference products and RK state
assembly for both execution paths. With gamma = q(N + 1)epsilon /
(1 - q(N + 1)epsilon), the state allowance is
8 gamma max(1, G)^2 S. The factor eight covers coefficient formation,
extended-precision reference calculation and the two compared paths; the
linear recurrence amplification is accounted for by G and the accumulated
operation count. Output allowances also multiply by the corresponding
absolute C/D/K row sums and include their local arithmetic allowance. This
bounded-fixture integration-consistency budget is not an arbitrary-model
accuracy certificate. Nonfinite budget terms or gamma outside its small-error
domain invalidate the comparison rather than widening the budget after a
failure.

## Consequences and deferred work

The change supplies an inspectable local linear plant/controller graph. It
does not add nonlinear aircraft blocks, vector/bus semantics, time-varying or
scheduled matrices, sampled controllers, saturation, events or algebraic-loop
solving. Published NT-33A reference tests remain separate from shared-engine
agreement. M2 desktop acceptance, independent aircraft-data review and
platform qualification remain open gates.
