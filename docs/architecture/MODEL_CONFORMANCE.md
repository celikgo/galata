# Continuous scalar model conformance protocol

**Status: predeclared M1 acceptance protocol.** Execution status is recorded in
[the M1 implementation record](../product/M1_IMPLEMENTATION.md). This is a
bounded synthetic benchmark for the first executable signal-flow model profile.
It records expected equations, rejection behavior and numerical budgets before
running the implementation. It does not establish aircraft-model validity,
handling-quality compliance, tool qualification or complete Simulink parity.

The protocol refines D07 in [discovery](../product/DISCOVERY.md) and W09 in
[delivery](../product/DELIVERY.md), under the
[executable-model architecture](EXECUTABLE_MODEL.md). The source model is a
signal-flow graph whose feedback can express differential equations. It is
distinct from the existing acyclic study pipeline, which orders completed
engineering operations.

The implementation owner and independent test owner must resolve any interface
choice inconsistent with this protocol before declaring a case passed. A
discovered failure changes the implementation or receives a documented scope
decision; observed output must not become the reference or justify widening a
budget. Record actual results in generated test evidence, not in this protocol.

### A second profile now exists

This document specifies `continuous-scalar.v1` only. The explicit
`continuous-linear.v1` profile adds one ordered `linear_combination` block kind
and is specified, with its own predeclared acceptance protocol and budgets, in
[ADR-0013](../adr/0013-typed-linear-graph-adapter.md). That protocol is the
sibling of this one rather than a revision of it: it builds directly on the
rounding allowance in B01 and the canonical-identity requirement in B03 below,
and it leaves every rule in this document effective for the scalar profile.

The scalar profile continues to reject `linear_combination`, and its canonical
byte contract and frame-equality checks are unchanged by the new profile's
existence. Anything stated here about the five scalar block kinds should be
read as scoped to those kinds. The two profiles share one compiler, evaluator
and RK4 executor, so the execution-conformance meaning established here carries
across; the numerical budget for a linear-row model does not, because a row's
allowance depends on its ordered term count.

## Accepted first profile

The initial profile contains finite binary64 real scalars, explicit SI signal
units and explicit frame identities. Its built-in blocks are constant, ordered
sum, gain, continuous integrator and output. Every model has stable block IDs,
unambiguous input-port identities, a supported schema/profile version and an
explicit output interface. Duplicate declarations or conflicting writers are
errors. Fan-out and connecting one source to several distinct input ports are
valid and retain their multiplicity.

Signal units are part of the model's engineering meaning. A dimensioned gain
multiplies the input unit by the gain unit. For example, a coefficient measured
in inverse seconds maps position in metres to a derivative in metres per
second. An integrator's input unit is its output unit divided by seconds; its
initial condition has the output unit. Constants declare their signal units,
sum inputs have the same unit, and outputs preserve their input unit. There is
no implicit conversion, frame rotation or assumption that two frame names are
equivalent. The supported unit vocabulary must be explicit; unsupported unit
expressions and quantity kinds are rejected rather than simplified silently.

Scalar frames denote the declared signal reference. They do not supply a
vector component, axis transformation or a physical interpretation of an
otherwise untyped number. Tests use distinct explicit frame identities to
exercise equality and refusal, without inferring aircraft frames from names.

The semantics of the five block types are:

| Block | Output | State/derivative | Instantaneous dependency |
|---|---|---|---|
| Constant | Declared finite value | None | None |
| Sum | Sum of its input values in declared input-port order | None | Every input |
| Gain | Declared coefficient multiplied by its input | None | Input, including when the coefficient is zero |
| Integrator | The value of its owned continuous state slot | Input is the derivative of that slot | Output has no direct dependence on the current input |
| Output | Connected signal value | None | Input |

At each derivative evaluation, integrator outputs come from the supplied
temporary state. Combinational outputs are then evaluated in the compiled
instantaneous order, and integrator inputs become the derivative vector. Every
state derivative is evaluated from the same temporary state. No block commits
state, reuses a prior stage's output or advances an accepted time sample during
an RK stage.

The compiler rejects every instantaneous cycle before execution, including
self-loops, cycles with a zero gain and cycles outside the selected output
paths. A continuous integrator breaks direct feedthrough while retaining its
feedback equation. The compiler must not delete an edge, inject an implicit
delay or attempt an algebraic solve to make a source graph executable.

Stateless graphs are supported as direct evaluations on the declared recording
time grid. They have no fabricated continuous state. Unsupported sampled
clocks, unit delays, hybrid events, external callbacks, variable-size signals,
Boolean/matrix blocks, subsystem recursion and implicit solvers are refused in
this first profile. Their appearance in the broader architecture is not an
implementation claim for this increment.

## Canonical compilation and observation

Canonical ordering uses stable semantic identities. Reordering source block or
connection declarations while preserving IDs and port assignments cannot
change state/output ordering, semantic representation, the schedule or results.
The reduction order of a sum is part of its port contract. It must not depend
on connection insertion order, pointer addresses, hash-container iteration or
topological traversal accidents. Source port order changes that alter the
floating-point reduction are semantic edits.

A compiled model owns a resolved snapshot of its source. Mutating or destroying
the original source document after compilation must not change a run. Separate
integrator instances own separate state, even if their parameter values are
identical. Evaluation is observationally pure: evaluating temporary state B
between two evaluations of state A cannot change the result at A.

Time is specified by a finite origin, a positive finite step and an integer
step count. Recording uses the initial sample, declared stride boundaries and
the final sample, without duplicating the final sample when it already lies on
a stride boundary. Zero steps produce exactly one initial sample. Recorded
times are formed from origin plus integer index times step; recording stride
does not change the derivative evaluations or final state.

Same-binary repeats compare the bits of recorded doubles, including the sign
of zero where the format preserves it, and compare semantic bytes exactly.
Cross-platform numerical comparisons use the stated analytic error budgets;
this new protocol does not silently expand the existing fingerprint battery's
scope. See [ADR-0004](../adr/0004-determinism-policy.md). A model semantic digest
alone is not a complete run identity: numerical settings and runtime/build
identity also matter.

## Independent mathematical references and budgets

The continuous references below follow directly from constant-coefficient
ordinary differential equations. The discrete references are independently
derived from the classical four-stage RK4 stability polynomial, rather than
obtained by calling the production integrator:

\[
R(z)=1+z+z^2/2+z^3/6+z^4/24.
\]

References are E. Hairer, S. P. Nørsett and G. Wanner, *Solving Ordinary
Differential Equations I: Nonstiff Problems*, second revised edition, Springer,
1993, and J. C. Butcher, *Numerical Methods for Ordinary Differential Equations*,
third edition, Wiley, 2016. These are also the mathematical references of the
[existing integration API](../../include/galata/numerics/integrator.hpp).
The elementary fixture equations and error derivations are written here so an
independent reviewer can reproduce the expected values without running Galata.

| Budget | Predeclared meaning |
|---|---|
| B01 — Floating arithmetic | For a path with q rounded elementary operations, use gamma(q) = q epsilon / (1 − q epsilon), with q epsilon < 1. Scale by the sum of absolute intermediate contributions, propagated through the stated linear recurrence; include the independent reference evaluation's operation allowance. Exactly representable dyadic fixtures require exact equality where every intermediate operation is exact. This budget does not cover overflow, underflow or ill-conditioned cancellation by pretending the result has small relative error. |
| B02 — Continuous solution | Add the independently derived RK4 truncation bound to B01. For the scalar decay fixture with 0 < h ≤ 1/8 and N steps, the absolute truncation error is at most N h^5 / 120 times the initial-state magnitude. For the oscillator, use N exp(h) h^5 / 120 in the Euclidean norm of the state normalized by its declared fixture units. These are deliberately conservative mathematical upper bounds for the declared nondimensional fixtures, not observed product errors. |
| B03 — Discrete RK4 solution | Compare against R(−h)^N for scalar decay and the real/negative-imaginary parts of R(i h)^N for oscillator position/velocity, allowing only B01 and the declared reference-rounding allowance. For h = 1/8, R(−h) = 86753/98304; this rational one-step reference exposes an incorrect stage schedule without deriving the reference from production code. |
| B04 — Refinement | Compare h, h/2 and h/4 at the same final time. Each run must satisfy B02/B03. Where reporting an observed order, compare with the independent polynomial recurrence and propagate B01 through the differences and logarithm. Do not require an order value when its differences are at or below the floating-point allowance. An unavailable order is explicit, never replaced with four. |

Here h is dimensionless time for fixtures with a one-second time scale. The
test must retain seconds in the graph's signal and simulation metadata. The
decay bound follows from the alternating exponential remainder and
0 < R(−h) ≤ 1: each local defect is at most h^5/120 and propagation is
contractive. The oscillator bound uses the exponential-series remainder and
the identity

\[
|R(i h)|^2 = 1-h^6/72+h^8/576 \leq 1
\]

for the selected step range. Summing the propagated local bounds gives B02.
These arguments do not provide an error certificate for arbitrary user models.

For the oscillator comparison, normalize position by one metre, velocity by
one metre per second and time by one second. The normalized variables obey
q prime = w and w prime = −q. Thus q_N is the real part of R(i h)^N and w_N is
its negative imaginary part. This normalization avoids adding errors with
different physical units in the norm comparison.

Test code must state its operation-count and scale choice when instantiating
B01. It must not select a single unexplained decimal tolerance for unrelated
fixtures. A conservative count may cover several elementary operations per
block/RK stage, but it must stay tied to the bounded fixture and its conditioning.
An analytical reference using exp, sin or cos also declares a small libm
rounding allowance; the rational/polynomial reference independently checks the
integrator where that allowance alone would be insufficient.

## Positive conformance cases

| Case | Fixture and independent expected behavior | Acceptance |
|---|---|---|
| MC01 | Stateless constants, ordered sum, gain and output form an exactly representable dyadic expression, such as twice the difference between three and one. | Output is four; no continuous state is allocated; metadata and recording times follow the declared interface. Exact arithmetic applies to this fixture. |
| MC02 | Constant-rate integral: x dot = 3/4 metre per second, x(0) = 1/2 metre; h = 1/8 second and N = 16. | Every retained state/output agrees with x(t) = 1/2 + 3t/4 under B01; intermediate recording and nonzero time origin do not shift initialization. |
| MC03 | Exponential feedback: x dot = −x/(1 second), x(0) = 1 metre; h = 1/8 second and N = 8, with two refinements. | Valid dynamical feedback compiles; x(t) = exp(−t) metres satisfies B02, and the discrete amplification satisfies B03/B04. Output equals the state at each accepted time. |
| MC04 | Affine feedback using all five block types: x dot = 1 metre per second − x/(1 second), x(0) = 0. | x(t) = 1 − exp(−t) metres and discrete x_N = 1 − R(−h)^N satisfy the corresponding B02/B03 budgets. Constants and feedback are reevaluated at temporary RK states. |
| MC05 | Coupled oscillator: position dot = velocity; velocity dot = −position/(1 second)^2; position(0) = 1 metre and velocity(0) = 0. | Position/velocity agree with cos(t) and −sin(t) in the declared units and with R(i h)^N under B02/B03. Reordering integrator declarations cannot serialize the simultaneous state update. |
| MC06 | Two independent decay-state instances have different initial values and separate outputs. Evaluate two supplied temporary states in interleaved order, then run twice. | State ownership, source-snapshot ownership and evaluator purity hold; each trajectory follows its own analytic initial value. |
| MC10 | Permute block and edge declaration order while preserving IDs and all input-port assignments. | Canonical semantic bytes, compiled state/output order and repeat-run samples are identical. Changing a coefficient, initial value, connection or signal type changes semantic identity. |
| MC11 | Ordered sum receives 10^16, −10^16 and 1 through explicitly ordered ports; one producer also fans out to two distinct ports in a separate exact fixture. | The declared reduction yields its binary64 result in fixed port order across source permutations; repeated input multiplicity is retained. Reassociation is not accepted as equivalent. |
| MC12 | Zero, one and several steps; strides dividing and not dividing the step count; a nonzero origin; repeated execution; output and integrator declarations in reversed order. | Exactly the declared initial/stride/final samples appear, with correct labels and unchanged final dynamics. Times and same-binary result bits repeat. |

MC03–MC05 must include direct derivative/output evaluation at a supplied state,
where the public contract exposes it. This separates equation-lowering defects
from the integration error, and catches a derivative that was constructed from
initial values alone. The oscillator output comparison must use signal/state
identities, not assume an insertion-dependent vector order.

## Refusal and bounded-resource cases

| Case | Invalid or extreme fixture | Required behavior |
|---|---|---|
| MC20 — Structure | Empty required IDs, duplicate block IDs, unknown endpoints, nonexistent ports, missing required inputs, more than one writer to a port, unsupported version and an empty model/output interface. | Refuse compilation with source-local diagnostics. No partially compiled object is accepted for execution. A valid fan-out is not mistaken for a duplicate writer. |
| MC21 — Types | Unequal sum units; an integrator with input unit equal to its state unit instead of state/time; a dimensionally incorrect gain; equal units with different frames; an unsupported quantity/unit spelling. | Refuse before numerical evaluation. Do not add a conversion, erase a frame or reinterpret an initial condition. |
| MC22 — Feedthrough | Gain self-loop; multi-block sum/gain algebraic loop; zero-gain cycle; unobserved algebraic cycle. | Refuse with the cycle's participating source identities. The valid integrator feedback in MC03/MC04 remains accepted. No hidden delay or iterative solve is permitted. |
| MC23 — Unsupported semantics | Discrete/sample-time declarations, rate transitions, delays, events, custom callbacks, unsupported block kinds or shapes. | Emit an unsupported-profile diagnostic, including when a value could be coerced into the scalar continuous profile. Do not accept and ignore a field. |
| MC24 — Finite arithmetic | NaN/infinite parameter or initial state; finite coefficients whose product/sum overflows; finite initial states producing a nonfinite derivative, RK temporary state or output. | Reject nonfinite declarations or terminate evaluation explicitly. No completed trajectory, zero replacement or engineering pass is returned. Include overflow in an output branch even if the derivative remains finite. |
| MC25 — Time | Nonpositive/nonfinite step; negative count; nonpositive stride; nonfinite origin/span; a positive step too small to advance the required RK stage times; a subnormal step whose RK4 weight vanishes. | Reject before a completed result. Zero steps retain the documented initialization checks and do not bypass invalid graph values. |
| MC26 — Budgets | Fixed profile caps at and just beyond block/edge/sum-port counts; step/sample/evaluation limits; checked size products near the integer maximum; a large fan-out or deep chain. | Accept practical graph-count boundaries and refuse the first excess without overflow, unbounded recursion or huge allocation. Validate size arithmetic before reserving buffers or iterating. Runtime excess cases must refuse before performing the large run; tests do not exhaust host memory. |

Resource limits are explicit run/compiler policy, not aircraft validity limits.
At minimum, graph counts, retained scalar sample counts and scheduled operation
counts must be bounded using checked arithmetic. Count initial and final
observations in the recording budget; a stateless model must not evade the
output budget merely because it has no continuous state. Record whether a
limit is a logical work/storage bound or an enforced process memory/time bound.
The first does not imply the second.

The initial API fixes these policy caps: 1024 blocks, 8192 connections, 64
inputs per sum, 1000000 steps, 1000000 recorded scalars including times, and
100000000 scheduled block evaluations. The source document byte cap is one
mebibyte. There is no configurable-limit API in this profile. State and output
counts are bounded by graph size and the recording limit. These are engineering
policy limits, not measured capacities or numerical-accuracy thresholds.

Parser byte/depth/alias limits are a separate boundary around the same source
contract. Their tests must include malformed and oversized documents and
unsupported keys, while direct C++ construction receives the same semantic
checks. Parser success is not permission to bypass compilation validation.

## Evidence and promotion

Tests report which profile, case and budget they exercise. Analytic kernel tests
belong to the unit tier; source-to-run contract tests belong to integration;
same-binary repeatability tests belong to determinism when added to that
battery. A case does not acquire an aircraft-validation label because it uses
metres, seconds or an oscillator. Follow the independent-reference rules in
[testing](../TESTING.md).

The first promotion requires the accepted positive and negative cases on the
supported build matrix, meaningful sanitizer coverage and review of the source
mapping and numerical metadata. A successful compiler/run establishes only
execution conformance for this bounded profile. Numerical reliability, model
validity and engineering acceptance retain separate evidence states under
[ADR-0008](../adr/0008-numerical-evidence-authority.md).

Sampled, hybrid, reset, multirate, subsystem and aircraft blocks require their
own predeclared protocols before admission. The continuous fixtures provide a
foundation for D07; they do not close its broader event/clock-order decisions.

`linear_combination` was admitted through exactly that route. Its protocol is
[ADR-0013](../adr/0013-typed-linear-graph-adapter.md), predeclared before the
implementation and carrying its own positive, refusal and integration-consistency
cases. Admitting it did not relax this protocol, and reconstructing an aircraft
plant through typed rows establishes agreement with the existing engine — not
independent aircraft validation, which remains a separate evidence dimension.
