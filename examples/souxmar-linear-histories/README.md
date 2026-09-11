# Declared input histories through a linear model

`sim.linear` integrates a continuous linear model. It used to take one constant
input for the whole run. It now also takes a **declared input history**, in the
same schema `sim.plant` reads for the nonlinear plant, so a history's timing
means the same thing on either path. Its values are the model's own inputs. For
a linearisation, those are deviations from the trim.

The model here is the native Souxmar plant's hover linearisation, read from
`models/souxmar-quad/souxmar-quad.yaml` in place. It has sixteen chart states,
including the four rotor speeds with their first-order lag. It has seven inputs:
four rotor-speed commands and three wind components.

## Run it

```bash
galata run examples/souxmar-linear-histories/study.yaml
```

| Stage | Capability | What it adds |
|---|---|---|
| `plant`, `hover`, `linear` | `model.quadrotor`, `trim.hover`, `linearize.extended` | The native model, its hover at 120 m, and the linearisation about it. |
| `rotors` | `model.channels` | The four rotor commands, observed through the altitude. |
| `all_inputs` | `model.channels` | All seven inputs, observed through the north position and the altitude. |
| `doublet` | `sim.linear` | A collective rotor doublet under a **zero-order** hold. |
| `spoolup` | `sim.linear` | A collective spool-up while a headwind builds, under a **linear** hold. |
| `doublet_csv`, `spoolup_csv`, `report` | `report.csv`, `report.markdown` | Every sample with the input in force at it, and the account of both runs. |

## The schema

```yaml
input_schedule:
  hold: zero_order          # or linear; required, no default
  extrapolation: hold       # or refuse; required, no default
  samples:
    - {time_s: 0.0, values: [0.0, 0.0, 0.0, 0.0]}
    - {time_s: 0.1, values: [3.0, 3.0, 3.0, 3.0]}
```

`input_schedule` and `constant_input` are alternatives:
- A stage given both is refused, because the run would leave it unsaid which
  input drove it.
- A stage given neither integrates with a zero input, as it always did.

Every sample carries exactly as many values as the model has inputs, in the
model's input order. A key the schema does not name is refused at either level.
So `interpolation: linear` written beside `hold` is an error, not a request that
looks honoured.

## What each part means

These are the integrator's definitions. They are stated beside the code in
`include/galata/sim/linear.hpp`.

- **Timestamps** are seconds from the start of the run. The initial state is at
  `t = 0`, and the run ends at `t = step_s × steps`. A history may start before
  the run or end after it. Only the part the run crosses is read.
- **Zero-order hold.** Each value is held from its sample until the next.
  - A change of value is an **event**. It must fall on an integration step
    boundary, and one that does not is refused. An RK4 step straddling a jump is
    only first order across it. Rounding the event to the nearest step would
    move it without saying so.
  - The value in force across a step is resolved once, at the event's own time,
    so all four RK4 stages of every step see it.
- **Linear hold.** The input is piecewise linear between samples, and it is
  evaluated at every RK4 stage.
  - A change of slope inside a step costs RK4 its order across that step. It is
    not refused, exactly as in `sim.plant`.
  - For the full order, put the sample times on the step lattice, as this study
    does.
- **Extrapolation.** Outside the sampled span:
  - `hold` keeps the nearest end value.
  - `refuse` rejects a run whose horizon leaves the span, before any step is
    taken, so there is no partial trajectory. `spoolup` declares `refuse` and
    ends exactly at its last sample.
- **Recording.** The input recorded at each sample is the value **in force
  after** any event at that instant, the one the next step integrates under.
  - The output's feedthrough `D u` uses that recorded value.
  - Under a history, the CSV carries one `input:` column per input. A
    constant-input run writes exactly the columns it always wrote.
- **Compatibility.** A history that holds one value throughout reproduces the
  constant-input run bit for bit.

## What checks this

`ExampleSouxmarLinearHistories`, in the `integration` tier, runs this exact study
file. It compares against references that are not the routine under test:

- `ExampleSouxmarLinearHistories.RunsEndToEndAndHonoursEveryDeclaredEvent`
  checks three things:
  - the stages;
  - that the doublet's three value changes are events at the declared steps,
    and the spool-up has none;
  - that the input recorded at every sample is the declared history's value in
    force there.
- `ExampleSouxmarLinearHistories.TheDoubletMatchesTheExactHeldSolutionWithinAnAPrioriBudget`
  compares every sample with the exact solution for a held input, computed from
  the block matrix exponential. The budget is derived beforehand from the model,
  the step and the history. Its negative control is the same comparison against
  the doublet moved one step late, which must fail that budget.
- `ExampleSouxmarLinearHistories.TheSpoolUpMatchesTheExactInterpolatedSolutionWithinAnAPrioriBudget`
  does the same for the linearly interpolated history. Its negative control is a
  reference that holds each value across the step instead of interpolating it,
  which must fail the budget.

`LinearHistoryWorkflow` holds the schema itself. It uses a plant whose response
is a polynomial that RK4 integrates exactly, and checks three things: exactness
through the pipeline, constant-input compatibility, and each refusal by name.
`LinearSimulation`, in the `unit` tier, holds the integrator.

## What this is not

- **Not a comparison with the nonlinear plant.** Both runs are of the
  linearisation. How far a linear prediction can be trusted against the
  nonlinear plant is the question `examples/souxmar-sampled-lqr` asks, and that
  study has its own budget.
- **Not a validated response.** The references check that the integrator
  integrates the declared history correctly. They say nothing about whether the
  model describes an aircraft. The Souxmar coefficients are an independent
  implementation's nominal set, not measured data, and
  `models/souxmar-quad/PROVENANCE.md` says so.
- **Not an event model.** An event here is a declared change of an input at a
  declared time. Nothing detects an event from the state, such as a threshold
  crossing.
- **Not an adaptive integrator.** The step is fixed and declared. An event off
  the step lattice is refused, not located.
