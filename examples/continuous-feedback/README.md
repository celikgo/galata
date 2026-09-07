# Continuous feedback model

This synthetic example exercises the first executable block-model profile. Its
equation is `dx/dt = 1 m/s - x/(1 s)`, with `x(0) = 0 m`; the exact continuous
solution is `x(t) = (1 - exp(-t/(1 s))) m`. The gain declares inverse-second
units and the integrator declares metre state units. It is infrastructure
evidence, with no aircraft data or aircraft validity claim.

From the repository root:

```sh
build/dev/src/cli/galata run examples/continuous-feedback/study.yaml --output-dir build/continuous-feedback
```

The installed CLI accepts the same study under `share/galata/examples`. Use a
fresh output directory or explicitly pass `--overwrite` for a repeat run.

`response.csv` records time, state `x` and output `y`, including the initial and
final samples when the stride does not divide the step count. `evidence.json`
contains the canonical model, semantic digest, source IDs, execution schedule,
solver settings and CSV digest. The enclosing content-addressed run manifest
records the exact original model/study bytes and build/runtime identities.
Interpret the evidence and trajectory together with that manifest; files left
by an interrupted multi-stage study do not establish a completed run.

Successful execution does not assess numerical accuracy, model validity or
engineering acceptance. The independent analytic tests and their predeclared
budgets are in [the model conformance protocol](../../docs/architecture/MODEL_CONFORMANCE.md).
Changing a step or a model requires reviewing accuracy for that run. This
profile has no sampled clocks, state machines, aircraft blocks or graphical
editor. See [the source format](../../docs/MODEL_FILES.md) and
[ADR-0010](../../docs/adr/0010-continuous-scalar-executable-model.md).
