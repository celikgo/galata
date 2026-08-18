# NT-33A lateral-directional modes

Turns a linear aircraft model into a labelled modal table. It is the smallest
end-to-end study galata can run, and every number it prints is checked against a
published document.

## Run it

```bash
galata run examples/nt33a-lateral-modes/modal-study.yaml
```

That writes `lateral-modes.md` next to the pipeline. Add `--output-dir` to put
it somewhere else.

## What it does

Three stages:

1. `model.linear.statespace` reads `nt33a-lateral.yaml` — a 4-state
   lateral-directional model of the NT-33A at sea level and M = 0.204.
2. `analyze.modes` decomposes it, computes the modal metrics and participation
   factors, and labels the classical modes.
3. `report.markdown` writes the table.

## Why this model

The NT-33A data comes from NASA CR-2144, *Aircraft Handling Qualities Data*
(Heffley and Jewell, Systems Technology Inc., 1972), which is a US Government
contractor report released with unlimited distribution. It publishes both the
stability derivatives **and** the resulting modal characteristics, which is what
makes it usable as a validation reference rather than merely as a demo: galata
computes the modes from the derivatives, and the published modes say whether it
got them right.

The published values for this flight condition, and what galata computes:

| Mode | Published | galata |
|---|---|---|
| Spiral | 1/T = 0.0318 1/s | 0.0319 |
| Roll subsidence | 1/T = 2.20 1/s | 2.1929 |
| Dutch roll | ζ = 0.0609, ω_n = 1.13 rad/s | 0.0607, 1.1297 |

The agreement is limited by the source's own precision, not by galata's: the
report prints its derivatives to three significant figures, and that rounding
propagates. `tests/validation/test_nt33a_modes.cpp` measures the propagation
rather than assuming a tolerance, and `docs/VERIFICATION.md` publishes the
result.

## What this example does not show

There is no trim and no linearisation here. The model is *given* as a state
matrix rather than computed by trimming a nonlinear aircraft and linearising
about the trim point — neither of those capabilities exists yet. When they do,
this example gains two stages ahead of `analyze.modes` and the derivatives
become an input to the aerodynamic model instead of an already-assembled matrix.
