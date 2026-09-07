# NT-33A lateral 2×2 loop — principal gains and sensitivity peaks

Takes the same aircraft as the other NT-33A examples and closes **two** loops
around it at once — aileron to bank angle, rudder to sideslip. It exists to show
the two things a single-loop analysis cannot see: that a MIMO plant has a *range*
of gains at each frequency rather than one, and that per-channel margins can look
comfortable while the loop as a whole is not.

## Run it

```bash
galata run examples/nt33a-lateral-mimo/mimo-study.yaml
```

That writes `lateral-mimo.md` next to the pipeline. Add `--output-dir` to put it
somewhere else.

## What it does

Five stages:

1. `model.linear.statespace` reads `nt33a-lateral-mimo.yaml`. A and B are the
   NT-33A lateral-directional dynamics; the two C rows are the control law.
2. `analyze.sigma` computes the principal gains of the 2×2 transfer matrix.
3. `analyze.sensitivity` computes M_S and M_T for the loop closed with negative
   unit feedback.
4. `analyze.margins` computes classical margins for the aileron→bank channel
   **alone**, so the report can show them being optimistic.
5. `report.markdown` writes it all out.

## One gain, or a range of them

At 0.01 rad/s this loop's principal gains are 43.8 and 0.458 — a condition
number of about 96. That single frequency has a largest gain almost a hundred
times its smallest, depending entirely on which *direction* the input points.
There is no single number that is "the gain" of this system at that frequency,
which is why a MIMO plant needs singular values rather than a grid of
element-by-element Bode plots.

A caution that matters here: the condition number depends on how the inputs and
outputs are **scaled**. Both feedback gains in this example are deliberately
equal at 0.5, so the spread above is the aircraft's own and not an artefact of
mixing units. Change the gains independently and you change the condition number
without changing the aeroplane.

## The per-channel margins are optimistic

The report puts these side by side:

| measure | value |
| --- | ---: |
| Aileron→bank channel, gain margin | not found in the searched band |
| Aileron→bank channel, phase margin | 48.5 deg |
| M_S, the whole 2×2 loop | 1.85 |
| M_T, the whole 2×2 loop | 1.80 |

The channel has a healthy sampled phase margin, but it was computed with the
rudder loop **held open**. It describes a different feedback system from the
closed two-channel loop. A missing gain-margin crossover is also a search
result, not evidence of unlimited tolerance.

The sampled M_S estimate is 1.85. Its reciprocal, about 0.54, estimates the
smallest singular value of I+L over the searched frequencies. This is a
multivariable singular-value separation, not the distance from a scalar Nyquist
curve to −1. The estimate is below the textbook rule of thumb of M_S around 2,
but a sampled lower norm estimate cannot establish that the full norm meets a
limit or that an aircraft design is accepted.

## Why no guaranteed margins are printed

Sampled sensitivity peaks are lower estimates of the true norms. The classical
sensitivity-to-margin formulas require upper norm bounds, so sampled results
cannot supply guarantees even for a single loop. Those formulas also apply only
to SISO systems: applying them independently to this MIMO loop would not
establish simultaneous robustness. The report states both limitations.

## What it does not tell you

M_S is a distance from the critical point, not a Nyquist encirclement count.
It establishes nothing about closed-loop stability on its own. The numerical
stability assessment must resolve the nominal loop before reporting a peak;
ill-conditioned cases are refused rather than declared stable.

The report prints the searched band and point count. The feedback gains here
are chosen to demonstrate the example, not designed for an aircraft. No
application-specific qualification or certification evidence is claimed.
