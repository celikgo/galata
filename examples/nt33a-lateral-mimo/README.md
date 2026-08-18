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
| Aileron→bank channel, gain margin | infinite |
| Aileron→bank channel, phase margin | 48.5 deg |
| M_S, the whole 2×2 loop | 1.85 |
| M_T, the whole 2×2 loop | 1.80 |

Read the first two rows alone and the loop looks excellent: unlimited gain
tolerance and a healthy phase margin. But those numbers were computed with the
rudder loop **held open** — they describe a different system from the one that
actually flies.

M_S = 1.85 is the honest number. It says the Nyquist curve of the multivariable
loop comes within 0.54 of the critical point, and it accounts for both channels
varying at once. Skogestad and Postlethwaite's rule of thumb is M_S below about
2, so this design passes — but with far less room than "infinite gain margin"
suggested.

## Why no guaranteed margins are printed

For a single loop, galata reports the classical margins that M_S and M_T
*guarantee*, from Skogestad & Postlethwaite equations (2.47) and (2.48). For
this example it refuses, and says so in the report.

That is the source's own scope, not caution added here. Those equations sit in a
chapter whose remit is SISO, and the book never restates them for MIMO. Its
spinning-satellite example shows a plant with excellent margins "when
considering one loop at a time" that is destabilised by small *simultaneous*
input gain errors — precisely the error that applying a per-channel bound to a
multi-loop system would reproduce.

## What it does not tell you

M_S is a distance from the critical point, not a Nyquist encirclement count.
It establishes nothing about closed-loop stability on its own; galata checks the
closed-loop eigenvalues before reporting a peak at all, and refuses if they are
in the right half-plane.

The peaks are grid maxima and so are **lower** bounds on the true H∞ norms —
the error is in the optimistic direction. The report prints the band and point
count searched. See `include/galata/analyze/sensitivity.hpp`.

The feedback gains here are chosen to make the example work, not designed.
Nothing in this directory is a control design, and nothing in galata is
certification evidence — see `docs/CHARTER.md`.
