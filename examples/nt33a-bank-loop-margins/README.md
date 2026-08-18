# NT-33A bank-angle loop — frequency response and margins

Takes a real aircraft, closes a simple bank-angle loop around it, and reports
all four margin types. It exists to show two things a smaller example cannot:
that a loop can have more than one phase margin, and that the classical margins
and the disk margin do not measure the same thing.

## Run it

```bash
galata run examples/nt33a-bank-loop-margins/margin-study.yaml
```

That writes `bank-loop-margins.md` next to the pipeline. Add `--output-dir` to
put it somewhere else.

## What it does

Five stages:

1. `model.linear.statespace` reads `nt33a-bank-loop.yaml`. The A and B matrices
   are the NT-33A lateral-directional dynamics; the C row is the control law.
2. `analyze.freqresp` evaluates G(jw) over four decades, solving
   (jwI − A)x = B at each frequency rather than forming an inverse.
3. `analyze.margins` finds every crossover and reports gain, phase and delay
   margins with the frequency of each.
4. `analyze.diskmargin` computes the symmetric disk margin.
5. `report.markdown` writes it all out.

## The loop is not the aircraft

Margins are a property of a **loop**. An aircraft on its own is not one, so this
example adds the smallest control law that makes the question meaningful:
measure bank angle, multiply by 0.5, drive the aileron. That gain is not tuned
and is not from any document — it is chosen because it makes the example show
something, and the model file says so.

The loop is broken at the **plant input**, between the gain and the aileron.
Breaking it at the output instead gives a different transfer function and
different margins for the same closed-loop system. That is a modelling decision;
galata will not make it for you.

## Three phase margins

The interesting part of the output:

| kind | w (rad/s) | margin |
| --- | ---: | ---: |
| \|L\| = 1 | 0.59 | 101 deg |
| \|L\| = 1 | 0.92 | 165 deg |
| \|L\| = 1 | 1.61 | 48 deg |

The magnitude dips below unity, comes back up over the Dutch roll, and falls
away again. An implementation that returned the first crossing it found would
report a phase margin of 101 degrees and call this loop very comfortable. The
governing margin is 48 degrees, at a frequency more than twice as high.

This is why `analyze.margins` returns every crossover and not just one, and why
the report prints them all.

## The gain margin is infinite, and that is not reassuring

The phase never reaches −180 degrees inside the searched band, so there is no
phase crossover and the gain margin is infinite. Taken alone that reads as a
loop with unlimited gain tolerance.

The disk margin says otherwise. It reports a guaranteed gain range of roughly
0.39 to 2.56 and a guaranteed phase range of about ±47 degrees — because it is
asking a different question: not "how much gain change alone" or "how much phase
change alone", but how much of **both together**. Real actuators and real
sensors vary in both.

## What it does not tell you

Nothing here is a handling-qualities assessment, and 48 degrees of phase margin
is not a certification argument. See `docs/CHARTER.md` on what galata is not.

The disk margin's peak is found by searching a frequency grid, so it is an
upper bound on the true margin — the error is in the optimistic direction. The
report prints the band and point count that were searched. See
`include/galata/analyze/disk_margin.hpp`.
