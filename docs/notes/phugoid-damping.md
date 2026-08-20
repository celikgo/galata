<!-- SPDX-License-Identifier: Apache-2.0 -->

# The 2.04% phugoid-damping discrepancy in the hand-assembled matrix

**Status: cause identified, not yet acted on.** This note records the
investigation behind the `known discrepancy` row in
[docs/VERIFICATION.md](../VERIFICATION.md). It is written because an open
number with no investigation behind it reads as an unexamined bug, and because
the conclusion below changes what the discrepancy *is*: the hand assembly does
not omit a term, it **includes one that should not be there**.

Everything numeric here is reproducible from the committed reference values in
[tests/validation/reference/nt33a_fc1.csv](../../tests/validation/reference/nt33a_fc1.csv)
and the assembly in
[tools/validation/nt33a_hand_assembly.cpp](../../tools/validation/nt33a_hand_assembly.cpp).

## The measurement

Two routes to the same published mode, from the same report:

| Route | ζ | ω_n (rad/s) | vs published ζ = 0.0948 |
|---|---|---|---|
| State matrix assembled **by hand** from Table II-3's dimensional derivatives | 0.09287 | 0.171911 | **−2.04%** |
| Nonlinear model from Table II-1's **non-dimensional** set, trimmed, then linearised by central differences | 0.094852 | 0.171399 | +0.05% |

The hand-assembled eigenvalue is −0.015965 ± 0.171168j against a published
−0.016306 ± 0.171225j. The imaginary parts agree to four parts in ten thousand.
The whole disagreement is 3.403 × 10⁻⁴ in the **real part**, which ζ = |Re|/|λ|
then divides by a small ω_n and turns into 2%.

## What was ruled out

### The published value's own rounding — ruled out

The report prints ζ = 0.0948 and ω_n = 0.172, so the published real part
−ζω_n carries ±5.6 × 10⁻⁵ from its own three-significant-figure printing. The
observed gap is **6.1× that band**. The published number cannot be moved far
enough to meet the hand assembly.

### The inputs' rounding — ruled out, and by a wider margin than previously stated

Each of the ten longitudinal inputs was perturbed by half a unit in **its own
last printed digit, in the units the report prints it in** — which matters,
because `M_u*` is printed per second-foot and stepping its per-second-metre
form understates its uncertainty by the 3.28 conversion factor.

| Input | ζ across its own rounding | swing |
|---|---|---|
| `X_u*` | 0.092724 … 0.093015 | 0.31% |
| `Z_u*` | 0.092771 … 0.092969 | 0.21% |
| `alpha`, `V` | — | ≤ 0.06% |
| `M_q`, `M_u*`, `M_wdot`, `X_w`, `M_w`, `Z_w` | — | ≤ 0.04% |

Enumerating **all 2¹⁰ corners** simultaneously gives ζ ∈ [0.092521, 0.093219],
that is −2.40% … −1.67%. The published 0.0948 is **not reachable**: even the
most favourable corner of the input rounding still falls 1.67% short. The
earlier wording — "about three times what the inputs' rounding allows" — was
too generous to the rounding hypothesis.

### A units error in the per-foot scaling — ruled out by ω_n

The pitching-moment row is per foot and the matrix is not invariant under a
change of length unit, so this is the obvious suspect. It is also the easiest
to eliminate, because **ω_n is the control variable**: it already agrees to
0.05%, and any scaling error destroys it.

| Hypothesis | ζ | ω_n | ω_n vs published |
|---|---|---|---|
| Scaling omitted | 0.11222 | 0.13582 | −21% |
| Scaling applied twice | 0.10362 | 0.19001 | +10.5% |

### A transcription error — ruled out

No single input can be moved to close the gap while leaving ω_n where it is.
`Z_u*` would have to become −0.2385, nineteen half-units from its printed
−0.248, and doing so drags ω_n to 0.16900 (−1.74%). `X_u*` would have to become
−0.03976, thirteen half-units from its printed −0.0391.

Independently: `X_u*` and `Z_u*` are the two inputs the phugoid is sensitive to,
and galata's own trim-and-linearise chain — which never reads Table II-3 —
computes −0.039146 and −0.247105 for them. The printed values are corroborated
to 0.12% and 0.36% by a route that shares no arithmetic with the transcription.

### The report printing a reduced-order approximation rather than the quartic root — ruled out

If the published 0.0948 came from a classical phugoid approximation rather than
from factoring the full quartic, that would explain a systematic offset. It does
not:

| Approximation | ζ | ω_n | vs published |
|---|---|---|---|
| Lanchester, ω_n = √2·g/V | — | 0.19957 | ω_n +16% |
| ζ = −X_u/(2ω_n), Lanchester ω_n | 0.09796 | — | ζ +3.3% |
| ζ = −X_u/(2ω_n), exact ω_n | 0.11372 | — | ζ +20% |
| Residualised 2×2 in (u, θ) | 0.13468 | 0.17221 | ζ +42% |

None of them lands on 0.0948, and the report's ω_n = 0.172 is the exact root,
not Lanchester's 0.19957. The report factored the quartic.

## What it actually is

Substituting the chain's linearisation for the hand assembly's **one matrix
entry at a time** localises the gap completely:

| Entry swapped for the chain's value | hand | chain | share of the gap |
|---|---|---|---|
| **A(3,4) = M_ẇ · (−g sin θ)/(1 − Z_ẇ)** | **+0.001877** | **0.000000** | **98.8%** |
| A(1,3) = −W_o + X_q | −2.667738 | −2.604860 | −28.2% |
| A(3,1) = M_u | 0.002280 | 0.002244 | 11.5% |
| every other entry | — | — | ≤ 11.1% each |

One entry carries the discrepancy. It is the term that couples pitch attitude
into pitch acceleration through the downwash lag, and it arises from the
descriptor form of NASA CR-2144 Appendix C p.C-1,

```
q̇ = M_u u + M_w w + M_q q + M_ẇ ẇ
```

being closed by substituting the **whole** ẇ equation, gravity term included,
into `M_ẇ ẇ`.

**That path does not physically exist.** `M_ẇ` is the downwash-lag derivative:
it multiplies the rate of change of *aerodynamic incidence*, and the convention
`M_ẇ ≡ M_α̇/U_o` holds only when ẇ stands for the aerodynamic part of ẇ. The
gravity component of ẇ changes the flight-path angle, not the incidence.
Gravity acts at the centre of gravity and produces no moment about it, so no
gravity term may reach q̇ at all.

The chain gets exactly zero there, and not by accident. It computes
α̇ = (u ẇ − w u̇)/(u² + w²) from the true state derivative
([src/model/aircraft.cpp:265](../../src/model/aircraft.cpp)), and gravity is
the only thing making u̇ and ẇ depend on θ:

```
∂u̇/∂θ = −g cos θ ,   ∂ẇ/∂θ = −g sin θ
∂α̇/∂θ = g(−u sin θ + w cos θ)/V² = g·sin(α − θ)/V = 0   when θ = α + γ, γ = 0
```

which is the same statement as α = θ − γ ⟹ α̇ = q − γ̇, and γ̇ carries no direct
θ dependence. The computed value is −1.5 × 10⁻¹⁸.

### The corrected assembly

Setting that one entry to zero and changing nothing else:

| Quantity | corrected | published | deviation |
|---|---|---|---|
| Phugoid ζ | 0.094829 | 0.0948 | **+0.03%** |
| Phugoid ω_n | 0.172016 | 0.172 | +0.01% |
| Short-period ζ | 0.621681 | 0.622 | −0.05% |
| Short-period ω_n | 1.594693 | 1.59 | +0.30% |

All four land inside the source's own printing precision. The published ζ band
is ±0.05% and the corrected value sits inside it.

### Why this and not the X_q hypothesis

The previous leading candidate was that `X_q`, left **blank** in Table II-3 and
read as zero, is not actually zero. That hypothesis still fits the single
number: X_q = −0.217 m/s per rad/s closes the gap exactly.

It should nonetheless be rejected as the primary explanation, because it is
**fitted** and this one is not. The M_ẇ term is worth 98.8% of the observed gap
with no free parameter — its size follows from `M_ẇ`, `g` and `θ`, all three
printed. A hypothesis that predicts the right number is worth more than one
tuned to it.

The two are also **separable by experiment**, which is the part that would
settle it (below): the spurious term scales with sin θ and the X_q term does
not.

| θ | with the term | without it | artefact |
|---|---|---|---|
| 2.20° (flight condition 1) | 0.092870 | 0.094829 | −2.07% |
| 4.20° | 0.104205 | 0.107939 | −3.46% |
| 7.20° | 0.121426 | 0.127819 | −5.00% |
| 12.20° | 0.150932 | 0.161773 | −6.70% |

At exactly level flight with γ = 0 and small α the artefact is small; it is not
small in a climb.

## What would settle it

Two transcriptions, neither yet made, each decisive on its own:

1. **The report's own longitudinal quartic coefficients.** Appendix C writes the
   characteristic polynomial in terms of `M_α` and `M_α̇`; its `D` and `E`
   coefficients for this flight condition were never transcribed. The phugoid
   roots are set by the low-order coefficients, so evaluating the report's
   printed `E` against both assemblies distinguishes them directly — it shows
   whether the report's own algebra carries the gravity–M_ẇ path.

2. **A second flight condition with γ ≠ 0.** Table II-2 gives eight. The
   artefact predicted here scales with sin θ; a fitted `X_q` does not. One
   climbing or descending condition, transcribed and run through both routes,
   separates the two hypotheses by a margin far larger than the rounding.

## What has not been changed, and why

The hand assembly is **still committed as it is**, and the regression lock
`Nt33aHandAssembled.PhugoidDampingDiscrepancyDoesNotGrow` still holds the gap at
its measured size.

That is deliberate. The hand assembly's value is that it is a *literal* reading
of the report's Appendix C, independent of galata's own modelling opinions. If
it is quietly corrected to agree with the chain, it stops being an independent
check and becomes a second implementation of the same opinion — and the
comparison that localised this discrepancy in the first place would no longer be
able to find the next one.

Acting on this note means one of two things, and it is a decision to take
explicitly rather than by editing a matrix:

* keep the literal assembly and **relabel** the row — the discrepancy belongs to
  the literal reading of Appendix C, not to galata; or
* add a **third** route, the corrected assembly, alongside the literal one, and
  gate all three.

Until that decision is made, the row stays marked `known discrepancy` and this
note is what stands behind it.
