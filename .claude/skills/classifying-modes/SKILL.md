---
name: classifying-modes
description: Why galata labels the classical aircraft modes by eigenvector participation rather than by frequency ordering, exactly what breaks under frequency ordering, how the signature table and greedy assignment actually work, what makes a mode Unclassified, and every place that has to change to add a new mode. Use when touching src/analyze/modes.cpp, include/galata/analyze/modes.hpp, StateRoles, the ModeLabel enum, or any test or report that reads a mode label or a label score.
---

# Classifying modes

A plain eigenvalue list is what every tool gives. A **labelled** modal table is what an engineer
wants, and how the label is decided is this repository's distinguishing feature.

galata labels by **eigenvector participation**: how much of each mode lives in the states that
define it. Not by frequency, not by damping, not by ordering.

## Why not frequency ordering

Frequency thresholds stop working on exactly the configurations that are worth running a modal
analysis on. Four concrete failures, all present in the tree:

**1. The unit test is built to fail a frequency classifier.** `tests/unit/test_modes.cpp` builds
block-diagonal systems with deliberately *unhelpful* frequencies — a "phugoid" (u, θ) block at
ω_n = 3.0 and a "short period" (α, q) block at 0.4 — and asserts that the phugoid label lands on
the 3.0 block. The file says why:

> a classifier that keys on frequency would pass a test built from realistic numbers while
> failing on the aft-CG configurations that are the entire reason to run a modal analysis.

**2. At an aft CG the short period and phugoid merge.** They approach each other and can become a
pair of real roots, one of which is the "tuck" divergence. There the frequency rule fails
outright while participation keeps working — and where the evidence genuinely *is* ambiguous the
scores fall and you get a low-confidence label rather than a confident wrong one.

**3. On the real NT-33A the frequencies already cross axes.** The Dutch roll is at
ω_n = 1.12933 rad/s and the short period at 1.59503 — the *lateral* oscillation is slower than
the longitudinal one — and the roll subsidence at 2.19923 is the fastest of all five roots. Any
rule that sorted the five modes by frequency and read labels off the ordering would be wrong
about this aircraft, which is a completely conventional one.

**4. Where frequency ordering *would* have worked, it would have been right for the wrong
reason.** The NT-33A's spiral and roll subsidence are a factor of 69 apart, so almost any rule
separates them. That is not evidence for the rule.

A related trap in reading the output: **the reported table is not frequency-ordered either.**
`mode_precedes` puts all non-oscillatory modes first, then sorts by ω_n, then by real part, then
by imaginary part. On the NT-33A that prints spiral (0.0319), roll subsidence (2.1992), Dutch
roll (1.1293) — the Dutch roll is slower than the roll subsidence and still appears last.
Reading the table as a frequency ladder inverts two of the three lateral rows.

## How it actually works

### Participation factors

For mode column *i* and state *k*, with `V` the right eigenvectors and `Vinv = V⁻¹`:

```
P_ki = |V(k,i) · Vinv(i,k)|
```

The complex factors sum to 1 across states by `(Vinv·V)(i,i) = 1`, exactly — but their
**magnitudes do not**, so there is a second, explicit renormalisation by the sum of magnitudes.
After it, each mode's participation vector is real, non-negative and sums to 1 (asserted to
1e-12).

Left eigenvectors are the plain inverse of the right eigenvector matrix, and are formed **only**
when the decomposition is well conditioned.

### The signature table

```cpp
// This table IS the classifier; everything else is bookkeeping.
struct Signature {
  ModeLabel label;
  bool oscillatory;
  std::array<int StateRoles::*, 2> roles;
};
```

| Label | Oscillatory | Roles |
|---|---|---|
| `ShortPeriod` | yes | angle of attack, pitch rate |
| `Phugoid` | yes | axial speed, pitch attitude |
| `DutchRoll` | yes | sideslip, yaw rate |
| `RollSubsidence` | no | roll rate (listed twice, deduplicated) |
| `Spiral` | no | bank angle (listed twice, deduplicated) |

Two things in that table surprise people. The **phugoid signature does not use α** — it is a
speed-and-attitude exchange, and including α would make it compete with the short period. The
**Dutch roll signature does not include roll rate** even though the comment beside it calls the
mode "a yaw-sideslip oscillation with rolling", because p participates but does not
*distinguish* it from the roll subsidence.

`signature_score` is a plain sum of the mode's participation over the distinct, present role
indices. An absent role (-1), a repeated one, or an out-of-range one contributes nothing.

### Assignment

Candidates are generated only where the signature's `oscillatory` flag matches the mode's
character — a hard gate — and only where the score is **strictly greater than zero**. They are
then sorted by descending score and assigned **greedily**, each mode and each label used at most
once:

> Greedy rather than a global optimum … a greedy rule is one a reader can follow when a label
> surprises them.

Ties break deterministically — higher score, then lower mode index, then lower signature index
in declaration order — *"so the table does not depend on sort stability"* (ADR-0004). Note that
the modes are sorted into report order **before** candidates are built, so the mode-index
tie-break refers to the reported order.

Duplicate labels are impossible by construction, and there is a regression test for it: two
similar oscillations that both score on `ShortPeriod` yield exactly one `ShortPeriod`, and the
second goes unlabelled rather than duplicating.

### `label_score` and `label_reason`

`label_score` is **exactly** the winning candidate's signature score — no rescaling, no penalty,
no clamp. `label_reason` is a fixed 3-decimal string built from the caller's own state names:
`"0.959 of participation in u and theta"`.

**There is no minimum-score threshold in the classifier.** The only gate is `score > 0.0`. The
0.5 / 0.8 / 0.9 numbers you will find live in *tests* and in an advisory header comment ("below
about 0.5 the label is a guess") — never in `src/`. So: **always look at the score before
quoting a label.** On the NT-33A the five scores are

| Mode | score |
|---|---|
| short period | 0.9926 |
| phugoid | 0.959357 |
| roll subsidence | 0.838181 |
| Dutch roll | 0.770542 |
| spiral | 0.717249 |

The spiral is the weakest-evidenced label on a completely ordinary aeroplane — 0.269 of its
participation sits in yaw rate. That is the honest picture, and it is why the score is a column
in every report rather than a debug value.

### `StateRoles`

Supplied by the caller rather than guessed, *"because a classifier that guesses is a classifier
that silently mislabels a model whose author spelled a state differently"*. `-1` means the role
is absent, which is normal — a purely longitudinal model has no roll rate.

`StateRoles::from_names` is a convenience that matches **lowercased names by exact equality**, no
prefixes and no substrings:

| Role | Accepted names |
|---|---|
| axial speed | `u`, `v_t`, `vt`, `speed`, `airspeed` |
| angle of attack | `w`, `alpha`, `aoa` |
| pitch rate | `q`, `pitch_rate` |
| pitch attitude | `theta`, `pitch`, `pitch_attitude` |
| sideslip | `v`, `beta`, `sideslip` |
| roll rate | `p`, `roll_rate` |
| yaw rate | `r`, `yaw_rate` |
| bank angle | `phi`, `bank`, `roll`, `bank_angle` |

A model with unusual names should set the fields directly. A role left at -1 simply takes no part.

Two API notes. `has_longitudinal()` and `has_lateral()` exist, are declared and defined — and are
called **nowhere**, including inside `analyze_modes`. And `analyze_modes` has **no notion of
longitudinal versus lateral** at all: the `axis` field in `docs/assets/modal-map.json` and in the
report run record is stamped by the *emitting tool*, which runs two separate 4-state
decompositions.

### Conditioning, and when there is no label

Participation factors are defined for a system with distinct eigenvalues. The eigenvector matrix
condition number is computed by SVD, and above `1e10` participation is declared meaningless:
no inverse is formed, the participation vectors stay **empty**, and classification returns early.
Eigenvalues and every modal metric are still reported.

This is not a pathological case in flight dynamics. It happens at the exact CG where two real
roots coalesce into a complex pair, and a CG sweep walks straight through one.

A mode ends up `Unclassified` in exactly these situations: the decomposition was
ill-conditioned; no signature of matching oscillatory character scored above zero; every
matching label was already claimed by a better-scoring mode; or the two-argument overload was
used, which passes a default `StateRoles` with every field at -1.

A conjugate pair is reported **once**, as the member with positive imaginary part, and the
partner is matched by nearest conjugate rather than by index *"because the solver does not
promise adjacency"*. A mode counts as oscillatory when |Im| exceeds `1e-9 · |λ|` — a **relative**
threshold, so it behaves the same for a spiral root at 0.03 rad/s and a structural mode at
300 rad/s.

## Adding a mode

Seven places. The compiler catches one of them; the rest are on you.

1. **`ModeLabel`** in `include/galata/analyze/modes.hpp`.
2. **`to_string`** in `src/analyze/modes.cpp`. The switch has **no `default:`**, so with `-Wall`
   and `-Werror` (`GALATA_WERROR`, on by default) an omitted case is a build failure. This is the
   one place that fails loudly — do not add a `default:` and take that away.
3. **`kSignatures`** — the array size is a literal `5` in the type, and the per-signature role
   array is fixed at `2`. Both are part of the type and both must change. If your mode needs
   three roles, that array size changes for every signature.
4. **`StateRoles`** — a new field defaulting to -1, a branch in `from_names`, and a decision about
   `has_longitudinal` / `has_lateral`.
5. **Unit tests** in `tests/unit/test_modes.cpp`. Follow
   `ClassifiesLongitudinalModesByParticipationNotByFrequency`: build a block-diagonal system with
   **deliberately unhelpful frequencies**, so the test cannot pass by accident on a frequency
   rule.
6. **The chain validation test** — `Nt33aChain.ModesAreLabelledCorrectlyFromParticipationAlone`
   asserts that *every* mode of *both* axes is non-`Unclassified` with a score above 0.5. A new
   label that steals a mode from an existing one shows up here.
7. **The artefact generators.** `scripts/gen-social-preview.py` looks modes up **by label string**
   and exits if one is missing; `scripts/gen-modal-map.sh` and `scripts/gen-report.sh` both fail
   on an `unclassified` label; the integration tests assert the label strings that appear in the
   generated example reports. Regenerate `docs/assets/modal-map.json`,
   `docs/assets/nt33a-fc1-run.json`, the social preview and the report page, and commit them.

Also update `tools/validation/case_registry.cpp` if the new mode is validated against a
published value — and it should be, or it is `Unvalidated` and says so.

## Never

- Add a frequency, damping or ordering threshold to the classifier. The whole design is that
  there is none.
- Quote a label without its score. The classifier cannot tell you a label is meaningless; it can
  only tell you the evidence was weak, and `label_score` is how it does that.
- Read the reported table as a frequency ladder.
- Report participation factors from an ill-conditioned decomposition. The code refuses; do not
  work around it.
- Assume `from_names` will match a state you spelled differently. It compares for equality
  against a fixed list, and a state it does not recognise silently takes no part in
  classification.
