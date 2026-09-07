<!-- SPDX-License-Identifier: Apache-2.0 -->

# RFC-0001: Control synthesis, and how it will be verified before it is written

- **Status:** accepted for the bounded offline implementation described below; the original proposal follows for context
- **Date:** 2026-08-24
- **Affects:** `synth.*` capabilities, `src/synth/`, `docs/VERIFICATION.md`, ADR-0004
- **Supersedes:** nothing. Refines the original analysis-and-design milestone; the bounded offline scope is recorded in [ROADMAP.md](../ROADMAP.md).

The README's Status table remains the authority on implemented capabilities.

Acceptance decision, 2026-09-06, under the owner's instruction to complete the
engineering product: the first implementation uses Laub's ordered complex Schur
method for dense continuous-time problems with positive-definite R. Cholesky
solves form the Hamiltonian without constructing an explicit inverse. Singular
or indefinite R, poorly separated spectra and ill-conditioned invariant
subspaces are rejected. The generalised-pencil solver remains future work;
this release does not claim the full CAREX parameter domain.

The reference questions are resolved as follows. The SLICOT project's
[current source licence](https://github.com/SLICOT/SLICOT-Reference/blob/main/LICENSE)
is BSD-3-Clause. This implementation does not copy its Fortran code or its full
benchmark collection. It quotes only the small worked-result tables from the
[BB01AD](https://www.slicot.org/objects/software/shared/doc/BB01AD.html) and
[SB02MD](https://www.slicot.org/objects/software/shared/doc/SB02MD.html)
documentation under ADR-0007, with reference provenance and budgets derived
from the printed decimals. BB01AD supplies one CAREX example, not the whole
collection. No published aircraft LQR design satisfying all of section 2.5's
criteria has been transcribed: that case is explicitly unvalidated. Aircraft
examples test the workflow and local nonlinear response; they do not claim
agreement with published aircraft controller gains.

Schur decomposition uses Eigen's deterministic same-binary implementation;
its internal QR convergence is data dependent, just as the existing modal
eigensolver's is. The original assertion below that Schur iteration consists
only of a fixed sequence is incorrect. Our Schur reordering is dimension
bounded, and repeatability is tested. This is not a worst-case execution-time
guarantee or an onboard controller implementation.

## Why this document exists at all

Control synthesis is, in the README's own words, the point of the project. It is also the first
capability where galata will produce a number that is a **design** rather than a measurement —
and a design has no published value to compare against, because it is new.

That is the whole problem. Everything validated so far had a document with the answer printed in
it: CR-2144 prints the NT-33A's modes, Seiler et al. print their disk margin, the 1976 Standard
Atmosphere prints its tables. A gain matrix galata computes for a plant nobody has published a
gain matrix for cannot be checked that way, and the temptation is to check it against nothing and
call the absence of a crash a result.

So the verification strategy is designed **first**, and the API is designed to make that strategy
possible. This is the repository's own methodology (`docs/CHARTER.md` rule 1, "CI exists before
the feature") applied to its own roadmap.

The order of the sections below is deliberate: §2 is the argument, §3 is the consequence.

---

## 1. Scope

**In scope for the first synthesis milestone:**

- `synth.lqr` — infinite-horizon continuous-time linear-quadratic regulator: state feedback
  `u = −Kx` minimising `∫ (xᵀQx + uᵀRu + 2xᵀNu) dt`, via the continuous-time algebraic Riccati
  equation (CARE).
- `synth.care` — the CARE solver itself, exposed as its own capability because it is the thing
  with a benchmark collection behind it and because a solver that can only be reached through a
  designer cannot be validated against one.
- `synth.pid` — a single-loop PID law, because it is what a reader will reach for first and
  because its verification story is entirely different and worth having side by side.

**Explicitly out of scope for this RFC:** LQG and Kalman filtering (see §5.5 — the margin story
changes completely and deserves its own record), H-infinity and mu-synthesis, gain scheduling,
discrete-time synthesis, and any form of automatic tuning.

---

## 2. How a synthesised law gets verified

Six layers. Each catches a class of error the one below it cannot, and each has a stated budget.
A layer with no budget is not a layer.

### 2.1 The equation is satisfied — a residual, not a comparison

The CARE

```
AᵀX + XA − (XB + N) R⁻¹ (BᵀX + Nᵀ) + Q = 0
```

is an equation, so the first check does not need a reference at all: form the residual and
require it small relative to the terms that made it.

```
||AᵀX + XA − (XB+N)R⁻¹(BᵀX+Nᵀ) + Q||_F  /  (||AᵀX||_F + ||XA||_F + ||Q||_F)
```

**Budget:** a stated multiple of machine epsilon times the conditioning of the Riccati equation,
reported alongside the residual — not a bare `1e-10`. The number the user sees is the residual
and its scaling, in the same way `TrimPoint` carries `residual_norm` and
`jacobian_condition_number` today (charter rule 9). Following `trim_level`: **failure is loud**.
A solver that cannot reach its residual throws; it does not return a matrix with a large number
attached, because that number gets dropped somewhere downstream.

This layer catches an implementation that solves *a* Riccati equation — the wrong sign
convention, the wrong cross-term handling — and nothing else does.

### 2.2 The structural properties hold

Independently checkable and cheap:

- `X` is symmetric to a stated tolerance, and positive semidefinite (eigenvalues of the symmetric
  part above `−tol`);
- the closed loop `A − BK` is Hurwitz, checked with the eigenvalue code that already exists;
- the closed-loop eigenvalues equal the stable half of the Hamiltonian spectrum, which is a
  different computation of the same quantity and therefore a real cross-check;
- `K = R⁻¹(BᵀX + Nᵀ)` is reproduced from `X`.

**Budget:** symmetry and the Hamiltonian-spectrum agreement get relative bounds stated in the
test; Hurwitz is a sign, not a tolerance.

### 2.3 Closed-form scalar and 2×2 cases

For scalar `a, b, q, r` the CARE is a quadratic with a closed-form positive root. A handful of
2×2 cases have closed forms too. These are exact answers, not published tables.

**Budget:** the closed-form value is exact, so the budget is pure floating-point accumulation —
a stated multiple of eps, on the pattern of the existing closed-form transfer-function gates in
`tests/unit/`.

### 2.4 A published benchmark collection

**Reference:** P. Benner, A. J. Laub and V. Mehrmann, *A collection of benchmark examples for the
numerical solution of algebraic Riccati equations, Part I: Continuous-time case*, Technical Report
SPC 95-22, Fakultät für Mathematik, TU Chemnitz-Zwickau, 1995 — the CAREX collection, already
named in `docs/ROADMAP.md`.

This is the right shape of reference for a solver: a set of problems designed to be *hard* in
specific, documented ways — ill-conditioned, nearly singular `R`, large parameter ranges — with
solutions or residual expectations published alongside. Several examples are parameterised, so
the same case can be walked toward its own singularity and the residual watched rather than
asserted once.

**Budget:** derived the same way as every other reference in this repository — from the
collection's own printed precision, per §3 of the `validating-against-a-published-source` skill.
Transcribe into `tests/validation/reference/carex1995_care.csv` with the file's own SOURCE,
RIGHTS and TRANSCRIPTION METHOD header, one `location` per value.

**Rights check first.** ADR-0007 governs. Establish the collection's rights position *before*
transcribing anything, and record where that position is printed. If it cannot be established
with confidence, the data does not ship, and the case ships as a generator plus instructions.

**Honest note:** the CAREX examples are matrices, not aeroplanes, and they will not catch an
error in how galata builds `Q` and `R` from a user's YAML. That is layer 2.5's job.

### 2.5 A published aircraft design example

**Candidate references, none yet transcribed:** B. L. Stevens, F. L. Lewis and E. N. Johnson,
*Aircraft Control and Simulation*, 3rd ed., Wiley, 2016 — already cited by the trim and
linearisation headers, and it works LQR designs on an aircraft model end to end. B. D. O. Anderson and
J. B. Moore, *Optimal Control: Linear Quadratic Methods*, Prentice-Hall, 1989. A. E. Bryson and
Y.-C. Ho, *Applied Optimal Control*, Hemisphere, 1975.

Which one is used is **not decided by this RFC**, and deliberately so: the choice depends on
finding an example whose plant, weights *and* resulting gains are all printed, which is rarer
than it sounds. Textbook LQR examples routinely print the design and omit one of the three.

The acceptance criterion for the reference is therefore stated as a property of the reference,
not as a name:

> A worked example is usable only if the plant matrices, the weighting matrices and the resulting
> gain (or the resulting closed-loop poles) are all printed, and the printed precision of each is
> visible. If only the gains are printed, it validates nothing: any `Q` and `R` can be
> reverse-engineered to produce a given `K`.

**Budget:** the source's printed precision, propagated through the design. Note that this is
harder than the modal case and the difficulty must be faced rather than hidden: `K` is a
*nonlinear* function of `Q`, `R` and the plant, so the input-rounding band has to be built the
way `tests/validation/test_nt33a_modes.cpp` builds it — by perturbing each printed input by half
a unit in its own last printed digit, **in the units the source prints it in**, and summing the
worst deviations.

**If no such example is found, the case is `Unvalidated` and says so.** An honest `Unvalidated`
is worth more than a fabricated match (charter rule 8). Layers 2.1–2.4 and 2.6 would still stand.

### 2.6 The theorem galata can check on any plant

This is the layer that makes synthesis verifiable *without* a published design, and it is the
reason the API in §3 returns what it returns.

For an LQR state-feedback loop broken at the **plant input**, with `N = 0`, the return-difference
**equality**

```
(I + L(jω))* R (I + L(jω)) = R + B*(−jωI − Aᵀ)⁻¹ Q (jωI − A)⁻¹ B ,   L = K(jωI − A)⁻¹B
```

holds exactly. `Q` is positive semidefinite, so the right-hand side dominates `R`; and when `R`
is a scalar — **a single-input plant** — or a multiple of the identity, `R` divides out and

```
|I + L(jω)| ≥ 1   for all ω
```

so the input sensitivity satisfies `|S(jω)| ≤ 1` at every frequency, hence **M_S ≤ 1**. Feed that
into the disk-margin relations already implemented in
[`include/galata/analyze/disk_margin.hpp`](../../include/galata/analyze/disk_margin.hpp), at skew
σ = +1 where `α = 1/M_S`:

| | |
|---|---|
| `α = 1/M_S ≥ 1` | |
| `γ_min = 1/(1+α) ≤ 1/2` | gain may be **halved** |
| `γ_max = 1/(1−α) → ∞` at α = 1 | gain may be increased **without bound** |
| `cos φ_m = (1 + γ_min γ_max)/(γ_min + γ_max) → γ_min = 1/2` | phase margin **≥ 60°** |

which is the classical LQR margin result, arrived at through formulas this repository has already
transcribed from a published source and already gates.

So the property test is: **synthesise an LQR law for an arbitrary stabilisable plant, form the
loop broken at the plant input, and require `M_S ≤ 1 + ε`, gain reduction tolerance to 1/2, and
a phase margin of at least 60°** — measured by `analyze.sensitivity` and `analyze.diskmargin`,
which exist, are validated, and were written without any of this in mind.

**Scope, stated rather than glossed.** For a general positive-definite `R` on a multi-input
plant, `R` does not divide out and the inequality holds only in the `R`-weighted norm. galata's
disk margin is in any case the **SISO** condition — `include/galata/analyze/disk_margin.hpp`
says so, and the multi-loop case needs a structured singular value galata does not have. So the
property test is stated for single-input plants and for `R = ρI`, and a multi-input plant with a
general `R` is a case the test **skips with a stated reason** rather than one it silently passes.

**Budget:** `ε` is not free. Both peaks in galata are **grid maxima**, and therefore lower bounds
on the true H-infinity norms — the error is optimistic, so a grid maximum can only make `M_S`
look *smaller* than it is. The interesting failure is therefore `M_S` measured *above* 1, and
`ε` covers only the finite-grid and floating-point slack, stated in the test.

This layer is worth more than it looks. It is a **published theorem checked numerically on plants
nobody has published a design for**, it composes two independently validated capabilities, and it
fails loudly if either the synthesis or the margin code drifts. It is also the one that catches
the single most likely implementation error: **breaking the loop in the wrong place.** The
guarantee holds at the plant input and not at the output, so a synthesis whose reported loop is
formed at the output will violate it on almost any plant.

Run the same property test with `N ≠ 0` and it should **fail**, because the guarantee does not
hold there — an assertion of a limit rather than of a capability, in the manner of
`DiskMarginSeiler2020.PublishedCriticalFrequencyDisagreesWithItsOwnPublishedPerturbation`.

---

## 3. The API, shaped by §2

### 3.1 The solver

```cpp
namespace galata::synth {

struct CareOptions {
  // Fixed, not a convergence criterion (ADR-0004). Any iterative refinement runs
  // a fixed number of steps and THEN reports the residual.
  int refinement_iterations = 0;
  // Relative residual above which solve_care throws rather than returning.
  double residual_tolerance = 1e-10;
};

struct CareSolution {
  Eigen::MatrixXd x;                    // the stabilising solution, symmetric PSD

  // Charter rule 9: the evidence travels with the answer.
  double relative_residual = 0.0;       // as defined in 2.1
  double symmetry_defect = 0.0;         // ||X - X^T||_F / ||X||_F
  double hamiltonian_separation = 0.0;  // spectral gap across the imaginary axis
  bool closed_loop_is_hurwitz = false;
  std::vector<std::complex<double>> closed_loop_eigenvalues;  // 1/s
};

// Solves A^T X + X A - (X B + N) R^-1 (B^T X + N^T) + Q = 0 for the stabilising X,
// by the generalised (ordered) Schur method.
// Throws std::invalid_argument on shape or definiteness violations; throws
// std::runtime_error if the residual is above tolerance.
[[nodiscard]] CareSolution solve_care(const Eigen::MatrixXd& a,
                                      const Eigen::MatrixXd& b,
                                      const Eigen::MatrixXd& q,
                                      const Eigen::MatrixXd& r,
                                      const Eigen::MatrixXd& n = {},
                                      const CareOptions& options = {});

}  // namespace galata::synth
```

**Why the generalised Schur method.** Reference: A. J. Laub, *A Schur method for solving algebraic
Riccati equations*, IEEE Transactions on Automatic Control, vol. 24, no. 6, pp. 913–921, 1979;
and W. F. Arnold and A. J. Laub, *Generalized eigenproblem algorithms and software for algebraic
Riccati equations*, Proceedings of the IEEE, vol. 72, no. 12, pp. 1746–1754, 1984. It is
deterministic — a fixed sequence of orthogonal transformations, no iteration count that depends on
a tolerance — which is what ADR-0004 requires of a gated numerical path. The generalised form
avoids forming `R⁻¹` explicitly, which matters exactly where `R` is nearly singular, and that is
half of what the CAREX collection is designed to probe.

### 3.2 The designer

```cpp
struct LqrRequest {
  Eigen::MatrixXd q;   // state weighting, symmetric PSD
  Eigen::MatrixXd r;   // control weighting, symmetric positive definite
  Eigen::MatrixXd n;   // cross term, may be empty
  // Which loop the margins below are measured on. There is no default:
  // LQR's guaranteed margins hold at the PLANT INPUT and not at the output,
  // so this is a modelling decision the tool refuses to make (the same
  // position analyze/margins.hpp already takes).
  LoopBreakPoint break_at;
};

struct LqrDesign {
  Eigen::MatrixXd k;                       // u = -K x
  CareSolution riccati;                    // carried, not discarded
  model::LinearSystem closed_loop;         // A - BK, ready for analyze.modes
  model::LinearSystem broken_loop;         // L(s), ready for analyze.margins
  std::vector<std::string> state_names;
  std::vector<std::string> input_names;
};
```

The two `LinearSystem` members are the load-bearing part of the design. They are what let
`analyze.modes`, `analyze.margins`, `analyze.diskmargin` and `analyze.sensitivity` be applied to a
synthesised law **without a single new line of analysis code**, which is what makes layer 2.6
possible and what makes the pipeline in §3.3 short.

`CareSolution` is carried into `LqrDesign` rather than discarded, so a report can print the
residual that stands behind the gain.

### 3.3 The pipeline surface

Synthesis is a capability like any other, so a design is a file:

```yaml
stages:
  - id: aircraft
    capability: model.aircraft.derivatives
    input: { path: ../../models/nt33a/nt33a-fc1.yaml }

  - id: trim
    capability: trim.level
    input: { aircraft: { from: aircraft }, altitude_m: 0.0, airspeed_m_s: 69.4944 }

  - id: plant
    capability: linearize.finitediff
    input: { trim_point: { from: trim }, axes: lateral }

  - id: law
    capability: synth.lqr
    input:
      system: { from: plant }
      q: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,10]]
      r: [[1,0],[0,1]]
      break_at: plant_input          # required, no default

  # Everything below is EXISTING, VALIDATED capability applied to a synthesised law.
  - id: modes
    capability: analyze.modes
    input: { system: { from: law }, use: closed_loop }

  - id: margins
    capability: analyze.diskmargin
    input: { system: { from: law }, use: broken_loop, input: aileron, output: aileron_command }
```

A new artefact kind `control_law` is added, with a Markdown section writer and a place in the
HTML report generator (`tools/report/`) alongside the trim point and the modal table. Charter
rule 2: the capability registry entry lands in the same commit as the code, and
`scripts/gen-status-table.sh --check` keeps the README honest.

### 3.4 `synth.pid`, and why its verification is different

A PID law has no Riccati equation and no return-difference theorem, so layers 2.1, 2.4 and 2.6 do
not apply to it at all. Its verification is:

- **exact transfer function** — `K_p + K_i/s + K_d s/(τ_d s + 1)` evaluated against a closed-form
  expression at a grid of frequencies, to a stated multiple of eps;
- **a published tuning example** — Åström & Murray, *Feedback Systems*, 2nd ed., Princeton, 2021,
  is already cited by
  [`include/galata/analyze/margins.hpp`](../../include/galata/analyze/margins.hpp) and works PID
  examples with printed values;
- **the derivative filter is mandatory and its absence is an error, not a default.** An unfiltered
  derivative term is not a proper transfer function, cannot be represented in state space, and
  would silently make every margin computed from it meaningless.

It is listed in this RFC precisely because putting it beside LQR makes the point that *the
verification strategy is per-capability and is designed before the capability*, not inherited.

---

## 4. Determinism

Synthesis is a gated numerical path, so ADR-0004 applies without dilution:

- the Schur method is a fixed sequence of orthogonal transformations — no tolerance-driven loop;
- any iterative refinement runs a **fixed** count and reports the residual afterwards;
- eigenvalue ordering across the imaginary axis must be a **total** order with an explicit
  tie-break, in the manner of the modal classifier's `score, then mode_index, then
  signature_index`, so the closed-loop eigenvalue list does not depend on sort stability;
- the design outputs join `tools/determinism/fingerprint.cpp`. Whether they can hold the tier-2
  1e-9 cross-platform bound is an **open question** (§6): `K` is downstream of an eigenvalue
  reordering, and near a degenerate Hamiltonian spectrum a libm disagreement could in principle
  select a different ordering. If it cannot hold, the values get the `tier1.` prefix **with a
  stated reason**, exactly as the linearisation does today. That decision is written down, not
  made silently.

---

## 5. What could go wrong, stated in advance

1. **Breaking the loop in the wrong place.** The single most likely error, and layer 2.6 catches
   it. Hence `break_at` has no default.
2. **A `Q` or `R` that is not what the user meant.** No layer catches this and none can. The
   report prints the weights it used, and the capability's `WHAT THIS IS NOT` block says that a
   design is only as meaningful as its weighting.
3. **Nearly singular `R`.** Where `R⁻¹` is formed the answer degrades quietly. The generalised
   Schur form avoids forming it; the CAREX cases probe it; the residual reports it.
4. **A published example that prints only the gains.** Unusable — see §2.5. Do not
   reverse-engineer the weights to make it fit; that is fitting, and the phugoid note explains at
   length why a fitted explanation is worth less than a predictive one.
5. **LQG.** Doyle, *Guaranteed margins for LQG regulators*, IEEE Transactions on Automatic
   Control, vol. 23, no. 4, pp. 756–757, 1978 — a one-page paper whose result is that there are
   none. Every guarantee in §2.6 is a property of **full state feedback**, and inserting an
   observer destroys it. When an estimator arrives, layer 2.6 does not extend to it and a separate
   record must say what replaces it. Writing that down now is cheaper than discovering it later.

---

## 6. Open questions this RFC does not settle

- Which published aircraft design example, if any, satisfies §2.5's criterion. Someone has to go
  and look.
- The CAREX collection's rights position under ADR-0007, and therefore whether its values ship
  in-tree or as a generator.
- Whether `K` can hold the tier-2 cross-platform bound, or needs the `tier1.` prefix (§4).
- Whether `synth.care` should also expose the DARE. The benchmark collection has a Part II for
  it; discrete-time synthesis is otherwise out of scope.
- Whether the guaranteed-margin property test belongs in `tests/property/` (it is an invariant
  over generated plants) or `tests/validation/` (it checks a published theorem). The tier
  determines what a failure means, and this one means both things.

## 7. Acceptance

This RFC is accepted when §2.5's reference question is answered — with a named example or with a
decision to ship the case as `Unvalidated` — and §6's rights question is settled. **No `src/synth/`
code merges before then**, because the API in §3 exists to serve the verification strategy in §2,
and a strategy with an unanswered reference question is not finished.
