---
name: adding-an-aircraft-model
description: How to add an aircraft to models/ — the strict-SI units contract and what happens at the boundary where aeronautical sources publish feet, slugs and pounds-force; the YAML schema key by key with what is required, what defaults and what is silently accepted; the non-dimensional to dimensional path and where the stability-to-body rotation happens; and the three things a new model must ship with — a citation, a validity envelope, and at least one validation case with a stated budget. Use when adding or editing anything under models/, touching src/model/aircraft.cpp, or writing a PROVENANCE.md.
---

# Adding an aircraft model

A model is **one YAML file plus a sibling `PROVENANCE.md`** under `models/<name>/`. There is no
registry to add it to and no schema document to update: `load_aircraft`
(`src/model/aircraft.cpp`) *is* the schema, and it is the only reader of a model file.

That has a consequence worth stating up front: **unknown and misspelled keys are silently
ignored.** Nothing iterates the document's keys. A typo in an optional key gives you a model
that loads, trims, linearises and is quietly wrong. Read the loader, not another model file.

---

## 1. The units contract

**The numerical core is strictly SI** (ADR-0003, charter rule 5). Metres, seconds, kilograms,
newtons, newton-metres, radians, radians per second, kelvin, pascals. **The model file is SI
too** — the key carries the unit, always: `wing_area_m2`, `mass_kg`, `reference_alpha_rad`,
`thrust_incidence_rad`. Never `altitude`; always `altitude_m`.

### The boundary, where aeronautical sources live

Published aircraft data is in feet, slugs, pounds-force and slug-ft². That conversion happens
**once, at transcription time, by hand, and is recorded** — not in the loader, and never in the
solver.

`scripts/check-si-boundary.sh` enforces this, and the failure mode it is really aimed at is not
someone calling `feet_to_metres` inside a solver, because nobody does that. It is someone
writing `alt * 0.3048` inline because it seemed obvious. So the gate greps for **conversion
factors as numeric literals** as well as for the function names, across `src/core`, `src/model`,
`src/numerics`, `src/trim`, `src/linearize`, `src/analyze`, `src/sim` and their headers. A
constant that legitimately happens to equal a conversion factor needs a `GALATA_SI_EXEMPT`
comment on the line explaining what it is.

`include/galata/units.hpp` is the single exemption, because it is the definition site. Every
factor there is **exact by definition**, and each carries the definition that makes it exact:

| From | To | Factor | Basis |
|---|---|---|---|
| international foot | metre | 0.3048 | exact, 1959 international yard and pound agreement |
| nautical mile | metre | 1852 | exact, by definition |
| knot | m/s | 1852 / 3600 | exact, from the nautical mile |
| degree | radian | π / 180 | exact |
| °C | K | + 273.15 | exact, by definition of the kelvin |
| pound-force | newton | 0.45359237 × 9.80665 | exact, from the pound and standard gravity |
| slug | kilogram | that / 0.3048 | exact, from the two above |

**Non-exact conversions are not permitted.** If a factor cannot be written exactly, the
underlying unit is not one this project converts — which is why the SI values in a model file
are *exact transcriptions* and not approximations, and why `PROVENANCE.md` can say so.

Note also what ADR-0003 rejects and why: a compile-time dimensional-analysis type system is the
strictly better engineering answer and is rejected on **interface cost, not on merit**, because
an aircraft's A matrix mixes 1/s, rad/s, m/s and dimensionless entries in the same matrix and a
dimensioned scalar does not compose with Eigen. Do not reopen that in a model PR.

---

## 2. The YAML contract, key by key

Everything below is what `load_aircraft` actually does.

### Top level

| Key | Required | Unit / type | Notes |
|---|---|---|---|
| `description` | no | free text | Default empty. **Never validated.** |
| `citation` | no | free text | Default empty. **A model with no citation loads without complaint** — the rule that it must have one is charter rule 7 and review, not the loader. |
| `thrust_incidence_rad` | no | rad | Default 0. Positive nose-up relative to the body x-axis. |
| `geometry` | **yes** | map | |
| `mass` | **yes** | map | |
| `aero` | **yes** | map | |

### `geometry` — all three required

`wing_area_m2`, `wing_span_m`, `mean_aerodynamic_chord_m`.

`validate()` rejects a chord longer than the span: *"One of them is probably in the wrong
units."*

### `mass`

Required: `mass_kg`, `inertia_xx_kg_m2`, `inertia_yy_kg_m2`, `inertia_zz_kg_m2` — about the CG,
in body axes.

Optional: `product_of_inertia_xz_kg_m2`, default 0.

> **Quote `product_of_inertia_xz_kg_m2` exactly as the source prints it.** The loader negates it,
> because in the aerospace convention a positive quoted I_xz is a *negative* off-diagonal entry.
> Pre-negating it to "help" flips the roll–yaw coupling handedness, and `MassProperties::validate()`
> will not catch it: a sign-flipped I_xz is still symmetric, still positive definite, and still
> satisfies the triangle inequality.

There are **no keys for I_xy or I_yz**; those entries are hard-coded zero, so lateral mass
asymmetry cannot be expressed.

`MassProperties::validate()` enforces positive mass, symmetry, positive definiteness (via
`SelfAdjointEigenSolver`) and the triangle inequality on the principal moments.

### `aero`

**Required, six numbers:** `reference_alpha_rad`, `lift_ref`, `drag_ref`, `lift_alpha`,
`pitching_moment_alpha`, `pitching_moment_elevator`.

**Required, one string:** `lateral_axes`, exactly `body` or `stability`. See §3 — this is the
one that bites.

**Optional, defaulting to 0:** `reference_mach`, `pitching_moment_ref`, `drag_alpha`,
`lift_pitch_rate`, `pitching_moment_pitch_rate`, `pitching_moment_alpha_dot`, `lift_elevator`,
`drag_elevator`, and the thirteen lateral-directional derivatives — `side_force_beta`,
`rolling_moment_beta`, `yawing_moment_beta`, `rolling_moment_roll_rate`,
`yawing_moment_roll_rate`, `rolling_moment_yaw_rate`, `yawing_moment_yaw_rate`,
`side_force_aileron`, `rolling_moment_aileron`, `yawing_moment_aileron`, `side_force_rudder`,
`rolling_moment_rudder`, `yawing_moment_rudder`.

**Two keys exist only to be rejected:** `lift_alpha_dot` and `drag_alpha_dot`. A non-zero value
throws, with the reason — an alpha-dot *force* derivative makes the model implicit, since the
vertical acceleration would depend on α̇ which depends on the vertical acceleration. Rejected
rather than silently dropped. `pitching_moment_alpha_dot` **is** supported.

**`reference_mach` is optional and it is a trap.** Omit it and it defaults to 0, so the envelope
advisory computes a Mach departure of |M − 0| = M, which exceeds `kAdvisoryMachLimit = 0.15` at
any real airspeed — and every result the model produces is flagged
`outside_advisory_envelope`. Set it.

### What `validate()` will *not* catch

Empty citation or description. A zero or wrong-sign `pitching_moment_alpha`. Zero lateral
derivatives, or zero aileron and rudder authority. A positive `drag_ref`. Any
`reference_alpha_rad`. Unknown keys. It does catch: non-positive geometry, chord > span,
non-zero α̇ force derivatives, non-positive `lift_alpha` (*"a sign error, not an exotic
configuration"*), and a zero `pitching_moment_elevator` (no longitudinal trim exists).

One more sharp edge: a **non-numeric value under a numeric key** escapes as a raw yaml-cpp
exception rather than as `std::invalid_argument`, because the `try`/`catch` wraps only
`YAML::Load`.

---

## 3. `lateral_axes` — the key that decides whether the model is right

Lateral-directional derivatives are published in **stability axes** as often as in body axes,
and the two differ by a rotation about the reference angle of attack. `load_aircraft` requires
you to say which, and rotates when you say `stability`, at **load time**, using the model's
`reference_alpha_rad` — not the trim alpha.

The absent-key message is long on purpose: *"the difference is tens of percent in the Dutch roll
damping, and it cannot be inferred from the values. State it."*

It is tempting to skip the rotation on the grounds that a 2.2° trim alpha has a cosine of 0.9993.
**That reasoning is wrong, and this repository has the scar.** The rotation *mixes* the rolling
and yawing moments, and on the NT-33A `C_l_beta` is 2.6 times `C_n_beta`, so the cross term
dominates the cosine: `C_n_beta` moves by 10%, not 0.07%. Left unrotated, the Dutch roll damping
came out **35% high** — and it was the comparison against the published *dimensional*
derivatives that caught it, not review.

Note also that `lateral_axes` is read **last**, after every numeric key. A newcomer who copies
only the numeric block gets a throw, which is the intended behaviour. A newcomer who writes
`lateral_axes: body` for a set that was published in stability axes gets a model that loads,
trims, linearises and is tens of percent wrong.

The rotation deliberately leaves the **side-force** derivatives untouched.

---

## 4. The non-dimensional to dimensional path

There is no dimensionalisation step and no dimensional derivative table. It happens entirely
inside `Aircraft::wrench`, per evaluation:

```
q_bar          = ½ ρ V²
reference_force = q_bar · S              the single force scaling

p̂ = p·b/(2V)     q̂ = q·c̄/(2V)     r̂ = r·b/(2V)     α̂̇ = α̇·c̄/(2V)

roll  moment = C_l · q_bar · S · b
pitch moment = C_m · q_bar · S · c̄
yaw   moment = C_n · q_bar · S · b
```

Lift and drag are built in **stability axes** and rotated into body axes; the side force is
added in body axes and is **not** rotated.

Below **1 m/s** the wrench throws: the rate non-dimensionalisation divides by V, *"so this model
is not defined at rest."*

The consequence for a new model is the useful part: **you supply non-dimensional coefficients and
geometry, and nothing else.** If your source publishes dimensional derivatives, they are not the
model input — they are a *check* on it, which is exactly how the NT-33A's Table II-7 comparison
is used.

---

## 5. What a new model must ship with

### (a) A `PROVENANCE.md`

Follow the shape of `models/nt33a/PROVENANCE.md`, whose headings are:

```
# <name> — provenance
## Source
## Rights
## Transcription
## The aircraft and the condition
## Conversions
## Choices made here that the source does not make
## What this model is not
```

**Source** names the report, its numbers (NTRS / DTIC / contract), a URL, and the **size, page
count and SHA-256** of the exact file the numbers were read from.

**Rights** quotes the source's own rights statement rather than asserting a conclusion. ADR-0007
governs: coefficient data ships in-tree only when it comes from a work whose rights position
permits it. Data traceable only to a copyrighted source ships as a loader plus instructions,
never as data. A dataset whose licensing cannot be established is not shipped at all.

**Transcription** says how the digits were obtained. The NT-33A entry is worth copying as a
standard: the OCR layer was unusable and *was not used*, every value was read visually from
rendered page images, and a second independent pass re-downloaded the document, confirmed its
SHA-256, and re-read twelve values without reference to the first.

**Conversions** states that every factor is exact by definition, so the SI values are exact
transcriptions and not approximations, with the factor table.

**Choices made here that the source does not make** is the section people skip and should not.
The NT-33A has two, and both change results: the source publishes no trim pitching moment and no
trim elevator, so the model is *defined* to be trimmed at its reference condition with the
elevator centred — which means the trimmed elevator it produces *is near zero by construction
and is not evidence of anything* — and the lateral set is in stability axes and is rotated.

**What this model is not** is the validity envelope: for the NT-33A, a first-order expansion
about **one** flight condition, with no stall, no Mach effects and no engine.

### (b) A reference file in the source's own units

The original-unit values go in `tests/validation/reference/<model>.csv`, with its own
SOURCE / RIGHTS / TRANSCRIPTION METHOD header and a `location` naming the table and page for
each value. Keep them **as printed** — the units the source prints them in — because that is
what an error budget has to be taken on. See the `validating-against-a-published-source` skill.

### (c) At least one validation case with a stated budget

Follow `tests/validation/test_nt33a_trim_linearize.cpp`. It loads the model, trims at the
published condition, and then asserts, in order:

- the trim's own evidence — residual and Jacobian conditioning — and that the trim is inside the
  model's advisory envelope;
- a **closed-form force balance** independent of the solver: L = mg − D·tan α, T = D/cos α;
- the trim's dynamic pressure and Mach against the published condition;
- published **dimensional** derivatives, at a budget derived from the source's printed precision;
- the classical **modes**, likewise;
- the linearisation's own health: neglected coupling, worst relative truncation, chart
  conditioning.

Then register the case in `tools/validation/case_registry.cpp` and regenerate
`docs/VERIFICATION.md`. A model with no validation case is `Unvalidated`, and that is a legitimate
status — but say so in the registry rather than leaving the question open.

---

## Checklist

- [ ] `models/<name>/<name>-<condition>.yaml` with every key carrying its SI unit.
- [ ] `lateral_axes` stated explicitly, and correct.
- [ ] `reference_mach` set — not left to default to 0.
- [ ] `product_of_inertia_xz_kg_m2` quoted as the source prints it, not pre-negated.
- [ ] `models/<name>/PROVENANCE.md` with all eight headings, including the choices the source
      does not make and the validity envelope.
- [ ] `tests/validation/reference/<name>.csv` in the source's **own** units, with per-value
      locations.
- [ ] A validation test that trims, linearises and compares against published values inside a
      budget derived from the source's printed precision.
- [ ] The case registered, and `docs/VERIFICATION.md` regenerated and committed.
- [ ] `scripts/check-si-boundary.sh` passes.

## Never

- Convert units inside the model loader or anywhere in the numerical core.
- Use a conversion factor that is not exact by definition.
- Copy the numeric block of an existing model file without reading `load_aircraft`. Unknown keys
  are ignored, so a typo is silent.
- Ship coefficient data whose rights position you have not established and written down.
- Add a model without a validity envelope. A model whose error direction is unstated is a model
  whose user cannot tell whether it is conservative.
