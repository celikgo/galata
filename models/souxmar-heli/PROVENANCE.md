# Souxmar F1 — where every number came from

<!-- SPDX-License-Identifier: Apache-2.0 -->

**This is a design study, not an aircraft.** No Souxmar helicopter has been
built, flown or measured. Nothing in this directory is flight-test data, wind-
tunnel data, or a published reference. A completed run against this model is
evidence about the equations, never about an aircraft.

## The source

Souxmar helicopter programme, **design revision I**, 12 September 2026,
`turboshaft_rev_i/results.json` and its `configuration_inputs` block, produced by
`turboshaft_rev_i/recalculate.py`. The package is the user's own preliminary
design work. Its own stated limitations are carried forward in full at the end
of this file.

The design package is a **sizing and mission study**. It computes hover power,
mission fuel and a weight statement. It does **not** contain stability
derivatives, a mass-properties tensor, blade inertias, actuator limits,
empennage geometry or a governor model — all of which a flight-dynamics model
needs. Every such quantity below is therefore marked **assumed**, and the basis
for the assumption is stated.

## Rights position

The design package is the repository owner's own work, used with their
direction. It is not a third-party publication and carries no third-party
rights. No proprietary engine deck, OEM data or copyrighted dataset is
reproduced here: the PW207D1 figures the design package cites are used only to
derive a drive-input torque limit, and the limit actually applied is the design
package's own 700 kW drive rating, which binds first.

## Transcription method

Values were read from `turboshaft_rev_i/results.json` programmatically, not
retyped. Derived quantities were computed from those values by the expressions
shown below and are reproducible by running them again. Angles in the design
package are stated in degrees where they appear at all; this file records the
conversion and the YAML carries radians, per ADR-0003.

---

## 1. Taken directly from the design package

| YAML key | Value | Source field | Note |
|---|---|---|---|
| `mass.mass_kg` | 2829.7 | `weight.gross_at_full_fuel_kg` | Empty 1809.7 + 480 payload + 540 usable fuel |
| `main_rotor.radius_m` | 6.0 | `configuration_inputs.radius_m` | 12 m diameter |
| `main_rotor.blade_count` | 4 | `configuration_inputs.blades` | |
| `main_rotor.profile_drag_coefficient` | 0.011 | `configuration_inputs.cd0` | |
| `main_rotor.induced_power_factor` | 1.15 | `configuration_inputs.kappa` | |
| `tail_rotor.radius_m` | 1.2 | `configuration_inputs.tail_radius_m` | 2.4 m diameter |
| `tail_rotor.profile_drag_coefficient` | 0.012 | `configuration_inputs.tail_cd0` | |
| `tail_rotor.induced_power_factor` | 1.2 | `configuration_inputs.tail_kappa` | |
| `airframe.flat_plate_area_m2` | 1.3 | `configuration_inputs.flat_plate_m2` | |
| `drivetrain.transmission_efficiency` | 0.95 | `configuration_inputs.drive_efficiency` | |
| `tail_rotor.position_cg_to_hub_body_m[0]` | −6.7 | `configuration_inputs.tail_arm_m` | Sign: aft is −x in FRD |
| `main_rotor.maximum_thrust_coefficient_solidity` | 0.11 | `configuration_inputs.main_loading_screen` | The design's own C_T/σ screen |
| `main_rotor.maximum_advance_ratio` | 0.35 | `configuration_inputs.max_screen_mu` | |

## 2. Derived from the design package, by stated arithmetic

| YAML key | Value | Derivation |
|---|---|---|
| `drivetrain.reference_rotor_speed_rad_s` | 32.5 | `tip_m_s / radius_m` = 195 / 6 |
| `main_rotor.chord_m` | 0.339292006587698 | `solidity · π · R / N_b` = 0.072 · π · 6 / 4 |
| `tail_rotor.chord_m` | 0.169646003293849 | `tail_solidity · π · R_t / N_bt` = 0.18 · π · 1.2 / 4, with N_bt = 4 assumed |
| `drivetrain.tail_gear_ratio` | 5.0 | `(tail_tip_m_s / tail_radius_m) / (tip_m_s / radius_m)` = 162.5 / 32.5 |
| `drivetrain.accessory_torque_n_m` | 615.4 | `accessories_kw · 1000 / Ω` = 20000 / 32.5 |
| `drivetrain.maximum_engine_torque_n_m` | 20461.5 | `proposed_pair_rotor_drive_input_limit_kw · 1000 · η / Ω` = 700000 · 0.95 / 32.5. **The drive limit, not the engine limit.** Two PW207D1 at MCP would give 26 600 N·m at this shaft; `drive.no_credit_for_full_910kw_on_existing_rotor_drive` in the design package says that credit is not taken, so the drive rating binds. |

Round-trip check: `main_rotor.solidity()` recomputes 0.072000 exactly, and
`tail_rotor.solidity()` recomputes 0.180000 exactly, from the chords above. A
unit test asserts both, so the chord and the design's declared solidity cannot
drift apart.

## 3. The inertia tensor — computed, rejected, and replaced

The design package's weight statement gives thirteen mass groups with **x and z
offsets but no y offsets**, and no longitudinal distribution for the tail boom.
Computing a point-mass inertia from it gives, about the gross CG:

```
Ixx  2030.3     Iyy  2271.6     Izz   241.3     Ixz   175.2   kg m^2
```

**These are not usable, and the reason is visible in the numbers themselves.**
With no lateral spread every mass lies in the x–z plane, so `Izz` collects only
the longitudinal offsets and comes out twenty times too small for an aircraft of
this size. The principal-moment triangle inequality `Izz + Ixx ≥ Iyy` is then
satisfied only marginally — 241.3 + 2030.3 = 2271.6 against 2271.6 — which is
the degeneracy announcing itself. `sim::MassProperties::validate()` would refuse
a tensor a little worse than this one.

**What is shipped instead.** Radii of gyration as fractions of the rotor radius,
which is standard preliminary-rotorcraft practice when a mass-properties
breakdown is not yet available:

```
k_x = 0.16 R    Ixx = m (k_x R)^2 = 2829.7 · (0.96)^2 = 2608
k_y = 0.30 R    Iyy = m (k_y R)^2 = 2829.7 · (1.80)^2 = 9168
k_z = 0.27 R    Izz = m (k_z R)^2 = 2829.7 · (1.62)^2 = 7426
```

`Ixz` is kept at the point-mass value of 175 kg·m², because the longitudinal and
vertical offsets the design package **does** supply are exactly what that
product of inertia is made of.

**Direction of the error.** The gyration fractions are typical of a light twin
of this class; a real Souxmar airframe with its actual equipment layout will
differ. `Iyy` and `Izz` are the two that set the pitch and yaw response, so an
error of ±20% in them moves the pitch and yaw mode frequencies by roughly ∓10%.
**Every modal result from this model inherits that uncertainty, and no modal
result should be quoted without it.** Replacing this block with a real
mass-properties statement is the single highest-value improvement available to
this model.

## 4. Assumed, with the basis stated

| YAML key | Value | Basis |
|---|---|---|
| `main_rotor.lift_curve_slope` | 5.73 /rad | 2π per radian reduced for finite thickness; the conventional rotorcraft value |
| `main_rotor.blade_twist_rad` | −0.1396 (−8°) | Typical linear washout for a four-blade articulated main rotor |
| `main_rotor.tip_loss_factor` | 0.97 | Prandtl tip-loss value conventional for this blade count and loading |
| `main_rotor.hinge_offset_m` | 0.18 (3% R) | Small articulated offset; sets the hub moment and therefore the control power |
| `main_rotor.flap_stiffness_n_m_rad` | 48 000 | Chosen with the hinge offset to give a flap frequency ratio near 1.05, typical of a moderately articulated rotor |
| `main_rotor.blade_flap_inertia_kg_m2` | 288 | Uniform blade, `m_b R² / 3` with `m_b` = 24 kg; blade mass apportioned from the 213.1 kg rotor-group weight |
| `main_rotor.polar_inertia_kg_m2` | 1152 | `N_b m_b R² / 3` with the same blade mass |
| `tail_rotor.polar_inertia_kg_m2` | 2.88 | Same form, `m_b` = 1.5 kg |
| `*.inflow_time_constant_s` | 0.1 main, 0.05 tail | Dynamic-inflow lag of order `0.85 / (4 λ_h Ω)`; conventional values for this disc loading |
| `main_rotor.shaft_tilt_forward_rad` | 0.0524 (3°) | Conventional forward shaft tilt so the fuselage sits near level in cruise |
| `main_rotor.position_cg_to_hub_body_m` | [0, 0, −2.5] | Rotor 3.4 m above ground (`rotor_height_m`) less the gross CG height of 0.88 m from the weight statement |
| `airframe.horizontal_tail_*` | 1.1 m², slope 3.8, −3° | Sized to the tail arm; low-aspect-ratio slope. **No wind-tunnel data exists for this airframe.** |
| `airframe.vertical_tail_*` | 1.0 m², slope 3.2 | As above |
| `airframe.horizontal_tail_downwash_factor` | 0.4 | Constant fraction of main-rotor induced velocity at the tail. The real value varies strongly with advance ratio; see the model header's `WHAT THIS IS NOT` |
| `tail_rotor_blockage_factor` | 0.92 | Fin blockage of the tail-rotor disc; conventional 5–10% loss |
| `drivetrain.governor_proportional_n_m_s` | 2500 | Chosen to hold steady-state droop under about 2% at full collective; not an engine control law |
| `drivetrain.governor_time_constant_s` | 0.3 | Turboshaft torque response; order-of-magnitude only |
| `actuators.*` | see YAML | Travels are conventional for this class; rate limits and lags are **assumed**, not measured. Actuator data is the most commonly missing item in a preliminary package and the most consequential for a control law |
| `*.maximum_*` envelope limits | see YAML | Advisory limits for `envelope()`, taken from the design's own screens where it has them |

## 5. Cross-check against the design package's own hover computation

The design package computes main-rotor hover power by an independent route (a
Python momentum-theory script) and prints `hover[0].main_kw = 504.86 kW` and
`hover[0].main_ct_sigma = 0.08545` at sea-level ISA, 3170 kg, with a 1.04 hover
download factor.

This model, asked for the collective that carries the same thrust at the same
condition, reaches **C_T/σ = 0.0852** against the package's **0.08545** — a
**0.3%** difference. That is a cross-implementation agreement, not a validation:
both are momentum theory, computed by different code from the same geometry. It
is recorded because a disagreement would have meant one of the two was wrong,
and it is gated by a validation-tier case with a budget derived before the
comparison.

## 6. The design package's own stated limitations, carried forward

Reproduced verbatim from `turboshaft_rev_i/results.json`:

- Constant 3170 kg power across mission; no fuel-burn relief
- SFC .32 kg/kWh is a project assumption, not a PW207D1 fuel map
- 35 kg fixed allowance for ground/climb/approach, no resolved segment model
- No calls to generic engine lapse/available function
- No installed performance or aircraft compliance established

To which this model adds:

- **No measured aerodynamic data of any kind exists for this airframe.**
- The inertia tensor is a gyration-fraction estimate, not a mass-properties
  statement (§3).
- Actuator rate limits and lags are assumed and are the parameters a control-law
  result is most sensitive to.
- Everything in the model header's `WHAT THIS IS NOT` block applies: no ground
  effect, no stall, no compressibility, no dynamic flapping, no lead-lag, no
  autorotative energy exchange, no ground contact.
