# UH-60A Level-1 study model — provenance and limits

<!-- SPDX-License-Identifier: Apache-2.0 -->

This directory adds a native Galata helicopter model for the UH-60A reference
component. It is a software integration and early design-study artifact, not a
validated or qualified Black Hawk model. No result from this YAML establishes
aircraft performance, handling qualities, flight safety, airworthiness or
certification.

## Controlled source inputs

The controlled reference is K. B. Hilbert, A Mathematical Model of the UH-60
Helicopter, NASA TM-85890, April 1984, NTRS 19840015585. The repository's
examples/uh60-reference-component/reference-aircraft.json records the
transcription and its page/table location.

The executable model promotes the following compatible subset from Table 1 (or
derives the value algebraically from Table 1 values). Compatibility here means
that the current Level-1 schema and equations can consume the value without
changing the model's meaning:

| Model field | Source value | Conversion |
| --- | ---: | --- |
| main_rotor.radius_m | 26.83 ft | × 0.3048 = 8.177784 m |
| main_rotor.chord_m | 1.73 ft | The component records a rounded source chord; the YAML uses the solidity-derived value from the controlled 0.08210 and four blades |
| main_rotor.blade_count | 4 | dimensionless |
| main_rotor.solidity | 0.08210 | dimensionless |
| main_rotor.omega | 27.0 rad/s | already SI |
| tail_rotor.radius_m | 5.5 ft | × 0.3048 = 1.6764 m |
| tail_rotor.chord_m | solidity 0.1875, 4 blades, R = 1.6764 m | σπR/N = 0.24687027771 m |
| drivetrain.tail_gear_ratio | 124.62/27.0 | = 4.61555555556 |
| mass.mass_kg | 16400 lbf | × 0.45359237 = 7438.914868 kg |
| mass.inertia_xx_kg_m2 | 5629 slug-ft² | × 1.35581794833 = 7631.89923262 kg m² |
| mass.inertia_yy_kg_m2 | 40000 slug-ft² | × 1.35581794833 = 54232.71794364 kg m² |
| mass.inertia_zz_kg_m2 | 37200 slug-ft² | × 1.35581794833 = 50436.42768758 kg m² |
| mass.product_of_inertia_xz_kg_m2 | 1670 slug-ft² | × 1.35581794833 = 2264.21597415 kg m² |
| horizontal_tail_area_m2 | 45 ft² | × 0.09290304 = 4.1806368 m² |
| horizontal_tail_lift_slope | 1.03 | direct Table 1 value |
| vertical_tail_area_m2 | 32.3 ft² | × 0.09290304 = 3.000768192 m² |
| vertical_tail_side_slope | 0.89 | direct Table 1 value |

The source also records main-rotor values that are not promoted into the
executable Level-1 model yet:

| Source quantity | Table 1 value | Current executable treatment |
| --- | ---: | --- |
| main-rotor blade twist | -0.3142 rad | The YAML retains the explicit Level-1 placeholder -0.12217304764 rad; the current equations do not implement the complete UH-60 flapping/trim convention needed to use the source value safely. |
| main-rotor hinge offset | 0.04659 R = 0.38100295656 m | The YAML retains the Level-1 placeholder 0.25 m; the current model does not yet expose the source hinge convention end-to-end. |
| main-rotor shaft tilt | 0.05236 rad | The YAML retains the Level-1 placeholder 0.03490658504 rad; the source definition is not yet represented by the current trim equations. |
| maximum main-rotor thrust coefficient/solidity | 0.1846 | The YAML retains the Level-1 placeholder 0.11; the current limit is not yet proven equivalent to the source envelope convention. |

The source component contains an acceptance tolerance for the solidity
recomputation and the exact conversion policy. The source chord is rounded to
two decimal feet, so the solidity-derived YAML chord is intentionally the
controlled value. This native model does not silently promote that component
transcription into a full aerodynamic model.

## Assumptions added for executable Level-1 coverage

The reference component does not provide the complete parameter set required by
Galata's common Level-1 helicopter equations. The following remain explicit
assumptions, not source facts:

- tail-rotor inertia, location, cant and loading limits;
- rotor blade lift/drag, tip loss, flap inertia, polar inertia and inflow parameters;
- fuselage flat-plate representation, empennage moment arms and incidences;
- rotor hub height/tilt and anti-torque orientation;
- main-rotor twist, hinge offset, flap stiffness, shaft-tilt convention and
  thrust-coefficient limit;
- governor, engine torque, transmission/accessory load and speed limits;
- actuator travel, rate and lag;
- rotor blockage and pedal-to-tail-collective mapping.

The values are conservative integration placeholders selected to keep the model
inside the shared parser, trim and linearisation contract. They are not an OEM
data substitute. Replace the remaining assumptions with controlled UH-60 data
before using any control-law or flight-envelope result. Promoting Table 1 values
does not validate the unsupported dynamics.

## What is now evidenced

examples/uh60a-hover-trim/study.yaml proves that this model loads through the
native helicopter schema, reaches a numerically converged hover trim and can be
linearised by the shared vehicle path. That is model/software-path evidence
only. It is not agreement with Hilbert's full UH-60 mathematical model and is
not flight-test evidence.
