# Response-metric migration

## Scope

Milestone response artifacts written before this change remain historical
evidence. New artifacts use the definitions below; a previous PASS is not
silently reinterpreted as a new PASS.

| Metric | Current definition | Unit | Historical difference |
|---|---|---|---|
| Peak tracking error | Maximum absolute measured-minus-reference error over the segment | signal unit | Replaces the overloaded `tracking_error` interpretation only when the new per-signal key is declared |
| Final tracking error | Absolute error at the final sample of the segment | signal unit | New explicit metric |
| RMS tracking error | Root mean square of sampled error over the segment | signal unit | New explicit metric |
| Settling band | Dedicated absolute error band | signal unit | Independent of peak allowance |
| Settling duration | Time from the segment event to the first continuous in-band dwell | s | No longer derived from the peak bound |
| Settling status | `already_within_band`, `demonstrated_recovery`, or `not_settled_within_observation_window` | enum | Historical already-at-trim cases are not called recovered |
| Overshoot | Positive excursion beyond the final value divided by the absolute signed reference-step amplitude | dimensionless fraction | Only applicable to a meaningful step; zero-amplitude, ramp, and disturbance-only segments report inapplicable |
| Open/closed peak ratio | Controlled peak tracking error divided by matched uncontrolled peak tracking error | dimensionless ratio | Renamed from the former relative-performance `overshoot` interpretation |
| Improvement | `1 - open_closed_peak_ratio` | dimensionless fraction | New explicit relative-performance metric |
| Requested-minus-limited | Requested command minus the actuator-limited command | actuator unit | Separate mismatch, not effort |
| Limited-minus-delayed | Limited command minus the delayed/applied command | actuator unit | New explicit delay metric |
| Delayed-minus-actual | Applied command minus actual actuator position | actuator unit | New explicit actuator-tracking metric |
| Control effort | Peak/RMS applied-command deviation from the first applied trim command | actuator unit | Replaces the overloaded `control_effort_rad` wording |

## Schema rules

`signal_requirements` is keyed by named signal and requires unit-compatible
names such as `peak_tracking_error_rad`, `settling_band_m`, or
`rms_tracking_error_rad_s`. Unknown requirement names and unit-incompatible
names are refused. The legacy aggregate aliases remain accepted only to keep
historical study files executable; they are not used by the new examples.

Each reference step is analyzed as its own segment. A ramp is evaluated for
tracking but is not treated as a collection of fictitious steps. A report lists
all segment statuses and times. Requirements are evaluated against the final
segment when a later reference change truncates an earlier observation window;
the earlier result remains visible.

## Reproduction

```sh
build/dev/src/cli/galata run examples/heli-performance-acceptance/study.yaml --output-dir build/heli-performance-acceptance
build/dev/src/cli/galata run examples/heli-reference-tracking/study.yaml --output-dir build/heli-reference-tracking
```
