# F-4C power-approach derivative workflow

This example loads the F-4C power-approach derivative set from
[`models/f4c/f4c-power-approach.yaml`](../../models/f4c/f4c-power-approach.yaml),
trims it at the published sea-level speed, finite-difference linearises both
axes, classifies the modes and writes a Markdown report.

```bash
galata run examples/f4c-power-approach/study.yaml \
  --output-dir build/f4c-power-approach-study
```

The derivative source is NASA CR-2144, Section IV, Figure IV-1 and Table IV-1.
See [`models/f4c/PROVENANCE.md`](../../models/f4c/PROVENANCE.md) for the
transcription, geometry reference, conversions and local validity boundary.

This is a fifth independently sourced fixed-wing reference condition. It is
not a global F-4C model, a flight-test validation, an airworthiness result or
a certification artifact.
