# Navion nominal derivative workflow

This example loads the Navion nominal derivative set from
[`models/navion/navion-nominal.yaml`](../../models/navion/navion-nominal.yaml),
trims it at sea level and the published 176 ft/s condition, finite-difference
linearises both axes, classifies the modes and writes a Markdown report.

```bash
galata run examples/navion-trim-and-modes/study.yaml \
  --output-dir build/navion-study
```

The source is NASA CR-96008, *Aircraft Stability and Control Data*, Section X,
Tables X-A through X-E. See [`models/navion/PROVENANCE.md`](../../models/navion/PROVENANCE.md)
for the transcription and conversions.

This is a second published-data aircraft model exercising the same shared
fixed-wing path as the NT-33A. It is not a global Navion model, a flight-test
validation, an airworthiness result or a certification artifact.
