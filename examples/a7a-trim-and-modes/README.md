# A-7A condition-1 derivative workflow

This example loads the clean flexible-airplane A-7A condition-1 derivative set
from [`models/a7a/a7a-fc1.yaml`](../../models/a7a/a7a-fc1.yaml), trims it at
sea level and the published 279 ft/s condition, finite-difference linearises
both axes, classifies the modes and writes a Markdown report.

```bash
galata run examples/a7a-trim-and-modes/study.yaml \
  --output-dir build/a7a-study
```

The source is NASA CR-96008, *Aircraft Stability and Control Data*, Section II,
Tables II-A through II-F. See [`models/a7a/PROVENANCE.md`](../../models/a7a/PROVENANCE.md)
for the transcription, conversions and local validity boundary.

This is a third published-data aircraft model exercising the shared fixed-wing
path. It is not a global A-7A model, a flight-test validation, an airworthiness
result or a certification artifact.
