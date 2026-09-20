# A-4D condition-1 derivative workflow

This example loads the clean flexible-airplane A-4D condition-1 derivative set
from [`models/a4d/a4d-fc1.yaml`](../../models/a4d/a4d-fc1.yaml), trims it at
sea level and the published 447 ft/s condition, finite-difference linearises
both axes, classifies the modes and writes a Markdown report.

```bash
galata run examples/a4d-trim-and-modes/study.yaml \
  --output-dir build/a4d-study
```

The source is NASA CR-96008, *Aircraft Stability and Control Data*, Section III,
Tables III-A through III-C. See [`models/a4d/PROVENANCE.md`](../../models/a4d/PROVENANCE.md)
for the transcription, conversions and local validity boundary.

This is a fourth published-data fixed-wing condition exercising the shared
aircraft path. It is not a global A-4D model, a flight-test validation, an
airworthiness result or a certification artifact.
