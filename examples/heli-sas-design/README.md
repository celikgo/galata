# Helicopter sampled SAS

```bash
galata run examples/heli-sas-design/study.yaml --output-dir build/heli-sas
```

Runs three rate-damping PID loops on the nonlinear helicopter at a 100 Hz
controller rate. `sas.csv.controller.csv` records named measurements, requests,
saturation and applied commands. This is an exploratory Level-1 design study,
not a stability-margin or aircraft-validity claim.
