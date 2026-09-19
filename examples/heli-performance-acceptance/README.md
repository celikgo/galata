# Measured helicopter control-performance acceptance

This study compares a disturbed open-loop helicopter response with a matched
sampled PID response. It declares the initial body-frame attitude rotation
vector, angular-rate perturbations, simulation horizon, and response budgets in
the study file. The response stage refuses an unmatched initial condition or
wind history and records peak error, final error, settling, actuator tracking,
saturation, rotor-speed excursion, and validity-envelope departures. Attitude
recovery and SAS rate recovery are reported as separate criteria. The LQR
synthesis stage remains as a diagnostic of the sampled design path; it is not
used as evidence of successful nonlinear control.

```bash
build/dev/src/cli/galata run examples/heli-performance-acceptance/study.yaml \
  --output-dir build/heli-performance-acceptance
```

This is a Souxmar Level-1 design study. It is controller-performance evidence
for this model, not validation of a measured aircraft.
