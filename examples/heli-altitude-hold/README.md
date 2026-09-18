# Helicopter sampled altitude hold

```bash
galata run examples/heli-altitude-hold/study.yaml --output-dir build/heli-altitude
```

Runs a collective altitude PID with explicit rate stabilisation, one controller
period of delay, actuator saturation and zero-order hold. The controller CSV
keeps the integrator and filtered-derivative states beside the plant history.
