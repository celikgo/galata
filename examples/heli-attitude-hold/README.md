# Helicopter sampled LQR attitude hold

```bash
galata run examples/heli-attitude-hold/study.yaml --output-dir build/heli-attitude
```

The study trims and linearises the helicopter, designs a continuous full-state
LQR, and executes that law against the nonlinear plant through the shared
sampled-loop path. The law is full-state feedback over named Euler/auxiliary
coordinates; no estimator is implied.
