# Helicopter wind, gust and turbulence evidence

This study uses one forward-flight trim to compare matched open-loop and
sampled-rate-damped responses under a smooth deterministic gust, a wind step,
and `gaussian_ou_v1` seeded turbulence. Wind-step boundaries are on the
integration lattice and rebase air-relative velocity so ground velocity is
continuous. A second open-loop gust member halves the RK4 step for refinement
evidence. The turbulence model is a piecewise-constant Gaussian
Ornstein–Uhlenbeck process initialized at its mean; this implementation is
supported only at positive airspeed and is not a Dryden or measured atmospheric
model.

```bash
build/dev/src/cli/galata run examples/heli-gust-response/study.yaml \
  --output-dir build/heli-gust-response
```
