# Helicopter deterministic noisy feedback

```bash
galata run examples/heli-noisy-feedback/study.yaml --output-dir build/heli-noisy
```

The named rate sensor uses the versioned `mt19937_64_box_muller_v1` algorithm,
independent channel streams, bias, white noise, quantisation, one-period
sample/transport timing and declared dropouts. The report records the seed,
algorithm, sample/latency periods and stream IDs, then compares the disturbed
open and noisy delayed closed-loop rate responses over 10 seconds. This is a
reproducible impairment primitive, not an estimator or a real-sensor
validation.
