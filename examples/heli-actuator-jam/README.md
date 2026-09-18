# Helicopter scheduled actuator jam

```bash
galata run examples/heli-actuator-jam/study.yaml --output-dir build/heli-jam
```

The lateral cyclic actuator jams at its current position during sampled PID
feedback. The event sidecar records the fixed-step application time, while the
controller sidecar preserves requested, saturated and delayed-applied commands
for diagnosing the remaining authority.
