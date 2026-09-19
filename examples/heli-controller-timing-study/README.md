# Controller sample-time and delay study

This study runs the same named rate-damping controller at 10 ms/no delay and
20 ms/one-period delay. The solver step is held at 2 ms, so controller timing
sensitivity is separated from integration-step sensitivity.

```bash
build/dev/src/cli/galata run examples/heli-controller-timing-study/study.yaml \
  --output-dir build/heli-controller-timing-study
```
