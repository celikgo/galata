# Flight-condition and altitude sweep

This executable study declares an altitude sweep and a forward-flight speed
sweep in declaration order. It records each trim independently; no continuation
is used, so the result does not depend on worker completion order.

```bash
build/dev/src/cli/galata run examples/heli-parameter-sweeps/study.yaml \
  --output-dir build/heli-parameter-sweeps
```
