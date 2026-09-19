# Actuator saturation and anti-windup release

This study commands a deliberately infeasible 0.30 rad roll step, records
requested, position-limited, delayed commands and actuator states, and releases
the reference after four seconds. The criteria separate successful simulation
completion from the declared saturation/recovery budget; the case is a
controller-behaviour study, not a claim that the aircraft can hold the
unreachable reference.

```bash
build/dev/src/cli/galata run examples/heli-saturation-recovery/study.yaml \
  --output-dir build/heli-saturation-recovery
```
