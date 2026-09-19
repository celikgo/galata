# Altitude reference tracking

This study changes the altitude reference from 100 m to 102 m and back after a
hover trim. It records the scheduled reference stream, actuator command versus
actual position, one controller-period delay, and the matched open-loop
comparison. The model is a Souxmar Level-1 design study, not measured aircraft
data.

```bash
build/dev/src/cli/galata run examples/heli-reference-tracking/study.yaml \
  --output-dir build/heli-reference-tracking
```
