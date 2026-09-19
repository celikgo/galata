# Seeded mass/CG sensitivity ensemble

Four independent forward-flight members vary mass and centre of gravity within
declared design ranges and use member-specific seeds. The aggregate and one
manifest per member preserve declaration order; `parallel: true` changes only
execution scheduling. The study emits both parallel and serial aggregates for
direct determinism comparison. These distributions are design assumptions, not
measured aircraft uncertainty.

```bash
build/dev/src/cli/galata run examples/heli-mass-cg-ensemble/study.yaml \
  --output-dir build/heli-mass-cg-ensemble
```
