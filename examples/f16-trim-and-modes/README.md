# NASA F-16 nominal derivative workflow

This example loads the [F-16 nominal derivative slice](../../models/f16/f16-nominal.yaml)
derived from NASA's public `simupy-flight` repository and runs the ordinary
Galata fixed-wing path: level trim, longitudinal and lateral finite-difference
linearisation, modal classification and a Markdown report.

The exact source revision, hashes, transformations and rights boundary are in
[`models/f16/PROVENANCE.md`](../../models/f16/PROVENANCE.md).

Run it with:

```sh
build/ci-macos/src/cli/galata run examples/f16-trim-and-modes/study.yaml \
  --output-dir build/f16-study
```

This is a first-order local coefficient slice, not a port of the NASA
nonlinear F-16 simulation. It has no global aerodynamic schedule, propulsion,
actuator, sensor, stores or structural model, and it is not flight validation,
airworthiness evidence or certification.
