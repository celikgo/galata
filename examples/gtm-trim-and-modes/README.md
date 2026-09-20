# NASA GTM T2 nominal derivative workflow

This example loads the [GTM T2 nominal derivative slice](../../models/gtm/gtm-t2-nominal.yaml)
and runs the ordinary Galata fixed-wing path: level trim, longitudinal and
lateral finite-difference linearisation, modal classification and a Markdown
report.

The model is derived from the public NASA GTM_DesignSim polynomial database;
the exact source revision, hashes, transformations and rights boundary are in
[`models/gtm/PROVENANCE.md`](../../models/gtm/PROVENANCE.md).

Run it with:

```sh
build/ci-macos/src/cli/galata run examples/gtm-trim-and-modes/study.yaml \
  --output-dir build/gtm-t2-study
```

This is a first-order coefficient slice, not a port of NASA's nonlinear
large-envelope simulation. It has no global aerodynamic schedule, propulsion,
actuator, sensor or damage model, and it is not flight validation,
airworthiness evidence or certification.
