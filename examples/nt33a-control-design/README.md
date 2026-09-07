# NT-33A local control-design study

This study loads the published reference airframe, trims and linearises it,
selects the elevator channel, computes an LQR law, examines its closed-loop
poles and plant-input margins, and integrates both the linear closed loop and
the nonlinear aircraft with bounded actuator dynamics. The CSV files contain
the actual computed histories; the Markdown report retains the controller
weights, gains, solver evidence and actuator-limit counts.

Run from the repository root:

```sh
build/dev/src/cli/galata run examples/nt33a-control-design/study.yaml --output-dir build/control-study
```

Existing output files require `--overwrite`. Each invocation also writes a
separate run manifest binding the consumed inputs and outputs to their hashes.

The initial pitch perturbation, control weights and actuator specifications
are illustrative study inputs. They are not published NT-33A controller gains
or verified hardware limits. The model remains local to its published power
approach condition. The nonlinear simulation stops if that model envelope is
exceeded. The linear response omits actuators; differences between the two
histories therefore include actuator lag and nonlinear effects.

To check step sensitivity, halve `step_s` and double `steps`, keeping duration
fixed. Compare corresponding physical states, not their raw mixed-unit vector
norm. To design for a different aircraft, supply validated derivative data,
correct hardware constraints and weights chosen for that application.
