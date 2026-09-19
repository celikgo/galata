# Error-driven integral-windup recovery

This matched study uses nonzero integral gain and an infeasible roll reference
for two seconds, then releases the reference. The actuator limits, delay,
initial condition and reference are identical. Only the PID back-calculation
anti-windup gain differs. Both runs record requested, limited, delayed and
actual actuator signals plus the integral state.

The older heli-saturation-recovery example is retained as the PD
position/rate/lag saturation-release case; it has ki: 0 and is not an
integral-windup acceptance case.

Run:

    build/dev/src/cli/galata run examples/heli-anti-windup-recovery/study.yaml \
      --output-dir build/heli-anti-windup-recovery --overwrite
