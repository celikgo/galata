# Reproducible host deployment fixture

This directory is a small, byte-bound integration fixture for the POSIX
onboard deployment path. The two text `.bin` files stand in for controlled
model and controller build outputs; they are not flight software and are not
aircraft-specific calibration data.

From the repository root:

```bash
CLI=build/ci-macos/src/cli/galata
MANIFEST=examples/onboard-deployment/onboard.manifest
MODEL=examples/onboard-deployment/model-artifact.bin
CONTROLLER=examples/onboard-deployment/controller-artifact.bin

"$CLI" onboard verify "$MANIFEST"
"$CLI" onboard deploy "$MANIFEST" \
  build/onboard-deployment-candidate \
  --runtime "$CLI" \
  --artifact model="$MODEL" \
  --artifact controller="$CONTROLLER"
"$CLI" onboard verify-deployment build/onboard-deployment-candidate
```

The resulting bundle contains the verified runtime, manifest, receipt and
artifacts. It is a host/POSIX deployment candidate only. Target signing,
hard-preemptive timing, watchdog and emergency-stop tests, HIL evidence,
deployment approval and qualification remain external requirements.

The files in [`host-sil-evidence`](host-sil-evidence) can also be assembled into a traceability
package after deployment:

```bash
"$CLI" onboard target create build/onboard-host-sil-evidence \
  build/onboard-deployment-candidate/onboard.manifest \
  --evidence-class host_sil \
  --controller-worst-case-s 0.001 \
  --cycle-worst-case-s 0.004 \
  --watchdog-response-s 0.008 \
  --file timing_report=examples/onboard-deployment/host-sil-evidence/timing_report \
  --file hardware_hil_report=examples/onboard-deployment/host-sil-evidence/hardware_hil_report \
  --file failsafe_report=examples/onboard-deployment/host-sil-evidence/failsafe_report \
  --file signing_record=examples/onboard-deployment/host-sil-evidence/signing_record \
  --file target_configuration=examples/onboard-deployment/host-sil-evidence/target_configuration
"$CLI" onboard target verify \
  build/onboard-deployment-candidate/onboard.manifest \
  build/onboard-host-sil-evidence/target-evidence.manifest
```

This package is intentionally `host_sil`, with physical tests and target
signing marked not applicable. It cannot satisfy the target-HIL or
flight-target production gate.
