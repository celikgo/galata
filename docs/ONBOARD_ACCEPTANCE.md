# Onboard/POSIX acceptance evidence

Date: 2026-09-20
Host: macOS arm64
Build: `build/ci-macos`
Scope: deterministic POSIX bench integration, not target-HIL or flight acceptance

The complete staged-runner test was executed with the built CLI and the built
versioned controller ABI fixture:

```text
GALATA_PROJECT_CLI=build/ci-macos/src/cli/galata
GALATA_TEST_CONTROLLER=build/ci-macos/tests/galata-test-controller.dylib
python3 tests/scripts/test_onboard_runner.py -v
```

Result: `2` tests passed in approximately `2.1 s`.

The test creates a manifest-bound model/controller package, deploys and verifies
the atomic POSIX runtime bundle, rejects an unexpected file and a symlink, starts
the staged runner over a real POSIX pseudo-terminal serial link, publishes one
CRC-checked actuator frame, and checks the runner's observed cycle telemetry.
The second case deploys the same bundle with a fixed-port UDP profile, sends
the first sensor frame before any actuator frame, and verifies the returned
CRC-checked actuator datagram from the peer socket.

This closes the repository-controlled POSIX deployment, serial-loopback and
fixed-local-port UDP bench contracts. It does not prove a real flight computer,
CAN installation, hard real-time preemption, watchdog or emergency-stop
behaviour, target signing, hardware-in-the-loop acceptance, airworthiness or
certification.

The repository fixture is also exercised end to end by
`tests/scripts/test_onboard_deployment_fixture.py`. It verifies the checked-in
manifest, deploys the current CLI and both declared artifacts into a fresh
bundle, creates the byte-bound `host_sil` target-evidence package, and verifies
that package against the newly deployed manifest. The current local run used
`build/onboard-deployment-candidate-v78` and
`build/onboard-host-sil-evidence-v78`; the evidence class remains deliberately
ineligible for a target-HIL or flight-target production decision.
