# Onboard handoff manifest

This example writes a deterministic, target-neutral handoff for a reviewed
integration boundary:

```bash
galata run examples/onboard-manifest/study.yaml --output-dir build/onboard-manifest
```

The result contains `onboard.manifest` and `onboard.manifest.sha256`. It has no
executable and is explicitly `qualification_state=not_qualified`. Replace the
placeholder model and controller hashes only with identities from a controlled
build. The example also declares a 5 ms controller callback budget and a replay
transport profile with bounded I/O and watchdog deadlines; the shared runtime
detects an overrun after return, while a target watchdog is still needed for
hard preemption. Replace the replay profile with a reviewed target profile
before staging. A real deployment still needs a target adapter, timing/fault
evidence, signing, HIL/SIL evidence and an authorized approval process.
