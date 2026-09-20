# Production-readiness position

This document records what the repository can honestly claim after the shared
vehicle and integration work.

The itemized project-level decision is in
[`FINALIZATION_AUDIT.md`](FINALIZATION_AUDIT.md).

| Area | Repository state | Remaining evidence |
|---|---|---|
| Deterministic numerical engine | Implemented and covered by the local suite | Independent release build and review |
| Fixed-wing, multirotor and helicopter model boundary | Implemented through VehicleModel adapters; seven published/source-derived fixed-wing nominal conditions (NT-33A, Navion, A-7A condition 1, A-4D condition 1, F-4C power approach, NASA GTM T2 and the NASA simupy-flight F-16 slice), the Souxmar helicopter design study, and a native UH-60A Level-1 study path are exercised end to end | Complete aircraft-specific parameter sets, broader envelope data and real-aircraft validation |
| Flight-test data comparison | Implemented as identify.validate.vehicle, with numerical and byte-verified campaign-evidence gates, mandatory provenance classification (`measured_flight`, `public_deidentified` or `synthetic_contract`), a package-backed execution receipt, a shared CLI/desktop verifier and a reproducible NASA DASHlink public-record import reference; public/synthetic packages remain unresolved | A matching aircraft/configuration model, calibrated `measured_flight` record, approved test plan and independent acceptance review |
| Hardware interface | Implemented as a transport-neutral contract, versioned CRC-protected frame codec, guarded replay transport, POSIX serial/UDP and Linux SocketCAN CAN-FD adapters, explicit arming gate, target identity binding, reviewed transport-profile contract, one-cycle fail-closed supervisor, observed SIL/bench timing telemetry and an end-to-end serial runner test; target evidence is versioned and provenance-classified as `host_sil`, `target_hil` or `flight_target` | Real target-specific bus/FC binding, hard timing/preemption, fault and emergency-stop evidence |
| Onboard handoff | Implemented as a deterministic manifest handoff bound to a hardware asset, flight-computer type, firmware build and emergency-stop identity, plus an atomic POSIX runtime bundle containing the executable runner, verified model/controller artifacts and deployment receipt; the runner loads a versioned C controller ABI and executes serial/UDP/CAN-FD cycles with an explicit software watchdog gate; the built POSIX serial-loopback acceptance is recorded in [`ONBOARD_ACCEPTANCE.md`](ONBOARD_ACCEPTANCE.md), and the reproducible host-deployment fixture plus byte-bound `host_sil` evidence are in [`examples/onboard-deployment`](../examples/onboard-deployment) | Target signing, target HIL evidence, measured timing/preemption proof and deployment approval from the responsible programme |
| Qualified or certified use | Not claimed; a bounded qualification-evidence package creator and verifier now bind requirements, release, flight, HIL, safety, review, authority-decision and maintenance records plus the complete staged deployment runtime and its executable hash, while retaining `not_qualified` | Real controlled evidence, independent review, authorized authority decision and application-specific qualification programme |
| Desktop product | macOS preview now runs saved projects and arbitrary Galata study YAMLs, including validation and flight-test workflows; it also keeps a bounded, revision-aware local recovery snapshot for unsaved drafts; manual native import/save/run/recovery acceptance is recorded in [`DESKTOP_ACCEPTANCE.md`](DESKTOP_ACCEPTANCE.md); CI and the local packaging path now build and mechanically verify both ZIP and DMG candidates carrying the complete shipped examples and model inputs | Developer ID/notarized distribution, accessibility, clean-machine and release acceptance |
| Broader aircraft support | Fixed-wing, multirotor and helicopter families share the generic path; seven independently sourced/derived fixed-wing reference conditions are exercised, including the NASA GTM T2 and simupy-flight F-16 slices, while UH-60A has a native assumption-labeled Level-1 path in addition to the Souxmar helicopter design study. The production audit now accepts this row only from a separate, hash-bound independent-validation evidence package containing at least two aircraft configurations. | Controlled UH-60 aerodynamic/inertial/actuator data, additional aircraft configurations, broader envelope data and real-aircraft validation |

“Implemented” in this table means the software boundary exists and is tested; it
does not mean a real aircraft, target computer or certification authority has
accepted it. The last four rows cannot be completed truthfully by source changes
alone because their evidence is external to this repository.

## Production preflight audit

The repository now provides one machine-readable preflight that runs the
independent flight-test, deployment, target-evidence and qualification
verifiers together and checks the desktop distribution metadata:

```text
python3 scripts/check-production-readiness.py \
  --cli build/ci-macos/src/cli/galata \
  --flighttest /controlled/campaign/campaign.manifest \
  --flight-validation-receipt /controlled/campaign/validation-receipt.json \
  --deployment /controlled/deployment/deployment.manifest \
  --deployment-dir /controlled/deployment/bundle \
  --target-evidence /controlled/target/target-evidence.manifest \
  --dossier /controlled/qualification/qualification.manifest \
  --desktop-package /controlled/release/Galata.package.json \
  --dmg-metadata /controlled/release/Galata.dmg.json \
  --aircraft-evidence /controlled/aircraft/aircraft-validation.manifest
```

The report is `galata.production-readiness-audit.v1`. Exit code `2` means the
bytes were not contradicted but one or more production prerequisites are
missing; exit code `1` means a supplied verifier failed. A future report can
reach `ready_for_external_review`, but the report always retains
`qualification_state=not_qualified` and false airworthiness/certification
claims. On macOS it also records the number of locally visible Developer ID
Application identities, so an ad-hoc candidate cannot be mistaken for a
publishable desktop release. This command is a release preflight, not an
authority or airworthiness decision.

The aircraft evidence manifest uses `galata.aircraft-validation-evidence.v1`.
Each record binds an aircraft and configuration to SHA-256-checked model,
calibration, operating-envelope, validation-report and independent-review
files. The verifier requires two distinct aircraft and completed independent
review for every record. Repository examples, public references and synthetic
fixtures do not satisfy this gate by themselves; the verifier checks supplied
evidence bytes but cannot perform the independent review or accept the aircraft.

For a local end-to-end contract test of the repository-controlled boundaries,
run [`examples/production-evidence-contract`](../examples/production-evidence-contract/README.md)
with `scripts/run-production-contract.py`. It creates a synthetic campaign,
package-backed validation receipt, onboard deployment, host-SIL target evidence
and qualification-chain dossier. Its expected result is `not_ready_by_design`;
it proves composition and traceability, not flight, target-HIL or authority
acceptance.

The broader-aircraft boundary has the same explicit contract in
[`examples/aircraft-validation-contract`](../examples/aircraft-validation-contract/README.md).
It binds the F-16 and GTM model slices into two distinct records and verifies
their hashes, while leaving independent review incomplete. Its expected result
is also `not_ready_by_design`; model count alone cannot substitute for
independent aircraft validation.
