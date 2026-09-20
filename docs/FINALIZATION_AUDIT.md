# Finalization audit

This is the project-by-project status at the current 0.3.0 candidate. The
word **finalized** is split deliberately:

- **Software-final** means the repository-controlled implementation and its
  declared tests are complete for the stated scope.
- **Candidate** means it is usable and packaged, but release or acceptance
  gates remain.
- **Not finalized** means the engineering evidence needed to make the broader
  claim is absent or the scope is explicitly experimental.

Passing tests do not upgrade a model into flight evidence or a candidate into
a certified product.

| Project / surface | Status | What is actually complete | What prevents finalization |
|---|---|---|---|
| Numerical C++ engine, CLI and capability registry | Software-final for 0.3.0 scope | Deterministic core, strict inputs/outputs, provenance, reports and 934-test local regression; installable CMake consumer also passes | Independent release review and broader advertised-domain acceptance |
| Project files and worker (`.galata`) | Candidate / experimental M2 | Create, import, save, revisions, restore, isolated run, review export/verify, bounded desktop draft recovery and desktop editing are implemented and tested | Project schema is not frozen; hosted Linux/macOS acceptance, accessibility and full v1 workflow acceptance remain open |
| NT-33A fixed-wing | Finalized only as one narrow numerical reference condition | Published-data transcription, trim/linearisation/mode/control studies and validation comparisons are covered | Not a global aircraft model: no stall, Mach schedule, engine, configuration changes, envelope or flight-test evidence |
| Navion nominal fixed-wing | Finalized only as a narrow published-data transcription | Nominal level-flight derivative model and trim/mode workflow are tested | No global Navion model, propulsion/configuration/structure/instrumentation model or flight validation |
| A-7A condition 1 | Finalized only as one narrow numerical reference condition | Published-data transcription, source-factor comparison and study path are tested | No global A-7A model, flexible-aircraft envelope, configuration schedule or flight validation |
| A-4D condition 1 | Finalized only as one narrow numerical reference condition | NASA CR-96008 dimensional derivative transcription, conversion checks, trim and local linearisation path are tested | No global A-4D model, flexible-aircraft envelope, configuration schedule or flight validation |
| F-4C power approach | Finalized only as one narrow numerical reference condition | NASA CR-2144 geometry/mass/derivative transcription, stability-to-body axis conversion, trim and local linearisation path are tested | No global F-4C model, propulsion/control-system schedule, configuration envelope or flight validation |
| NASA GTM T2 nominal derivative slice | Finalized only as one narrow numerical reference condition | Public NASA GTM_DesignSim source-bound geometry, mass/inertia and coefficient derivation; load, trim and finite-dynamics checks are tested | Not the nonlinear NASA database: no global schedule, propulsion, actuator/sensor/damage model or flight validation |
| NASA F-16 nominal derivative slice | Finalized only as one narrow numerical reference condition | Public NASA simupy-flight source-bound geometry, mass/inertia and local coefficient derivation; load, trim and finite-dynamics checks are tested | Not the nonlinear F-16 model: no global schedule, propulsion, actuator/sensor/stores/structure model or flight validation |
| Shared fixed-wing/multirotor/helicopter adapters | Software-final as an interface contract | All three families use common trim, linearise, execute and report artifacts | Family-specific model quality and real-aircraft evidence are outside the adapter contract |
| Souxmar helicopter | Design-study complete, not aircraft-final | End-to-end Level-1 helicopter simulation, controller, failure and response studies | User design inputs omit measured mass properties, derivatives, actuator/governor data and flight evidence; assumptions remain explicit |
| Native UH-60A Level-1 path | Not finalized | Executable hover-trim/study path with source-derived subset and explicit provenance/assumption labels | Controlled aerodynamic, inertial, actuator and configuration data plus real-aircraft validation are missing |
| Souxmar quadrotor | Cross-implementation software validation only | Nonlinear plant, sampled LQR and declared comparison/acceptance cases are reproducible | No published or measured aircraft anchor; external trajectory fixture is not shipped and the model is not the built airframe |
| Quadrotor identification workflow | Synthetic identification workflow complete | Training/held-out separation, parameter recovery and refusal cases are tested | Synthetic truth data is not measured flight data; no aircraft parameter campaign or validation acceptance |
| Fixed-wing validation contract | Numerical contract complete, public-record importable, not flight-final | Disjoint windows, prediction metrics, provenance, synthetic contract pass, a reproducible converter for NASA DASHlink recorder windows and a package-backed execution receipt that binds every campaign byte into a run | The public record is de-identified and lacks a matching aircraft/configuration identity, calibration package, approved test plan and independent review |
| Flight-test evidence package | Tooling complete, evidence not final | CLI and desktop create/verify a versioned five-role package atomically with copied-byte hashes, explicit `evidence_class` provenance and safety-review confirmation; the validation receipt executes and checks the bound study; only verified `measured_flight` packages can reach the evidence-gate pass | The creator cannot make records genuine, representative or accepted; real campaign artifacts are absent |
| Hardware transports and onboard runtime | Host/POSIX integration candidate | Serial, fixed-port UDP, SocketCAN CAN-FD, framing/CRC, explicit target identity binding, arming gate, fail-closed runtime with software watchdog, controller ABI v2, atomic deployment, recorded serial-loopback and fixed-port UDP bench acceptance, observed SIL/bench timing telemetry and a versioned target-evidence gate with class-correct `host_sil`/`target_hil`/`flight_target` provenance | No real target flight computer/bus binding, hard timing/preemption, watchdog/E-stop/HIL evidence, target signing or deployment approval has been supplied |
| Qualification dossier | Traceability tooling complete, not qualified | CLI and desktop create/verify a nine-role dossier atomically; the qualification-chain gate verifies the dossier, a gate-passed campaign validation receipt, complete flight campaign, deployment-bound target evidence, the complete staged runtime package and manifest links, reports readiness separately from traceability, and retains `not_qualified` | Controlled requirements, measured-flight/target-HIL records, independent review and authorized authority decision are absent |
| macOS desktop | Local candidate, not release-final | Complete shipped examples/models, arbitrary study runner, project editor, bounded revision-aware draft recovery, validation/onboard/flight-test/qualification actions; manual native import/save/run/recovery acceptance is recorded in [`DESKTOP_ACCEPTANCE.md`](DESKTOP_ACCEPTANCE.md); CI and the local packaging path mechanically verify ZIP and DMG candidates | Ad-hoc signing, no notarization/Developer ID, clean-machine, accessibility and release acceptance gates remain |

The current local candidate artifacts are the ZIP and DMG recorded in
[`DESKTOP_ACCEPTANCE.md`](DESKTOP_ACCEPTANCE.md). Their package metadata keeps
the source tree dirty and the distribution channel `local-candidate`; the
artifacts are not being presented as a stable or notarized release.

## Bottom line

Galata is not finalized as a production flight-engineering product yet. The
repository-controlled software is now a strong, tested candidate with clear
boundaries. The remaining blockers are primarily external evidence and release
authority: real calibrated flight data, target hardware/HIL measurements,
signed/notarized distribution, independent review and an authorized
qualification decision. Source changes alone cannot honestly manufacture those
artifacts.

The broader-aircraft readiness gate is now backed by the strict
[`check-aircraft-validation.py`](../scripts/check-aircraft-validation.py)
verifier. Repository model examples remain explicitly non-qualifying until a
supplied package contains independently reviewed evidence for at least two
aircraft.

The detailed implementation boundaries are maintained in
[`PRODUCTION_READINESS.md`](PRODUCTION_READINESS.md),
[`FLIGHT_TEST_VALIDATION.md`](FLIGHT_TEST_VALIDATION.md),
[`HARDWARE_ONBOARD.md`](HARDWARE_ONBOARD.md) and
[`QUALIFICATION_EVIDENCE.md`](QUALIFICATION_EVIDENCE.md).
