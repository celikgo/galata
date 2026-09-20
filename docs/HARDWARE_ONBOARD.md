# Hardware and onboard boundary

Galata does not open a serial, CAN, UDP or vendor flight-controller link by
default. The production boundary is explicit:

- galata::hardware::InterfaceSpec defines SI units, frames, channel order and
  sample period;
- galata::hardware::Frame carries a sequence number, timestamp and finite
  values;
- galata::hardware::FrameCodec defines a versioned little-endian packet with
  explicit channel count and CRC-32 corruption detection;
- galata::hardware::Transport is the injection point for a reviewed
  target-specific adapter;
- galata::hardware::UdpTransport is a concrete POSIX IPv4/IPv6 adapter with
  independently bounded receive and non-blocking transmit polling, strict
  frame ordering/timing checks and fault-closed handling for malformed
  datagrams or socket errors. UDP itself is not
  authenticated and provides no delivery or failsafe guarantee;
- galata::hardware::SerialTransport is a concrete POSIX raw-8N1 adapter with
  independently bounded exact-packet reads/writes and the same fault-closed
  checks;
- galata::hardware::CanFdTransport is a Linux SocketCAN CAN-FD adapter with
  explicit extended CAN IDs, bounded non-blocking I/O and the same checks. It
  refuses contracts whose FrameCodec packet cannot fit in one 64-byte CAN-FD
  payload; classic CAN and unreviewed fragmentation are not silently accepted;
- galata::hardware::ReplayTransport provides deterministic record/replay
  testing and stores actuator output instead of sending it; and
- galata::hardware::TransportProfile records the reviewed transport kind,
  endpoint identity, bounded receive/transmit timeouts, watchdog deadline and
  independent emergency-stop requirement. It is validated against the
  interface sample period before it can enter an onboard manifest;
- galata::hardware::ArmingInterlock keeps link readiness separate from
  operator arming. GuardedTransport puts that interlock directly on the
  actuator-output path and disarms on disconnect, any observed non-ready
  state, or an I/O/validation exception.
- galata::onboard::Runtime is the shared one-cycle supervisor for SIL, HIL and
  a reviewed target adapter. It requires an explicit arm, consumes one sensor
  frame, produces one same-sequence/same-timestamp actuator frame and sends it
  only through GuardedTransport. A timeout, callback exception, invalid output,
  execution-budget overrun or identity mismatch disarms and disconnects before
  reporting the failure. The execution budget is detected after a callback
  returns; it is not preemption, so a target watchdog/RTOS must still enforce a
  hard deadline for a non-returning controller.
- galata::onboard::ControllerPlugin loads a controlled native controller
  artifact through `controller_abi.h`. The ABI is versioned, passes the
  verified model-artifact path into controller creation, uses caller-owned POD
  buffers and never permits a controller exception to cross the boundary; the
  runtime still re-validates every output frame before it reaches the guarded
  transport.

The packet format is intentionally small and reviewable: four-byte `GLHW`
magic, version byte, zero reserved-flags byte, little-endian `uint16` channel
count, little-endian `uint64` sequence, IEEE-754 binary64 timestamp, one
binary64 value per channel, and a little-endian CRC-32 over every preceding
byte. The codec rejects wrong sizes, versions, flags, channel counts,
checksums, timestamps and non-finite values before a packet reaches a model or
actuator adapter.

The CAN-FD adapter carries one complete codec packet per CAN-FD frame. Its
interface name, receive/transmit IDs and I/O deadlines are configuration, not
proof that a bus is wired, terminated, clocked or accepted by a flight
controller.

galata::onboard::build_manifest_package creates a deterministic, hashed
manifest-only handoff. It records the target platform, concrete hardware asset,
flight-computer type, firmware build and independent emergency-stop-chain
identity, as well as model/controller identities, failsafe action, interface
contract and transport profile. These target fields bind the handoff to one
controlled integration; they are identity declarations, not discovery or proof
that the named hardware is present. The profile is a declaration of the
reviewed link boundary, not a discovery of a live bus or flight controller. It
contains no executable and is always marked `not_qualified`.

For a reviewed handoff directory, `galata::onboard::stage_manifest_package`
also verifies each explicitly supplied regular file against its declared
SHA-256 and verifies that the artifact role/hash map is exactly the one encoded
in the manifest, rejects symlinks and existing destinations, writes the
manifest, checksum and artifacts into a private staging directory, and
re-checks the copied bytes before publishing that directory with a no-overwrite
rename. This is an integrity and review step;
it is not an installer and does not make the package executable or qualified.

Studies can write the same handoff through onboard.manifest. The stage takes
the interface, transport profile, controller execution budget and artifact
SHA-256 identities as explicit YAML data and writes both the manifest and a
checksum file. This makes deployment reviewable from a pipeline run while
keeping executable generation and target installation out of the numerical
worker. The generated manifest records runtime.max_controller_time_s and the
hardware.* profile fields; the runtime budget and watchdog are required
positive finite deadlines.

The CLI exposes the same boundary for a handoff received outside a pipeline:

```text
galata onboard verify <onboard.manifest>
galata onboard stage <onboard.manifest> <new-directory> \
  --artifact model=/controlled/path/model-artifact \
  --artifact controller=/controlled/path/controller-artifact
galata onboard deploy <onboard.manifest> <new-directory> \
  --runtime /controlled/path/galata \
  --artifact model=/controlled/path/model-artifact \
  --artifact controller=/controlled/path/controller-artifact
galata onboard verify-deployment <new-directory>
galata onboard target create <new-directory> <onboard.manifest> \
  --evidence-class target_hil \
  --controller-worst-case-s <s> --cycle-worst-case-s <s> \
  --watchdog-response-s <s> --emergency-stop-passed true \
  --loss-of-link-passed true --hil-passed true --signing-verified true \
  --file timing_report=/controlled/path/timing-report \
  --file hardware_hil_report=/controlled/path/hil-report \
  --file failsafe_report=/controlled/path/failsafe-report \
  --file signing_record=/controlled/path/signing-record \
  --file target_configuration=/controlled/path/target-configuration
galata onboard target verify <onboard.manifest> <target-evidence.manifest>
galata onboard run <onboard.manifest> \
  --model <new-directory>/artifacts/model \
  --controller <new-directory>/artifacts/controller \
  --operator <operator-id> --confirm ARM --cycles <count>
galata onboard self-test
```

`verify` checks the bounded manifest file and returns a machine-readable
verified result. `stage` requires one explicit source file for every manifest
role, verifies each source digest, refuses symlinks and existing destinations,
and publishes the private staged directory atomically. The result remains a
manifest-only, `not_qualified` handoff; it is not a flight-computer installer.
`deploy` performs the same checks and additionally stages an executable POSIX
runner at `bin/galata`, writes `deployment.receipt`, and records the runtime
digest. `verify-deployment` re-hashes that complete bundle. `run` requires the
model and controller paths to match the manifest identities, requires an
explicit `ARM` confirmation, and can open the concrete `serial`, `udp` or
`can_fd` adapter. Endpoint syntax is `device|baud` for serial,
`host:port[|local_port]` (or `[IPv6]:port[|local_port]`) for UDP, and
`interface,receive_id,transmit_id` for CAN-FD. `--linger-ms` is available for
bench consumers that need the link to remain open briefly after a finite run.
These are integrity and execution boundaries, not evidence that a target bus,
clock, watchdog or controller is safe for flight.
For a deployed UDP target, `local_port` is mandatory; the manifest verifier
rejects an ephemeral source port. The lower-level transport still permits zero
for a controlled one-way or bench consumer, but a fixed local port is required
for a flight computer to address the runner with its first sensor frame.
`self-test` runs three deterministic replay cycles through the same guarded
one-cycle supervisor and reports a machine-readable pass. It is a host/SIL
contract test only; it does not claim target timing, a deployed executable,
flight-controller compatibility or qualification.

`run` and `self-test` also report `observed_controller_worst_case_s`,
`observed_cycle_worst_case_s` and `observed_cycles`. These are measurements made
by the supervising process and are useful for a SIL or POSIX bench report. They
are deliberately separate from `target_hil` or `flight_target` evidence: the
host scheduler, transport and callback cannot be hard-preempted by this runtime,
so these fields never establish target timing, watchdog response or airworthiness.

`target create` is the controlled handoff for evidence produced by an external
target-integration programme. It requires an explicit `evidence_class`:
`host_sil`, `target_hil` or `flight_target`. The first is a software-only
contract: omit the four physical-test and target-signing options, and the
manifest records those states as `not_applicable`. The latter two identify
evidence from the intended hardware context and require explicit `true`
confirmations for emergency stop, loss-of-link, HIL and signing results. None
of the classes is an approval decision. All classes copy exactly the five
required evidence roles into a new directory, record their SHA-256 identities,
and publish the package atomically. Galata performs the inventory, hash and
budget checks; it does not perform those physical tests or turn their results
into certification.

`target verify` is the target-integration evidence gate. The target-evidence
manifest must bind the exact onboard manifest hash, target identity, interface
and transport profile, and must include byte-addressed records for timing,
failsafe behaviour, signing/qualification state and target configuration. It
also declares measured worst-case controller time, complete cycle time and
watchdog response; the verifier rejects values outside the deployment budgets,
class-inconsistent physical-test or signing states, tampered files, symlink
components and extra evidence files. The manifest is versioned and carries the
evidence class so a host/SIL record cannot be silently described as target HIL.
A successful result means that the supplied evidence is traceable and
contract-compatible; it remains `qualification_state=not_qualified` and cannot
establish airworthiness or certification by itself.

Deployment verification also rejects symlink path components and any file or
directory outside the exact runtime inventory (`bin/galata`, the three receipts,
and the manifest-declared artifact files). An augmented or redirected bundle
therefore cannot be mistaken for the bytes that were originally reviewed.

The POSIX runner bundle is an integration release, not a flight release. A real
target still needs a target-specific bus/flight-controller profile, bounded I/O,
clock and jitter measurements, sensor/actuator calibration, loss-of-link and
emergency-stop behaviour, hardware-in-the-loop evidence, code-generation or
hand-coded implementation control, signing, and an authorized qualification
process. The library will not manufacture those facts from a manifest, a
packet checksum or a replay.
