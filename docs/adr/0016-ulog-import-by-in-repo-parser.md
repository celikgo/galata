# ADR-0016: PX4 ULog is imported by an in-repo parser, not a dependency

- **Status:** accepted
- **Date:** 2026-09-10
- **Deciders:** project maintainer

## Context

RFC-0002's WP3 asks for `data.import.ulog`: read a PX4 flight log, resample to a
declared rate, tag every channel with its unit and frame, and refuse what it
cannot identify. The Souxmar quadrotor programme flies a Pixhawk 6C Mini running
PX4, so its flight records are ULog and there is no second format to fall back
on.

Reading ULog means decoding a documented binary container: a header, then a
message stream of format definitions (`F`), subscription records (`A`), packed
data records (`D`), info and parameter records (`I`, `M`, `P`), and dropout
markers. The wire format is versioned and stable.

Three ways to obtain that decoding, judged against what this repository already
requires of everything inside it — the vcpkg manifest, the two supported
platform families of ADR-0015, and the determinism policy of ADR-0004.

## Decision

**Write the parser in this repository**, covering a declared subset of the
format and refusing everything outside it by name.

The parser is ours, built with the rest of the engine, with no addition to
`vcpkg.json` and no vendored source in `third_party/`. Its PROPOSED location is
`src/data/ulog/`; that directory does not exist yet and this record decides the
approach rather than announcing the code. `scripts/doc-references-allow.txt`
carries the forward reference until the capability lands.

## Alternatives considered, and why they were rejected

**A vcpkg dependency on PX4/ulog_cpp.** Rejected on build grounds, not on
licence or quality. `ulog_cpp` is BSD-3-Clause, which is compatible with this
repository's Apache-2.0 and would have raised no licensing question. It is **not
in the vcpkg registry** at our pinned baseline
(`56bb2411609227288b70117ead2c47585ba07713`), so taking it means an overlay port
or a second registry, pinned to a git revision, maintained by us, and made to
work on Linux/GCC, Linux/Clang and macOS/AppleClang. `vcpkg.json` currently
names three packages, all first-tier and all in the registry. The machinery to
add a fourth that is not costs more than the code it would import.

**Vendoring ulog_cpp into `third_party/`.** Rejected on precedent and on
boundary. `third_party/` has never held source — only licence texts — and
vendoring would put code we did not write inside the determinism boundary that
ADR-0004 makes claims about. We would then re-validate its output anyway,
because the refusals WP3 requires are not behaviours a general-purpose reader
provides: an unknown channel, a unit-less channel or an unsupported layout must
be refused rather than returned. Wrapping a dependency to re-impose our own
contract on every value is most of the work of parsing, with a supply chain
attached.

**Writing the parser.** The subset is small and specified, the refusals are
ours, and determinism is under our control: ordered containers on every path
that reaches an output, no locale-dependent conversion, no tolerance-based
early exit. It also keeps ADR-0007's posture — what ships in-tree is ours or is
public-domain-clean — without a new category of exception.

## What this costs, stated rather than glossed

**We own correctness, and a hand-written binary decoder can mis-decode
silently.** That is the real risk of this decision and it is not small: a
misread scale factor or a wrong field offset produces plausible numbers, and
plausible numbers are what identification then fits a model to.

Three things hold it, and the capability is not complete without them:

1. **Fixtures written by an independent implementation.** `pyulog` (BSD-3-Clause,
   PX4's own Python tooling) generates the test logs. It is a **test-time**
   tool, invoked to produce fixtures; it is not a dependency of the library, is
   not shipped, and nothing built from it enters a release. A fixture we both
   wrote and read would prove only that we are self-consistent.
2. **Refusal over best effort.** An unrecognised format version, an unsupported
   message type, a topic whose fields do not match the declared layout, or a
   channel with no unit is refused by name. "Unsupported" is a status, not a
   silent zero.
3. **Sign and direction checks per axis**, and an explicit quaternion order and
   direction check, because PX4's NED/FRD convention matching ADR-0002 is the
   kind of agreement that is easy to assert and easy to get backwards.

## Consequences

- `vcpkg.json` is unchanged; `THIRD_PARTY_LICENSES.md` gains no entry.
- The supported ULog subset is declared in the capability's own documentation
  and is the contract; growing it is a normal change with its own tests.
- PX4 actuator channels are **not** treated as measured rotor speeds. They are
  commands in their own units, and turning them into rad/s requires an explicit
  declared calibration — see WP3 and the capability header.
- If PX4 changes the wire format incompatibly, we carry that. The refusal path
  means we find out by being refused rather than by fitting a model to
  mis-decoded numbers.
