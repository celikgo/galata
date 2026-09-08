# Testing

Five tiers. A test belongs to exactly one, and the tier determines what a
failure means.

| Tier | Directory | What it proves | Exists |
|---|---|---|---|
| Unit | `tests/unit/`, `tests/desktop/` | One algorithm against a closed-form answer | yes |
| Property | `tests/property/` | Invariants over many generated inputs, seeded | yes |
| Integration | `tests/integration/`, `tests/scripts/` | One pipeline stage against a frozen capability contract | yes |
| Validation | `tests/validation/` | Output against published reference data | yes |
| Determinism | `tests/determinism/` | Bit-identical repeat runs; cross-platform agreement | yes |
| Evals | `evals/` | Agent behaviour against checkable outcomes | no |

The "Exists" column is the honest state of this repository right now. A
directory appears in `tests/CMakeLists.txt` in the same commit that adds its
first test, never in advance.

The tier is the label, not the directory. Two directories join a tier they do
not share a name with, and one of them is conditional on the platform:

- `tests/desktop/` carries the `unit` label. It holds headless presentation
  geometry for the optional native macOS preview — routing, connection and
  diagram editing — built only when `APPLE` and the `galata_desktop` target are
  both present. It links AppKit but opens no window and starts no application.
- `tests/scripts/` supplies five `integration` tests registered by name rather
  than by discovery: `ProjectWorkflow`, `ProjectLinearImport`, `ProjectRecovery`,
  `ProjectRoutes` and `DesktopPackageContract`. The first four drive the
  experimental project CLI as a child process over its public JSON contract,
  which is why they are integration tests written against
  [`docs/PROJECT_FILES.md`](PROJECT_FILES.md) rather than unit tests of
  `src/cli/project.cpp`. They are registered when the `galata_cli` target is
  built on a UNIX host, which both supported platforms are, so in practice the
  target is the condition. They carry a longer timeout because an instrumented
  worker hashes its whole executable and runtime inventory on every run.

The rest of `tests/scripts/` — the assurance, provenance, release and worktree
gates — is not registered with ctest at all. CI runs it separately as
`python3 -m unittest discover -s tests/scripts`, because those tests check the
Python gates themselves rather than anything the C++ build produces. Running
only `ctest` therefore leaves that set unexercised.

## Rules that make the tiers mean something

**Reference values come from published sources, never from the
implementation.** A test that captures current output as its expected value
proves only that the code still does what it did, which is worth having but is
not validation. Such a test is labelled a regression-lock in its own name and
carries a comment naming the validated case it is anchored to. Every other test
cites a document.

**Integration tests are written without reading the implementation.** They are
written against the capability's documented contract. A test written by reading
the code tests the code's opinion of itself.

**Property-based tests carry the invariants that example-based tests miss.**
Round-trip conversions, quaternion normalisation, and the frame relationships
that must hold at every attitude — that the wind and body rotations are mutual
inverses, that the stability frame never moves the body y-axis, that elementary
rotations add their angles. These are the tests that catch a transposed rotation
matrix, which no single hand-picked example reliably does. Composition
associativity is *not* among them; it would be worth adding.

**No number in the V&V report is typed.** Every figure it states about galata's
own behaviour is measured at render time and referred to in prose by name. The
generator refuses to produce a document if a case note contains a value it also
computes, and `scripts/gen-verification.sh` fails on an unresolved placeholder —
which is left visible rather than dropped, because a sentence with the number
silently removed still reads fine and says nothing.

Anything shared between a test and the report lives in one place for the same
reason: the determinism battery, the perturbation-amplification study and the
hand-assembled NT-33A matrices are all libraries, not copies. Two
implementations would be two answers to the same question.

**The V&V report's summary table is generated, and its claims are checked.**
`docs/VERIFICATION.md` is produced from `tools/validation/case_registry.cpp`,
which declares every validation case: its reference, its status, and the tests
that stand behind it. Four tests reconcile that declaration with reality —
evidence must name a test that is actually registered in the binary it claims
to live in; a capability declaring *implemented and validated* must be backed
by a case that validates it; a case marked *not implemented* may not name a
capability that exists. The reasoning is in that file's header, and it comes
down to this: a hand-typed status line is a claim nothing checks, and this one
drifted three times before it was generated.

**Coverage is a diagnostic, not a target.** The project aims at a test-to-source
line ratio of 0.3 or better and treats a sudden drop as a signal to look, not as
a gate to satisfy.

## Running them

```
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
```

`ctest --preset dev -L unit` runs one tier. Labels match the table above.
The label is authoritative: `-L unit` on macOS also runs the desktop geometry
tests, and `-L integration` also runs the project CLI tests.

Every test preset runs ctest with one job per available core. Tests must
therefore be independent of one another: no test that depends on another having
run first, no assumption about the order they are picked up in, and no fixed
scratch path two of them could both be writing at once. A test that needs a
directory of its own builds a unique name for it.
