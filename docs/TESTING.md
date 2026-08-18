# Testing

Five tiers. A test belongs to exactly one, and the tier determines what a
failure means.

| Tier | Directory | What it proves | Exists |
|---|---|---|---|
| Unit | `tests/unit/` | One algorithm against a closed-form answer | yes |
| Property | `tests/property/` | Invariants over many generated inputs, seeded | yes |
| Integration | `tests/integration/` | One pipeline stage against a frozen capability contract | yes |
| Validation | `tests/validation/` | Output against published reference data | yes |
| Determinism | `tests/determinism/` | Bit-identical repeat runs; cross-platform agreement | yes |
| Evals | `evals/` | Agent behaviour against checkable outcomes | no |

The "Exists" column is the honest state of this repository right now. A
directory appears in `tests/CMakeLists.txt` in the same commit that adds its
first test, never in advance.

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
Round-trip conversions, quaternion normalisation, frame-transformation
composition and associativity. These are the tests that catch a transposed
rotation matrix, which no single hand-picked example reliably does.

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
