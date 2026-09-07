# Architecture decision records

One record per non-obvious decision, numbered without gaps. A decision that a
stranger would have made differently, or that costs something real to reverse,
gets a record. A decision that follows from the charter does not.

The format is in [0000-template.md](0000-template.md).

| # | Title | Status |
|---|---|---|
| [0001](0001-independent-c-abi.md) | Plugin C ABI is an independent sibling of `souxmar-c` | accepted |
| [0002](0002-state-and-frame-conventions.md) | State vector, frames and attitude conventions | accepted |
| [0003](0003-strict-si-and-boundary-conversion.md) | Strict SI internally, conversion only at the boundary | accepted |
| [0004](0004-determinism-policy.md) | Determinism policy, and what it does not cover | accepted |
| [0005](0005-single-source-of-version.md) | One source of version truth | accepted |
| [0006](0006-equations-of-motion-about-the-cg.md) | The equations of motion are written about the centre of gravity | accepted |
| [0007](0007-reference-values-from-copyrighted-sources.md) | Scalar reference values may be quoted from copyrighted sources; datasets may not | accepted |
| [0008](0008-numerical-evidence-authority.md) | Numerical evidence retains its scope and authority | implemented; maintainer review pending |
| [0009](0009-release-evidence-and-source-identity.md) | Releases consume complete evidence for one immutable source | implemented; hosted verification pending |
| [0010](0010-continuous-scalar-executable-model.md) | Bounded continuous scalar executable model | implemented feasibility profile; review pending |
| [0011](0011-source-model-identity-and-project-boundary.md) | Source-model identity and project document boundary | proposed |
