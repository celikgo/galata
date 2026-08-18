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
