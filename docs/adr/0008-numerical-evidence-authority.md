# ADR-0008: Numerical evidence retains its scope and authority

- **Status:** implemented; pending maintainer review
- **Date:** 2026-09-07
- **Deciders:** implementation team within the authorized M0 reliability work

## Context

The [audit baseline](../product/AUDIT_BASELINE.md) identified successful runs
that produced misleading assurance: sampled sensitivity maxima became guaranteed
margins, an ill-conditioned eigensystem appeared stable, and linearization lost
its chart and equilibrium diagnostics when converted into a linear system.
Passing the existing reference suite did not detect these counterexamples.

The numerical library serves both the CLI and embedded C++ callers. File-format
validation alone cannot protect the latter. A flight model is mutable, and a
stored trim residual can become stale after a model or control change.

## Decision

Preserve the direction and scope of evidence at public API boundaries.

1. `SensitivityPeaks` supplies sampled lower norm estimates. It is no longer an
   accepted argument to `guaranteed_margins`. Callers must provide the explicit
   `SensitivityNormUpperBounds` contract, including evidence of upper norms and
   the nominal-stability prerequisite. A nonempty evidence description is an
   explicit caller assertion, not independent verification of that assertion.
   Floating-point Hamiltonian brackets remain numerical bounds, not interval
   certificates or approved engineering tolerances.
2. Nominal-stability consumers share one internal eigensystem assessment. It
   checks finiteness, residual, eigenvector conditioning and scale-dependent
   separation from the imaginary axis. Unresolved conditioning refuses the
   calculation. A numerically unresolved axis boundary cannot establish
   positive delay tolerance. The conditioning cutoff is a conservative policy;
   stable defective realizations can be refused. Rescaling or a future separately
   verified stability algorithm may resolve them.
3. Every linearization recomputes the actual dynamic equilibrium from the
   supplied model, state and controls under an independently chosen finite
   positive budget. The supported straight-line trim and Euler chart envelope
   are checked before differentiation and at perturbations. Stored metadata
   cannot authorize an invalid numerical operation.
4. An `Artifact` retains immutable source `Linearisation` records keyed by the
   originating study stage. The executor propagates their union through every
   downstream stage, including branch joins. Reports identify them as source
   diagnostics; the run manifest retains step vectors, truncation matrices,
   operating point, actual residual, acceptance budget and chart conditioning.
   These diagnostics do not become an error bound for a transformed model.
5. Public model, solver and evaluator boundaries reject nonfinite values and
   invalid dimensions before arithmetic or matrix indexing can manufacture a
   plausible answer. Failure is explicit; there is no silent best effort.

The dependency remains pipeline → linearize → model. `LinearSystem` stays a
mathematical state-space value: it does not gain a reverse dependency on the
linearization module. The pipeline's forward-declared immutable evidence
pointer adds no new numerical-core dependency.

## Alternatives considered

**Keep the existing API and strengthen report caveats.** This preserves source
compatibility, but embedded callers can still turn sampled estimates into
claimed guarantees without a compiler error. The explicit type makes that
migration reviewable.

**Attach diagnostics to every transformed linear-system value.** This makes
propagation convenient but suggests that a source Jacobian's truncation estimate
also bounds an interconnection or controller result. Keeping source records
separate preserves their meaning and avoids a module dependency cycle.

**Accept any eigenvalue with a negative real part.** This permits more
realizations but ignores the observed failure on extremely nonnormal matrices.
An eventual Lyapunov or verified-Schur alternative should be evaluated with
independent adversarial tests before expanding the accepted domain.

## Consequences

This is a source-breaking pre-1.0 library correction: callers of
`guaranteed_margins(SensitivityPeaks)` must migrate. Finite searches still provide
plots and estimates; reports no longer label them guaranteed ranges, and a
missing classical crossover means not found in the searched band.

Previously accepted invalid/nonfinite inputs now throw. Source evidence adds
storage to artifacts and manifests. Serializing the complete source diagnostic
makes downstream reports reviewable without pretending to solve uncertainty
propagation. These changes do not qualify the tool for airborne software or
establish aircraft model validity.

## Revisit when

A verified algorithm can resolve the conservatively refused eigensystems; a
non-Euler linearization chart is introduced; or the proposed executable-model
IR requires additional evidence kinds. Extend evidence by explicit scope and
schema migration, without collapsing sampled, numerical, model-validity and
engineering-acceptance statuses into one successful-run flag.
