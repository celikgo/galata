# Qualification evidence dossier

Galata now has a single, bounded package format for assembling the evidence
that an application-specific qualification review must examine:

```text
galata qualification create <new-directory> \
  --product-id galata --product-version 0.3.0 \
  --intended-use "controlled engineering review" \
  --aircraft-id airframe-01 --configuration configuration-2026-09 \
  --qualification-basis "application-specific review basis" \
  --authority-id external-authority-pending \
  --file requirements_matrix=... --file software_release=... \
  --file verification_report=... --file flight_test_campaign=... \
  --file hardware_hil_report=... --file safety_case=... \
  --file independent_review=... --file authority_decision=... \
  --file maintenance_plan=...
galata qualification verify <new-directory>/qualification.manifest
galata qualification verify <dossier.manifest>
galata qualification verify-chain <dossier.manifest> \
  --flighttest <campaign.manifest> \
  --flight-validation-receipt <validation-receipt.json> \
  --deployment <deployment.manifest> \
  --deployment-dir <verified-runtime-package> \
  --target-evidence <target-evidence.manifest>
```

The chain verifier requires the complete staged runtime directory as well as
the manifest. It re-verifies `onboard.manifest`, `deployment.receipt`, the
executable runtime and every declared artifact, then binds the runtime SHA-256
to the deployment manifest before reporting traceability. A manifest-only
deployment is not sufficient for the production chain.

The `create` command copies the nine declared source files into a new
package, hashes the copied bytes, writes the manifest, verifies the complete
package, and publishes it with an atomic directory rename. It refuses an
existing destination and leaves no staging directory after a failed create.
The command assembles traceability; it does not inspect the truth of the
records or approve the application.

The format is `galata-qualification-evidence-v1`. Its manifest declares the
product and version, intended use, aircraft and configuration identity, the
qualification basis, and the external authority that owns the decision. It
must set:

```text
acceptance_state=not_qualified
```

The verifier will not accept a manifest that promotes itself to `qualified`.
That is intentional: a SHA-256 check can establish byte identity, but it
cannot establish that a test was representative, that a safety argument is
adequate, or that an authority accepted the application.

Every dossier must contain exactly one regular, non-symlink file for each of
these roles:

1. `requirements_matrix`
2. `software_release`
3. `verification_report`
4. `flight_test_campaign`
5. `hardware_hil_report`
6. `safety_case`
7. `independent_review`
8. `authority_decision`
9. `maintenance_plan`

`verify-chain` is the end-to-end traceability gate. In addition to verifying the
dossier itself, it verifies the complete flight-test campaign, requires a
gate-passed package-backed flight-validation receipt whose campaign digest
matches the supplied campaign, verifies the target-evidence package against the
exact onboard deployment manifest, and requires the dossier's
`flight_test_campaign` and `hardware_hil_report` bytes to be the supplied
campaign and target-evidence manifests. The aircraft
identity and configuration must also match the campaign. This closes the
repository-controlled evidence chain while retaining
`qualification_state=not_qualified`; it does not decide whether the flight,
hardware tests or authority review are genuine or acceptable.

The chain result reports readiness separately from traceability. Its
`qualification_eligibility` is `not_ready` unless the campaign is explicitly
`measured_flight`, its validation receipt is `gate_passed` and bound to the
campaign, target evidence is `target_hil` or `flight_target`, and the dossier
identifies an external authority instead of `external-authority-pending`. If
those declared prerequisites are present, the result is only
`eligible_for_authority_review`; `qualification_state` remains
`not_qualified` and the authority still has to examine the underlying records.
Synthetic campaigns and `host_sil` target evidence therefore produce a
traceable but explicitly not-ready chain.

Paths are relative to the manifest, must remain inside the package, and are
checked for symlink components. Each file is bounded at 256 MiB and its
declared SHA-256 is recomputed. A successful command returns
`galata.qualification-evidence-verification.v1` with
`evidence_package_complete=true`, while still reporting
`qualification_state=not_qualified` and
`external_authority_acceptance_required=true`.

This package closes the repository-controlled traceability boundary. It does
not create the required controlled flight records, target HIL measurements,
independent review, legal approval, airworthiness decision, or tool
qualification. Those remain external evidence and must be supplied by the
responsible programme and authority before any qualified or certified use.

The native macOS preview exposes the same operation under **File → Verify
Qualification Dossier…** and displays the product, configuration, evidence
roles and non-qualified state returned by the shared CLI.
