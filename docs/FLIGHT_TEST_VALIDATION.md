# Flight-test validation workflow

Galata now has one vehicle-neutral numerical comparison path:
identify.validate.vehicle. It accepts the built-in fixed-wing, multirotor and
helicopter VehicleModel adapters and compares a declared model prediction with
a measured data::Record.

The stage requires all of the information that materially changes the result:

- an explicit initial state or a trim artifact belonging to the same model;
- a fixed integration step that lies exactly on the record time lattice;
- one command channel for every model control, in model order;
- measured output channels mapped to model state names;
- the atmospheric environment, including an optional wind vector; and
- the identity of the record used for estimation, either as a supplied record
  or as a declared SHA-256 digest.

The result reports RMSE, maximum and mean error, fit fraction where the
measured channel varies, lag-one residual autocorrelation, record lineage and
the separation classification. A digest inequality by itself is never treated
as held-out evidence. Two non-overlapping windows of one imported record can be
classified as VerifiedDisjoint; different flights remain a caller declaration
unless an external evidence process establishes that fact.

This closes the numerical comparison workflow. It does not close aircraft
validation. The implementation deliberately labels its assumptions as a
numerical model comparison and refuses to call the result flight-test approval,
airworthiness evidence or tool qualification. A real validation campaign still
needs an identified aircraft, calibrated instrumentation, configuration control,
an approved test plan, independent review and target-specific acceptance
criteria.

The lower-level C++ entry point is:

    galata::identify::validate_vehicle_model(model, record, request);

The pipeline capability produces the existing validation artifact, so reports
can retain the model identity and the record lineage without inventing a new
report format.

The shipped [`fixed-wing-validation-contract`](../examples/fixed-wing-validation-contract/README.md)
example exercises this path with a synthetic NT-33A trim trajectory. It is a
software contract test: its numerical gate can pass and its flight-test
evidence gate remains unresolved by design. It must not be cited as aircraft
validation.

## Public flight-data import reference

The repository also ships a dependency-free converter for the public NASA
DASHlink four-class recorder data:
[`nasa-dashlink-flight-data-reference`](../examples/nasa-dashlink-flight-data-reference/README.md).
NASA's resource describes actual onboard data from a de-identified
regional-jet type, including control-surface, attitude, airspeed,
acceleration and environmental channels. The converter exports one NPZ window
to Galata CSV and writes a provenance sidecar, but requires the operator to
provide the sample period explicitly. It never invents an aircraft model,
configuration, calibration package or test-plan identity.

This closes a reproducible public-record import path, not the campaign gate.
The public resource does not identify the aircraft/configuration or provide the
calibration, approved test-plan and independent-review artifacts required for
a model-specific flight-test claim. The record therefore cannot be paired
with NT-33A, Navion, A-7A, A-4D, F-4C, NASA GTM T2, NASA F-16 or another unrelated Galata model and called
validated. A reviewer must supply a matching model and controlled evidence
package before `identify.validate.vehicle` can produce a flight-test evidence
pass.

`identify.validate.vehicle` can also take an `acceptance` map with one or more
`outputs` criteria. Each criterion names a measured channel/state pair and at
least one predeclared `max_rmse`, `max_absolute_error` or `min_fit_fraction`
budget. The resulting gate is `pass` only when every budget is met and the
estimation/validation records are verified disjoint windows of one imported
record. An actual second flight remains `unresolved` unless an independent
evidence process establishes its relationship; a filename or unequal digest
does not turn a caller assertion into proof. Failed or unresolved gates are
retained in the report and never become an airworthiness or certification
decision.

For a campaign-completeness review, the same `acceptance` map may include an
`evidence` map with all of the following fields:

    aircraft_id: "airframe-01"
    aircraft_configuration: "configuration-2026-09"
    evidence_class: "measured_flight"
    test_plan_id: "FT-001"
    calibration_manifest_sha256: "...64 hexadecimal characters..."
    configuration_manifest_sha256: "...64 hexadecimal characters..."
    reviewer_id: "independent-reviewer"
    reviewer_attestation_sha256: "...64 hexadecimal characters..."
    safety_review_complete: true

The resulting flight-test evidence gate is `pass` only when the numerical gate
passes, a verified campaign package declares `evidence_class=measured_flight`,
every required identity/reference is present and well formed, the validation
result retains both record lineages and the safety review is attested complete.
`public_deidentified` and `synthetic_contract` packages remain `unresolved`
even when their bytes and numerical budgets are valid. Missing references are
`unresolved`; malformed SHA-256 references or a failed numerical gate are
`fail`. This checks evidence completeness and configuration traceability, not
the truth of the referenced documents. It cannot replace an approved test
organisation, authority, airworthiness process, independent flight evidence or
tool qualification.

For a package-backed validation, put the verified campaign manifest in the
acceptance map:

    acceptance:
      campaign_manifest: /controlled/campaign/campaign.manifest
      outputs:
        - {channel: position_north_m, state: position_north_m, max_rmse: 2.0}

The pipeline verifies the manifest and every referenced file, records those
bytes in the run input ledger, derives the campaign evidence fields from the
verified package, and requires the package's `flight_record` SHA-256 to match
the imported validation record. The generated report retains the campaign
manifest SHA-256. Manual evidence fields remain available for lower-level API
use, but a package-backed run is the operator-facing traceability path.

## Controlled campaign package verification

The references above can be checked as a package before a reviewer consumes
them. The command is:

    galata flighttest verify <campaign.manifest>

For a controlled package assembled from source files, use the atomic creator:

    galata flighttest create <new-directory> \
      --aircraft-id airframe-01 \
      --configuration configuration-2026-09 \
      --evidence-class measured_flight \
      --test-plan-id FT-001 \
      --reviewer-id independent-reviewer \
      --safety-review-complete true \
      --file test_plan=/controlled/test-plan.pdf \
      --file flight_record=/controlled/flight-record.csv \
      --file calibration_manifest=/controlled/calibration.json \
      --file configuration_manifest=/controlled/configuration.json \
      --file reviewer_attestation=/controlled/reviewer-attestation.txt

The creator refuses missing or duplicate roles, symlink sources, an existing
destination and any safety marker other than an explicit `true`. It copies
each source into the package's evidence directory, hashes the copied bytes, writes the deterministic
manifest, verifies the complete staged directory and publishes it with a
no-overwrite rename. This removes manual hash/copy errors; it does not inspect
the truth of the flight record or grant qualification.

The manifest is a bounded, deterministic text file. Its five required records
are `test_plan`, `flight_record`, `calibration_manifest`,
`configuration_manifest` and `reviewer_attestation`:

    format=galata-flight-test-evidence-v2
    aircraft_id=airframe-01
    aircraft_configuration=configuration-2026-09
    evidence_class=measured_flight
    test_plan_id=FT-001
    reviewer_id=independent-reviewer
    safety_review_complete=true
    file.0.role=test_plan
    file.0.path=test-plan.pdf
    file.0.sha256=<64 hexadecimal characters>
    file.1.role=flight_record
    file.1.path=flight-record.csv
    file.1.sha256=<64 hexadecimal characters>
    file.2.role=calibration_manifest
    file.2.path=calibration.json
    file.2.sha256=<64 hexadecimal characters>
    file.3.role=configuration_manifest
    file.3.path=configuration.json
    file.3.sha256=<64 hexadecimal characters>
    file.4.role=reviewer_attestation
    file.4.path=reviewer-attestation.txt
    file.4.sha256=<64 hexadecimal characters>

The verifier requires every path to be relative to the manifest directory,
rejects `.`/`..` escapes and symlink components, bounds each file at 256 MiB,
and recomputes every declared SHA-256 over the bytes it reads. The
`evidence_class` is mandatory and must be one of `measured_flight`,
`public_deidentified` or `synthetic_contract`. A successful result means the
package is complete and byte-consistent at verification time; it does not mean
the flight record is genuine, representative or accepted. Only the first
class is eligible for a flight-test evidence-gate pass, and even that remains
an evidence-completeness result rather than an airworthiness or certification
decision. The CLI therefore reports `qualification_state=not_qualified` and
makes explicit that it is making no airworthiness or certification claim.

## Reproducible package-backed execution

The campaign verifier proves that the package is complete and byte-consistent;
it does not prove that a study actually used it. The execution receipt closes
that traceability gap:

    galata flighttest validate /controlled/campaign/campaign.manifest \
      /controlled/study.yaml \
      --output-dir /controlled/validation-run \
      --receipt /controlled/campaign/validation-receipt.json

    python3 scripts/run-flight-test-validation.py \
      --cli build/ci-macos/src/cli/galata \
      --campaign-manifest /controlled/campaign/campaign.manifest \
      --study /controlled/study.yaml \
      --output-dir /controlled/validation-run \
      --receipt /controlled/campaign/validation-receipt.json

The command verifies the campaign, runs the supplied study, requires an
`identify.validate.vehicle` stage, checks that the campaign manifest and every
campaign artifact are present in the immutable run input ledger, and checks the
generated report's numerical and flight-test gate results. It returns zero only
for a `measured_flight` package whose declared gates pass. Synthetic and public
de-identified packages can execute the same software contract, but produce a
`not_ready` receipt and exit code 2. Tampering, a failed run or a missing
traceability record produces a failed receipt and exit code 1. The receipt
always retains `qualification_state=not_qualified` and makes no airworthiness,
certification or authority-acceptance claim.

The production preflight applies the same binding rule independently: the
receipt must declare the supplied campaign manifest path, carry the matching
campaign digest, and set `campaign_binding_verified=true` and
`run_inputs_verified=true`. A gate-passed receipt for a different campaign is
rejected even when its digest and numerical fields are otherwise valid.
