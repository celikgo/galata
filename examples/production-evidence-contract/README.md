# Reproducible production-evidence contract

This example runs the repository-controlled evidence chain end to end with
explicitly synthetic inputs:

```sh
python3 scripts/run-production-contract.py \
  --output-dir build/production-contract-v1
```

The runner creates and verifies:

1. a `synthetic_contract` flight-test package and package-backed validation
   receipt;
2. a POSIX onboard deployment bundle;
3. a `host_sil` target-evidence package; and
4. a complete nine-role qualification dossier and chain result.

The command is expected to finish with `status=not_ready_by_design`. This is a
traceability and integration contract, not flight evidence. The synthetic
campaign, host-SIL target evidence, placeholder review records and pending
authority decision are deliberately ineligible for production qualification.
The generated directory is disposable and is not a release artifact.
