# Reproducible broader-aircraft evidence contract

Run:

```sh
python3 scripts/run-aircraft-validation-contract.py \
  --output-dir build/aircraft-validation-contract-v1
```

The contract binds the existing Galata F-16 and NASA GTM model slices into two
distinct evidence records, checks model/calibration/envelope/report/review
bytes, and runs the broader-aircraft verifier.

Its expected result is `status=not_ready_by_design`: both records are
traceable, but their independent reviews are explicitly incomplete. This
demonstrates the production gate without fabricating independent review,
aircraft calibration or flight validation.
