# NT-33A longitudinal modes

The phugoid and the short period — the two modes every flight-dynamics course
opens with — computed and labelled from a published model.

## Run it

```bash
galata run examples/nt33a-longitudinal-modes/modal-study.yaml
```

## What it shows

The classification is done by eigenvector participation, not by frequency. That
distinction is the whole point: here the phugoid happens to be the slow mode and
the short period the fast one, as it is for any conventional aeroplane in
ordinary flight, so a classifier keying on frequency would get the right answer
for the wrong reason. At an aft centre of gravity the two modes approach each
other and can merge into a pair of real roots, and there the frequency rule
fails while participation keeps working — which is precisely the configuration
worth studying.

Against NASA CR-2144's published values for this flight condition:

| Mode | Published | galata |
|---|---|---|
| Phugoid | ζ = 0.0948, ω_n = 0.172 rad/s | 0.0929, 0.1719 |
| Short period | ζ = 0.622, ω_n = 1.59 rad/s | 0.6220, 1.5945 |

Three of those four reproduce within the precision the source prints. **The
phugoid damping ratio does not** — it disagrees by 2%. That is a known, open
discrepancy, not a rounding artefact: it is written up in
[`docs/VERIFICATION.md`](../../docs/VERIFICATION.md) with its measured size and
the two candidate explanations, and a regression lock in the validation suite
stops it growing unnoticed.

It is worth understanding why the disagreement is concentrated where it is. The
phugoid eigenvalue itself is close: −0.015974 ± 0.171170j against a published
−0.016306 ± 0.171226j, agreeing to 6e-5 in the imaginary part. But ζ = |Re|/|λ|
divides a small real part by a small natural frequency, so a 3.3e-4 residual in
the real part becomes a 2% error in the ratio. The mode is in nearly the right
place; the derived number amplifies what is left over.

## What this example does not show

No trim, no linearisation. The state matrix is given, not computed from a
nonlinear aircraft — neither capability exists yet.
