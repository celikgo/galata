# NT-33A — provenance

## Source

Robert K. Heffley and Wayne F. Jewell, **"Aircraft Handling Qualities Data"**,
NASA CR-2144. Also Systems Technology Inc. Technical Report 1004-1, and DTIC
AD-A277031. Prepared under NASA contract NAS 4-1729.
Systems Technology, Inc., Hawthorne, California, December 1972.

NTRS document ID 19730003312 — <https://ntrs.nasa.gov/citations/19730003312>

The file used: 7,101,468 bytes, 352 pages,
SHA-256 `f2976b2d9a3f62471de276c019f314af58b83c92ebd61f915779bea4d784349a`.

## Rights

The report's own NASA Report Documentation Page prints:

> 18. Distribution Statement: Unclassified - Unlimited
> 19. Security Classif. (of this report): Unclassified
> 20. Security Classif. (of this page): Unclassified

No copyright notice, no proprietary or limited-rights legend and no
export-control marking appears anywhere in the document. It is a NASA
contractor report prepared under a government contract and released without
restriction.

This satisfies the project's rule that coefficient data ships in-tree only when
it comes from a US Government work.

## Transcription

**The scan's OCR layer is unusable and was not used.** The PDF is a bitonal
CCITTFax G4 microfilm scan, 2544x3300 at 300 dpi, with an OCR layer bolted on
that handles prose passably and every numeric table not at all — the Table II-2
page extracts as `P-I 0 ,.-I t-.-.1 bid .,rl ,-4 0 .,,-I .4,}`.

Every value was read visually from rendered page images, with load-bearing
digits and superscript signs re-cropped at native resolution and read again. A
second, independent pass re-downloaded the document, confirmed its SHA-256, and
re-read twelve of the values without reference to the first pass. All twelve
agreed exactly.

The values in their original units are in
[`tests/validation/reference/nt33a_fc1.csv`](../../tests/validation/reference/nt33a_fc1.csv),
which carries the table and page number for each one.

## The aircraft and the condition

NT-33A — a variable-stability T-33 operated by Cornell Aeronautical
Laboratory. Flight condition 1 of the eight the report tabulates (Table II-2,
report p.22): sea level, M = 0.204, power-approach configuration — 230 gal tip
tanks, 25% internal fuel, full flaps, gear down, 1.4 Vs, W = 11,800 lb, c.g. at
0.260 c-bar.

## Conversions

Every conversion factor is exact by definition, so the SI values in
`nt33a-fc1.yaml` are exact transcriptions and not approximations:

| From | Factor |
|---|---|
| foot | 0.3048 m exactly (1959 international yard and pound agreement) |
| pound-force | 0.45359237 × 9.80665 N exactly |
| slug | pound-force / foot |
| slug ft² | slug × foot² |

## Choices made here that the source does not make

Two, and both change results, so both are stated rather than buried.

**The trim pitching moment is defined as zero at the reference condition.** The
report publishes the derivatives and the trim lift coefficient but neither a
trim pitching moment nor a trim elevator deflection. The model is therefore
*defined* to be trimmed at its reference angle of attack with the elevator
centred. A consequence: the trimmed elevator this model produces is near zero
by construction and is not evidence of anything.

**The lateral-directional derivatives are declared stability-axis and rotated.**
The report gives them in stability axes; the equations of motion are body-axis.
The model file declares `lateral_axes: stability` and galata rotates them.

Skipping that rotation is tempting — the trim angle of attack is 2.2 degrees
and cos(2.2°) is 0.9993 — and it is wrong. The rotation *mixes* the rolling and
yawing moments, and C_l_beta is 2.6 times C_n_beta, so the cross term dominates
the cosine: C_n_beta moves by 10%, not 0.07%. Left unrotated, this model's
Dutch roll damping came out 35% high. The error was found by comparing the
linearised dimensional derivatives against the report's Table II-7, which is
why that comparison is a validation test and not a debugging aid.

## What this model is not

Read the "WHAT THIS IS NOT" block in
[`include/galata/model/aircraft.hpp`](../../include/galata/model/aircraft.hpp)
before using it for anything. In short: it is a first-order expansion about one
flight condition. It has **no stall** — lift rises linearly with angle of attack
for ever — no Mach effects, no engine model, no ground effect, and no
configuration changes. It is good within a few degrees of 2.2° angle of attack
and at speeds near 228 ft/s, and it is worthless outside that.

`galata` reports how far a query has strayed from the reference condition in
every result. Nothing stops you ignoring that.
