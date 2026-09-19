# Independent rotorcraft reference case — C2 discovery record

This record starts the independent-validation work without promoting it to
validation. The Souxmar F1 model remains a separate design-study configuration;
none of the sources below is evidence about Souxmar.

## Selected reference aircraft

The selected reference aircraft is the UH-60A Black Hawk, using K. B. Hilbert,
*A mathematical model of the UH-60 helicopter*, NASA-TM-85890,
USAAVSCOM-TM-84-A-2 (1984), NASA Technical Reports Server citation
`19840015585`: <https://ntrs.nasa.gov/citations/19840015585>.

The complete report PDF was inspected, including the configuration tables and
the level-flight trim table. It describes a ten-degree-of-freedom
full-flight-envelope model, lists the UH-60-specific fuselage,
canted-tail-rotor, stabilator and pitch-bias extensions, and states that
configuration and physical parameters are provided. It is a public U.S.
Government work. The executable comparison target is deliberately narrow: a
component-level reconstruction of the published main-rotor geometry and
weight condition, with no claim that the Galata Souxmar model reproduces the
UH-60 aircraft.

The companion flight-data source is D. L. Key et al., *Helicopter simulation
validation using flight data*, NASA-TM-84291 (1982), NASA Technical Reports
Server citation `19830004842`: <https://ntrs.nasa.gov/citations/19830004842>.
That record identifies the subject aircraft as the UH-60A and describes
flight-data validation of the mathematical model. It is useful for a later
measured-data comparison, but it is not silently substituted for the
configuration source.

## Reuse and provenance

- Rights: the Hilbert report is identified by NTRS as a U.S. Government work;
  the flight-data report is likewise identified as public. The repository will
  transcribe only values needed by the defined comparison and retain the source
  citation next to every reference file.
- Parameter provenance: all reference-aircraft geometry, mass properties,
  derivatives, controls, and flight condition must be tagged as transcribed,
  derived, or assumed. Assumed values are not reference values.
- Transcription uncertainty: report table precision, graph-read values and any
  unit/sign conversion will be recorded before comparison. The acceptance budget
  will be derived from printed precision before the first result is read.

## Executable component evidence

`examples/uh60-reference-component/compare.py` is an executable, source-backed
component comparison. It reads the checked-in reference-aircraft configuration,
recomputes SI geometry and derived disk quantities, and checks the published
main-rotor radius, solidity, rotor speed, and aircraft weight transcription.
The script emits machine-readable JSON and records `souxmar_validation: false`.
This is analytical/source-transcription verification, not a published
simulation comparison and not flight-data validation.

## Source-completeness matrix

| Required input | Source location | Units/convention | Transcription and uncertainty | Status |
|---|---|---|---|---|
| Main-rotor radius, chord, blade count, solidity, rotor speed | Hilbert, NASA-TM-85890, PDF p. 17, Table 1 | ft, rad/s, dimensionless; converted to SI | Table transcription; printed precision budget | executable |
| Aircraft weight | Hilbert, NASA-TM-85890, PDF p. 18, Table 1 | lb; converted to N | Table transcription; printed precision budget | executable |
| Level-flight trim at 40 knots | Hilbert, NASA-TM-85890, PDF p. 21, Table 4 | knots and report-specific trim/control columns | Table is identified, but Galata does not yet have the complete UH-60 force/moment model needed to reproduce it | outstanding |
| Flight-data time histories | Key et al., NASA-TM-84291, report sections on validation | report-specific channels and sampling | Catalogue and report describe the programme; synchronized machine-readable channels are not supplied here | outstanding |

The executable comparison therefore closes only the component/transcription
check. The full-aircraft published-simulation comparison and measured-data
validation remain open because their force/moment inputs, conventions, and
time histories are not complete in the checked-in source set. No undocumented
reference values are filled in to make those comparisons appear executable.
