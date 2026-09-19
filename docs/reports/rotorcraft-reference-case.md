# Independent rotorcraft reference case — C2 discovery record

This record starts the independent-validation work without promoting it to
validation. The Souxmar F1 model remains a separate design-study configuration;
none of the sources below is evidence about Souxmar.

## Selected reference aircraft

The selected reference aircraft is the UH-60A Black Hawk, using K. B. Hilbert,
*A mathematical model of the UH-60 helicopter*, NASA-TM-85890,
USAAVSCOM-TM-84-A-2 (1984), NASA Technical Reports Server citation
`19840015585`: <https://ntrs.nasa.gov/citations/19840015585>.

The NTRS record describes a ten-degree-of-freedom full-flight-envelope model,
lists the UH-60-specific fuselage, canted-tail-rotor, stabilator and pitch-bias
extensions, and states that configuration and physical parameters are provided.
It is a public U.S. Government work. The comparison target is deliberately
narrow: reproduce one published UH-60 trim/stability condition and compare
named trim quantities and linear stability derivatives or eigenvalues.

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

## Current dependency and honest status

The accessible NTRS catalogue records establish the aircraft and the existence
of configuration/validation material, but this checkout does not yet contain a
machine-readable transcription of a single complete condition. Before an
executable comparison can be added, the following source inputs must be
transcribed and independently checked from the report pages:

1. one exact flight condition (airspeed, altitude/density, weight, CG, rotor
   speed and control convention);
2. the complete parameter subset needed by Galata's separate reference-aircraft
   configuration, including inertia and rotor/tail geometry;
3. the published trim quantities and derivative/eigenvalue table at that
   condition, with units and printed precision; and
4. for a measured-data comparison, time histories with synchronized controls,
   rates/attitude or acceleration channels, sampling metadata, and the
   kinematic/sign-convention mapping.

Until those inputs are transcribed and the comparison report is executable,
C2 remains **discovery started, comparison outstanding**. An analytical check
against the UH-60 published simulation and an agreement with measured aircraft
data will be labelled as different evidence classes; neither will be called
validation of the Souxmar F1 configuration.
