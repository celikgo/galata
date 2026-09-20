# Navion nominal model provenance

The derivative set is transcribed from Gary L. Teper, *Aircraft Stability and
Control Data*, NASA CR-96008, Systems Technology, Inc. for NASA Ames Research
Center, April 1969, NASA document number N69-31783. The public NASA technical
report is available from the NASA Technical Reports Server:

<https://ntrs.nasa.gov/citations/19690022405>

The Navion data are in Section X, pages 101–102, Tables X-A through X-E. The
report identifies this as a nominal level-flight condition at sea level,
Mach 0.158, 176 ft/s and 2750 lb. The source gives geometry, mass and inertia
in ft, ft², lb and slug-ft², and gives the aerodynamic derivatives per radian
in stability axes. The YAML stores only SI values, per the repository's
strict-unit policy.

Conversions are exact definitions used by the source boundary:

- `1 ft = 0.3048 m`;
- `1 slug = 14.593902937206362 kg`;
- `1 lbf = 4.4482216152605 N`; and
- standard gravity is `9.80665 m/s²`, so mass is weight divided by standard
  gravity.

The source's dimensional elevator moment derivative is
`M_delta_e = -11.1892 / s²` in Table X-D. The model stores the equivalent
non-dimensional coefficient
`C_m_delta_e = M_delta_e / (qbar S c / I_y) = -0.8890478007`, using the source
dynamic pressure `qbar = 36.8 lbf/ft²` and the converted reference geometry and
inertia. The source does not print a trim pitching-moment intercept, so
`pitching_moment_ref: 0` is an explicit model convention, not an additional
measurement. The reference angle is the source's 0.6 degree level-flight value.

This model is a reproducible published-data transcription, not evidence that
the current Galata equations reproduce a real Navion or that the result is
valid outside this local linearisation. It has no stall, propulsion, Mach,
configuration, structural, instrumentation or flight-test campaign model.
