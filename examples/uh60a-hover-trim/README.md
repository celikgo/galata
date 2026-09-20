# UH-60A Level-1 study path

Run from the repository root:

    galata run examples/uh60a-hover-trim/study.yaml --output-dir build/uh60a-hover-trim

This is the first native UH-60A model path in Galata. It exercises the shared
helicopter parser, hover trim, vehicle linearisation and modal reporting.

It is not a validated UH-60A simulation. The NASA reference component controls
only a small published geometry/weight subset; the executable model's remaining
rotor, empennage, inertia, drivetrain and actuator parameters are explicitly
assumed. See models/uh60a/PROVENANCE.md.
