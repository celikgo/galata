# Python engineering workflow

The installable galata-engineering package is a thin, machine-readable
adapter around the authoritative Galata CLI:

    python3 -m pip install -e .
    PYTHONPATH=python python3 examples/heli-python-workflow.py

The executable example calls load, configure, trim, linearize, design,
simulation, evaluation, comparison, plotting and export operations. Numerical
results are never recovered from Markdown prose. The CLI run --json response
points to the versioned galata.run.v1 manifest; CSV channels are selected by
their named headers and retain native units.

The offline plotter writes SVG without a GUI or network. Named time histories
can add reference/event/saturation markers; open/closed comparisons, sweep
curves and ensemble plots preserve failed/excluded member markers. PNG export
is an optional cairosvg adapter and must be enabled with:

    pip install -e '.[png]'

Trajectory playback uses the declared NED source convention and produces a
static isometric N/E/up projection (`up = -down`), with start/end time labels.
The workflow is orchestration and inspection, not a second numerical
implementation: trim, linearisation, controller design and simulation
continue to run through the C++ services used by the CLI.
