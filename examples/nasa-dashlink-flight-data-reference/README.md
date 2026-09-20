# NASA DASHlink public flight-data reference

This reference documents a reproducible import path for the public NASA
DASHlink four-class data set. The source resource describes actual onboard
recorder data from a de-identified regional-jet type, with 160-row time
windows and 20 recorded variables including control-surface positions,
airspeed, attitude, acceleration and wind. The source documentation describes
the window in seconds, but this reference still requires the operator to
declare the sample period rather than silently assuming one.

The source is intentionally not vendored into Galata: the raw NPZ is about
1.6 GB and the resource does not state a license in its metadata. Download it
from the [NASA DASHlink resource](https://c3.ndc.nasa.gov/dashlink/resources/1018/),
then convert one window:

```bash
python3 scripts/convert-nasa-dashlink.py \
  /controlled/DASHlink_full_fourclass_raw.npz \
  build/dashlink-flight-window.csv \
  --metadata build/dashlink-flight-window.json \
  --index 0 \
  --sample-period-s 1.0
```

The converter requires the sample period explicitly. It does not infer units,
aircraft identity, configuration, calibration or test-plan status. The JSON
sidecar retains the source URLs, selected instance and explicit limitations;
the CSV can then be mapped through `data.import.csv` after a reviewer has
declared the channel units and frames for the intended analysis.

This is a public flight-data import reference, not a completed Galata
flight-test validation case. NASA describes the records as de-identified and
not part of an airline FOQA program; the public resource does not supply the
aircraft/configuration identity, calibrated instrumentation package, approved
test plan or independent acceptance review needed by Galata's campaign gate.
It must not be used as evidence for airworthiness, certification or a
model-specific validation claim without that missing evidence and a matching
aircraft model.
