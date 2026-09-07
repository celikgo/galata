# Study files and run records

`galata run study.yaml --output-dir results` executes a study and writes its reports
and a run manifest. Missing output directories are created. Use `--overwrite` to
replace an existing report deliberately; the default refuses replacement.

## Input contract

A study has one YAML document with `version: 1` and a `stages` sequence. Each stage
has `id`, `capability`, and an optional `input` map. Stage identifiers are unique,
and `{from: stage_id}` wires an upstream artifact into a later stage.

Unknown keys are rejected at the study, stage, capability-input and model levels.
Duplicate keys are rejected in every map, including nested actuator settings and
model data. A misspelled optional setting cannot silently select its default.
Quoted scalars remain strings; numeric inputs must be finite, and counts and
indices must be integers. YAML aliases, anchors, custom tags, multiple documents
and nesting beyond the parser's limit are rejected.

Relative model paths resolve against the directory from which the study was
loaded. Absolute input paths are allowed. The runner snapshots declared model
inputs before executing stages and parses those exact bytes. If the original file
changes during the run, the snapshot remains the input for that run.

## Output contract

Report paths are portable relative paths under the operator's output directory.
Absolute output paths, `..`, symlinks and Windows device names are refused. Two
stages cannot write the same destination, including names that differ only in
ASCII letter case. An output cannot replace the executable, study or model input,
even with `--overwrite`.

Each file is written completely to a temporary file before publication. Without
`--overwrite`, publication refuses a destination that another process created
after preflight. Replacement publishes a complete new file rather than truncating
the existing report before writing it. These guarantees apply per file; a later
stage or filesystem failure can leave earlier reports in the output directory.
The process exit status and completed run manifest indicate whether the entire
study finished.

The report formats use the same computed artifacts:

- `report.markdown`: readable tables, numerical checks and model limitations.
- `report.html`: self-contained HTML tables, matrix blocks, and simulation
  time-history plots with local styling and print layout.
  Text is escaped; reports contain no scripts, remote images, fonts or other
  resources that cause network requests.
- `report.csv`: named columns for linear and nonlinear trajectory samples.

Reports refuse artifact kinds for which no writer exists rather than returning
success with an empty section.

HTML charts use the selected trajectory artifacts, identify the originating
stage, and group only quantities with common units. Each chart has its own
vertical scale. Linear plots show at most the first eight output channels; CSV
retains all recorded channels. Long traces are reduced to at most 2,002 display
points per series while retaining each time bucket's minimum and maximum, and
the caption identifies the reduction. This does not change the CSV data.

## Run manifest

A successful run prints the path of `run-<sha256>.json`. The filename is the
SHA-256 digest of the manifest's exact bytes. Its schema is `galata.run.v1`, and it
records:

- The original study and every declared model input, including full original
  bytes encoded as hex, byte counts and SHA-256 digests.
- The actual stage input tree and execution order, plus the input and output
  directories used for path resolution.
- Output filenames, byte counts and digests.
- The executing binary's path and digest; project/compiler/platform identity;
  source Git revision and clean, dirty or unknown status; dependency versions and
  the dependency-manifest digest.

The input snapshots are complete copies of the study data. A manifest contains
the model content, not only filenames or checksums. The binary itself is
identified by its digest and is not copied into the manifest.
Its bytes are snapshotted before stages execute and checked again at completion.
A concurrent executable replacement makes the run fail instead of attributing
the outputs to the replacement binary.

An identical manifest can be reused. A manifest already on disk with different
bytes is never replaced, including under `--overwrite`. This is a content-identity
record, not a digital signature or tamper-resistant storage. Git metadata is
refreshed at build time; a source archive without Git metadata records `unknown`.

## Adding capabilities

`Capability::input_keys` declares the complete top-level input vocabulary.
`input_file_keys` and `output_file_keys` additionally mark file inputs and outputs
for snapshots and preflight. Both role lists refer to keys in `input_keys`.
An empty vocabulary accepts no keys; it does not disable validation. Capabilities
with nested settings must validate those settings' keys and types too.

Capabilities read files with `StageContext::read_input` and publish output with
`StageContext::write_output`. Model adapters parse the returned bytes rather than
reopening the original path. `StageContext::resolve_output_path` supplies the
published artifact's filename. Direct file writes bypass the run's containment
and provenance contract and are not an extension mechanism.

The library's `RunOptions` exposes the same overwrite policy. Embedding programs
can explicitly disable manifest writing when they own a different record format;
the command-line runner always writes a manifest on success.
