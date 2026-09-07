# M2 project/worker, linear graph, recovery and native macOS preview

**Status: three bounded preview increments are implemented and locally
verified on macOS arm64.** The first increment
adds portable project revisions, an isolated CLI worker and an optional native
macOS authoring/review surface to the 0.3.0 baseline. The second reconstructs
the existing local NT-33A plant/controller study as an editable typed graph
with retained source evidence. The third adds a Dim appearance, native block
and sample tables, and explicit saved-revision review and restoration. This advances the [M2 delivery work](DELIVERY.md)
without completing M2 acceptance, selecting the product's desktop toolkit or
closing the remaining external [M1 contracts](M1_CONTRACTS.md).

## Implemented scope

- A versioned local directory project with separate model, presentation and
  simulation settings; immutable revisions and stale-save conflict refusal.
- Draft editing that permits unfinished wiring and compile-time graph errors
  while retaining strict structural, finite-value and resource validation.
- Create, inspect, save and run commands using the same existing model compiler,
  fixed-step simulator and study pipeline as headless automation.
- Immutable submitted source snapshots, cooperative cancellation, process-lock
  recovery, retained terminal diagnostics and verification of completed artifacts.
- An optional AppKit/Objective-C++ macOS preview with a block canvas, connections,
  JSON property editing, save/run controls, run history, trajectory plots and
  evidence/manifest inspection.
- [ADR-0012](../adr/0012-project-worker-preview.md), the
  [project-file contract](../PROJECT_FILES.md), public CLI acceptance cases and
  pipeline cancellation integration tests.
- An explicit `continuous-linear.v1` profile with ordered typed linear rows,
  retaining the existing scalar profile's frame and canonical-byte contracts.
- A pure bounded state-space adapter and `model.linear_graph` capability that
  lower a linear plant and optional LQR feedback through the existing graph
  compiler/evaluator/RK4 executor, with source-order channel mappings.
- A [local NT-33A graph study](../../examples/nt33a-graph-design/README.md) with
  generated model/adapter evidence, graph history, a `sim.linear` reference
  history and reports retaining source linearization/controller diagnostics.
- `project import-linear`, immutable source-origin attachments, v2 imported
  drafts and explicit matching/modified/uncompiled relation to the original
  graph. Reruns retain origin bytes as context while keeping current graph
  evidence separate from source-operation diagnostics.
- Native study import, typed-row editing and an original-source inspector
  for matrices, channel mappings, controller and linearization evidence.
- [ADR-0013](../adr/0013-typed-linear-graph-adapter.md) and independent
  typed-row, lowering, pipeline and project-import acceptance cases.
- A Dim appearance across the native preview and keyboard-selectable native
  tables for blocks and numerical samples, alongside the diagram and line plot.
  Native table accessibility semantics expose block identifiers, kinds, original
  channel labels and numerical sample values for review.
- Bounded saved-revision listing, selected-draft inspection and explicit restore
  through the shared CLI and a read-only native review window. Stale heads,
  invalid retained drafts, schema changes and different source origins are
  refused; all newer revisions and immutable run evidence remain retained.
- [ADR-0014](../adr/0014-project-revision-recovery.md) and independent public
  recovery cases, with GUI, sanitizer and representative-workload verification
  recorded separately from the first two increments below.
- Diagram fitting and zoom, expanded diagram focus, spaced input ports and
  obstacle-avoiding connection routes. Explicit Arrange is an undoable
  presentation edit; view navigation preserves saved coordinates.

The synthetic starter uses the same continuous scalar feedback profile as M1.
The adapter adds graph construction and explicit coordinate-coupling semantics;
it introduces no separate integration solver or aircraft validity claim.
The continuous-model capabilities remain implemented and unvalidated in the
registry. Numerical accuracy, model validity and engineering acceptance remain
explicitly separate from execution completion.

## Try the project workflow

Build the existing development configuration, then create, inspect and run a
project in a new or empty directory:

```bash
cmake --preset dev
cmake --build --preset dev
./build/dev/src/cli/galata project create build/feedback.galata
./build/dev/src/cli/galata project inspect build/feedback.galata
./build/dev/src/cli/galata project run build/feedback.galata
```

To reconstruct the existing local aircraft/controller study in a new project:

```bash
./build/dev/src/cli/galata project import-linear build/nt33a.galata examples/nt33a-graph-design/study.yaml
./build/dev/src/cli/galata project inspect build/nt33a.galata
./build/dev/src/cli/galata project run build/nt33a.galata
```

Import executes a study with exactly one linear-graph export and one directly
connected graph simulation. It retains the complete import outputs and source
manifest, creates a typed graph draft and adopts that simulation's options.
Source channel types are explicit SI dimensions and frames. Imported source
evidence survives editing; it does not certify that later coefficients,
connections or initial conditions preserve the original plant or controller.

Save accepts a closed draft document and the exact revision observed when
editing began:

```text
galata project save <directory> <draft.json> --expected-revision <64-hex-revision>
```

The [project-file guide](../PROJECT_FILES.md) explains constructing that draft
from an inspect response, the object schemas and the generated directory layout.
Changing presentation creates a project revision without changing the engine's
semantic model digest. Changing simulation settings changes the submitted run
configuration. Saving a newer draft cannot mutate an already submitted request.

Review saved drafts and restore a compatible retained revision explicitly:

```text
galata project revisions <directory>
galata project revision <directory> <64-hex-revision>
galata project restore <directory> <64-hex-revision> --expected-revision <observed-head>
```

The listing orders retained filenames, without inferring chronology or ancestry.
Damaged entries remain visible with diagnostics. Restore moves only the current
head pointer; it does not delete newer revisions, alter submitted runs or remove
an imported source attachment. The current and selected drafts must both verify.
An invalid storage pointer or missing source origin needs a separate repair
protocol.

## Build the optional macOS preview

On macOS, enable the desktop target in the configured development build:

```bash
cmake -S . -B build/dev -DGALATA_BUILD_DESKTOP=ON
cmake --build --preset dev
```

The application bundle is **build/dev/src/desktop/Galata Preview.app**.
Open it in Finder. The bundle includes the same CLI worker, and its version
uses the existing product-version mechanism.
This is a development app bundle; signing, notarization, installer acceptance
and a clean-machine offline installation have not been established.

Create or open a project directory, or use Import Study to create an imported
project. The canvas supports the five M1 block kinds and typed linear rows in
the linear profile,
adding/removing and moving blocks, and connection source/target/input selectors.
The block JSON inspector exposes parameters, dimensions and frames. Apply
updates the working draft; Save validates it through the CLI and rejects stale
revisions. Incomplete wiring can be saved, and Run Saved applies full compiler
checks. Model edits have session Undo/Redo; saving clears that history.

The Block List provides native table selection by stable ID, kind and original
channel label. The Samples view exposes every finite parsed row within the CSV
preview limit; the line plot may display fewer points and retains its first and
final samples. Saved Revisions opens a separate read-only draft review before
restoration. Working edits must be saved first. These surfaces use the same
model and run records as the diagram, properties and evidence views.

Command-1 through Command-5 select Diagram, Block List, Plot, Samples and Block
Properties respectively; Command-H opens Saved Revisions. Native table semantics
improve keyboard and assistive-technology access, but complete VoiceOver and
accessibility acceptance remain unmeasured.

Run Saved launches the shared CLI as a separate process. Cancel requests worker
termination, and inspection distinguishes recorded cancellation from an
interrupted worker. Run history links the saved request to its results. The
native line plot offers trajectory-column selection; evidence and manifest
views retain the underlying diagnostic records. Plot display limits and
decimation do not change the stored trajectory or assess numerical reliability.
The Original Study view retains the imported matrices, labels, controller and
full source diagnostics. Current model identity is shown as matching the
import, modified from it or an uncompiled draft. A source label remains an
origin label after edits.
The [desktop guide](../../src/desktop/README.md) records interaction details and
display limits.

## Third increment verification

Observed local verification on 2026-09-07:

- Development and AddressSanitizer/UBSan builds pass. All three affected project
  CTest entries pass in development: 20 worker cases, six linear-import cases
  and ten new independent recovery cases. All ten recovery cases also pass under
  AddressSanitizer/UBSan. The complete numerical suites recorded for the second
  increment were not rerun for these UI and project-recovery changes.
- Recovery cases cover exact retained-draft restoration, selected-draft review,
  stale-head and invalid-target refusal, unchanged origins and retained runs,
  unfinished drafts, schema/origin incompatibility, malformed history entries,
  the 1,024-entry bound and denied publication writes. Refusals preserve the
  saved head and retained evidence.
- Actual native GUI checks pass for keyboard block selection, property focus,
  numerical sample access, read-only revision review, edit/save/restore,
  stale-head refusal and damaged retained entries. The table exposes all 401
  NT-33A samples; inspected initial, second and final values match the stored
  CSV. Both prior run directories remain byte-identical after restoration.
- The Dim palette is applied to the workspace, native tables, JSON editors,
  plots and review windows. Actual screenshots were inspected in the tool
  transcript. Opaque text/background palette pairs were also checked using
  sRGB relative luminance; these calculations and native accessibility exposure
  do not establish full VoiceOver or accessibility acceptance.
- Representative worker measurements cover 1, 16 and 64 scalar channels and
  the imported NT-33A graph, with three runs each. The
  [measurement script](../../scripts/measure-project-workflow.py) records command
  times, exact inputs, build identities and retained artifact sizes. Observed
  review costs increase with retained history. These Debug-build observations
  do not establish UI rendering performance or supported workload thresholds.
- Relocated installed C++ consumers, all four shipped CLI studies and the
  bundled worker pass. The local Dim example also passes review, save, restore
  and a new bundled-worker run while retaining its prior runs unchanged.
- Both affected CLI translation units pass GCC 15.1 strict object compilation.
  All 154 C/C++/Objective-C++ sources pass pinned formatting. Python syntax,
  SI/version and documentation-reference checks pass. The documentation link
  checker passes with seven allowed host-refusal warnings; the final added
  measurement-script link also resolves locally.

Development project records are build/dev/m2-dim-projects.xml and its companion
log; sanitizer recovery records use build/asan/m2-dim-recovery. GUI observations,
artifact hashes and palette calculations are under build/m2-dim-gui. Workload
inputs, exact measurements and their summary are under build/m2-recovery-workloads.
The installed local bundle, restored example and bundled-worker verification are
under build/m2-dim-preview. Actual screenshots remain in the tool transcript;
no standalone PNG is claimed. The final native guard-message and review-invalidation
rebuilds are recorded separately in the GUI report. Reopening a project invalidates
the old review snapshot even when its content ID is unchanged, so repaired storage
cannot leave a stale diagnostic on screen. These are local development observations, without
signing, clean-machine installation, hosted verification or full M2 acceptance.

## Second increment verification

Observed local verification on 2026-09-07:

- The complete development CTest suite passed all 465 entries: 463 C++ cases
  and two Python project suites containing 26 acceptance cases. This includes
  13 typed-row/adapter unit cases, seven adapter/context integration cases and
  six new public import/edit/run cases.
- The NT-33A comparison checks every one of the fixture's 401 samples, source
  state mappings, plant outputs and actual controls against the existing linear
  solver under the predeclared ADR-0013 rounding budget. Original matrices,
  channel types, LQR data, complete linearization diagnostics, consumed bytes
  and build identity remain bound to the imported study. This is integration
  consistency, not new aircraft validation.
- The complete AddressSanitizer/UBSan CTest suite also passed all 465 entries,
  including both project suites. No sanitizer failure or test timeout occurred.
- Eleven affected translation units pass GCC 15.1 object compilation with
  optimization and the complete GNU warning policy, including warnings as errors.
  This is a local compilation check, not a GCC runtime or hosted Linux claim.
- Relocated installed C++ consumers exercise the public lowering API and its
  transitive model dependency. Installed CLI examples, including the new NT-33A
  graph study, pass. The native bundle includes the shared worker and notices.
- Actual native GUI testing passed import, saved run and plot, separate Original
  Study inspection, controller-coefficient editing, save and rerun. The saved
  graph and new run show modified origin relation while the retained baseline
  run keeps its original relation. Finder reveal of the original manifest works.
  The baseline project was preserved. Cancel availability was observed; this
  second GUI smoke does not claim an interactive cancellation exercise.
- All 151 C/C++/Objective-C++ sources pass pinned formatting. The 28 non-project
  Python assurance tests, SI/version checks, documentation references and
  ShellCheck's CI threshold pass. All 363 documentation links pass the checker,
  with six allowed host-refusal warnings. Generated modal/report artifacts are unchanged;
  the generated V&V capability inventory gains the unvalidated adapter row.

Initial targeted checks exposed a relative-output-path assumption in import
and quoted CSV headers in the independent comparison reader. Both were corrected
before the complete development run. Optional source attachments now have an
explicit optional file-role declaration; required file-role preflight remains
unchanged. A malformed custom control-law artifact with an empty gain is refused.
GCC found a redundant test-only cast, which was removed. No numerical tolerance
or published discrepancy lock was relaxed.

The development/sanitizer suite records use the m2-linear prefix in their build
directories. Governance logs are under build/dev/m2-linear-checks; GCC records
are under build/gcc-m2. The native smoke report, observed accessibility excerpts,
artifact hashes and original/edited projects are under build/m2-aircraft-gui.
Actual screenshots were inspected in the tool transcript; no local PNG files
are claimed. The installed local bundle and example project are under
build/m2-aircraft-preview. These artifacts are local development evidence, not a
signed release or external acceptance dossier. The final CLI help addition is
documentation-only; it was rebuilt and checked after the complete development
run. The GUI manifests preserve each worker's exact executable identity across
intervening builds; the GUI smoke is not a same-binary numerical comparison.

## First increment verification record

Observed local verification of the first project/worker preview on 2026-09-07:

- All 443 C++ tests passed in the development and AddressSanitizer/UBSan builds,
  including the six independently authored pipeline cancellation tests.
- All 20 project/worker acceptance cases passed in development. The complete
  Python assurance suite also passed: 48 tests including those project cases.
- The sanitizer project run passed 19 cases and exceeded the cancellation
  fixture's ten-second deadline during provenance hashing. With that deadline
  aligned to the existing 45-second command allowance, the remaining case
  passed in 11.618 seconds. Its required cancelled status and retained request
  were unchanged. The earlier overall sanitizer fixture timeout was increased
  from 180 to 600 seconds; the subsequent complete run took 225.64 seconds.
- Nine affected translation units compile with GCC 15.1, optimization, the
  complete project warning set and warnings as errors. An errno-alias warning
  was corrected without changing platform behavior; its failure log is retained.
- Relocated installed C++ consumers, shipped CLI studies, project commands and
  the app's bundled worker passed. Bundle checks verify the worker matches the
  installed CLI and includes the required dependency notices. Loader inspection
  found system libraries/frameworks only in this local build.
- Native GUI smoke testing exercised Create/Open, property editing, Apply,
  Undo/Redo, drag/add/delete, Save, a real completed plot/evidence view, invalid
  wiring refusal with the old plot cleared, and cancellation/recovery.
- Pinned formatting, SI/version rules, generated reports/capability records,
  deterministic repeat/locale checks, documentation references/links, shell lint
  and whitespace checks passed. Six external documentation links returned the
  link checker's allowed host-refusal warnings.

Initial acceptance and review exposed quoted-number admission, subnormal/large
integer serialization and source-manifest binding defects. Corrections preserve
numeric lexemes and schema-defined strings, retain strict structural validation,
and verify the manifest's exact consumed source. Their independent regressions
pass. A macOS path-alias error in the tamper test fixture was corrected before
its successful rerun. No numerical tolerance or published discrepancy lock was
relaxed.

Development and sanitizer records are retained under their build directories
with the m2 prefix: complete C++ suite logs/JUnit, project reruns, the final
sanitizer cancellation case, Python and installed-consumer checks. GCC records
are under build/gcc-m2; actual GUI artifacts and its smoke report are under
build/m2-gui. The installed development bundle and starter project are under
build/m2-preview. These local artifacts are not a hosted release or external
acceptance dossier. Final changes after the complete C++ runs were formatting,
the equivalent errno portability guard, packaging metadata/checks, documentation
and test timeout/path corrections; affected project and installation paths were
verified afterward.

Hosted macOS/Linux verification and remote CI remain pending. The project
worker implements the POSIX boundary for macOS and Linux, but this record does
not claim a hosted Linux run or a supported Linux desktop. Existing Windows
engine CI remains separate; project commands refuse unsupported Windows use.

## Diagram usability follow-up

Hands-on use on 2026-09-07 exposed diagram defects that the earlier keyboard and
native-table smoke did not cover: crowded input labels, feedback wires hidden
behind cards, clipped layouts, unstable added-block placement and poor panel
resizing. The preview now provides spaced ports, routes around cards, single-wire
tracing, fitted opening and zoom, expanded diagram focus, and an explicit
undoable Arrange action. Negative saved coordinates use a display offset.

The native GUI check exercised the 19-block imported aircraft graph and the
five-block scalar starter, including selection, zoom, connection inspection,
invalid input refusal, dragging, Add, Undo/Redo, Arrange/Save, results switching
and window resizing. Undoing the only edit now returns to the saved state and
re-enables Run Saved. Viewing shifted-coordinate projects does not rewrite their
saved positions. The arranged aircraft copy changed presentation only; its model,
simulation, origin and original retained runs were preserved.

Seven permanent desktop routing tests pass in the strict development build.
They check endpoint direction, actual card intersection, forward/feedback/self
connections, the starter at translated positions, dense and varied layouts,
repeatability and explicit refusal cases. The normal starter check caught an
overly wide routing margin before delivery. These are presentation checks; the
full numerical and sanitizer suites were not repeated for this UI follow-up.

Local observations and artifact identities are retained under
build/diagram-usability, with an installed development preview and arranged
example under build/diagram-preview. This follow-up repairs the observed views;
it does not complete broader desktop or M2 acceptance.

## Wire editing follow-up

Further hands-on use on 2026-09-07 exposed the lack of direct connector editing
and difficulty distinguishing wires. The canvas now supports wire selection,
port dragging to create connections, and selected endpoint handles to reconnect
them. The connection controls offer explicit Create Wire, Update Wire, New Wire
and Remove Wire actions. Delete on a selected canvas wire removes the connection
while retaining its blocks. Occupied inputs are refused atomically; unchanged
updates do not dirty the draft. Undo/Redo covers connection edits.

Batch routing separates long shared paths where space permits. Colors and line
patterns follow each target block/input, with crossing gaps, endpoint labels and
single-wire emphasis. Unrelated removal preserves the remaining styles; W numbers
still reflect current list order. Shared output shafts and narrow passages can
overlap. The renderer exposes its 128-block and 256-connection limits.

Actual GUI checks selected an aircraft wire at 36% zoom and updated its source.
The saved QA draft changed exactly one connection, retaining 34 connections and
19 blocks; Undo/Redo restored and reapplied the edit. A six-block scalar fixture
at 87% zoom exercised output-to-input creation, source-handle reconnection and
target-handle reconnection from an occupied input to a free input. Empty-space
drops and occupied-input refusals preserved the existing wire without extra Undo
entries. Delete retained all six blocks; two Undo operations returned to the saved
state. An unchanged Update remained clean. A target-handle edit at 100% zoom was
also saved successfully. Escape cancellation is implemented but was not exercised
interactively in this check.

The strict development build and all 18 desktop tests pass: eleven routing cases
and seven connection-editing cases. These cover route geometry and separation,
atomic endpoint replacement, occupied or invalid inputs, stale selections and
unchanged edits. Local QA projects, saved-draft comparisons and the test record
are under build/wire-editing. These UI checks do not replace the numerical suite,
full assistive-technology workflows or broader M2/product acceptance.

## Line shaping and snapping follow-up

The 2026-09-07 follow-up adds round segment handles for reshaping orthogonal
wires while preserving their endpoints. Square endpoint handles still reconnect
the model. Straight wires can gain a dogleg. A blocked or out-of-bounds path is
refused without a history entry. Drag previews remain outside the draft until
release; effective moves form one Undo action. Selection, Undo/Redo and deletion
cancel pending gestures. Reset Route restores automatic routing.

Snap starts enabled: a 16-point logical canvas grid applies to blocks and wire
segments, and block edges/centers align within six screen points. Snap Off or
Option allows free movement. Guides preview alignment. Moving an attached block
resets its manual routes; Arrange resets all routes, with Undo restoring both
positions and shapes. A saved route obstructed by another block falls back to an
automatic path with a visible diagnostic. Valid custom lanes, including negative
coordinates, contribute to the canvas extent.

Custom routes use the bounded `galata.presentation.v2` format, identified by
the exact source/target/input connection tuple. Existing presentation v1 remains
accepted and is not upgraded by viewing. These are presentation edits: neither
the model graph, simulation settings nor imported origin changes. Older workers
that only accept presentation v1 refuse custom-route drafts. See the
[presentation contract](../PROJECT_FILES.md#presentation-and-manual-wire-routes).

Actual native GUI checks on isolated projects exercised straight-wire shaping,
blocked segment refusal, Undo/Redo, saving and reopening the custom shape,
Reset Route and Undo, square-handle reconnection and Undo, attached-block moves,
grid placement, peer-column alignment, and free movement with Snap Off. Saved
positions were checked as `(640, 320)` for grid placement, `(690, 336)` for
alignment to a peer at x=690, and fractional coordinates for free movement.
A manual lane at y=-32 remained visible with Fit All. On the 19-block, 34-wire
aircraft diagram, a wire was reshaped at 36% zoom and saved with its lower lane
at y=720; model, simulation, origin and block positions were unchanged. Escape
and Option bypass are implemented but were not exercised interactively here.

The strict development build passed. Final targeted CTest ran 30 desktop cases
and all four public project suites successfully. The new geometry cases cover
segment movement, fixed endpoints, card clearance, bounds, no-op edits, grid and
peer snapping, and display-coordinate roundoff. The seven ProjectRoutes cases
cover exact-byte v1 compatibility, both presentation versions, refusal without
head mutation, save/restore, imported origin and unchanged executable identity
and trajectories. Local artifacts and the native interaction record are under
build/line-snapping. A final native zoom check also repaired stale handle sizes
by invalidating the canvas after magnification changes; the corrected 36%-to-100%
transition was visually checked and all 30 desktop cases passed again. This does
not close broader desktop or M2 acceptance.

## Remaining delivery work

The current feature set has a [desktop candidate packaging
workflow](../desktop-packaging.md) with a separate optimized build, matching
deployment targets for dependencies, a post-link UI/worker provenance stamp,
archive integrity checks and a relocated saved-project smoke test. These local
candidate checks are part of the macOS CI job; hosted execution and public
distribution approval remain separate. JSON text editing now has its own Undo
history, model actions preserve pending valid edits, and closing the main editor
uses one unsaved-change guard. Finder opening is declared for `.galata` packages.

The preview is macOS first. Complete accessibility workflows, clean-machine
offline operation, representative workload performance, minimum OS/hardware
targets and recovery under broader storage faults need measured acceptance. The Qt
versus local web/native comparison remains open under D11; an installed
toolchain and a working preview do not select the long-term desktop stack.

The typed linear adapter reconstructs the existing local NT-33A
aircraft/controller study. It still needs broader desktop acceptance;
shared-engine agreement does not replace independent aircraft-data review. Nonlinear aircraft blocks, sampled
and hybrid execution, nested subsystems, reusable block libraries, migration,
autosave, campaigns and shared editing remain later increments. JSON property
editors, the bounded diagram renderer and the lack of multiselect remain
feasibility UI constraints.

Cancellation is cooperative at declared work boundaries. Process separation
does not establish hostile-code sandboxing, hard resource containment or a
wall-time guarantee. Run verification retains exact source/output/build
bindings; it does not establish cryptographic authorship, qualification,
airworthiness, customer acceptance or long-term evidence compatibility.

Named reviewers, aircraft data/rights decisions, support ownership and complete
external M1/M2 acceptance remain open. The next delivery step is to complete the
desktop comparison and the broader recovery, accessibility and
representative-workload evaluations against reviewed acceptance criteria.
Explicit saved-revision restore and native table access advance those
evaluations without closing them.
