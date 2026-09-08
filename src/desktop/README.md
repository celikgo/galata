# Native macOS feasibility preview

This optional AppKit/Objective-C++ application exercises the shared project and
modeling engine. It is a feasibility preview, not a release-ready desktop product.
It uses the system AppKit frameworks and the existing C++ numerical worker.

The fixed dark appearance follows Twitter's Dim palette: navy `#15202B`,
surface `#192734`, raised surface `#22303C`, muted text `#8899AC`, and a blue
`#1DA1F2` accent. The [published Dim palette](https://www.color-hex.com/color-palette/99156)
provides the navy and surface references. Primary text uses `#F7F9F9` and borders
use `#38444D`; primary blue buttons use dark navy text for contrast. The theme
also covers native tables, JSON editors and source/revision review windows,
and stays dark when the host system uses a light appearance. Shared values live
in [`dim_theme.hpp`](dim_theme.hpp).

The `galata_desktop` CMake target creates `Galata Preview.app` and copies the shared
`galata` CLI into `Contents/MacOS`. Development launches can override its location
with `--engine /absolute/path/to/galata` and open a project with
`--project /absolute/path/to/example.galata`.
The bundle's minimum system version follows `CMAKE_OSX_DEPLOYMENT_TARGET` when
set, or the build host's macOS version otherwise. This is the build's deployment
requirement, not a tested minimum-OS support claim.

Create/Open choose a project directory. Import Study chooses a study YAML and a
new project destination, then runs `project import-linear` asynchronously through
the same CLI. For example, select
[`examples/nt33a-graph-design/study.yaml`](../../examples/nt33a-graph-design/study.yaml).
The study must export exactly one `model.linear_graph` and directly connect
exactly one `sim.model` stage. Cancel also applies to import. Failed or cancelled
imports retain incomplete files without publishing a project head; retry uses a
new or empty destination. See the [project protocol](../../docs/PROJECT_FILES.md)
for import bounds, origin storage and verification.

The canvas supports the five `continuous-scalar.v1` block kinds, plus
`linear_combination` rows when the project uses `continuous-linear.v1`. It
supports dragging, adding/removing blocks, direct wire editing and connection
source/target/input selectors. The block JSON inspector exposes
all existing parameters, dimensions and frames; keep `id` and `kind` unchanged.
Apply commits the inspector text to the draft; Save also applies pending text,
validates the draft through the shared CLI and uses an expected revision to reject
stale writes. Drafts may retain incomplete wiring; Run applies the full executable
compiler checks. Rejected drafts remain editable. Draft edits have 50 levels of
session Undo/Redo; successful Save clears that history. Run Saved requires a saved
draft and launches the same CLI as batch use. Cancel sends the child a termination
signal for cooperative cancellation; inspection recognizes abrupt worker death as
an interrupted run. Project switching and quitting guard unsaved drafts and active
commands.

Imported rows expose their ordered `terms` array in the block JSON inspector.
Each term declares an input type and a dimensional coefficient. Connection
selectors expose one port per term. Apply and Save preserve the imported draft
schema and its immutable origin hash; they cannot replace the source attachment.
The canvas expands its scrolling extent for larger imported layouts.

The diagram opens fitted to its content. **Fit All** returns to that overview;
the minus/plus controls change zoom and the percentage button restores 100%
detail. **Focus Diagram** expands the canvas while retaining connection controls;
**Show Results**, Command-3 or Command-4 brings the results back. Window resizing
keeps the inspector and result controls within their panels.

Cards provide spaced, numbered input ports and show original channel names when
available. Wires route around cards with separate lanes where space permits.
Colors, line patterns, crossing gaps and W labels distinguish connections;
selecting a block highlights its connections, while selecting one wire dims the
others. Colors and patterns follow the target block and input port, so removing
an unrelated wire does not change them. W numbers identify the current connection
list order and can change after removal. Shared output shafts and narrow passages
can still overlap.

Click a wire, or choose it in the connection selector, to populate its source,
target and input controls. Change those values and choose **Update Wire** to
replace that connection. **New Wire** leaves selection mode; choose endpoints
and **Create Wire** to add a connection. Repeated clicks at a crossing cycle the
nearby wires. **Remove Wire**, or Delete while a canvas wire is selected, removes
only that connection.

Drag an output port to an input to create a wire. Drag either square handle of
a selected wire to change its source or target; dragging an occupied input moves
that existing wire's target. A preview indicates the proposed drop and any
refusal. Dropping on empty space or pressing Escape cancels the wire drag.
Occupied inputs and nonexistent ports are refused without replacing another
wire. Successful connection edits have Undo/Redo; an unchanged update does not
dirty the draft.

To edit a line's shape, select the wire and drag a **round middle handle**, or
drag a visible segment itself. Horizontal segments move up/down; vertical
segments move left/right. The two ports stay connected. Moving a terminal
segment, including a straight wire, introduces bends with short endpoint stubs.
The square endpoint handles remain the controls for reconnecting ports. A path
that crosses a card or exceeds the canvas bounds is refused; releasing it leaves
the prior shape in place. Escape cancels the drag. A completed block or segment
drag is one Undo action; a click or an unchanged resulting position or shape
makes no edit.

**Snap On** starts enabled. Blocks and wire segments snap to a 16-point canvas
grid; blocks also align their edges or centers to another block within six
screen points, with visible alignment guides. **Snap Off** disables both, and
holding Option bypasses snapping during a drag. The snap preference changes the
view without changing the saved draft. Dragging supports edge scrolling, and
newly added blocks receive a free draft position and are brought into view.

**Reset Route** removes the selected wire's custom shape and restores automatic
routing; Undo restores that shape. Moving a block resets custom shapes on its
attached wires, and Undo restores both its position and those shapes. A moved
unrelated block can obstruct a retained custom route: the diagram then displays
an automatic path and a visible diagnostic. Select that wire to reshape it or
choose Reset Route. Save retains manual shapes in presentation data separately
from the model's connections and equations; see the
[presentation contract](../../docs/PROJECT_FILES.md#presentation-and-manual-wire-routes).

**Arrange** expands the current column/row order into a spaced layout. It is an
explicit presentation edit that resets all custom routes; Undo restores the
previous positions and shapes, and Save retains the arranged layout. Opening, fitting,
zooming and resizing do not rewrite saved coordinates, including negative ones.
Wire routing is bounded to 128 cards, 256 connections and a fixed search-work
limit. Overlapping cards, invalid ports or a rendering limit produce a visible
diagnostic; the block list and connection selector remain available. Routing
does not assess whether the model compiles or change execution semantics.

The Dim interface offers Diagram and Block List modes over the same draft.
Block List is a native table containing each stable block ID, kind and original
channel label when available. Arrow keys select rows and populate the existing
property editor; typing can locate a block. Selection stays synchronized with
the diagram. The list provides native table accessibility semantics in addition
to the custom canvas.

Saved Revisions opens a separate read-only window. Select a retained revision
to verify and inspect its complete saved draft before choosing Restore Reviewed
Revision. Save working edits before restoring. The command uses the observed
head to reject stale changes and requires the same draft schema and original
source attachment. Restore changes the saved head to that exact revision while
retaining all newer drafts and runs. Invalid entries remain visible with their
diagnostics; this does not repair a damaged current draft, pointer or origin.
The order is by retained filename, not inferred chronological save order. See
[ADR-0014](../../docs/adr/0014-project-revision-recovery.md) for the recovery boundary.

Original Study opens a separate read-only view containing the original adapter
and full manifest, including source matrices, channel types/mappings, LQR weights
and CARE diagnostics, and source linearization evidence. Reveal Original Manifest
locates that retained file in Finder. Selected imported state/control/output
blocks show their ORIGINAL channel labels alongside the stable graph ID. Those
labels describe the source channels even after parameter edits.

The saved graph relation is shown as matching the imported model, modified from
it or an uncompiled draft. Unsaved edits are identified separately; the saved
relation does not describe them. The original evidence remains source context.
Selected-run evidence and plots retain the executed revision's own identity and
relation, and do not inherit aircraft/controller validity from the origin.

Run history shows immutable execution records, JSON evidence/manifest, and a
native line plot with selectable trajectory columns, plus a native Samples
table. Completion is execution evidence only and does not claim numerical
acceptance. Samples exposes every finite parsed CSV row within the 16 MiB preview
limit; malformed or non-finite rows are omitted. The plot displays at most 12,000
samples, retaining the first and final parsed samples. A displayed count makes
the table/plot distinction visible. CSV files larger than 16 MiB are not
previewed; full artifacts remain in the project. Evidence and
selected-run manifest previews each have a 2 MiB limit. Subprocess output has an
8 MiB limit, including the project view used by Original Study.

Known feasibility limits: macOS only; JSON property editors rather than polished
per-kind forms; no multiselect, block renaming, project migration, autosave,
signing or notarization.
Keyboard shortcuts cover New/Open/Import/Save/Run, Undo/Redo, and ordinary text
editing. Additional navigation shortcuts are:

| Shortcut | Surface |
|---|---|
| Command-1 | Diagram |
| Command-2 | Block List |
| Command-3 | Plot |
| Command-4 | Samples |
| Command-5 | Block Properties |
| Command-H | Saved Revisions |

Arrow keys cycle canvas blocks; Delete removes the selected canvas wire, or the
selected block when no wire is selected. Native tables expose labelled rows and
columns, keyboard selection and sample values.
Core controls have accessibility labels. These interfaces have not yet completed
full VoiceOver or assistive-technology workflow acceptance; the custom canvas
and plot remain complemented by the native tables.
The app does not interpret a study's acceptance criteria or promise GUI parity
with all CLI capabilities.
