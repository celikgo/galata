# Native desktop acceptance evidence

This is the manual acceptance record for the local macOS desktop candidate. It
is deliberately separate from automated C++ and CLI tests: a visible desktop
completion state proves the shell can drive the worker and publish evidence,
but it does not prove numerical accuracy, aircraft validity or certification.

## Latest v57 native accessibility-tree smoke

Date: 2026-09-20
Host: macOS arm64
Build: `build/ci-macos`
Scope: native UI workflow and accessibility-tree smoke, not a full VoiceOver,
clean-machine or release-signing acceptance

The rebuilt **Galata Preview** window was inspected through the macOS
accessibility tree. The tree exposed labeled actions for New/Open/Import/Save,
Run Saved, review-package verification, onboard manifest/staging/deployment,
target evidence, onboard SIL self-test, flight-test package creation and
qualification-chain verification. The View menu exposed Diagram, Block List,
Plot, Samples, Block Properties and Full Screen. The Block List appeared as an
accessible table with block ID, kind and original-channel columns. The onboard
SIL self-test was invoked through the File menu and visibly reported:
`Onboard SIL self-test passed`, three cycles, and
`qualification_state=not_qualified`.

This records a native accessibility-tree smoke and confirms the UI does not
hide the major production workflows. It does not close a formal VoiceOver
review, keyboard-only usability review, clean-machine installation, Developer
ID signing, notarization or stable-release acceptance.

## Current candidate package

Date: 2026-09-20
Host: macOS arm64
Candidate: `v0.3.0`, local ad-hoc candidate built from the rebuilt `build/ci-macos`
tree

Candidate ZIP:
`build/desktop-candidate-assets-final-20260920-v88/galata-desktop-v0.3.0-candidate-macos-arm64.zip`
The SHA-256 is recorded in the adjacent `.zip.sha256` file.

Candidate DMG:
`build/desktop-dmg-final-20260920-v88/galata-desktop-v0.3.0-candidate-macos-arm64.dmg`
The SHA-256 is recorded in the adjacent `.dmg.json` check record.

The DMG checker reported `status=verified`, `volume_inventory=passed` and
`bundle_signature=ad-hoc verified`. The package remains a local candidate:
Developer ID signing, notarization, CI approval and stable-release status are
not claimed. The CI macOS job now builds and runs the same DMG inventory check
alongside the ZIP candidate check. The native File menu now also exposes **Run Flight-Test
Validation…**, which invokes the shared package-backed CLI receipt workflow.
The qualification-chain dialog displays traceability separately from
`qualification_eligibility` and lists the blocking reasons while retaining
`qualification_state=not_qualified`.

## Historical v44 run

Date: 2026-09-20
Host: macOS arm64
Candidate: `v0.3.0`, local ad-hoc candidate produced by the desktop packaging
workflow
Candidate ZIP:
`build/desktop-candidate-assets-final-20260920-v44/galata-desktop-v0.3.0-candidate-macos-arm64.zip`
Candidate DMG:
`build/desktop-dmg-final-20260920-v44/galata-desktop-v0.3.0-candidate-macos-arm64.dmg`

The DMG inventory and digest verification reported `verified`; the candidate
is ad-hoc signed, not Developer ID signed or notarized.

## Observed workflows

1. Imported the published NT-33A graph study from
   `examples/nt33a-graph-design/study.yaml` through **File → Import Study**.
   Saved the imported project as
   `build/native-ui-acceptance-20260920.galata`. The desktop showed
   `ORIGINAL STUDY · Saved graph matches imported model`.
2. Used **File → Run Saved Project**. The desktop reported completion for run
   `db63496a8534738992ccc1ba9176fe5bcfb9615ec10d588916d80241763f6fdf5`,
   displayed the evidence pane as completed and produced a `response.csv` in
   the run output directory.
3. Used **File → Run Study** on
   `examples/a7a-trim-and-modes/study.yaml`, selecting
   `build/a7a-native-study-run-20260920` as the output directory. The desktop
   reported `Study completed` and exposed the worker log through the evidence
   pane.

These checks close the previously unobserved native import/save/run path. They
do not close the remaining release gates: accessibility review, clean-machine
installation, Developer ID signing, notarization, flight-test evidence,
target-HIL evidence or qualification authority acceptance.

## Latest v55 recovery acceptance

Date: 2026-09-20
Host: macOS arm64
Candidate package: `v0.3.0`, local ad-hoc v55 candidate
Candidate ZIP:
`build/desktop-candidate-assets-final-20260920-v55/galata-desktop-v0.3.0-candidate-macos-arm64.zip`
Candidate DMG:
`build/desktop-dmg-final-20260920-v55/galata-desktop-v0.3.0-candidate-macos-arm64.dmg`

Using the rebuilt native app, changed the saved project's simulation JSON without
applying or saving it, then terminated the app abruptly. The app wrote one bounded
user-local recovery snapshot containing the normalized project path, saved base
revision, draft schema and pending editor text. Reopening the same project displayed
**Recover unsaved draft?**; **Recover Draft** restored the pending `sample_stride: 20`
edit and marked the document unsaved. **Save** validated and published the next
revision, returning the setting to the fixture's original `sample_stride: 10` after
the test and removing the recovery snapshot. A snapshot based on a different saved
revision is refused rather than merged. This closes the local crash-recovery path;
it does not close the remaining signing, notarization, accessibility, clean-machine,
flight-test, target-HIL or qualification gates.

## Latest v47 spot check

Date: 2026-09-20
Host: macOS arm64
Candidate package: `v0.3.0`, local ad-hoc v47 candidate
Candidate ZIP:
`build/desktop-candidate-assets-final-20260920-v47/galata-desktop-v0.3.0-candidate-macos-arm64.zip`
Candidate DMG:
`build/desktop-dmg-final-20260920-v47/galata-desktop-v0.3.0-candidate-macos-arm64.dmg`

From the native **File → Run Study** flow, selected
`examples/a4d-trim-and-modes/study.yaml` and accepted the default output
directory `examples/a4d-trim-and-modes/study-run`. The desktop reported
`Study completed`, exposed the run evidence pane, and showed:

- 7 stages completed;
- 401 rows and 401 plotted samples;
- A-4D trim at alpha `4.420 deg`, elevator `0.623 deg`, thrust `8172 N`,
  residual `1.8e-15`;
- longitudinal and lateral finite-difference linearisation completed;
- `trim-and-modes.md` and the run manifest written to
  `examples/a4d-trim-and-modes/study-run`.

The desktop evidence also explicitly states that execution completion is not
numerical acceptance, flight-test validation, airworthiness or certification.
This v47 spot check confirms the current native runner path; it does not close
the remaining release gates listed above.
