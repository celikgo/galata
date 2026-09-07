# Desktop candidate packaging

The macOS desktop can be assembled as a self-contained local candidate. This
workflow produces an app and an integrity-checked ZIP; it does not create a
stable public release. The current desktop acceptance limits remain in the
[M2 implementation record](product/M2_IMPLEMENTATION.md). Public release evidence
is governed separately by [ADR-0009](adr/0009-release-evidence-and-source-identity.md).

Use macOS with Xcode command-line tools, CMake, Ninja, Python and the repository's
vcpkg manifest prerequisites. The [ci-macos preset](../CMakePresets.json) enables
the desktop and tests with RelWithDebInfo. It declares a macOS 14.0 deployment
target and uses the [desktop dependency triplets](../cmake/desktop-triplets) so
dependencies use the same target. A deployment target describes build
compatibility; it does not establish completed testing on every eligible OS or
hardware configuration. The package identifies its actual single architecture.

From the checkout root, build and test a frozen source tree before packaging:

```bash
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset ci-macos
cmake --build --preset ci-macos
ctest --preset ci-macos
python3 scripts/package-desktop.py build/ci-macos --output-dir build/desktop-candidate-assets
```

The output directory must be new or empty. Choose another directory for a second
candidate; the packager refuses to overwrite existing artifacts. Keep source
files, dependencies and build configuration unchanged between the build, checks
and packaging. Source changes, including documentation changes, invalidate the
source digest and require another build and the appropriate checks. Packaging
does not rebuild or rerun the complete test suite after the tested inputs have
been selected. Debug, sanitizer and shared-library builds are refused.

The [desktop target](../src/desktop/CMakeLists.txt) copies the built CLI into the
app and creates a POST_BUILD stamp. That stamp binds the UI executable hash,
worker executable hash, source inventory digest and effective build configuration
digest. The [packager](../scripts/package-desktop.py) requires this stamp to match
the current tree and binaries, then copies the app into a temporary directory
outside the checkout. It adds help and actual dependency notices, seals that
copy with an ad-hoc signature, and records the UI hash both before and after
sealing. The packaged worker must remain byte-identical to the built CLI.

The output contains a ZIP, its sibling SHA256 file and a package JSON record. The
ZIP contains:

- Galata Preview.app, including its worker, help, license notices and build
  identification inside the bundle.
- Startup instructions and selected scalar-feedback and NT-33A study/model
  inputs, with their relative directory structure retained.
- A complete source snapshot archive, its canonical inventory, effective build
  configuration and compilation commands, and the desktop POST_BUILD stamp.
- Package metadata with every payload file's SHA256 and executable mode.

The payload inventory excludes its own package metadata to avoid a recursive
digest. The sibling archive checksum identifies the complete ZIP, including
that metadata. Source snapshots use the existing
[source inventory contract](../scripts/provenance_support.py), which retains
tracked and nonignored source files while excluding declared build and generated
output locations. Dirty local source is permitted and identified explicitly;
it cannot be presented as a clean, CI-approved release.

Before returning any output, the packager invokes the
[archive checker](../scripts/check-desktop-package.py). It validates every ZIP
member before extraction, refusing traversal paths, symlinks, special files,
unsafe modes, duplicate or case-colliding paths and oversized archives. It then
checks the complete file/source inventories, build stamp, executable identities,
bundle metadata and dependency notices. After extraction and relocation to a
path containing spaces, it checks macOS loader dependencies and minimum OS
requirements, verifies the ad-hoc bundle and executable signatures, and runs the
bundled worker with a minimal environment.

The worker smoke creates and opens a project, edits and saves its presentation,
reopens the saved revision, runs it, and inspects the retained completed result.
It verifies the run manifest's executable/source/configuration identities, moves
the project, and checks the retained revision and result again. This exercises
the public project workflow and its integrity review; native GUI gestures and
clean-machine installation require separate acceptance.

To repeat verification, pass the actual emitted ZIP path:

```bash
python3 scripts/check-desktop-package.py /path/to/candidate.zip
```

Add `--metadata-only` to inspect archive bytes without executing packaged code.
That mode is portable and reports runtime verification as not run. The
[packaging contract tests](../tests/scripts/test_desktop_packaging.py) cover
malformed archives, changed files or modes, false release claims, stale build
stamps, mismatched source/configuration, missing source files and external loader
paths. Their synthetic files do not establish native application acceptance.

The [CI workflow](../.github/workflows/ci.yml) runs the macOS packaging step after
the engine CTest suite and retains the result as a candidate artifact. The
packager does not consume a complete CI approval record and deliberately labels
the artifact's CI approval as not claimed. A successful local invocation or a
candidate artifact is not proof that every required hosted job succeeded for an
immutable release commit; that is the separate ADR-0009 gate.

Extract the ZIP before launching the app. The entire app can be moved to another
directory without a compiler, Python installation, vcpkg checkout or network
service. Keep a project's whole directory together when moving or backing it up.
The [project file guide](PROJECT_FILES.md) explains retained revisions, saved-run
identity and presentation compatibility. Keep examples and models together if
using the supplied study inputs outside the app.

Ad-hoc signing establishes a local code/resource seal; it supplies no Developer
ID publisher identity or notarization. Gatekeeper may refuse a downloaded copy.
This workflow does not disable macOS security or provide a public signing step.
A stable desktop release still requires a committed/tagged candidate with the
complete required release evidence, Developer ID signing and notarization,
clean-machine offline installation acceptance, and measured desktop workflow,
accessibility, recovery and performance acceptance. Local checksums and source
stamps identify bytes; they do not establish authorship, independently reproduced
builds, engineering acceptance or tool qualification.
