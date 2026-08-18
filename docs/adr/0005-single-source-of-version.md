# ADR-0005: One source of version truth

- **Status:** accepted
- **Date:** 2026-08-18
- **Deciders:** project authors

## Context

A project with a desktop app, a CLI, a C++ library, a C plugin ABI, Python
bindings and a package manifest has at least six places a version number can be
written. They drift. The failure is not dramatic — a CLI reporting 0.4.1 while
the About box says 0.4.0 — but it undermines the one thing this project is
selling, which is that its outputs are traceable.

The plugin ABI version is a separate concern that is deliberately *not* tied to
the product version: a patch release must not imply an ABI change, and an
additive ABI ratchet must not force a product major.

## Decision

The `VERSION` file at the repository root contains a three-component semantic
version and nothing else. It is the only place a product version is written by
hand.

Everything else derives from it:

- CMake reads it with `file(READ)` and passes it to `project(... VERSION ...)`.
- `include/galata/build_config.hpp` is generated from `.in` by `configure_file`,
  carrying the version plus the compiler identification, build type and target
  system, so a result file can name the binary that produced it.
- The CLI's `--version`, the Python binding's `__version__` and the desktop
  About box read the generated header, directly or through the library. This is
  binding on each surface as it is added; none of the three exists yet, and the
  consistency gate lists them as skipped by path until they do.
- `vcpkg.json`'s `version-string` matches, checked rather than derived, because
  a JSON manifest cannot include a file.

The plugin ABI major version is a *separate* constant, `GALATA_ABI_VERSION`,
pinned to 1 for the life of the 1.x series, with an independently ratcheting
minor. See [ADR-0001](0001-independent-c-abi.md).

`scripts/check-version-consistency.sh` gates all of this. It checks values where
values can be compared and *mechanisms* where they cannot: CMake is checked for
`file(READ ... VERSION)` rather than for a matching literal, because a hardcoded
literal that happens to match today is exactly the defect this ADR prevents and
a value comparison would pass it.

The script names surfaces that do not exist yet as *skipped*, by path. A gate
that silently checks nothing is worse than no gate, and the skip list is the
honest record of what remains to be wired.

## Alternatives considered

**Derive the version from the git tag.** Standard practice, and it removes the
file entirely. Rejected: it makes a source tarball without git history
unversionable, it makes the version of an uncommitted working tree ambiguous,
and it means the number cannot be read without running a program. A file that a
human can `cat` is worth the manual edit at release time.

**Put the version in `vcpkg.json` and read it from there.** One fewer file.
Rejected: it couples the product version to a dependency manifest that exists to
serve one package manager, and CMake would have to parse JSON to bootstrap.

**Keep the ABI version and the product version the same number.** Simpler to
explain. Rejected: it forces a product major bump for an ABI break and, worse,
implies an ABI break on every product major. The two version the same thing for
different audiences and must move independently.

## Consequences

- Releasing means editing two files: `VERSION` and `vcpkg.json`. CI fails if
  only one is edited, which is the point.
- Every generated result carries `build_identification()`, which names version,
  compiler, build type and target system.
- Adding a new version-reporting surface means adding it to the gate's list in
  the same commit, or the gate will report it as an unchecked skip.

## Revisit when

The project gains a release-automation step that could own the edit, at which
point the manual edit could be replaced by a checked-in generator — but the file
stays.
