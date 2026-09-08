# Contributing

## Build

```bash
export VCPKG_ROOT=/path/to/vcpkg
cmake --preset dev
cmake --build --preset dev
ctest --preset dev
```

Presets are in `CMakePresets.json`. `dev` is Debug with tests; `asan` adds
AddressSanitizer and UBSan. The `ci-*` presets are what CI runs and are
reproducible locally.

## Before you push

Run the gates CI will run. All five are fast, and only the link check needs a
network.

```bash
scripts/check-version-consistency.sh
scripts/check-si-boundary.sh
scripts/check-doc-references.sh
scripts/check-doc-links.sh

pip install 'clang-format==20.1.8'      # the exact version CI uses
# Tracked AND new files: `git ls-files` alone lists only what is already
# tracked, so a brand-new source is silently skipped and CI catches it instead.
{ git ls-files '*.cpp' '*.hpp' '*.h'
  git ls-files --others --exclude-standard '*.cpp' '*.hpp' '*.h'
} | sort -u | xargs clang-format -i
shellcheck --severity=warning scripts/*.sh
```

**If you develop on macOS, compile once with real GCC before pushing.** AppleClang
accepts several things GCC rejects under this project's warning set, and the CI
matrix will find them after you have pushed rather than before:

```bash
brew install gcc                          # provides g++-15
EIGEN=build/dev/vcpkg_installed/arm64-osx/include/eigen3
g++-15 -std=c++20 -O2 -Wall -Wextra -Wshadow -Wold-style-cast -Werror \
  -I include -I build/dev/generated/include -isystem "$EIGEN" \
  -c src/path/to/your.cpp -o /dev/null
```

Two classes account for most of it: `-Wshadow` inside GoogleTest macros, which
expand to a scope containing names you did not write, and `-Wold-style-cast`
reaching into third-party headers.

**A header must include what it uses, and nothing checks this for you.**
libstdc++ and libc++ pull `<stdexcept>`, `<algorithm>` and friends in
transitively through other standard headers, so a header that uses
`std::runtime_error` while including only `<map>` compiles on both supported
platforms. It is still wrong: its correctness depends on which standard library
it is compiled against and on what that library happens to include today, so it
breaks on a toolchain bump rather than on a change to this repository — and a
reader cannot tell what the header actually needs.

The MSVC job used to catch this, because MSVC's standard library does not carry
those transitive includes. Windows is no longer a supported platform and that
job is gone, so no gate catches it now: not the local GCC check above, not the
Linux and macOS builds, not clang-tidy, which CI runs with the analyzer checks
only. Include what you use because the header is incomplete without it, and
check it by reading the file — there is no longer a job that will tell you after
the fact.

**clang-format is pinned to 20.1.8.** Its output changes between major versions,
so an unpinned formatter means the gate fails on a change you cannot reproduce.
Install it from pip, not from your system package manager.

## What review will ask

Read [`docs/CHARTER.md`](docs/CHARTER.md) first — the nine rules there are gates,
not preferences, and most review comments are one of them restated.

The three that catch people:

- **Every public struct field carries its unit in a comment.** Metres, seconds,
  kilograms, radians. No exceptions in the numerical core
  ([ADR-0003](docs/adr/0003-strict-si-and-boundary-conversion.md)).
- **Every physics or numerics file cites its source** — author, title,
  publication, year — and states the model's validity envelope and the direction
  and magnitude of its known error.
- **Test reference values come from published documents, not from the code.** A
  test that freezes current output is a regression-lock, is named as one, and
  says which validated case it is anchored to
  ([`docs/TESTING.md`](docs/TESTING.md)).

## Conventions

Frames, attitude and the state vector are fixed by
[ADR-0002](docs/adr/0002-state-and-frame-conventions.md). Read it before writing
anything that touches a rotation. Quaternions are Hamilton, scalar-first,
body-to-NED; getting this wrong produces plausible numbers rather than errors,
which is why it is written down rather than inferred.

## Commits

Conventional-commit prefixes (`feat:`, `fix:`, `docs:`, `test:`, `build:`,
`refactor:`). One logical change per commit. A commit that adds a capability
adds its CI gate and its tests in the same commit — charter rule 1.

## Numerical changes and architecture

Before changing a public numerical contract, record its assumptions, units,
accepted domain, failure behavior and evidence type in an ADR. Include the
migration needed by library callers. Use the [ADR template](docs/adr/0000-template.md)
and [numerical evidence decision](docs/adr/0008-numerical-evidence-authority.md).
A pipeline execution success is not a numerical reliability or engineering
acceptance decision.

For a numerical defect, first retain an independent counterexample: an analytic
identity, a cited reference or a separately derived calculation. Test the direct
C++ entry point and the user-visible report when both expose the defect. Ask a
reviewer who did not implement the fix to check its assumptions and failure
cases. Do not loosen a published discrepancy lock or turn a current output into
its own reference value.

Run `python3 -m unittest discover -s tests/scripts` when changing build,
provenance or release gates. Include negative cases: missing checks, a different
commit, stale configuration or altered input must be refused. Release work is
reviewed against [ADR-0009](docs/adr/0009-release-evidence-and-source-identity.md).

Future block libraries and model examples must document their redistribution
license and source. The Simulink-style roadmap describes compatibility goals;
it does not grant permission to copy proprietary blocks, models or vendor code.
