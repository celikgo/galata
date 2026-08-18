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

Run the gates CI will run. All four are fast.

```bash
scripts/check-version-consistency.sh
scripts/check-si-boundary.sh
scripts/check-doc-links.sh

pip install 'clang-format==20.1.8'      # the exact version CI uses
git ls-files '*.cpp' '*.hpp' '*.h' | xargs clang-format -i
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

**A header must include what it uses**, and MSVC is the only compiler that will
tell you when it does not. libstdc++ and libc++ pull `<stdexcept>`,
`<algorithm>` and friends in transitively through other standard headers;
MSVC's standard library does not. `std::runtime_error` used in a header that
only includes `<map>` compiles on Linux and macOS and fails on Windows. Neither
the local GCC check above nor CI's Linux and macOS jobs catch it — only the
Windows job does, after you have pushed.

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
