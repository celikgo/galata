#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Every file path and every test name a document names in backticks must exist.
#
# WHY THIS EXISTS. The ADRs are enforced by review, and a correction pass found
# nine untrue claims across seven records with every gate in the repository
# green — because nothing reads a Markdown assertion and checks it. Three of
# those nine were of exactly one kind: a backticked NAME that had rotted.
# ADR-0003 pointed at `galata/core/units.hpp` for a header that has always been
# `galata/units.hpp`; ADR-0004 named a test that does not exist; ADR-0003 listed
# directories the SI gate had long since stopped skipping.
#
# A name is the one part of a documentation claim a machine can check. This gate
# checks that part, and only that part.
#
# WHAT IT DOES NOT CATCH, stated so the green tick is not read as more than it
# is. It cannot tell whether a sentence about a file is TRUE — only that the file
# is there. ADR-0006 had the sign of a moment-transfer equation reversed and
# ADR-0004 promised a bound with an unstated one-third carve-out; both name
# things that exist, and nothing here would have flagged either. Those still
# belong to review.
#
# A path that deliberately does not exist yet goes in
# scripts/doc-references-allow.txt with a justification. That file is the
# reviewable record of every place a document is describing an intention, which
# is charter rule 2 written down rather than assumed.

set -euo pipefail

cd "$(dirname "$0")/.."

docs="$(git ls-files '*.md')"
if [ -z "$docs" ]; then
  printf '::error::no Markdown files found — this gate would be vacuously green\n'
  exit 1
fi

sources="$(git ls-files 'tests/*.cpp')"
if [ -z "$sources" ]; then
  printf '::error::no test sources found — the test-name half of this gate would be vacuously green\n'
  exit 1
fi

python3 - "$PWD" <<'PY'
import os
import pathlib
import re
import subprocess
import sys

root = pathlib.Path(sys.argv[1])

# A backticked span is a PATH candidate only if it could not be anything else:
# no spaces, no operators, at least one slash, and either a known file extension
# or a trailing slash. That deliberately excludes the compiler flags, equations,
# YAML keys and C++ type names the documents are full of — `/fp:precise`,
# `M_cg = M_ref + r_cg_to_ref x F`, `altitude_m`, `std::mt19937_64`.
EXTENSIONS = ("md", "cpp", "hpp", "h", "c", "cc", "py", "sh", "yml", "yaml",
              "json", "txt", "cmake", "csv", "in", "html", "svg", "png")
PATH_SPAN = re.compile(r"[A-Za-z0-9._/-]+$")
PATH_TAIL = re.compile(r"\.(" + "|".join(EXTENSIONS) + r")$")

# A gtest name: two UpperCamel words joined by a dot. A value-parameterised name
# carries an instantiation prefix and a parameter suffix — Shapes/Suite.Test/0 —
# and the middle is what the macros declare, so it is stripped to that.
TEST_SPAN = re.compile(r"^(?:[A-Za-z0-9_]+/)?([A-Z][A-Za-z0-9_]*\.[A-Z][A-Za-z0-9_]*)(?:/[A-Za-z0-9_]+)?$")
TEST_MACRO = re.compile(r"\bTEST(?:_F|_P)?\(\s*([A-Za-z0-9_]+)\s*,\s*([A-Za-z0-9_]+)\s*\)")


def tracked(pattern):
    out = subprocess.run(["git", "ls-files", pattern], capture_output=True, text=True, check=True)
    return [line for line in out.stdout.split("\n") if line]


# --- what actually exists -------------------------------------------------
registered = set()
for source in tracked("tests/*.cpp"):
    text = (root / source).read_text(encoding="utf-8")
    for match in TEST_MACRO.finditer(text):
        registered.add(f"{match.group(1)}.{match.group(2)}")

allowed = {}
allow_file = root / "scripts" / "doc-references-allow.txt"
for number, line in enumerate(allow_file.read_text(encoding="utf-8").splitlines(), 1):
    if not line.strip() or line.lstrip().startswith("#"):
        continue
    entry, _, reason = line.partition("#")
    entry, reason = entry.strip(), reason.strip()
    if not reason:
        print(f"::error file=scripts/doc-references-allow.txt,line={number}::"
              f"'{entry}' has no justification. An exemption whose reason is not "
              f"written down is indistinguishable from a mistake nobody noticed.")
        sys.exit(1)
    allowed[entry] = reason


def path_resolves(candidate):
    if (root / candidate).exists():
        return True
    # A public header is written the way it is INCLUDED — `galata/units.hpp` —
    # not the way it sits on disk. That distinction is the whole reason
    # ADR-0003's wrong path survived review, so resolve both spellings.
    if (root / "include" / candidate).exists():
        return True
    # build_config.hpp is generated at configure time from a committed template.
    if (root / (candidate + ".in")).exists() or (root / "include" / (candidate + ".in")).exists():
        return True
    return False


# --- what the documents claim ---------------------------------------------
problems = []
paths_checked = tests_checked = 0
used_allowances = set()

for document in tracked("*.md"):
    fenced = False
    for number, line in enumerate((root / document).read_text(encoding="utf-8").splitlines(), 1):
        if line.lstrip().startswith("```"):
            fenced = not fenced
            continue
        # Fenced blocks are shell transcripts and file listings, full of paths
        # that are examples rather than claims — `/path/to/vcpkg` is not a
        # promise that the directory is in this repository.
        if fenced:
            continue

        for span in re.findall(r"`([^`]+)`", line):
            if PATH_SPAN.fullmatch(span) and "/" in span \
                    and not span.startswith(("/", "~", ".", "-")) \
                    and (PATH_TAIL.search(span) or span.endswith("/")):
                paths_checked += 1
                if span in allowed:
                    used_allowances.add(span)
                elif not path_resolves(span):
                    problems.append((document, number, "path", span))
                continue

            match = TEST_SPAN.match(span)
            if match:
                name = match.group(1)
                tests_checked += 1
                if span in allowed or name in allowed:
                    used_allowances.add(span if span in allowed else name)
                elif name not in registered:
                    problems.append((document, number, "test", name))

# --- report ----------------------------------------------------------------
print(f"Doc references: {paths_checked} path(s) and {tests_checked} test name(s) "
      f"across {len(tracked('*.md'))} document(s); "
      f"{len(registered)} registered tests; {len(allowed)} allowed exception(s).")

stale = sorted(set(allowed) - used_allowances)
if stale:
    # An allowance nothing uses is an admission that has outlived its document.
    print("::error::these entries in scripts/doc-references-allow.txt are no longer "
          "referenced by any document, so they are stale and should be deleted:")
    for entry in stale:
        print(f"  {entry}")
    sys.exit(1)

if problems:
    print(f"::error::{len(problems)} documentation reference(s) name something that does not exist:")
    for document, number, kind, span in problems:
        if kind == "path":
            print(f"::error file={document},line={number}::no such file or directory: {span}")
        else:
            print(f"::error file={document},line={number}::no test registered as {span}")
    print()
    print("  A name is the one part of a documentation claim a machine can check.")
    print("  Fix the name, or — if the reference is deliberately forward-looking —")
    print("  add it to scripts/doc-references-allow.txt with a justification.")
    sys.exit(1)

print("Doc reference check OK.")
PY
