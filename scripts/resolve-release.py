#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Resolve a version tag to an immutable commit, and optionally recheck it."""
import argparse
import os
import re
import subprocess
import sys

from ci_evidence import source_sha


def resolve(tag, expected=None):
    if not re.fullmatch(r"v[0-9]+\.[0-9]+\.[0-9]+", tag):
        raise ValueError("release tag must be v followed by the three-part source version")
    commit = subprocess.run(["git", "rev-parse", "--verify", f"refs/tags/{tag}^{{commit}}"],
                            check=True, capture_output=True, text=True).stdout.strip()
    source_sha(commit)
    if expected is not None and commit != source_sha(expected):
        raise ValueError("release tag moved after its source was verified")
    version = subprocess.run(["git", "show", f"{commit}:VERSION"], check=True,
                             capture_output=True, text=True).stdout.strip()
    if tag != "v" + version:
        raise ValueError("release tag does not name the source version")
    return commit


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tag")
    parser.add_argument("--expected-sha")
    args = parser.parse_args()
    try:
        commit = resolve(args.tag, args.expected_sha)
        if os.environ.get("GITHUB_OUTPUT"):
            with open(os.environ["GITHUB_OUTPUT"], "a", encoding="utf-8") as output:
                output.write(f"tag={args.tag}\nsource_sha={commit}\n")
        print(f"Release source: {args.tag} -> {commit}")
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        sys.exit(f"release resolution failed: {error}")
