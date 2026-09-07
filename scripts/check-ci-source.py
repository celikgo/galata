#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify a job's checkout before it can contribute evidence for a source SHA."""
import os
import subprocess
import sys

from ci_evidence import source_sha


try:
    expected = source_sha(os.environ["EXPECTED_SOURCE_SHA"])
    if "VERIFIED_SOURCE_SHA" in os.environ and os.environ["VERIFIED_SOURCE_SHA"] != expected:
        raise ValueError("complete CI did not verify the requested source SHA")
    actual = subprocess.run(["git", "rev-parse", "HEAD"], check=True,
                            capture_output=True, text=True).stdout.strip()
    if actual != expected:
        raise ValueError(f"checkout is {actual}, expected {expected}")
    with open(os.environ["GITHUB_OUTPUT"], "a", encoding="utf-8") as output:
        output.write(f"source_sha={actual}\n")
    print(f"Verified source checkout: {actual}")
except (KeyError, OSError, ValueError, subprocess.CalledProcessError) as error:
    sys.exit(f"::error::{error}")
