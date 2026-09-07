#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail closed unless every dependency of the required CI job succeeded."""
import json
import os
import sys
from pathlib import Path

from ci_evidence import check_results

try:
    needs = json.loads(os.environ["NEEDS_JSON"])
    expected = os.environ["EXPECTED_SOURCE_SHA"]
    check_results(needs, expected)
    evidence = {"schema": "galata.ci-evidence.v1", "source_sha": expected, "jobs": needs,
                "workflow_ref": os.environ.get("GITHUB_WORKFLOW_REF", "local"),
                "workflow_sha": os.environ.get("GITHUB_WORKFLOW_SHA", "unknown"),
                "run_id": os.environ.get("GITHUB_RUN_ID", "local"),
                "run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT", "local")}
    if os.environ.get("CI_EVIDENCE_PATH"):
        Path(os.environ["CI_EVIDENCE_PATH"]).write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if os.environ.get("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a", encoding="utf-8") as output:
            output.write(f"source_sha={expected}\n")
except (KeyError, OSError, ValueError) as error:
    sys.exit(f"::error::{error}")
print(f"All required jobs passed for {expected}.")
