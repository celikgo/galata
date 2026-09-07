# SPDX-License-Identifier: Apache-2.0
"""Fail-closed identity and result contracts for CI and release consumers."""
import re


REQUIRED_JOBS = {"format", "governance", "engine", "sanitizer", "static-analysis", "determinism"}


def source_sha(value):
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{40}", value):
        raise ValueError("source identity must be a full lowercase commit SHA")
    return value


def check_results(needs, expected_sha):
    source_sha(expected_sha)
    if not isinstance(needs, dict) or set(needs) != REQUIRED_JOBS:
        raise ValueError("required job results must name exactly: " + ", ".join(sorted(REQUIRED_JOBS)))
    for name, job in needs.items():
        if not isinstance(job, dict) or job.get("result") != "success":
            raise ValueError(f"required job did not succeed: {name}")
        outputs = job.get("outputs")
        if not isinstance(outputs, dict) or outputs.get("source_sha") != expected_sha:
            raise ValueError(f"required job has missing or mismatched source identity: {name}")


def check_evidence(evidence, expected_sha):
    if (not isinstance(evidence, dict) or evidence.get("schema") != "galata.ci-evidence.v1"
            or evidence.get("source_sha") != source_sha(expected_sha)):
        raise ValueError("CI evidence has missing or mismatched source identity/schema")
    check_results(evidence.get("jobs"), expected_sha)
    return evidence
