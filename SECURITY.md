# Security policy

## What galata is, for the purpose of this policy

galata is an offline engineering tool. It is a command-line program and a set of
C++ libraries that read files from the local filesystem and write files to it.
It opens no sockets, serves no requests, runs no code it did not link at build
time, and has no authentication, session or privilege boundary of its own. It
runs with exactly the permissions of the user who invoked it.

That shape determines what a vulnerability in galata can be. The realistic
attack surface is **untrusted input parsed by a trusted process**: a study
`.yaml`, an aircraft model `.yaml`, or a validation reference `.csv` obtained
from somewhere the operator does not control, fed to a binary running as the
operator. Memory-safety faults reachable from that path are the class of bug
this policy exists for.

## Supported versions

| Version | Supported |
|---|---|
| `main` | yes |
| latest release | yes |
| everything older | no |

This is a pre-1.0 project with a single maintainer. Fixes land on `main` and
appear in the next release; there is no backport branch, and claiming one would
be a promise this project cannot keep.

## Reporting a vulnerability

**Report privately, not in a public issue.**

Use GitHub's private vulnerability reporting, which is enabled on this
repository:

**<https://github.com/celikgo/galata/security/advisories/new>**

That form is visible on the repository's Security tab. It creates a private
advisory that only you and the maintainer can read, and it lets the fix and the
disclosure be prepared together. If you have not used it before, GitHub
documents the flow at
<https://docs.github.com/en/code-security/security-advisories/guidance-on-reporting-and-writing-information-about-vulnerabilities/privately-reporting-a-security-vulnerability>.

A useful report contains the galata version (`galata --version`), the platform
and compiler, the input file that triggers it, and what you observed. A
reproducer that a stranger can run is worth more than a description of one.

### What to expect

This is a single-maintainer project, so these are honest targets rather than a
service-level agreement:

| Stage | Target |
|---|---|
| Acknowledgement that the report was read | 7 days |
| An assessment — accepted, needs more information, or out of scope, with the reasoning | 30 days |
| Fix on `main` for an accepted report | negotiated with you, based on severity |

If 7 days pass with no acknowledgement, the report has not reached anyone.
Open a public issue saying only that you have filed a private report and had no
response — no details — and that will surface it.

Disclosure is coordinated: the advisory is published once a fix exists, crediting
you unless you ask otherwise. If you intend to disclose on your own timetable,
say so in the report, and that will be worked with rather than argued about.

## What is in scope

- Memory-safety faults reachable from a malformed or hostile input file —
  out-of-bounds read or write, use-after-free, uncontrolled allocation driven by
  a field in the input.
- Path traversal or unintended file writes from a study file — a pipeline that
  writes outside the directory the operator nominated.
- Anything that causes galata to execute code named by an input file.
- A vulnerability in a pinned dependency (Eigen, fmt, yaml-cpp, GoogleTest) that
  is actually reachable through galata's use of it. If it is not reachable, it
  is still worth a normal issue, and it will be pinned forward.

## What is out of scope

These are not security vulnerabilities in this project. Several of them are
nonetheless serious bugs, and the right place for those is named.

- **A wrong number.** An incorrect derivative, a misclassified mode, a margin
  computed against the wrong convention: report these as ordinary issues, and
  they are treated as the most serious class of bug this project has. They are
  not a security matter, because galata's output crosses no trust boundary.
  See [`docs/VERIFICATION.md`](docs/VERIFICATION.md) for what is checked against
  a published document and what is explicitly unvalidated.
- **A crash on input the operator wrote themselves.** A malformed study file
  from the person running the tool should produce a clear diagnostic, and a
  crash instead is a bug — file it as one.
- **Denial of service by way of a large or deeply nested input.** A local tool
  that the operator can also just not run, or terminate, does not have a
  meaningful availability boundary.
- **Numerical non-determinism across platforms.** This is documented, bounded
  and gated. See [ADR-0004](docs/adr/0004-determinism-policy.md).
- **Missing hardening flags, or a scanner's output pasted without a reachable
  path through galata.** A finding with no route from an input file to the
  faulting code is a suggestion, not a report.

## What this project does not claim

Nothing in galata is DO-178C qualified, and this policy does not make it so.
It must never be used as evidence in a certification package. A clean security
report says that no one has reported a memory-safety fault; it says nothing
about airworthiness.
