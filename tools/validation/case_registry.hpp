// SPDX-License-Identifier: Apache-2.0
//
// The validation-case registry: the single declared source for
// docs/VERIFICATION.md's summary table.
//
// WHY THIS EXISTS. That table was typed by hand into the report generator, and
// it drifted three separate times — most memorably by still saying "there is no
// aerodynamic model yet" in the same commit that added one. Every GENERATED
// table in this repository has held. The conclusion is not that more care is
// needed; it is that a hand-typed status line is a claim nothing checks.
//
// WHY NOT SIMPLY "GENERATE IT FROM THE CAPABILITY REGISTRY". Because the
// capability registry cannot answer the question. It knows about CAPABILITIES —
// `trim.level`, `analyze.modes` — while a validation case is a comparison
// against a published document. The two do not correspond one to one: the
// atmosphere has four cases and no capability of its own, the trim-and-linearise
// chain is one case spanning three capabilities, and "the quaternion conventions
// are self-consistent" belongs to no capability at all.
//
// So the cases are declared here, and they are RECONCILED with the capability
// registry by tests rather than by hope:
//
//   * every case whose status claims a comparison must name a reference and
//     must name its evidence;
//   * every piece of evidence must name a test that is actually registered in
//     the binary the case says it lives in — a renamed or deleted test fails
//     the check, so evidence cannot be fictional;
//   * every capability declaring `implemented and validated` must be covered by
//     at least one case that validates it, which makes that declaration
//     unfalsifiable by hand;
//   * every capability a case names must exist;
//   * a case marked `not implemented` must not name a capability that does
//     exist, so the row cannot survive the thing being built.
//
// The report generator then renders this list and nothing else.

#ifndef GALATA_TOOLS_VALIDATION_CASE_REGISTRY_HPP
#define GALATA_TOOLS_VALIDATION_CASE_REGISTRY_HPP

#include <string>
#include <vector>

namespace galata::validation {

enum class Status {
  // Compared against a published reference and agrees within a stated bound.
  Validated,
  // Compared and agrees, but only with a caveat the report spells out.
  ValidatedWithCaveat,
  // Compared and does NOT agree. The gap is measured, published, and held by a
  // labelled regression lock.
  KnownDiscrepancy,
  // Implemented, but no published reference has been compared against — either
  // none was found, or none was transcribed.
  Unvalidated,
  // Cross-checked internally against an independent implementation, which is
  // not the same as being validated against a document.
  SelfConsistent,
  // Does not exist yet.
  NotImplemented,
};

[[nodiscard]] std::string to_string(Status status);

// True for the statuses that assert a comparison was actually made, and
// therefore require a reference and evidence.
[[nodiscard]] bool claims_a_comparison(Status status);

// Which test binary a piece of evidence lives in. The registration check runs
// once per binary and verifies its own subset, because a binary can only
// enumerate the tests linked into it.
enum class Binary { Unit, Property, Integration, Validation, Determinism };

[[nodiscard]] std::string to_string(Binary binary);

struct Evidence {
  Binary binary = Binary::Validation;
  // The full gtest name, "Suite.Test". Checked against the tests actually
  // registered in that binary.
  std::string test;
};

struct Case {
  // Stable identifier. Appears nowhere in the rendered document; it exists so
  // that a case can be referred to in a commit message or an issue without
  // quoting its whole title.
  std::string id;
  std::string title;
  // Capability ids this case exercises. May be empty: plenty of what needs
  // validating is library-level and has no capability.
  std::vector<std::string> capabilities;
  // The publication compared against, with enough detail to find the table.
  std::string reference;
  Status status = Status::NotImplemented;
  std::vector<Evidence> evidence;
  // The measured agreement, the caveat, or why it is unvalidated. Rendered
  // into the table, so it is the place a reader looks for the number.
  std::string note;
};

[[nodiscard]] const std::vector<Case>& validation_cases();

}  // namespace galata::validation

#endif  // GALATA_TOOLS_VALIDATION_CASE_REGISTRY_HPP
