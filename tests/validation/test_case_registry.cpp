// SPDX-License-Identifier: Apache-2.0
//
// The validation-case registry reconciled against the capability registry.
//
// docs/VERIFICATION.md's summary table is generated from the case registry.
// That removes the drift, but it does not by itself make the table TRUE: a
// hand-written case could still claim a validation that nothing performs. These
// tests are what close that gap.
//
// The most valuable of them is CapabilitiesClaimingValidationAreBackedByACase.
// `Capability::State::Implemented` renders as "implemented and validated" in
// the README, in `galata capabilities` and in the V&V report. It is a string in
// a struct initialiser and nothing stopped anyone typing it. Now something
// does.

#include "galata/pipeline/registry.hpp"

#include "case_registry.hpp"
#include <gtest/gtest.h>

#include <set>
#include <string>

namespace {

using galata::validation::Case;
using galata::validation::claims_a_comparison;
using galata::validation::Status;
using galata::validation::validation_cases;

TEST(ValidationCaseRegistry, IsNotEmptyAndHasUniqueIds) {
  const auto& cases = validation_cases();
  ASSERT_GE(cases.size(), 10U) << "the registry is suspiciously thin";

  std::set<std::string> ids;
  for (const Case& validation_case : cases) {
    EXPECT_FALSE(validation_case.id.empty());
    EXPECT_FALSE(validation_case.title.empty()) << validation_case.id;
    EXPECT_TRUE(ids.insert(validation_case.id).second)
        << "duplicate case id '" << validation_case.id << "'";
  }
}

TEST(ValidationCaseRegistry, EveryCaseClaimingAComparisonNamesAReferenceAndItsEvidence) {
  // A row saying "validated" with no reference is an assertion, not a
  // validation. A row saying "validated" with no evidence is worse: it looks
  // like one.
  for (const Case& validation_case : validation_cases()) {
    if (!claims_a_comparison(validation_case.status)) {
      continue;
    }
    EXPECT_FALSE(validation_case.reference.empty())
        << validation_case.id << " claims a comparison but names no reference";
    EXPECT_FALSE(validation_case.evidence.empty())
        << validation_case.id << " claims a comparison but names no evidence";
    EXPECT_FALSE(validation_case.note.empty())
        << validation_case.id
        << " claims a comparison but says nothing about the agreement measured";
  }
}

TEST(ValidationCaseRegistry, NotImplementedCasesCarryNoEvidence) {
  // There cannot be a passing test for something that does not exist. A case
  // that has both is one whose status was never updated.
  for (const Case& validation_case : validation_cases()) {
    if (validation_case.status != Status::NotImplemented) {
      continue;
    }
    EXPECT_TRUE(validation_case.evidence.empty())
        << validation_case.id << " is marked not implemented but cites evidence";
    EXPECT_TRUE(validation_case.reference.empty())
        << validation_case.id << " is marked not implemented but names a reference";
  }
}

TEST(ValidationCaseRegistry, EveryCapabilityNamedByACaseExists) {
  // Catches a capability renamed in the registry and not in the case list,
  // which would otherwise leave the reconciliation table silently short a row.
  const auto& registry = galata::pipeline::builtin_registry();
  for (const Case& validation_case : validation_cases()) {
    for (const std::string& capability : validation_case.capabilities) {
      EXPECT_NE(registry.find(capability), nullptr) << validation_case.id << " names capability '"
                                                    << capability << "', which is not registered";
    }
  }
}

TEST(ValidationCaseRegistry, CapabilitiesClaimingValidationAreBackedByACase) {
  // THE point of this file.
  //
  // `implemented and validated` appears in the README, in `galata
  // capabilities` and in the V&V report. Until now it was a value in a struct
  // initialiser that nothing checked. A capability may perfectly well be
  // `implemented, unvalidated` — that is an honest state — but it may not
  // claim validation without a case that performs one.
  for (const auto* capability : galata::pipeline::builtin_registry().all()) {
    if (capability->state != galata::pipeline::Capability::State::Implemented) {
      continue;
    }
    bool backed = false;
    for (const Case& validation_case : validation_cases()) {
      if (!claims_a_comparison(validation_case.status)) {
        continue;
      }
      for (const std::string& named : validation_case.capabilities) {
        if (named == capability->id) {
          backed = true;
        }
      }
    }
    EXPECT_TRUE(backed)
        << "capability '" << capability->id
        << "' declares State::Implemented, which renders as 'implemented and validated', but "
           "no validation case backs it. Either add a case to "
           "tools/validation/case_registry.cpp, or declare it "
           "State::ImplementedUnvalidated — which is an honest state and not a lesser one.";
  }
}

TEST(ValidationCaseRegistry, NotImplementedCasesDoNotNameACapabilityThatExists) {
  // The row that outlives the gap. If `sim.nonlinear` is ever registered, the
  // case still saying "not implemented" fails here rather than sitting in the
  // published report contradicting the capability list two sections below it.
  const auto& registry = galata::pipeline::builtin_registry();
  for (const Case& validation_case : validation_cases()) {
    if (validation_case.status != Status::NotImplemented) {
      continue;
    }
    for (const std::string& capability : validation_case.capabilities) {
      EXPECT_EQ(registry.find(capability), nullptr)
          << validation_case.id << " is marked not implemented, but capability '" << capability
          << "' is registered. The thing was built and the row was not updated.";
    }
  }
}

TEST(ValidationCaseRegistry, EveryStatusRendersToSomethingLegible) {
  for (const Status status : {Status::Validated,
                              Status::ValidatedWithCaveat,
                              Status::KnownDiscrepancy,
                              Status::Unvalidated,
                              Status::SelfConsistent,
                              Status::NotImplemented}) {
    const std::string rendered = galata::validation::to_string(status);
    EXPECT_FALSE(rendered.empty());
    EXPECT_NE(rendered, "unknown");
  }
}

}  // namespace
