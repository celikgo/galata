// SPDX-License-Identifier: Apache-2.0
//
// Compiled into EVERY test binary, each with its own GALATA_TEST_BINARY.
//
// A process can only enumerate the tests linked into it, so no single place can
// verify the whole validation-case registry. This file is that single place
// compiled five times, and between them they cover every piece of evidence the
// registry declares.
//
// What it catches: a case citing a test that has been renamed, deleted, or that
// never existed. Without it, docs/VERIFICATION.md could cite evidence that is
// not there and nothing would notice — which is precisely the failure mode the
// generated table was introduced to end.

#include "case_registry_check.hpp"
#include <gtest/gtest.h>

#include <string>

namespace {

TEST(ValidationCaseRegistry, EveryPieceOfEvidenceInThisBinaryNamesARegisteredTest) {
  const std::vector<std::string> problems =
      galata::validation::unregistered_evidence(GALATA_TEST_BINARY);
  for (const std::string& problem : problems) {
    ADD_FAILURE() << problem;
  }
  if (!problems.empty()) {
    ADD_FAILURE() << "docs/VERIFICATION.md would cite evidence that does not exist. Either "
                     "restore the test, or update tools/validation/case_registry.cpp — but do "
                     "not leave the citation pointing at nothing.";
  }
}

}  // namespace
