// SPDX-License-Identifier: Apache-2.0

#include "case_registry_check.hpp"

#include <gtest/gtest.h>

#include <set>
#include <string>

namespace galata::validation {

std::vector<std::string> unregistered_evidence(Binary binary) {
  // Everything GoogleTest knows about in THIS process. Parameterised suites
  // report their instantiated names, which is what the registry cites.
  std::set<std::string> registered;
  const ::testing::UnitTest& unit_test = *::testing::UnitTest::GetInstance();
  for (int suite = 0; suite < unit_test.total_test_suite_count(); ++suite) {
    const ::testing::TestSuite& test_suite = *unit_test.GetTestSuite(suite);
    for (int index = 0; index < test_suite.total_test_count(); ++index) {
      registered.insert(std::string(test_suite.name()) + "."
                        + test_suite.GetTestInfo(index)->name());
    }
  }

  std::vector<std::string> missing;
  for (const Case& validation_case : validation_cases()) {
    for (const Evidence& evidence : validation_case.evidence) {
      if (evidence.binary != binary) {
        continue;
      }
      if (registered.count(evidence.test) == 0) {
        missing.push_back("case '" + validation_case.id + "' cites " + to_string(binary) + " test '"
                          + evidence.test + "', which is not registered");
      }
    }
  }
  return missing;
}

}  // namespace galata::validation
